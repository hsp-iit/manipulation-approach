# SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause
# Author: Simone Micheletti
import math
import threading
import time

import numpy as np

# Grounding DINO
from groundingdino.util.inference import load_model, predict, annotate
from PIL import Image as PIL_Image
import groundingdino.datasets.transforms as T
import cv2

import torch
from torchvision.ops import box_convert
# ROS2
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformListener, Buffer, TransformException
from tf2_geometry_msgs import do_transform_point

import message_filters
from ultralytics import SAM
from surface_detector_interfaces.msg import DetectionResults, SegmentedPointcloud
from surface_detector_interfaces.srv import SegmentObject
from surface_detector_interfaces.action import LowerHands, ReachObject

import utils

# Hands raise stage of a request
HANDS_WAITING = 0   # waiting for the surface height
HANDS_SENDING = 1   # sending the commands to the arm controllers
HANDS_MOVING = 2    # waiting for the end of the motion
HANDS_READY = 3     # hands above the surface: approach goals can reach navigation

class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")
        ### ROS2 Param declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('img_topic_name', "/camera/rgbd/img"),         # ergocub: /camera/rgbd/img
                ('depth_topic_name', "/camera/rgbd/depth"),     # ergocub: /camera/rgbd/depth
                ('camera_info_topic', "/camera/rgbd/camera_info"),   # ergocub: /camera/rgbd/camera_info
                ('use_camera_info_topic', True),
                ('cam_calib_mat', [386.0, 0.0, 321.0, 0.0, 386.0, 238.0, 0.0, 0.0, 1.0]),
                ('camera_reference_frame', "realsense_compensated"),  # Reference frame of the camera, if empty will use the one from the ros message
                ('robot_base_frame','geometric_unicycle'),  # mobile_base_body_link for R1, geometric_unicycle for ergoCub
                ('object_pointcloud_topic', 'seg_object_pointcloud'),
                ('full_pointcloud_topic', 'full_pointcloud'),
                ('box_threshold', 0.35),    #confidence threshold of the bbox to consider
                ('text_threshold', 0.35),   #threshold of the text
                ('planner_goal_topic', '/approach_planner/goal_pose'),  # approach goals computed by the approach_planner
                ('navigate_to_pose_action', 'navigate_to_pose'),
                ('nav_server_wait_timeout', 5.0),
                ('goal_update_min_distance', 0.15),     # [m] a new approach goal is sent to navigation only if it moved more than this
                ('goal_update_min_angle_deg', 10.0),    # [deg] or if it rotated more than this
                ('goal_update_min_interval', 1.0),      # [s] minimum time between two goals sent to navigation
                ('navigation_start_timeout', 20.0),     # [s] max time from the request to the first goal accepted by navigation
                ('dino_config_path', "/home/user1/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"),
                ('dino_weights_path', "/home/user1/GroundingDINO/weights/groundingdino_swint_ogc.pth"),
                ('sam_model', "sam2.1_l.pt"),
                ('raise_hands_before_approach', True),  # hands above the surface before any approach goal reaches navigation
                ('surface_results_topic', '/surface_detector/results'),
                ('left_arm_rpc_port', '/cartesian-control/left_arm/rpc:i'),
                ('right_arm_rpc_port', '/cartesian-control/right_arm/rpc:i'),
                ('arm_reference_frame', 'root_link'),   # root frame of the arm controllers
                ('arm_rpc_timeout', 1.0),               # [s]
                ('hand_forward_offset', 0.4),           # [m] hands position in the surface results frame, must match the approach_planner
                ('hand_lateral_offset', 0.2),           # [m] left hand at +offset, right hand at -offset
                ('hand_surface_clearance', 0.1),        # [m] hands height above the surface
                ('hands_raise_min_distance', 0.5),      # [m] minimum distance from the surface contour to raise the hands
                ('hand_retract_forward_offset', 0.15),  # [m] /lower_hands first pulls the hands back to here, in the base frame
                ('hands_motion_duration', 4.0),         # [s] trajectory duration
                ('hands_motion_timeout', 10.0)          # [s] max time for the arms to report the end of the motion
                ])
        # if self.object_string == "" we skip the callback
        self.object_string = ""
        # Name of the rgb image topic
        img_topic = self.get_parameter('img_topic_name').value
        # Name of the depth image topic
        depth_topic = self.get_parameter('depth_topic_name').value
        # Boolean flag to wether read the calibration matrix from topic (if true) or from parameters
        use_camera_info_topic = self.get_parameter('use_camera_info_topic').value
        # Name of the camera_info topic
        camera_info_topic = self.get_parameter('camera_info_topic').value
        # Reference frame of the camera, if empty will use the one from the ros message
        self.camera_reference_frame = self.get_parameter('camera_reference_frame').value
        # Base frame of the robot, counts as the robot pose
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        # Dino thresholds
        self.box_threshold = self.get_parameter("box_threshold").value
        self.text_threshold = self.get_parameter("text_threshold").value
        # Topic names
        self.object_pointcloud_topic = self.get_parameter("object_pointcloud_topic").value
        self.full_pointcloud_topic = self.get_parameter("full_pointcloud_topic").value
        planner_goal_topic = self.get_parameter("planner_goal_topic").value
        self.nav_action_name = self.get_parameter("navigate_to_pose_action").value
        self.nav_server_wait_timeout = self.get_parameter("nav_server_wait_timeout").value
        # Goal update thresholds, to avoid preempting navigation at every detection
        self.goal_update_min_distance = self.get_parameter("goal_update_min_distance").value
        self.goal_update_min_angle = math.radians(self.get_parameter("goal_update_min_angle_deg").value)
        self.goal_update_min_interval = self.get_parameter("goal_update_min_interval").value
        self.device = "cuda"
        self.navigation_start_timeout = self.get_parameter("navigation_start_timeout").value
        # Hands raise
        self.raise_hands = self.get_parameter("raise_hands_before_approach").value
        self.arm_reference_frame = self.get_parameter("arm_reference_frame").value
        self.hand_forward_offset = self.get_parameter("hand_forward_offset").value
        self.hand_lateral_offset = self.get_parameter("hand_lateral_offset").value
        self.hand_surface_clearance = self.get_parameter("hand_surface_clearance").value
        self.hands_raise_min_distance = self.get_parameter("hands_raise_min_distance").value
        self.hand_retract_forward_offset = self.get_parameter("hand_retract_forward_offset").value
        self.hands_motion_duration = self.get_parameter("hands_motion_duration").value
        self.hands_motion_timeout = self.get_parameter("hands_motion_timeout").value

        # Request and navigation state, shared by the action server, camera and navigation callbacks
        self._lock = threading.Lock()
        self._busy = False                  # a reach_object request is running
        self._request_id = 0
        self._request_first_stamp = None    # stamp of the first camera frame processed for the current request
        self._last_sent_goal = None         # (PoseStamped, send time) of the last goal sent to navigation
        self._nav_goal_seq = 0              # id of the latest goal sent to navigation
        self._nav_goal_handle = None        # handle of the latest navigation goal accepted
        self._nav_started = False
        self._nav_goal_rejected = False
        self._nav_result_status = None      # terminal status of the latest navigation goal
        self._hands_state = HANDS_READY
        self._hands_error = ""
        self._hands_move_start = None       # time.monotonic() of the arm commands
        self.feedback_dist = 0.0

        self.get_logger().info(f'Using parameters: {img_topic=}  {depth_topic=}  {use_camera_info_topic=}  {camera_info_topic=} \n'
                               f'{self.object_pointcloud_topic=}  {self.robot_base_frame=}  {planner_goal_topic=}  {self.nav_action_name=}')

        self.cb_grp = ReentrantCallbackGroup()

        # Enable camera info subscriber or load params
        if use_camera_info_topic == True:
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                camera_info_topic,
                self.camera_info_callback,
                10,
                callback_group=self.cb_grp
            )
            self.camera_info_available = False
        else:
            self.calib_mat = np.array(self.get_parameter('cam_calib_mat').value).reshape((3, 3))
            self.camera_info_available = True

        ### tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ### ROS2 subscribers
        # Keep only the latest frame: inference is slower than the camera, so queued frames would be processed late
        self.img_sub = message_filters.Subscriber(self, Image, img_topic, qos_profile=1)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=1)
        self.tss = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.depth_sub], 1, slop=0.1)
        self.tss.registerCallback(self.camera_callback)
        # Publishers
        self.object_pointcloud_pub = self.create_publisher(PointCloud2, self.get_name() + "/" + self.object_pointcloud_topic, 10)
        self.full_pointcloud_pub = self.create_publisher(PointCloud2, self.get_name() + "/" + self.full_pointcloud_topic, 10)
        self.ransac_plane_pub = self.create_publisher(PointCloud2, self.get_name() + "/ransac_plane", 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_name() + '/plane_markers', 10)
        self.seg_pc_pub = self.create_publisher(SegmentedPointcloud, self.get_name() + "/segmented_pointcloud", 10)
        # Services
        #self.segment_object_srv = self.create_service(SegmentObject, self.get_name() + "/object_to_find", self.object_to_find)

        # DINO model
        self.dino_model = load_model(self.get_parameter("dino_config_path").value,
                                     self.get_parameter("dino_weights_path").value)
        # Annotated img pub (debug only)
        self.annotated_img_pub = self.create_publisher(Image, "/annotated_dino_img", 10)
        # SAM
        self.sam = SAM(self.get_parameter("sam_model").value)
        self.annotated_sam_pub = self.create_publisher(Image, "/sam_mask_img", 10)  # for debug
        # Action server
        self.reach_object_action_server = ActionServer(self, action_type=ReachObject,
                                                       action_name="/reach_object",
                                                       execute_callback=self.reach_object_callback,
                                                       goal_callback=self.reach_object_goal_callback,
                                                       cancel_callback=lambda _: CancelResponse.ACCEPT,
                                                       callback_group=self.cb_grp)
        # Navigation: this node is the only one sending the approach goals to NavigateToPose
        # Not in the reentrant group: rclpy action clients can process the same goal response twice when run concurrently
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name,
                                       callback_group=MutuallyExclusiveCallbackGroup())
        # The hands are moved only with the surface known, and only this node commands them during the approach
        self.arms = None
        if self.raise_hands:
            from arm_interface import ArmInterface  # only here, so that YARP is needed only if the hands are moved
            self.arms = ArmInterface("/" + self.get_name(),
                                     {"left": self.get_parameter("left_arm_rpc_port").value,
                                      "right": self.get_parameter("right_arm_rpc_port").value},
                                     self.get_parameter("arm_rpc_timeout").value)
            self.lower_hands_action_server = ActionServer(self, action_type=LowerHands,
                                                          action_name="/lower_hands",
                                                          execute_callback=self.lower_hands_callback,
                                                          goal_callback=self.lower_hands_goal_callback,
                                                          cancel_callback=lambda _: CancelResponse.ACCEPT,
                                                          callback_group=self.cb_grp)
            self.surface_sub = self.create_subscription(DetectionResults,
                                                        self.get_parameter("surface_results_topic").value,
                                                        self.surface_results_callback, 10,
                                                        callback_group=MutuallyExclusiveCallbackGroup())
        # Mutually exclusive, so that navigation goals are sent in order
        self.planner_goal_sub = self.create_subscription(PoseStamped, planner_goal_topic, self.planner_goal_callback, 10,
                                                         callback_group=MutuallyExclusiveCallbackGroup())

    def camera_callback(self, img_msg : Image, depth_msg : Image):
        if not self.camera_info_available:
            self.get_logger().warn("Waiting for camera_info topic to become available")
            return
        # Do the callback only if we have to search for an object
        with self._lock:
            object_string = self.object_string
            if object_string != "" and self._request_first_stamp is None:
                self._request_first_stamp = Time.from_msg(depth_msg.header.stamp)
        if object_string == "":
            return
        start = time.time()
        # Convert ROS Image message to NumPy array (raw byte data) and then to Tensor
        rgb = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 3)
        rgb_torch = torch.tensor(rgb, device=self.device)
        depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)
        depth_torch = torch.tensor(depth, device=self.device)

        # Image preparation for model
        transform = T.Compose(
            [
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        image_pillow = PIL_Image.fromarray(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
        image_transformed, _ = transform(image_pillow, None)
        # Find categories
        boxes, logits, phrases = predict(
            model=self.dino_model,
            image=image_transformed,
            caption=object_string,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold
        )
        # Debug pub
        annotated_frame = annotate(image_source=rgb, boxes=boxes, logits=logits, phrases=phrases)
        debug_img_msg = utils.numpy_to_ros2_image(annotated_frame,
                                                                   img_msg.header.stamp,
                                                                   img_msg.header.frame_id)
        self.annotated_img_pub.publish(debug_img_msg)

        # If found something:
        if len(logits) > 0:
            # Find bbox with higher score
            highest_score = logits.max()
            bbox = boxes[logits == highest_score]
            # Convert bbox
            h, w, _ = rgb.shape
            bbox = bbox * torch.Tensor([w, h, w, h])
            xyxy = box_convert(boxes=bbox, in_fmt="cxcywh", out_fmt="xyxy").cpu().numpy()

            # Segment with SAM2
            sam_results = self.sam.predict(rgb, bboxes=xyxy, labels=[1])
            mask_bool = sam_results[0].masks.data.cpu().numpy()
            mask = mask_bool.astype(np.uint8)
            color_red = np.array([255, 0, 0], dtype=np.uint8)
            colored_mask = np.zeros_like(rgb, dtype=np.uint8)
            overlay = rgb.copy()
            for c in range(3):
                colored_mask[:, :, c] = mask * color_red[c]
            overlay = cv2.addWeighted(overlay, 1.0, colored_mask, 0.5, 0)
            sam_img = utils.numpy_to_ros2_image(overlay,
                                                                 img_msg.header.stamp,
                                                                 img_msg.header.frame_id,
                                                                 encoding = 'rgb8')
            self.annotated_sam_pub.publish(sam_img)
            # Convert to pointcloud2
            frame_id = self.camera_reference_frame if self.camera_reference_frame != "" else depth_msg.header.frame_id
            pc_msg, full_pc_msg = utils.project_depth_to_pc_torch(depth_torch,
                                                                                   rgb_torch,
                                                                                   self.calib_mat,
                                                                                   frame_id,
                                                                                   depth_msg.header.stamp,
                                                                                   mask=mask,
                                                                                   max_depth=3.0)
            self.object_pointcloud_pub.publish(pc_msg)
            self.full_pointcloud_pub.publish(full_pc_msg)
            seg_pc = SegmentedPointcloud()
            seg_pc.segmented_object = pc_msg
            seg_pc.full_pointcloud = full_pc_msg
            self.seg_pc_pub.publish(seg_pc)
            time_1 = time.time()
            self.get_logger().info(f"DINO + SAM2 time: {time_1 - start}")
            return

    def camera_info_callback(self, msg : CameraInfo):
        """
        Saves the calib matrix of the camera intrinsic parameters
        """
        if not self.camera_info_available:
            self.calib_mat = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            self.camera_info_available = True

    def reach_object_goal_callback(self, goal_request):
        """
        Accepts a new request only if no other one is running, since they would share the navigation goal
        """
        with self._lock:
            if self._busy:
                self.get_logger().warn(f"Rejecting request to reach {goal_request.object_string}: another request is running")
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def reach_object_callback(self, goal_handle : ServerGoalHandle):
        self.get_logger().info(f"Received request to reach object {goal_handle.request.object_string}")
        outcome = "abort"
        try:
            outcome, error_msg = self.execute_reach_request(goal_handle)
        finally:
            # Stop the object search, and the robot if we are not at the goal
            self.stop_request(cancel_navigation=(outcome != "succeed"))

        result = ReachObject.Result()
        result.reached = (outcome == "succeed")
        result.error_msg = error_msg
        if outcome == "succeed":
            goal_handle.succeed()
        elif outcome == "cancel":
            goal_handle.canceled()
        else:
            self.get_logger().warn(f"Request aborted: {error_msg}")
            goal_handle.abort()
        return result

    def execute_reach_request(self, goal_handle : ServerGoalHandle):
        """
        Runs the object search until navigation reaches the approach goal.
        Returns the outcome ("succeed", "cancel" or "abort") and the error message
        """
        if not self.nav_client.wait_for_server(timeout_sec=self.nav_server_wait_timeout):
            return "abort", f"Navigation action server {self.nav_action_name} not available"
        if self.raise_hands:
            missing_arms = self.arms.connect()
            if missing_arms:
                return "abort", f"Arm controllers not available: {', '.join(missing_arms)}"

        with self._lock:
            self._request_id += 1
            self._request_first_stamp = None
            self._last_sent_goal = None
            self._nav_goal_handle = None
            self._nav_started = False
            self._nav_goal_rejected = False
            self._nav_result_status = None
            self._hands_state = HANDS_WAITING if self.raise_hands else HANDS_READY
            self._hands_error = ""
            self._hands_move_start = None
            self.feedback_dist = 0.0
            # Start the object search
            self.object_string = goal_handle.request.object_string

        # Enable other nodes? -> TODO think how to do it (probably using srv, but it's an optional feature)

        feedback_msg = ReachObject.Feedback()
        start_wait_time = time.time()
        while True:
            time.sleep(0.2)
            if goal_handle.is_cancel_requested:
                return "cancel", "Request cancelled"
            with self._lock:
                nav_started = self._nav_started
                nav_goal_rejected = self._nav_goal_rejected
                nav_result_status = self._nav_result_status
                feedback_msg.distance_remaining = self.feedback_dist
                hands_error = self._hands_error
                hands_moving = self._hands_state == HANDS_MOVING
            if hands_error:
                return "abort", hands_error
            if hands_moving:
                hands_error = self.update_hands_motion()
                if hands_error:
                    return "abort", hands_error
            if nav_goal_rejected:
                return "abort", "Approach goal rejected by the navigation server"
            # Wait for the navigation to start
            if not nav_started:
                if (time.time() - start_wait_time) > self.navigation_start_timeout:
                    return "abort", "Timeout while starting the approach pipeline untill navigation"
                continue
            # Navigation started -> Now wait for it's end
            if nav_result_status is None:
                goal_handle.publish_feedback(feedback_msg)
                continue
            if nav_result_status == GoalStatus.STATUS_SUCCEEDED:
                return "succeed", ""
            return "abort", f"Goal failed with status: {nav_result_status}"

    def stop_request(self, cancel_navigation):
        with self._lock:
            self.object_string = "" # IMPORTANT: stop the search of objects
            self._request_first_stamp = None
            nav_goal_handle = self._nav_goal_handle
            self._nav_goal_handle = None
            hands_in_motion = self._hands_state in (HANDS_SENDING, HANDS_MOVING)
            self._hands_state = HANDS_READY
            self._busy = False
        if hands_in_motion:
            self.stop_arms()
        # Goals still being sent to navigation are cancelled in nav_goal_response_callback
        if cancel_navigation and nav_goal_handle is not None:
            self.get_logger().info("Cancelling the navigation goal")
            nav_goal_handle.cancel_goal_async()

    def lower_hands_goal_callback(self, _goal_request):
        """
        Accepts the request only if no other one is running, since they would share the arms
        """
        with self._lock:
            if self._busy:
                self.get_logger().warn("Rejecting request to lower the hands: another request is running")
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def lower_hands_callback(self, goal_handle : ServerGoalHandle):
        self.get_logger().info("Received request to lower the hands")
        outcome, error_msg = "abort", ""
        try:
            outcome, error_msg = self.execute_lower_hands(goal_handle)
        except (RuntimeError, TransformException) as error:
            error_msg = f"Could not lower the hands: {error}"
        finally:
            if outcome != "succeed":
                self.stop_arms()
            with self._lock:
                self._busy = False

        result = LowerHands.Result()
        result.lowered = (outcome == "succeed")
        result.error_msg = error_msg
        if outcome == "succeed":
            goal_handle.succeed()
        elif outcome == "cancel":
            goal_handle.canceled()
        else:
            self.get_logger().warn(f"Request aborted: {error_msg}")
            goal_handle.abort()
        return result

    def execute_lower_hands(self, goal_handle : ServerGoalHandle):
        """
        Lowers the hands in two stages, because go_home is a straight line and the arms have no collision
        avoidance: with the hands over a surface, it could cross the surface edge. So they first come back at the
        same height, behind any surface edge. Returns the outcome ("succeed", "cancel" or "abort") and the error
        """
        missing_arms = self.arms.connect()
        if missing_arms:
            return "abort", f"Arm controllers not available: {', '.join(missing_arms)}"
        targets = self.compute_retract_targets()
        if targets:
            outcome, error_msg = self.run_hands_stage(goal_handle, "retract",
                lambda arm: self.arms.go_to_position(arm, targets[arm], self.hands_motion_duration), targets)
            if outcome != "succeed":
                return outcome, error_msg
        return self.run_hands_stage(goal_handle, "go home", self.arms.go_home, self.arms.ports)

    def compute_retract_targets(self):
        """
        Positions behind any surface edge at the current height, in the arm controllers frame, of the hands that
        are ahead of them. It starts from the measured positions, as the hands can be anywhere after a stopped motion
        """
        to_base = self.tf_buffer.lookup_transform(self.robot_base_frame, self.arm_reference_frame, Time(),
                                                  timeout=Duration(seconds=1.0))
        to_arm = self.tf_buffer.lookup_transform(self.arm_reference_frame, self.robot_base_frame, Time(),
                                                 timeout=Duration(seconds=1.0))
        targets = {}
        for arm in self.arms.ports:
            hand = PointStamped()
            hand.point.x, hand.point.y, hand.point.z = self.arms.get_position(arm)
            hand = do_transform_point(hand, to_base)
            if hand.point.x > self.hand_retract_forward_offset:
                hand.point.x = self.hand_retract_forward_offset
                target = do_transform_point(hand, to_arm).point
                targets[arm] = [target.x, target.y, target.z]
        return targets

    def run_hands_stage(self, goal_handle, name, command, arms):
        """
        Commands the arms and waits for the end of the motion. Returns the outcome and the error message
        """
        for arm in arms:
            if not command(arm):
                return "abort", f"The {arm} arm controller refused to {name}"
        start = time.monotonic()
        while True:
            time.sleep(0.2)
            if goal_handle.is_cancel_requested:
                return "cancel", "Request cancelled"
            elapsed = time.monotonic() - start
            # The elapsed time guards from a status read before the controllers started the motion
            if elapsed >= self.hands_motion_duration and all(self.arms.is_motion_done(arm) for arm in arms):
                return "succeed", ""
            if elapsed > self.hands_motion_timeout:
                return "abort", f"Timeout during the {name} stage of lowering the hands"

    def stop_arms(self):
        for arm in self.arms.ports:
            try:
                self.arms.stop(arm)
            except RuntimeError as error:
                self.get_logger().error(f"Could not stop the arm: {error}")

    def surface_results_callback(self, msg : DetectionResults):
        """
        Starts raising the hands above the surface. It's done once per request, from the first valid surface
        """
        if len(msg.surface_contours) == 0:
            return
        with self._lock:
            if self._hands_state != HANDS_WAITING or self._request_first_stamp is None:
                return
            # Skip surfaces computed from camera frames older than the current request
            if Time.from_msg(msg.segmented_object.header.stamp) < self._request_first_stamp:
                return
            self._hands_state = HANDS_SENDING
            request_id = self._request_id
        try:
            targets = self.compute_hand_targets(msg)
            for arm, target in targets.items():
                if not self.arms.go_to_position(arm, target, self.hands_motion_duration):
                    raise RuntimeError(f"The {arm} arm controller refused the motion")
        except (RuntimeError, TransformException) as error:
            with self._lock:
                self._hands_error = f"Could not raise the hands: {error}"
            return
        with self._lock:
            request_active = self._hands_state == HANDS_SENDING and request_id == self._request_id
            if request_active:
                self._hands_state = HANDS_MOVING
                self._hands_move_start = time.monotonic()
        if not request_active:
            # The request ended while the commands were being sent: don't let the arms move
            self.stop_arms()

    def compute_hand_targets(self, msg : DetectionResults):
        """
        Hand positions above the surface, in the arm controllers frame. The arms have no collision avoidance
        and move in a straight line, so they are raised only far from the surface, to not hit it from below
        """
        contour = np.array([(p.point.x, p.point.y) for p in msg.surface_contours])
        distance = distance_to_polyline(contour)
        if distance < self.hands_raise_min_distance:
            raise RuntimeError(f"too close to the surface ({distance:.2f} m, minimum {self.hands_raise_min_distance:.2f} m)")
        header = msg.surface_contours[0].header
        transform = self.tf_buffer.lookup_transform(self.arm_reference_frame, header.frame_id, Time(),
                                                    timeout=Duration(seconds=1.0))
        targets = {}
        for arm, side in (("left", 1.0), ("right", -1.0)):
            hand = PointStamped(header=header)
            hand.point.x = self.hand_forward_offset
            hand.point.y = side * self.hand_lateral_offset
            hand.point.z = msg.surface_contours[0].point.z + self.hand_surface_clearance
            target = do_transform_point(hand, transform).point
            targets[arm] = [target.x, target.y, target.z]
        return targets

    def update_hands_motion(self):
        """
        Checks the end of the hands motion. Returns the error message, if any
        """
        elapsed = time.monotonic() - self._hands_move_start
        # The elapsed time guards from a status read before the controllers started the motion
        try:
            done = elapsed >= self.hands_motion_duration and all(self.arms.is_motion_done(arm) for arm in self.arms.ports)
        except RuntimeError as error:
            return f"Could not raise the hands: {error}"
        if done:
            with self._lock:
                if self._hands_state == HANDS_MOVING:
                    self._hands_state = HANDS_READY
            return ""
        if elapsed > self.hands_motion_timeout:
            return "Timeout while raising the hands"
        return ""

    def planner_goal_callback(self, msg : PoseStamped):
        """
        Forwards the approach goals of the planner to navigation, skipping stale goals
        and goals too similar to the last one sent (each new goal preempts the navigation)
        """
        with self._lock:
            if self.object_string == "" or self._request_first_stamp is None:
                return
            # The hands must be above the surface before the robot gets close to it
            if self._hands_state != HANDS_READY:
                return
            # Skip goals computed from camera frames older than the current request
            if Time.from_msg(msg.header.stamp) < self._request_first_stamp:
                return
            now = time.monotonic()
            if self._last_sent_goal is not None:
                last_goal, last_time = self._last_sent_goal
                if now - last_time < self.goal_update_min_interval:
                    return
                if (last_goal.header.frame_id == msg.header.frame_id and
                        goal_distance(last_goal, msg) < self.goal_update_min_distance and
                        goal_yaw_difference(last_goal, msg) < self.goal_update_min_angle):
                    return
            self._last_sent_goal = (msg, now)
            self._nav_goal_seq += 1
            seq = self._nav_goal_seq
            request_id = self._request_id
            self._nav_result_status = None

        self.get_logger().info(f"Sending approach goal to navigation: X: {msg.pose.position.x:.3f} Y: {msg.pose.position.y:.3f}")
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = msg
        future = self.nav_client.send_goal_async(nav_goal,
                                                 feedback_callback=lambda feedback: self.nav_feedback_callback(feedback, seq))
        future.add_done_callback(lambda f: self.nav_goal_response_callback(f, seq, request_id))

    def nav_goal_response_callback(self, future, seq, request_id):
        nav_goal_handle = future.result()
        with self._lock:
            request_active = self.object_string != "" and request_id == self._request_id
            is_latest = seq == self._nav_goal_seq
            if request_active and is_latest:
                if nav_goal_handle.accepted:
                    self._nav_goal_handle = nav_goal_handle
                    self._nav_started = True
                else:
                    self._nav_goal_rejected = True
        if not nav_goal_handle.accepted:
            return
        if not request_active:
            # The request ended while the goal was being sent: don't let the robot move
            nav_goal_handle.cancel_goal_async()
            return
        # Older goals are preempted by the newer one, so we don't track them
        if is_latest:
            nav_goal_handle.get_result_async().add_done_callback(lambda f: self.nav_result_callback(f, seq))

    def nav_result_callback(self, future, seq):
        with self._lock:
            if seq == self._nav_goal_seq:
                self._nav_result_status = future.result().status

    def nav_feedback_callback(self, feedback_msg : NavigateToPose.Impl.FeedbackMessage, seq):
        with self._lock:
            if seq == self._nav_goal_seq:
                self.feedback_dist = feedback_msg.feedback.distance_remaining

def distance_to_polyline(points):
    """
    Distance of the origin from the closed polyline with the given Nx2 points
    """
    A = points
    AB = np.roll(points, -1, axis=0) - A
    length_sq = np.maximum(np.sum(AB * AB, axis=1), 1e-12)
    t = np.clip(-np.sum(A * AB, axis=1) / length_sq, 0.0, 1.0)
    return float(np.min(np.linalg.norm(A + t[:, None] * AB, axis=1)))

def goal_distance(a : PoseStamped, b : PoseStamped):
    return math.hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y)

def goal_yaw_difference(a : PoseStamped, b : PoseStamped):
    diff = yaw_from_quaternion(a.pose.orientation) - yaw_from_quaternion(b.pose.orientation)
    return abs(math.atan2(math.sin(diff), math.cos(diff)))

def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def main():
    rclpy.init()
    node = ObjectDetector()
    print(f"Created {node.get_name()}")
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    print(f"Spinning Node {node.get_name()}")
    try:
        exe.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.arms is not None:
            node.arms.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
