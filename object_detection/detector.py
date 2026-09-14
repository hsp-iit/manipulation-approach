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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformListener, Buffer

import message_filters
from ultralytics import SAM
from surface_detector_interfaces.msg import SegmentedPointcloud
from surface_detector_interfaces.srv import SegmentObject
from surface_detector_interfaces.action import ReachObject

import utils

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
                ('goal_update_min_interval', 1.0)       # [s] minimum time between two goals sent to navigation
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
        self.navigation_start_timeout = 20.0

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
        self.img_sub = message_filters.Subscriber(self, Image, img_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
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

        # DINO model : TODO set params for DINO configs path
        self.dino_model = load_model("/home/user1/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py",
                                     "/home/user1/GroundingDINO/weights/groundingdino_swint_ogc.pth")
        # Annotated img pub (debug only)
        self.annotated_img_pub = self.create_publisher(Image, "/annotated_dino_img", 10)
        # SAM
        self.sam = SAM("sam2.1_l.pt")
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
            pc_msg, full_pc_msg = utils.project_depth_to_pc_torch(depth_torch,
                                                                                   rgb_torch,
                                                                                   self.calib_mat,
                                                                                   self.camera_reference_frame,
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

        with self._lock:
            self._request_id += 1
            self._request_first_stamp = None
            self._last_sent_goal = None
            self._nav_goal_handle = None
            self._nav_started = False
            self._nav_goal_rejected = False
            self._nav_result_status = None
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
            self._busy = False
        # Goals still being sent to navigation are cancelled in nav_goal_response_callback
        if cancel_navigation and nav_goal_handle is not None:
            self.get_logger().info("Cancelling the navigation goal")
            nav_goal_handle.cancel_goal_async()

    def planner_goal_callback(self, msg : PoseStamped):
        """
        Forwards the approach goals of the planner to navigation, skipping stale goals
        and goals too similar to the last one sent (each new goal preempts the navigation)
        """
        with self._lock:
            if self.object_string == "" or self._request_first_stamp is None:
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
    exe.spin()

if __name__ == "__main__":
    main()
