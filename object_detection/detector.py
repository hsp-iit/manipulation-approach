# SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause
# Author: Simone Micheletti
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
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformListener, Buffer
import message_filters
from ultralytics import SAM
from surface_detector_interfaces.msg import SegmentedPointcloud
from surface_detector_interfaces.srv import SegmentObject

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
                ('text_threshold', 0.35)    #threshold of the text
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
        self.device = "cuda"

        self.get_logger().info(f'Using parameters: {img_topic=}  {depth_topic=}  {use_camera_info_topic=}  {camera_info_topic=} \n'
                               f'{self.object_pointcloud_topic=}  {self.robot_base_frame=}')

        # Enable camera info subscriber or load params
        if use_camera_info_topic == True:
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                camera_info_topic,
                self.camera_info_callback,
                10
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
        self.segment_object_srv = self.create_service(SegmentObject, self.get_name() + "/object_to_find", self.object_to_find)

        # DINO model : TODO set params for DINO configs path
        self.dino_model = load_model("/home/user1/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py", "/home/user1/GroundingDINO/weights/groundingdino_swint_ogc.pth")
        # Annotated img pub (debug only)
        self.annotated_img_pub = self.create_publisher(Image, "/annotated_dino_img", 10)
        # SAM
        self.sam = SAM("sam2.1_l.pt")
        self.annotated_sam_pub = self.create_publisher(Image, "/sam_mask_img", 10)  # for debug


    def camera_callback(self, img_msg : Image, depth_msg : Image):
        if not self.camera_info_available:
            self.get_logger().warn("Waiting for camera_info topic to become available")
            return
        # Do the callback only if we have to search for an object
        if self.object_string == "":
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
            caption=self.object_string,
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

    def object_to_find(self, request : SegmentObject.Request, response : SegmentObject.Response):
        if request.object_string is not None:
            self.object_string = request.object_string
            response.is_ok = True
            if self.object_string == "":
                self.get_logger().info("[object_to_find] Received empty string. Stopping looking for objects")
            else:
                self.get_logger().info(f"[object_to_find] Looking for object {self.object_string=}")
        else:
            response.is_ok = False
            response.error_msg = f"[object_to_find] None object received as: {request.object_string=}"
            self.get_logger().error(response.error_msg)
        
        return response

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
