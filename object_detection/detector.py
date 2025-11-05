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
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, CameraInfo, Image, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import TransformListener, Buffer
import message_filters
import open3d as o3d
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
from ultralytics import YOLOWorld, SAM
from sensor_msgs_py import point_cloud2
from scipy.spatial import cKDTree
from shapely import Polygon
from shapely import Point as Point_Shapely

TEXT_PROMPT = "bottle"
BOX_TRESHOLD = 0.35
TEXT_TRESHOLD = 0.35    #0.25

def numpy_to_ros2_image(rgb: np.ndarray, stamp, frame_id="camera_rgb_frame", encoding = 'bgr8') -> Image:
    msg = Image()
    msg.height = rgb.shape[0]
    msg.width = rgb.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = rgb.shape[1] * 3
    msg.data = rgb.tobytes()

    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    return msg

def numpy_to_pc2_msg(pointcloud: np.ndarray, color: np.ndarray, header):
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize
    fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyzrgb')]
    nbytes = 6
    xyzrgb = np.array(np.hstack([pointcloud, color/255]), dtype=np.float32)
    pc2_msg = PointCloud2(header=header, 
                      height = 1, 
                      width= pointcloud.shape[0], 
                      fields=fields, 
                      is_dense= False, 
                      is_bigedian=False, 
                      point_step=(itemsize * nbytes), 
                      row_step = (itemsize * nbytes * pointcloud.shape[0]), 
                      data=xyzrgb.tobytes())
    return pc2_msg

def project_depth_to_pc_torch(depth : torch.Tensor, color_img : torch.Tensor, calib_matrix, frame_id, stamp, mask = None, min_depth = 0.2, max_depth = 10.0, depth_factor=1.0):
    """
    Creates the 3D pointcloud, in camera frame, from the depth and alignes RGB color for each 3D point.
    Uses tensors to speed up the process. Uses GPU
    
    :param depth: matrix of shape (W , H), depth image from the camera
    :param color_img: matrix of shape (W , H), color image image from the camera
    :param calib_matrix: matrix of shape (3, 3) containing the intrinsic parameters of the camera in matrix form
    :param min_depth: (float) filters out the points below this Z distance: must be positive
    :param max_depth: (float) filters out the points above this Z distance:  must be positive
    :param depth_factor: (float) scale factor for the depth image (it divides the depth z values)
    :return: Pontcloud2 msg
    """
    fx = calib_matrix[0, 0]
    fy = calib_matrix[1, 1]
    cx = calib_matrix[0, 2]
    cy = calib_matrix[1, 2]
    #intrisics = [[fx, 0.0, cx],
    #             [0.0, fy, cy],
    #             [0.0, 0.0, 1.0 / depth_factor]]

    # Hardcoded sanity check
    if min_depth < 0.0:
          min_depth = 0.2
    if max_depth < 0.0:
          max_depth = 6.0
    if mask is None:
        mask = np.ones_like(depth.shape(), dtype=np.uint8)
    mask_torch = torch.tensor(mask, dtype=torch.uint8, device=depth.device)

    # filter depth coords based on z distance
    uu, vv = torch.where((depth > min_depth) & (depth < max_depth))
    full_xx = (vv - cx) * depth[uu, vv] / fx
    full_yy = (uu - cy) * depth[uu, vv] / fy
    full_zz = depth[uu, vv] / depth_factor
    xx = (vv - cx) * depth[uu, vv] * mask_torch[0][uu, vv] / fx
    yy = (uu - cy) * depth[uu, vv] * mask_torch[0][uu, vv]/ fy
    zz = depth[uu, vv] * mask_torch[0][uu, vv] / depth_factor
    condition = (xx != 0) & (yy != 0) & (zz != 0)
    xx = xx[condition]
    yy = yy[condition]
    zz = zz[condition]
    color = color_img[uu, vv, :]
    color_cpu = color[condition].detach().cpu().numpy()
    full_color_cpu = color.detach().cpu().numpy()
    pointcloud = torch.cat((xx.unsqueeze(1), yy.unsqueeze(1), zz.unsqueeze(1)), 1).detach().cpu().numpy()
    full_pointcloud = torch.cat((full_xx.unsqueeze(1), full_yy.unsqueeze(1), full_zz.unsqueeze(1)), 1).detach().cpu().numpy()
    #uu, vv = uu.cpu().detach().numpy(), vv.detach().cpu().numpy()

    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize
    fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyzrgb')]
    nbytes = 6
    xyzrgb = np.array(np.hstack([pointcloud, color_cpu/255]), dtype=np.float32)
    msg = PointCloud2(header=header, 
                      height = 1, 
                      width= pointcloud.shape[0], 
                      fields=fields, 
                      is_dense= False, 
                      is_bigedian=False, 
                      point_step=(itemsize * nbytes), 
                      row_step = (itemsize * nbytes * pointcloud.shape[0]), 
                      data=xyzrgb.tobytes())
    #
    full_xyzrgb = np.array(np.hstack([full_pointcloud, full_color_cpu/255]), dtype=np.float32)
    full_msg = PointCloud2(header=header, 
                      height = 1, 
                      width= full_pointcloud.shape[0], 
                      fields=fields, 
                      is_dense= False, 
                      is_bigedian=False, 
                      point_step=(itemsize * nbytes), 
                      row_step = (itemsize * nbytes * full_pointcloud.shape[0]), 
                      data=full_xyzrgb.tobytes())

    return msg, full_msg

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
                ])
        self.use_yolo = False   # TODO remove
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
        # Topic names
        self.object_pointcloud_topic = self.get_parameter("object_pointcloud_topic").value
        self.full_pointcloud_topic = self.get_parameter("full_pointcloud_topic").value
        self.device = "cuda"

        self.get_logger().info(f'Using parameters: {img_topic=}  {depth_topic=}  {use_camera_info_topic=}  {camera_info_topic=} \n'
                               f'{self.object_pointcloud_topic=}  {self.robot_base_frame=}')

        # Enable camera ingo subscriber or load params
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
        self.object_pointcloud_pub = self.create_publisher(PointCloud2, self.object_pointcloud_topic, 10)
        self.full_pointcloud_pub = self.create_publisher(PointCloud2, self.full_pointcloud_topic, 10)
        self.ransac_plane_pub = self.create_publisher(PointCloud2, "/ransac_plane", 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/plane_markers', 10)

        # DINO model
        self.dino_model = load_model("/home/user1/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py", "/home/user1/GroundingDINO/weights/groundingdino_swint_ogc.pth")
        # Annotated img pub (debug only)
        self.annotated_img_pub = self.create_publisher(Image, "/annotated_dino_img", 10)
        # YOLO (TODO remove)
        if self.use_yolo:
            self.yolo = YOLOWorld("yolov8l-worldv2")
            self.yolo.set_classes(["chair" , "laptop" , "mouse" , "bag" , "box" , "backpack" , "mug" , "bottle"])
            self.annotated_yolo_pub = self.create_publisher(Image, "/annotated_yolo_img", 10)
        # SAM
        self.sam = SAM("sam2.1_l.pt")
        self.annotated_sam_pub = self.create_publisher(Image, "/sam_mask_img", 10)


    def camera_callback(self, img_msg : Image, depth_msg : Image):
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
            caption=TEXT_PROMPT,
            box_threshold=BOX_TRESHOLD,
            text_threshold=TEXT_TRESHOLD
        )
        # Debug pub
        #if len(logits) > 0:
        annotated_frame = annotate(image_source=rgb, boxes=boxes, logits=logits, phrases=phrases)
        debug_img_msg = numpy_to_ros2_image(annotated_frame, img_msg.header.stamp, img_msg.header.frame_id)
        self.annotated_img_pub.publish(debug_img_msg)

        # Compare with yolo (TODO remove)
        if self.use_yolo:
            yolo_results = self.yolo.predict(rgb)
            yolo_annotated_rgb = yolo_results[0].plot()
            yolo_annotated_bgr = cv2.cvtColor(yolo_annotated_rgb, cv2.COLOR_RGB2BGR)
            yolo_debug_img_msg = numpy_to_ros2_image(yolo_annotated_bgr, img_msg.header.stamp, img_msg.header.frame_id)
            self.annotated_yolo_pub.publish(yolo_debug_img_msg)

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
            sam_img = numpy_to_ros2_image(overlay, img_msg.header.stamp, img_msg.header.frame_id, encoding = 'rgb8')
            self.annotated_sam_pub.publish(sam_img)
            # Convert to pointcloud2
            pc_msg, full_pc_msg = project_depth_to_pc_torch(depth_torch, rgb_torch, self.calib_mat, self.camera_reference_frame, depth_msg.header.stamp, mask=mask, max_depth=3.0)
            self.object_pointcloud_pub.publish(pc_msg)
            self.full_pointcloud_pub.publish(full_pc_msg)
            time_1 = time.time()
            self.get_logger().info(f"DINO + SAM2 time: {time_1 - start}")
            # Transform to base frame
            try:
                tf = self.tf_buffer.lookup_transform(self.robot_base_frame, "realsense_compensated", img_msg.header.stamp)
                full_pc_transformed = tf2_sensor_msgs.do_transform_cloud(full_pc_msg, tf)
                full_points = np.array([
                    [p[0], p[1], p[2]]
                    for p in point_cloud2.read_points(full_pc_transformed, field_names=("x", "y", "z"), skip_nans=True)
                    ], dtype=np.float32)
                if len(full_points) == 0:
                    self.get_logger().warn("Empty full point cloud received.")
                    return
                obj_pc_transformed = tf2_sensor_msgs.do_transform_cloud(pc_msg, tf)
                obj_points = np.array([
                    [p[0], p[1], p[2]]
                    for p in point_cloud2.read_points(obj_pc_transformed, field_names=("x", "y", "z"), skip_nans=True)
                    ], dtype=np.float32)
                if len(obj_points) == 0:
                    self.get_logger().warn("Empty object point cloud received.")
                    return
            except Exception as ex:
                self.get_logger().warn(f"Couldn't transform: {ex=}")
                return

            # Filter based on the object height
            obj_height = obj_points[:, 2].min()
            height_mask = (full_points[:, 2] > obj_height - 0.2) & (full_points[:, 2] <= obj_height + 0.05)
            filtered_points = full_points[height_mask]
            if len(filtered_points) == 0:
                self.get_logger().warn("No pc points in height range.")
                return
            
            # RANSAC
            cloud_filtered = o3d.geometry.PointCloud()
            cloud_filtered.points = o3d.utility.Vector3dVector(filtered_points)
            plane_model, inliers = cloud_filtered.segment_plane(
                                    distance_threshold=0.05,
                                    ransac_n=3,
                                    num_iterations=1000
                                    )
            if len(inliers) == 0:
                self.get_logger().error("No plane detected.")
                return
            plane_cloud = cloud_filtered.select_by_index(inliers)
            plane_points_np = np.asarray(plane_cloud.points)
            time_2 = time.time()
            self.get_logger().info(f"RANSAC time: {time_2 - time_1}")
            # Convert plane to ros2 msg
            plane_points_color = np.zeros_like(plane_points_np,dtype=np.uint16)
            plane_points_color[:, -1] = 1
            header = Header()
            header.frame_id = full_pc_transformed.header.frame_id
            header.stamp = img_msg.header.stamp
            plane_msg = numpy_to_pc2_msg(plane_points_np, plane_points_color, header)
            self.ransac_plane_pub.publish(plane_msg)

            # Cluster the plane
            labels = np.array(plane_cloud.cluster_dbscan(eps=0.05, min_points=40, print_progress=False))
            if len(labels) == 0:
                self.get_logger().info("No clusters found.")
                return
            self.get_logger().info(f"Detected {labels.max() + 1} clusters on plane.")
            # Object Position
            object_pos = np.array([obj_points[:,0].mean(),
                                   obj_points[:,1].mean(),
                                   obj_points[:,2].mean()
                          ])
            
            ## Find the closest cluster to the object
            iter = 0
            closest_cluster = None
            for cluster_id in range(labels.max() + 1):
                cluster_mask = labels == cluster_id
                cluster_points = plane_points_np[cluster_mask]
                poly = Polygon(cluster_points[:,:2])
                dist = poly.distance(Point_Shapely(object_pos[:2]))
                print(f"{dist=}")
                if iter == 0:
                    min_dist = dist
                    iter+=1
                    closest_cluster = cluster_points
                    continue
                iter+=1
                if dist < min_dist:
                    closest_cluster = cluster_points
                    min_dist = dist
            if closest_cluster is None:
                print("No closest cluster found")
                return
            time_3 = time.time()
            self.get_logger().info(f"Clusters time: {time_3 - time_2}")

            # Concave hull
            try:
                closest_cluster_pcd = o3d.geometry.PointCloud()
                closest_cluster_pcd.points = o3d.utility.Vector3dVector(closest_cluster)
                #hull_mesh, _ = closest_cluster_pcd.compute_convex_hull()
                hull_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
                    closest_cluster_pcd, 0.1
                )
                hull_points = np.asarray(hull_mesh.vertices)
                self.get_logger().info(f"Concave hull vertices: {len(hull_points)}")
            except Exception as e:
                self.get_logger().warn(f"Concave hull computation failed: {e}")
                return
            time_4 = time.time()
            self.get_logger().info(f"Convex Hull: {time_4 - time_3}")
            # Publish hull verticies
            marker_array = MarkerArray()
            marker = Marker()
            marker.header = header
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.01
            marker.color.r = 1.0
            marker.color.g = 0.7
            marker.color.a = 1.0
            #hull_points_2d = hull_points[:,:1]
            #hull_points_2d = np.unique(hull_points_2d)
            for p in hull_points:
                pt = Point()
                pt.x, pt.y, pt.z = p.tolist()
                marker.points.append(pt)
            marker_array.markers.append(marker)
            self.marker_pub.publish(marker_array)
            time_5 = time.time()
            self.get_logger().info(f"Final: {time_5 - start}")

    def camera_info_callback(self, msg : CameraInfo):
        """
        Saves the calib matrix of the camera intrinsic parameters
        """
        if not self.camera_info_available:
            self.calib_mat = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            self.camera_info_available = True


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
