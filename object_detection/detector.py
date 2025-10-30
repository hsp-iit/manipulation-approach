import numpy as np

# Grounding DINO
from groundingdino.util.inference import load_model, load_image, predict, annotate
from PIL import Image as PIL_Image
import groundingdino.datasets.transforms as T
import cv2

import torch
from torchvision.ops import box_convert
# ROS2
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, CameraInfo, Image, PointField
from tf2_ros import TransformListener, Buffer
import message_filters

from ultralytics import YOLOWorld, SAM


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
                ('depth_downsampling', 10),     #10 for resolution 640x480 36 for res 1280x720
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
        # category list to display frame by frame
        self.depth_downsampling = self.get_parameter('depth_downsampling').value
        # Reference frame of the camera, if empty will use the one from the ros message
        self.camera_reference_frame = self.get_parameter('camera_reference_frame').value
        # Base frame of the robot, counts as the robot pose
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        # Topic names
        self.object_pointcloud_topic = self.get_parameter("object_pointcloud_topic").value
        self.full_pointcloud_topic = self.get_parameter("full_pointcloud_topic").value
        self.device = "cuda"

        self.get_logger().info(f'Using parameters: {img_topic=}  {depth_topic=}  {use_camera_info_topic=}  {camera_info_topic=} \n'
                               f'{self.object_pointcloud_topic=}  {self.robot_base_frame=}  {self.depth_downsampling=}')

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
        #self.tf_buffer = Buffer()
        #self.tf_listener = TransformListener(self.tf_buffer, self)

        ### ROS2 subscribers
        self.img_sub = message_filters.Subscriber(self, Image, img_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.tss = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.depth_sub], 1, slop=0.1)   
        self.tss.registerCallback(self.camera_callback)
        # Publishers
        self.object_pointcloud_pub = self.create_publisher(PointCloud2, self.object_pointcloud_topic, 10)
        self.full_pointcloud_pub = self.create_publisher(PointCloud2, self.full_pointcloud_topic, 10)

        # DINO model
        self.dino_model = load_model("/home/user1/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py", "/home/user1/GroundingDINO/weights/groundingdino_swint_ogc.pth")
        # Annotated img pub (debug only)
        self.annotated_img_pub = self.create_publisher(Image, "/annotated_dino_img", 10)
        # YOLO
        if self.use_yolo:
            self.yolo = YOLOWorld("yolov8l-worldv2")
            self.yolo.set_classes(["chair" , "laptop" , "mouse" , "bag" , "box" , "backpack" , "mug" , "bottle"])
            self.annotated_yolo_pub = self.create_publisher(Image, "/annotated_yolo_img", 10)
        # SAM
        self.sam = SAM("sam2.1_l.pt")
        self.annotated_sam_pub = self.create_publisher(Image, "/sam_mask_img", 10)


    def camera_callback(self, img_msg : Image, depth_msg : Image):

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
            #color = np.random.randint(0, 255, (3,), dtype=np.uint8)
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
