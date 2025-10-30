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
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from tf2_ros import TransformListener, Buffer
import message_filters

from ultralytics import YOLOWorld, SAM


TEXT_PROMPT = "bottle"
BOX_TRESHOLD = 0.35
TEXT_TRESHOLD = 0.25

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
                ('object_pointcloud_topic', 'seg_object_pointcloud')
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
        self.object_pointcloud_topic = self.get_parameter("object_pointcloud_topic").value

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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ### ROS2 subscribers
        self.img_sub = message_filters.Subscriber(self, Image, img_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.tss = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.depth_sub], 1, slop=0.3)   
        self.tss.registerCallback(self.camera_callback)

        self.object_pointcloud_pub = self.create_publisher(PointCloud2, self.object_pointcloud_topic, 10)

        # DINO model
        self.dino_model = load_model("/home/user1/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py", "/home/user1/GroundingDINO/weights/groundingdino_swint_ogc.pth")
        # YOLO
        if self.use_yolo:
            self.yolo = YOLOWorld("yolov8l-worldv2")
            self.yolo.set_classes(["chair" , "laptop" , "mouse" , "bag" , "box" , "backpack" , "mug" , "bottle"])
            self.annotated_yolo_pub = self.create_publisher(Image, "/annotated_yolo_img", 10)
        # SAM
        self.sam = SAM("sam2.1_l.pt")
        self.annotated_sam_pub = self.create_publisher(Image, "/sam_mask_img", 10)

        # Annotated img pub (debug only)
        self.annotated_img_pub = self.create_publisher(Image, "/annotated_dino_img", 10)


    def camera_callback(self, img_msg : Image, depth_msg : Image):

        # Convert ROS Image message to NumPy array (raw byte data) and then to Tensor
        rgb = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 3)
        #rgb_torch = torch.tensor(rgb, device=self.device)
        #depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)
        #depth_torch = torch.tensor(depth, device=self.device)

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
            mask = sam_results[0].masks.data.cpu().numpy()
            mask = mask.astype(np.uint8)
            #color = np.random.randint(0, 255, (3,), dtype=np.uint8)
            color_red = np.array([255, 0, 0], dtype=np.uint8)
            colored_mask = np.zeros_like(rgb, dtype=np.uint8)
            overlay = rgb.copy()
            for c in range(3):
                colored_mask[:, :, c] = mask * color_red[c]
            overlay = cv2.addWeighted(overlay, 1.0, colored_mask, 0.5, 0)
            sam_img = numpy_to_ros2_image(overlay, img_msg.header.stamp, img_msg.header.frame_id, encoding = 'rgb8')
            self.annotated_sam_pub.publish(sam_img)


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
