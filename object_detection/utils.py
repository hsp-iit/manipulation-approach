from sensor_msgs.msg import Image, PointField, PointCloud2
from std_msgs.msg import Header
import torch
import numpy as np

def numpy_to_ros2_image(rgb: np.ndarray, stamp, frame_id="camera_rgb_frame", encoding = 'bgr8') -> Image:
    """
    Convert a NumPy image array into a ROS 2 Image message.

    This function wraps a NumPy RGB/BGR image into a `sensor_msgs.msg.Image`
    message, setting the appropriate metadata such as dimensions, encoding,
    and header information.

    Args:
        rgb (np.ndarray): Image array of shape (H, W, 3) containing color data.
            The channel order must match the specified encoding.
        stamp: ROS 2 timestamp to assign to the image header
            (typically `builtin_interfaces.msg.Time`).
        frame_id (str, optional): Frame ID to associate with the image.
            Defaults to "camera_rgb_frame".
        encoding (str, optional): ROS image encoding string (e.g., 'bgr8',
            'rgb8'). Defaults to 'bgr8'.

    Returns:
        Image: A ROS 2 `sensor_msgs.msg.Image` message containing the image data.
    """
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
    """
    Convert NumPy point cloud and color arrays into a ROS PointCloud2 message.

    This function takes an Nx3 point cloud array and an Nx3 color array and
    packs them into a `sensor_msgs.msg.PointCloud2` message with fields
    ['x', 'y', 'z', 'r', 'g', 'b'], where color values are normalized to [0, 1].

    Args:
        pointcloud (np.ndarray): Array of shape (N, 3) containing XYZ coordinates
            as float values.
        color (np.ndarray): Array of shape (N, 3) containing RGB color values
            in the range [0, 255].
        header: ROS message header (typically `std_msgs.msg.Header`) containing
            the frame ID and timestamp.

    Returns:
        PointCloud2: A ROS PointCloud2 message representing the input point cloud
        with per-point RGB color information.

    Notes:
        - The resulting PointCloud2 message is unorganized (height = 1).
        - Color values are stored as float32 and normalized by dividing by 255.
        - The point layout is: x, y, z, r, g, b (each as FLOAT32).
        - `is_dense` is set to False and endianness is assumed to be little-endian.
    """
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
    Project a depth image into a colored 3D point cloud using PyTorch (GPU).

    This function back-projects a depth image into 3D space using the camera
    intrinsic matrix and associates each 3D point with RGB color information.
    All heavy computation is performed using PyTorch tensors and can run on GPU.

    Two PointCloud2 messages are produced:
      1) A masked and depth-filtered point cloud.
      2) A full point cloud filtered only by depth limits.

    Args:
        depth (torch.Tensor): Depth image tensor of shape (H, W), containing
            per-pixel depth values in meters (or scaled by `depth_factor`).
        color_img (torch.Tensor): Color image tensor of shape (H, W, 3),
            aligned with the depth image.
        calib_matrix: Camera intrinsic matrix of shape (3, 3) with
            focal lengths and principal point (fx, fy, cx, cy).
        frame_id (str): Reference frame ID for the generated point clouds.
        stamp: ROS 2 timestamp to assign to the PointCloud2 headers
            (typically `builtin_interfaces.msg.Time`).
        mask (np.ndarray or torch.Tensor, optional): Binary mask of shape (H, W)
            indicating which pixels to include (1 = keep, 0 = discard).
            If None, all pixels are considered.
        min_depth (float, optional): Minimum valid depth value (in meters).
            Points closer than this are discarded. Defaults to 0.2.
        max_depth (float, optional): Maximum valid depth value (in meters).
            Points farther than this are discarded. Defaults to 10.0.
        depth_factor (float, optional): Scale factor applied to depth values.
            Final Z values are computed as depth / depth_factor. Defaults to 1.0.

    Returns:
        Tuple[PointCloud2, PointCloud2]:
            - msg: PointCloud2 containing masked and depth-filtered points.
            - full_msg: PointCloud2 containing all valid depth points
              (ignores the mask).

    Notes:
        - Point clouds are generated in the camera coordinate frame.
        - The output point layout is: x, y, z, r, g, b (FLOAT32).
        - Color values are normalized to the range [0, 1].
        - The resulting PointCloud2 messages are unorganized (height = 1).
        - Data is transferred from GPU to CPU before message construction.
        - Endianness is assumed to be little-endian.
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