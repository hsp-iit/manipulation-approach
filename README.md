# manipulation-approach
Pipeline to safely approach desks/tables where tasks need to be performed or have objects to be grasped. 

## Install
Compile the docker using the `build_docker.sh` script inside the `docker` folder. Setting accordingly the TAG and DOCKER_NAME variables inside the script.

Launch the docker using the `run.sh` script, setting the name and tag of the docker that you set in the previous step when building the image.

## Usage
Inside the docker you will have a ros2 workspace in the home folder: `ws_ros2`. If you need to compile the ws, you can do: `colcon build --symlink-install`.
Inside the docker, this repo will be cloned inside the `src` folder of this ros2 workspace.
If you want to use this code in your system/environment, just create the ros2 workspace and clone this repo in the `src` folder, as the usual ros2 procedure.

### 1- Object Detection
---
Launch the object detection node inside the `~/ws_ros2/src/manipulation-approach` folder by:
```
python3 object_detection/detector.py 
```

This node requires the following topics:
- `/camera/rgbd/img`
- `/camera/rgbd/depth`
- `/camera/rgbd/camera_info`
- `/tf`

Publishes:
- `/object_detector/segmented_pointcloud` (needed by the `surface_detector` node)
- `/seg_object_pointcloud` (for visualization debug)
- `/full_pointcloud` (for visualization debug)

To set which object to look for in the environment use the service `/object_detector/object_to_find`:

```
ros2 service call /object_detector/object_to_find surface_detector_interfaces/srv/SegmentObject "{object_string: name of what you want to look for}"
```

### 2- Plane Segmentation
---
Make sure to have sourced the workspace local setup (already done inside the docker)
Launch the node by:
```
ros2 launch surface_detection surface_detector.launch.py
```

Subscribes to the topic `/object_detector/segmented_pointcloud` and publishes the following topics:
- `/surface_detector/detected_plane` pointcloud of the plane below the segmented object
- `/surface_detector/marker` marker of the contour of the plane underneath the object