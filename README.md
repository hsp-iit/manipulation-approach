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

To start the pipeline to reach a certain object use the action `/reach_object`:

```
ros2 action call /reach_object surface_detector_interfaces/action/ReachObject "{object_string: 'name of what you want to look for'}"
```

### 2- Pipeline Launch
---
Make sure to have sourced the workspace local setup (already done inside the docker)

Launch the segmentation and planner nodes using the launch file:
```
ros2 launch approach_path_planning approach_pipeline.launch.py
```
It will also launch the visualization gui Rviz2.

## Nodes Explaination

### 1- Plane Segmentation
---

If you want to launch the node by itself do:
```
ros2 launch surface_detection surface_detector.launch.py
```

Subscribes to the topic `/object_detector/segmented_pointcloud` and publishes the following topics:
- `/surface_detector/detected_plane` pointcloud of the plane below the segmented object
- `/surface_detector/marker` marker of the contour of the plane underneath the object

### 2- Planner
---

If you want to launch the node by itself do:
```
ros2 launch approach_path_planning approach_planner.launch.py
```

Subscribes to the topic `/surface_detector/results` and the costmap `/global_costmap/costmap_raw`.
It publishes the desired goal (if any) on the topic: `/goal_pose`.
Publishes also the following visualization/debug topics:
- `/candidate_goals_marker`: Marker msg containing the possible poses outside the contour of the suface below the object to grasp
- `/filtered_candidate_marker`: Marker msg that shows the poses from `/candidate_goals_marker` filtered in such a way that are reachable and in reach of the object to grasp.
