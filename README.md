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

Optionally, launch the minimal coordinator node (for action forwarding/orchestration) by:
```
python3 object_detection/coordinator.py
```

This node requires the following topics:
- `/camera/rgbd/img`
- `/camera/rgbd/depth`
- `/camera/rgbd/camera_info`
- `/tf`

Publishes:
- `/object_detector/segmented_pointcloud` (needed by the `surface_detector` node)
- `/object_detector/seg_object_pointcloud` (for visualization debug)
- `/object_detector/full_pointcloud` (for visualization debug)
- `/annotated_dino_img` (debug image)
- `/sam_mask_img` (debug image)

To start the pipeline to reach a certain object use the action `/reach_object`:

```
ros2 action call /reach_object surface_detector_interfaces/action/ReachObject "{object_string: 'name of what you want to look for'}"
```

If using the coordinator, call:
```
ros2 action call /reach_object_coordinator surface_detector_interfaces/action/ReachObject "{object_string: 'name of what you want to look for'}"
```

### 2- Pipeline Launch
---
Make sure to have sourced the workspace local setup (already done inside the docker)

To launch detector + coordinator + segmentation + planner + RViz in one command:
```
ros2 launch approach_path_planning approach_full_stack.launch.py
```

If your repository path is different from `~/ws_ros2/src/manipulation-approach`, set it with:
```
ros2 launch approach_path_planning approach_full_stack.launch.py repo_root:=/absolute/path/to/manipulation-approach
```

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
- `/approach_planner/candidate_goals_marker`: Marker msg containing the possible poses outside the contour of the suface below the object to grasp
- `/approach_planner/filtered_candidate_marker`: Marker msg that shows the poses from `/approach_planner/candidate_goals_marker` filtered in such a way that are reachable and in reach of the object to grasp.

## Parameters Reference

This section summarizes all runtime parameters currently used by the pipeline nodes in this repository.

### `approach_planner` parameters
Configured in `approach_path_planning/param/approach_path_planning.yaml`.

| Parameter | Default | Description |
|---|---:|---|
| `base_frame` | `geometric_unicycle` | Robot base frame used by planner transforms/logic. |
| `costmap_topic_name` | `/global_costmap/costmap_raw` | Topic used to subscribe to costmap data. |
| `contours_topic_name` | `/surface_detector/results` | Input topic with detected object + surface contours. |
| `robot_radius` | `0.2` | Robot radius in meters used for candidate offsets and clearance checks. |
| `max_costmap_val` | `253` | Cost threshold considered non-traversable (`>=` this value is rejected). |
| `dist_threshold` | `0.6` | Maximum distance (m) allowed between candidate goal and object for grasp feasibility. |
| `enable_window_tangent_orientation` | `true` | Enables robust orientation computation from local contour window tangent. |
| `orientation_window_size` | `2` | Half-window size (in contour points) used to smooth local tangent estimation. |
| `orientation_object_weight` | `0.7` | Weight for the object-facing direction in blended heading computation. |
| `orientation_contour_normal_weight` | `0.3` | Weight for inward contour-normal direction in blended heading computation. |
| `orientation_face_tolerance_deg` | `40.0` | Max angular tolerance (deg) for “facing the object” validation. |
| `orientation_perp_tolerance_deg` | `40.0` | Max angular tolerance (deg) for “near-perpendicular to contour” validation. |
| `enable_goal_clearance_check` | `true` | Enables neighborhood occupancy check around each goal cell. |
| `goal_clearance_radius_cells` | `5` | Clearance radius in cells. If set to `-1`, it is auto-computed as `ceil(robot_radius / resolution)`. |

### `surface_detector` parameters
Configured in `surface_detection/param/surface_detector.yaml`.

| Parameter | Default | Description |
|---|---:|---|
| `pointcloud_topic` | `/object_detector/segmented_pointcloud` | Input segmented object cloud topic. |
| `reference_frame` | `geometric_unicycle` | Target frame used for processing and output. |
| `min_cluster_size` | `200` | Minimum number of points for cluster extraction. |
| `cluster_tolerance` | `0.05` | Euclidean cluster tolerance (meters). |
| `height_offset` | `0.2` | Vertical offset used in surface/object extraction steps. |
| `ransac_eps` | `0.2` | Epsilon/constraint parameter used in RANSAC stage. |
| `ransac_distance_threshold` | `0.03` | Inlier distance threshold (meters) for plane RANSAC. |
| `thicken_ransac` | `true` | Enables thickening/expansion of RANSAC-selected support region. |
| `delta_ransac_height` | `0.03` | Height delta used when `thicken_ransac` is enabled. |
| `enable_visualization` | `true` | Publishes extra visualization/debug outputs. |

### `global_costmap` (costmap-from-bag) parameters
Configured in `approach_path_planning/param/costmap_from_bag.yaml`.

These are standard Nav2 costmap parameters. Main values currently set in this repo:

- Frames and timing:
	- `global_frame: map`
	- `robot_base_frame: geometric_unicycle`
	- `update_frequency: 5.0`
	- `publish_frequency: 10.0`
	- `resolution: 0.05`
	- `track_unknown_space: true`
	- `use_sim_time: false`
	- `robot_radius: 0.1`
	- `trinary_costmap: false`
	- `always_send_full_costmap: true`
- Plugins:
	- `static_layer`
	- `obstacle_layer`
	- `inflation_layer`
- Obstacle/inflation tuning includes:
	- marking/clearing sources from `/object_detector/full_pointcloud`
	- `inflation_radius: 4.0`
	- `cost_scaling_factor: 1.5`

> Note: planner behavior is most sensitive to `max_costmap_val`, `dist_threshold`, orientation parameters, and goal-clearance parameters.
