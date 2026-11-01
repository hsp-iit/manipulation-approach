# Reference

Topics, interfaces, parameters and fixed values. For guidance on which parameter to change, see
[tuning](tuning_and_troubleshooting.md#3-tuning-guide).

---

## 1. Interfaces

Package `surface_detector_interfaces`.

**`action/ReachObject`**

```text
string object_string        # text prompt
---
bool reached                # Nav2 reported success for the latest goal
string error_msg            # reason, if not reached
---
float32 distance_remaining  # from Nav2, 0.0 before its first feedback
```

**`msg/SegmentedPointcloud`**

```text
sensor_msgs/PointCloud2 segmented_object    # object points
sensor_msgs/PointCloud2 full_pointcloud     # all points within the depth range
```

**`msg/DetectionResults`**

```text
geometry_msgs/PointStamped segmented_object     # XY centroid, Z = maximum height
geometry_msgs/PointStamped[] surface_contours   # closed polygon: the last point repeats the first
```

**`srv/SegmentObject`** is defined but unused: its service is commented out in `detector.py`.

---

## 2. Actions

| Name | Type | Role | Node |
|---|---|---|---|
| `/reach_object` | `ReachObject` | Server | `object_detector` |
| `/reach_object_coordinator` | `ReachObject` | Server | `reach_coordinator` |
| `/reach_object` | `ReachObject` | Client | `reach_coordinator` |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | Client | `object_detector` |

---

## 3. YARP

With `raise_hands_before_approach`, `object_detector` is an RPC client of two `ergocub-cartesian-control`
controllers. It opens `/object_detector/left/rpc:o` and `/object_detector/right/rpc:o`, and calls
`go_to_position`, `is_motion_done` and `stop`. Targets are in `arm_reference_frame`.

---

## 3b. Topics

### `object_detector`

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Sub | `/camera/rgbd/img` | `Image` | QoS depth 1. |
| Sub | `/camera/rgbd/depth` | `Image` | QoS depth 1. |
| Sub | `/camera/rgbd/camera_info` | `CameraInfo` | Used once. |
| Sub | `/approach_planner/goal_pose` | `PoseStamped` | Candidates for forwarding. |
| Sub | `/surface_detector/results` | `DetectionResults` | Surface height for the hands. Only with `raise_hands_before_approach`. |
| Pub | `/object_detector/segmented_pointcloud` | `SegmentedPointcloud` | Pipeline input. |
| Pub | `/object_detector/seg_object_pointcloud` | `PointCloud2` | Debug. |
| Pub | `/object_detector/full_pointcloud` | `PointCloud2` | Debug, and costmap input in the bag setup. |
| Pub | `/annotated_dino_img` | `Image` | Debug, each processed frame. |
| Pub | `/sam_mask_img` | `Image` | Debug, frames with a detection. |

`/object_detector/ransac_plane` and `/object_detector/plane_markers` are created but never published.

### `surface_detector`

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Sub | `/object_detector/segmented_pointcloud` | `SegmentedPointcloud` | |
| Pub | `/surface_detector/results` | `DetectionResults` | Pipeline output. Always published. |
| Pub | `/surface_detector/marker` | `Marker` | Contour. Needs `enable_visualization`. |
| Pub | `/surface_detector/detected_plane` | `PointCloud2` | Height slab. Needs `enable_visualization`. |
| Pub | `/surface_detector/closer_cluster` | `PointCloud2` | Selected cluster. Needs `enable_visualization`. |

### `approach_planner`

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Sub | `/surface_detector/results` | `DetectionResults` | |
| Sub | `/global_costmap/costmap_raw` | `nav2_msgs/Costmap` | Sensor-data QoS (best effort). |
| Pub | `/approach_planner/goal_pose` | `PoseStamped` | In `map`. |
| Pub | `/approach_planner/candidate_goals_marker` | `Marker` | Debug, blue. |
| Pub | `/approach_planner/filtered_candidate_marker` | `Marker` | Debug, yellow. |

---

## 4. Parameters

### `object_detector`

Set on the command line, with `--ros-args -p name:=value`. Read once at startup.

| Parameter | Default | Description |
|---|---|---|
| `img_topic_name` | `/camera/rgbd/img` | Color topic. |
| `depth_topic_name` | `/camera/rgbd/depth` | Depth topic. |
| `camera_info_topic` | `/camera/rgbd/camera_info` | Intrinsics topic. |
| `use_camera_info_topic` | `true` | If false, uses `cam_calib_mat`. |
| `cam_calib_mat` | `[386, 0, 321, 0, 386, 238, 0, 0, 1]` | Row-major intrinsic matrix. |
| `camera_reference_frame` | `realsense_compensated` | Frame written into the clouds. Must be optical. Empty uses the depth message's frame. |
| `robot_base_frame` | `geometric_unicycle` | Read and logged, not otherwise used. |
| `object_pointcloud_topic` | `seg_object_pointcloud` | Suffix under `/object_detector/`. |
| `full_pointcloud_topic` | `full_pointcloud` | Suffix under `/object_detector/`. |
| `box_threshold` | `0.35` | Minimum DINO box score. |
| `text_threshold` | `0.35` | Minimum DINO token score. |
| `planner_goal_topic` | `/approach_planner/goal_pose` | Goals from the planner. |
| `navigate_to_pose_action` | `navigate_to_pose` | Nav2 action. |
| `nav_server_wait_timeout` | `5.0` | Seconds to wait for Nav2 at request start. |
| `goal_update_min_distance` | `0.15` | Deadband in metres. |
| `goal_update_min_angle_deg` | `10.0` | Deadband in degrees. |
| `goal_update_min_interval` | `1.0` | Minimum seconds between Nav2 goals. |
| `navigation_start_timeout` | `20.0` | Seconds to reach a first accepted goal. |
| `dino_config_path` | `/home/user1/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py` | Model config. |
| `dino_weights_path` | `/home/user1/GroundingDINO/weights/groundingdino_swint_ogc.pth` | Model weights. |
| `sam_model` | `sam2.1_l.pt` | SAM 2 weights, downloaded by ultralytics if missing. |
| `raise_hands_before_approach` | `true` | Raise the hands before any goal is forwarded. Needs YARP and the arm controllers. |
| `surface_results_topic` | `/surface_detector/results` | Surface height source. |
| `left_arm_rpc_port` | `/cartesian-control/left_arm/rpc:i` | Left arm controller, as in its real robot config. |
| `right_arm_rpc_port` | `/cartesian-control/right_arm/rpc:i` | Right arm controller. |
| `arm_reference_frame` | `root_link` | Root frame of the arm controllers (`mobile_base_body_link` on R1). |
| `arm_rpc_timeout` | `1.0` | Seconds to wait for an RPC reply. |
| `hand_forward_offset` | `0.4` | Hands distance ahead of the base frame, in metres. Must match `approach_planner`. |
| `hand_lateral_offset` | `0.2` | Left hand at +offset, right hand at −offset, in metres. Must match `approach_planner`. |
| `hand_surface_clearance` | `0.1` | Hands height above the surface, in metres. |
| `hands_raise_min_distance` | `0.5` | Minimum distance from the contour to raise the hands, in metres. |
| `hands_motion_duration` | `4.0` | Trajectory duration, in seconds. |
| `hands_motion_timeout` | `10.0` | Maximum time for the arms to finish, in seconds. |

### `reach_coordinator`

| Parameter | Default | Description |
|---|---|---|
| `coordinator_action_name` | `/reach_object_coordinator` | Action served. |
| `detector_action_name` | `/reach_object` | Action called. |
| `detector_server_wait_timeout` | `5.0` | Seconds to wait for the detector. |

### `surface_detector`

Set in `surface_detection/param/surface_detector.yaml`, read during configure. The YAML overrides the
compiled defaults, which differ, so a node started without the YAML behaves very differently.

| Parameter | YAML | Compiled | Description |
|---|---:|---:|---|
| `pointcloud_topic` | `/object_detector/segmented_pointcloud` | Same | Input topic. |
| `reference_frame` | `geometric_unicycle` | Same | Z-up working and output frame. |
| `height_offset` | `0.2` | `0.01` | Slab extent below the object's estimated base, in metres. |
| `ransac_eps` | `0.2` | `0.02` | Maximum plane tilt, in radians. |
| `ransac_distance_threshold` | `0.03` | `0.01` | RANSAC inlier distance, in metres. |
| `thicken_ransac` | `true` | `true` | Rebuild the plane from raw points. See [performance](surface_detection.md#7-performance). |
| `delta_ransac_height` | `0.03` | `0.02` | Half-height of the thickened slab, in metres. |
| `cluster_tolerance` | `0.05` | `0.02` | Clustering connection distance, in metres. |
| `min_cluster_size` | `200` | `100` | Minimum points in a cluster. |
| `enable_visualization` | `true` | `true` | Publish the debug topics. |

### `approach_planner`

Set in `approach_path_planning/param/approach_path_planning.yaml`, read during configure. YAML and
compiled defaults match.

| Parameter | Default | Description |
|---|---:|---|
| `base_frame` | `geometric_unicycle` | Robot pose frame. |
| `costmap_topic_name` | `/global_costmap/costmap_raw` | Costmap input. |
| `contours_topic_name` | `/surface_detector/results` | Contour input. |
| `goal_topic_name` | `/approach_planner/goal_pose` | Goal output. |
| `robot_radius` | `0.2` | Radius for the offset, clearance and escape disc, in metres. |
| `goal_edge_margin` | `0.05` | Extra offset from the contour, in metres. |
| `max_costmap_val` | `253` | Cells with cost at or above this are blocked. |
| `dist_threshold` | `0.6` | Maximum distance from goal to object, in metres. |
| `goal_robot_distance_weight` | `0.25` | `λ` in the goal score `d_object + λ · d_robot`. Negative values are clamped to 0. |
| `enable_hands_over_surface_check` | `true` | Reject stances with a hand off the surface. |
| `hand_forward_offset` | `0.4` | Hands distance ahead of the base frame at the goal, in metres. Must match `object_detector`. |
| `hand_lateral_offset` | `0.2` | Hands distance from the centre line, in metres. Must match `object_detector`. |
| `hand_surface_margin` | `0.05` | Minimum distance of each hand from the contour, in metres. |
| `enable_window_tangent_orientation` | `true` | Heading blend and gate. If false, face along the segment normal. |
| `orientation_window_size` | `2` | Tangent half-window in vertices, minimum 1. |
| `orientation_object_weight` | `0.7` | Weight of the direction to the object. |
| `orientation_contour_normal_weight` | `0.3` | Weight of the inward normal. |
| `orientation_face_tolerance_deg` | `40.0` | Face check. Inactive with the default weights. |
| `orientation_perp_tolerance_deg` | `40.0` | Perpendicular check. |
| `enable_goal_clearance_check` | `true` | Enable the circular clearance check. |
| `goal_clearance_radius_cells` | `-1` | Radius in cells. `-1` computes it from `robot_radius` and the costmap resolution, `0` is clamped to 1. |

### `global_costmap` (bag setup)

`approach_path_planning/param/costmap_from_bag.yaml` holds standard Nav2 costmap parameters. The ones
that affect this pipeline:

| Parameter | Value | Relevance |
|---|---:|---|
| `global_frame` | `map` | Must be `map`. |
| `resolution` | `0.05` | Sets the cell size of all planner checks. |
| `robot_radius` | `0.1` | Differs from the planner's 0.2. |
| `inflation_radius` | `4.0` | Extent of the soft tail. |
| `cost_scaling_factor` | `1.5` | Decay of the soft tail. |
| `track_unknown_space` | `true` | Unknown cells reject candidates. |
| `observation_sources` | `pointcloud_mark pointcloud_clear` | Both read `/object_detector/full_pointcloud`. |

---

## 5. Hard-coded values

Changing these needs a code edit.

| Value | Where | Meaning |
|---|---|---|
| `"cuda"` | `detector.py`, `self.device` | Inference device. |
| `max_depth=3.0` | `detector.py`, `camera_callback` | Depth range limit, metres. |
| `min_depth=0.2` | `utils.py`, default argument | Depth range limit, metres. |
| `slop=0.1`, queue 1 | `detector.py` | Color and depth synchronisation. |
| `0.2 s` | `detector.py`, `execute_reach_request` | Request loop period. |
| `m_voxel_size = 0.02` | `surface_detector.hpp` | Voxel leaf, metres. |
| `setMaxIterations(200)` | `surface_detector.cpp` | RANSAC iterations. |
| `setAlpha(0.2)` | `surface_detector.cpp` | Alpha shape radius, metres. |
| `"map"` | `planner.cpp` | Global frame name. |
| `costmap_search_radius = 2` | `planner.cpp` | Fallback window radius, cells. |
| `4-connected` | `planner.cpp` | BFS connectivity. |

---

## 6. Launch arguments

| Launch file | Argument | Default | Note |
|---|---|---|---|
| `approach_full_stack` | `repo_root` | `~/ws_ros2/src/manipulation-approach` | |
| `approach_full_stack` | `detector_script` | `<repo_root>/object_detection/detector.py` | |
| `approach_full_stack` | `coordinator_script` | `<repo_root>/object_detection/coordinator.py` | |
| `approach_planner` | `approach_path_planning` | Installed YAML | Parameter file. |
| `surface_detector` | `surface_detector` | Installed YAML | Parameter file. Undeclared but honoured. |
| `approach_planner`, `surface_detector` | `use_sim_time` | `false` | **No effect**: never passed to the node. |
| `costmap_from_bag` | `params_file` | Installed YAML | |
| `costmap_from_bag` | `use_sim_time` | `false` | Applied. |
