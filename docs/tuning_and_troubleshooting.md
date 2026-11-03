# Tuning and troubleshooting

Start from what you see, find the stage at fault, then change the right parameter.

---

## 1. Symptom table

| Symptom | Likely cause | What to do |
|---|---|---|
| No topic publishes anything | No active request. | Expected. The detector works only during a request. |
| `Waiting for camera_info topic to become available` | Wrong topic or no publisher. | Check `camera_info_topic`, or set `use_camera_info_topic:=false`. |
| `/annotated_dino_img` silent during a request | Color and depth never pair. | Compare stamps: they must be within 0.1 s. Check the topic names. |
| Exception at `reshape` in `camera_callback` | Unexpected encoding or row padding. | See the [input contract](object_detection.md#2-input-contract). |
| `Timeout while starting the approach pipeline...` | Any stage failed for 20 s. | Follow [section 2](#2-finding-the-failing-stage). |
| `Navigation action server ... not available` | Nav2 not running, or on another domain. | `ros2 action list`, and check `ROS_DOMAIN_ID`. |
| `Cannot transfor: ...` | No TF from the cloud frame to `reference_frame`. | `ros2 run tf2_ros tf2_echo <base> <camera>`. |
| Surface or object floats or tilts in RViz | `camera_reference_frame` is not an optical frame. | See [frames](architecture_overview.md#3-coordinate-frames). |
| `Empty pointcloud received` | Object beyond 3 m or depth missing inside the mask. | Move closer, and check the depth image. |
| `Obtained an empty pc after heigh filtering` | Nothing within the slab. | The surface may be out of view. Increase `height_offset` if the object's base is misjudged. |
| `Could not estimate a planar model` | No horizontal plane found. | Check that the base frame's Z is vertical. Raise `ransac_eps` a little. |
| `Unable to cluster` | Every cluster below `min_cluster_size`. | Lower it, particularly with `thicken_ransac: false`. |
| `No valid surface contour found` | Degenerate hull, usually too few points. | Move closer so more surface is in view. |
| `Costmap not available!` | No costmap message yet. | Check `/global_costmap/costmap_raw`, and that the costmap node is active. |
| `Arm controllers not available: ...` | Controller not running, or wrong port. | `yarp name list`, check `left_arm_rpc_port` and `right_arm_rpc_port`. |
| `Could not raise the hands: too close to the surface` | The caller stopped too near. | Stop farther away, or lower `hands_raise_min_distance` only if the path of the hands is clear. |
| `Timeout while raising the hands` | Arm controller in error, or a slow trajectory. | Check the controller log. Raise `hands_motion_timeout` for long trajectories. |
| `ModuleNotFoundError: yarp` at startup | YARP Python bindings missing. | Install them, or set `raise_hands_before_approach:=false`. |
| `Skipped N goal candidates with the hands off the surface` | Normal near corners. | If no goal is left, review `hand_forward_offset`, `hand_lateral_offset` and `hand_surface_margin`. |
| `Robot pose is outside costmap bounds` | Robot off the map, or wrong frame. | Check the `map` to base TF. |
| `Unable to find valid goal candidates` | All candidates filtered. | Follow [section 4](#4-why-did-the-planner-reject-everything). |
| `Skipped N unreachable goal candidates` | Reachability rejected some. | Normal if others remain. See section 4 otherwise. |
| The robot stops beside the object | `goal_robot_distance_weight` too high. | Lower it. See [goal selection](#goal-selection). |
| The robot walks around the table | `goal_robot_distance_weight` too low. | Raise it. See [goal selection](#goal-selection). |
| The object is hard to reach from the goal | The reach check is 2D and base-centred. | See [L3](known_limitations.md#l3-reach-ignores-height-and-the-arms). |
| The robot jitters near the goal | Goal noise is above the deadband. | Raise `goal_update_min_distance`, or `goal_update_min_interval`. |
| The goal jumps between objects | DINO's top box changes per frame. | Write a more specific prompt, or raise `box_threshold`. |
| The pipeline slows as the robot gets close | Surface detector cost. | See [performance](surface_detection.md#7-performance). |
| Wrong colors in `/sam_mask_img` | Published as `rgb8`, but the data is BGR. | Harmless. It is a display issue only. |

---

## 2. Finding the failing stage

A request timeout hides which stage failed. Check the chain in order, during an active request. The
first silent topic is the problem.

```bash
ros2 topic hz /camera/rgbd/img                        # is the camera alive?
ros2 topic hz /annotated_dino_img                     # are frames processed?
ros2 topic hz /sam_mask_img                           # was the object detected?
ros2 topic hz /object_detector/segmented_pointcloud   # were clouds built?
ros2 topic hz /surface_detector/results               # was a contour found?
ros2 topic hz /approach_planner/goal_pose             # was a goal found?
```

To test later stages without a request, note that `surface_detector` and `approach_planner` react to
their inputs at any time. Replaying a recorded `/object_detector/segmented_pointcloud` therefore
exercises them alone, and `goal_pose` will publish even with no request active.

---

## 3. Tuning guide

### Adapting to a different robot

| Change | Why |
|---|---|
| `robot_radius` to the robot's real radius | Sets the distance from the edge and the clearance. |
| `dist_threshold` to the arm's comfortable reach, measured from the base centre | Sets which objects can be approached. |
| `base_frame`, `reference_frame`, `robot_base_frame` | Your Z-up planar frame. |
| `camera_reference_frame` | Your optical frame. |

For an elongated footprint, `robot_radius` is a compromise, since the planner models a circle. For
ergoCub, the commented-out footprint in `costmap_from_bag.yaml` is 0.4 × 0.7 m. Use the half-depth if
the robot approaches facing the edge.

### Goal selection

`goal_robot_distance_weight` (`λ`) sets the trade between reach and travel: a pose must be `λ` metres
closer to the object per metre of extra distance from the robot. Lower it to favour reach, and raise it
to avoid walking around surfaces. In the narrow table scene, the far side won below about 0.05. Negative
values are clamped to 0.

### Hands

Measure `hand_forward_offset` and `hand_lateral_offset` from the real pre-grasp pose, and set them on both
`object_detector` and `approach_planner`. With `enable_hands_over_surface_check` (off by default),
`hand_forward_offset` must exceed the stance offset (`robot_radius + goal_edge_margin`) by at least
`hand_surface_margin`, or no straight approach can pass.
Keep `hands_raise_min_distance` above `hand_forward_offset`, and set `hand_surface_clearance` above the
tallest object in the path of the hands.

### Margin from the edge

With the defaults, `goal_edge_margin` leaves one 5 cm cell of slack between the clearance circle and
the surface. In a simple model, with edges at random positions within a cell and 1 cm contour noise,
about 8 % of the near-edge candidates fail the clearance check at 0.05 m, and none at 0.075 m. Raise
the margin if near-edge goals are often missing on real data, but note that it moves the robot away
from the object by the same amount.

### The deepest object you can reach

Objects up to `dist_threshold - robot_radius - goal_edge_margin` from an edge can be approached: 0.35 m
by default. Raising `dist_threshold` extends this, but only as far as the arms really reach.

### Orientation

| Goal | Change |
|---|---|
| Face the object more | Raise `orientation_object_weight`. |
| Stay square to the edge | Raise `orientation_contour_normal_weight`. |
| Accept more oblique stances | Raise `orientation_perp_tolerance_deg` above 40°. |
| Make the face check active | Lower its tolerance below `asin(w_n / w_o)`, 25.4° by default. |
| Bypass the gate | `enable_window_tangent_orientation: false`. |

Only the ratio of the two weights matters, since the blend is normalised.

### Surface detection

| Problem | Change |
|---|---|
| Too slow near the table | `thicken_ransac: false` with `min_cluster_size: 20`, after [checking on real data](surface_detection.md#7-performance). |
| Two desks merge into one | Lower `cluster_tolerance`. |
| A desk splits into fragments | Raise `cluster_tolerance`. |
| A lower shelf wins | Lower `height_offset`. |
| The surface is thin or noisy | Raise `ransac_distance_threshold` and `delta_ransac_height`. |

### Goal stability

Lower `goal_update_min_distance` to refine the stance more finely, at the cost of more preemptions.
Keep it above the frame-to-frame goal noise, which you can read from the `Found goal` log lines.

### Timeouts

Raise `navigation_start_timeout` when the robot starts far from the object or inference is slow. The
20 s budget has to absorb detection attempts, plus up to 10 s per frame in the surface detector at
close range.

---

## 4. Why did the planner reject everything?

Compare the two marker topics in RViz, and work through the cases:

| Observation | Cause |
|---|---|
| No blue spheres | All candidates failed the orientation gate, or no contour. Disable the gate with `enable_window_tangent_orientation: false` to confirm. |
| Blue spheres, none near the object | Object too deep: farther than `dist_threshold - 0.25` m from every edge. |
| Blue spheres near the object, all in dark cells | The surface is lethal in the costmap with large inflation. Check that `r_inscribed` is below the goal offset. |
| Blue spheres in free cells, still rejected | Clearance check, or unknown cells nearby. Check for unknown space in the costmap. |
| Log says `Skipped N unreachable goal candidates` | The robot's cell is enclosed, or the path is narrower than the inflated corridor. |

To test a hypothesis, change the parameter in the YAML and restart: `approach_planner` reads
parameters only when configured.

Each marker is published only when it has at least one point, and an old marker stays visible in RViz
until a new one replaces it. Do not read a stale marker as a current result.
