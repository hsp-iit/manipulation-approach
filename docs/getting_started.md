# Getting started

From a clean machine to the robot driving to an object. For what each node does, see the
[architecture overview](architecture_overview.md).

---

## 1. What the pipeline needs from the robot

This repository does not drive the robot or build the map. It expects these to be running already:

| Requirement | Detail |
|---|---|
| RGB-D camera | Aligned `bgr8` color and `32FC1` depth in metres, same resolution, plus `camera_info`. See [input contract](object_detection.md#2-input-contract). |
| TF tree | `map` → `geometric_unicycle` → `realsense_compensated`. The frame names are parameters except `map`, which is hard-coded in the planner. |
| Global costmap | `nav2_msgs/Costmap` on `/global_costmap/costmap_raw`, with `map` as global frame. |
| Nav2 | A `NavigateToPose` action server on `navigate_to_pose`. |
| GPU | CUDA is required: the detector hard-codes `self.device = "cuda"`. |
| Arm controllers | Both `ergocub-cartesian-control` arms running on the YARP network, and the `yarp` Python module. Without them every request aborts, unless `raise_hands_before_approach:=false`. |

The object and its surface must be in view and closer than 3 m (`max_depth=3.0` in `detector.py`)
when the request starts. The pipeline approaches what it can see. It does not search for the object.

---

## 2. Install

### Docker (recommended)

Run the build from the `docker` folder, since the script builds the current directory:

```bash
cd docker
# Optionally edit DOCKER_NAME and TAG in build_docker.sh
./build_docker.sh <git_username> <git_email> <git_token>
```

The three arguments configure git and clone the repository during the build.

> **Warning:** the token is passed as a `--build-arg`, so it remains readable in the image history
> (`docker history`). Use a short-lived token and do not publish the image.

Before the first run, edit `docker/run.sh`: set `NAME` and `TAG` to match the build, and replace the
`/home/smicheletti-iit.local/rosbags_vlm` volume with your own path.

```bash
./run.sh
```

The container uses host networking and inherits `ROS_DOMAIN_ID`. The image contains ROS 2 Jazzy,
CycloneDDS, Grounding DINO (the `cuda12.8-fix` branch of `SimoneMic/GroundingDINO`, with weights),
ultralytics, and the Nav2 costmap packages. It does **not** contain the Nav2 servers, so Nav2 must
run on the robot or in another container on the same network and domain.

The image does **not** contain YARP either, and `detector.py` fails at startup without it while
`raise_hands_before_approach` is true. Install the Python bindings, or disable that parameter.

The SAM 2 weights (`sam2.1_l.pt`) are not in the image: ultralytics downloads them on first use, so
the first run needs network access.

### Existing ROS 2 Jazzy system

Clone the repository into `src` of a workspace, then install the same dependencies listed in
`docker/Dockerfile`: `pcl_ros`, `nav2_costmap_2d`, `nav2_msgs`, PyTorch with CUDA, Grounding DINO and
ultralytics.

Grounding DINO paths default to `/home/user1/GroundingDINO/...`. If yours differ, set
`dino_config_path` and `dino_weights_path` on the detector.

---

## 3. Build

```bash
cd ~/ws_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Three packages are built: `surface_detector_interfaces`, `surface_detection` and
`approach_path_planning`. The Python nodes in `object_detection/` are plain scripts, not a ROS
package, so they run directly from the source tree and need no rebuild.

A Release build (`--cmake-args -DCMAKE_BUILD_TYPE=Release`) made the planner about 2.5× faster here
(2.3 ms to 0.9 ms per contour) but did not change the surface detector's time, which is spent in the
precompiled PCL libraries. See [performance](surface_detection.md#7-performance).

---

## 4. Run

Start everything in this repository with:

```bash
ros2 launch approach_path_planning approach_full_stack.launch.py
```

This starts the detector, coordinator, surface detector, planner and RViz. It does not start Nav2 or
a costmap. If the repository is not at `~/ws_ros2/src/manipulation-approach`:

```bash
ros2 launch approach_path_planning approach_full_stack.launch.py repo_root:=/absolute/path/to/manipulation-approach
```

To run the stages separately, for example to restart the detector without reloading the rest:

```bash
# Terminal 1: surface detector, planner and RViz
ros2 launch approach_path_planning approach_pipeline.launch.py

# Terminal 2: object detector, from the repository root
python3 object_detection/detector.py

# Optional terminal 3: coordinator
python3 object_detection/coordinator.py
```

Other launch files:

| Launch file | Starts | Argument |
|---|---|---|
| `surface_detection surface_detector.launch.py` | Surface detector | `surface_detector:=<params.yaml>` |
| `approach_path_planning approach_planner.launch.py` | Planner | `approach_path_planning:=<params.yaml>` |
| `approach_path_planning rviz.launch.py` | RViz with `param/rviz/vis.rviz` | |
| `approach_path_planning costmap_from_bag.launch.py` | Standalone global costmap | `params_file:=<params.yaml>`, `use_sim_time:=true` |

The detector's parameters are set on its command line:

```bash
python3 object_detection/detector.py --ros-args -p box_threshold:=0.4 -p camera_reference_frame:=my_optical_frame
```

---

## 5. Send a request

```bash
ros2 action send_goal --feedback /reach_object \
  surface_detector_interfaces/action/ReachObject "{object_string: 'red mug'}"
```

The README's `ros2 action call` does not exist in Jazzy. The verb is `send_goal`.

Use `/reach_object_coordinator` instead of `/reach_object` to go through the coordinator. Both
accept the same goal and return the same result.

`object_string` is the Grounding DINO text prompt, so describe what is visible. Press `Ctrl+C` to
cancel, which also cancels navigation.

A successful result looks like this:

```yaml
reached: true
error_msg: ''
```

If `reached` is false, `error_msg` gives the reason. See
[failure modes](request_lifecycle.md#6-outcomes-and-failure-modes).

---

## 6. Watch it work

The RViz config displays these topics, with `map` as fixed frame:

| Display | Topic | What to expect |
|---|---|---|
| Object cloud | `/object_detector/seg_object_pointcloud` | Only the requested object. |
| Scene cloud | `/object_detector/full_pointcloud` | Everything within 3 m. |
| Surface cluster | `/surface_detector/closer_cluster` | The support surface, flattened. |
| Contour | `/surface_detector/marker` | Green outline of the surface. |
| Candidates | `/approach_planner/candidate_goals_marker` | Blue spheres around the contour. |
| Filtered candidates | `/approach_planner/filtered_candidate_marker` | Yellow spheres that survived filtering. |
| Goal | `/approach_planner/goal_pose` | The chosen pose. |
| Costmap | `/costmap` | Matches `costmap_from_bag.launch.py`. With a full Nav2 stack, change it to `/global_costmap/costmap`. |

Add the image topics `/annotated_dino_img` and `/sam_mask_img` to inspect the detections. If nothing
appears in these topics, see [troubleshooting](tuning_and_troubleshooting.md#1-symptom-table): the
detector publishes only while a request is active and only on frames with a detection.

---

## 7. Replay a bag

A bag needs the camera topics, `/tf`, `/tf_static` and `/map`. The pipeline can then run without a
robot, using `costmap_from_bag.launch.py` in place of Nav2's costmap:

```bash
ros2 launch approach_path_planning costmap_from_bag.launch.py use_sim_time:=true
ros2 launch approach_path_planning approach_pipeline.launch.py
python3 object_detection/detector.py --ros-args -p use_sim_time:=true
ros2 bag play <bag> --clock
```

Three things to know:

- **Disable the hands stage** with `-p raise_hands_before_approach:=false` on the detector, unless arm
  controllers are running.
- **No Nav2 means no goal output by default.** `/approach_planner/goal_pose` is only forwarded by the
  detector during a request, and a request aborts within 5 s without a `navigate_to_pose` server.
  Run a stub server, such as `FakeNav` from `test_reach.py`, or start Nav2.
- **`use_sim_time:=true` has no effect on the surface detector and planner launch files.** They
  declare the argument but never pass it to the node. Add `use_sim_time: true` to their parameter
  YAML files if you need it.
- **`costmap_from_bag.yaml` marks obstacles from `/object_detector/full_pointcloud`**, which exists
  only on frames with a detection. Until a detection happens, the obstacle layer is empty and the
  costmap shows only the static map.

---

## 8. Test on the robot, stand-alone

Start with the robot about 1 m from the surface, with the object and the surface edge in view. Add one
risk at a time. Nav2 and the SLAM or localization that provides `map` come from your navigation stack:
nothing in this repo starts them.

Check `xy_goal_tolerance` first. The goal is usually less than 1 m away, so a large tolerance (0.4 m in
`nav2_params_r1.yaml`) stops the robot short, or reports success without moving.

`approach_full_stack.launch.py` cannot pass parameters to the detector, so start it by hand.

```bash
# Always: surface detector, planner, RViz
ros2 launch approach_path_planning approach_pipeline.launch.py
```

**Step 1, nothing moves.** Goals go to a logging server, and the arms are not used:

```bash
python3 object_detection/test/fake_nav.py --ros-args -r navigate_to_pose:=approach_dry_run
python3 object_detection/detector.py --ros-args \
  -p navigate_to_pose_action:=approach_dry_run -p raise_hands_before_approach:=false
```

Check the contour and goals in RViz and in the `fake_nav` log.

**Step 2, arms only.** Start both arm controllers and remove `raise_hands_before_approach:=false`. The
hands rise, the base stays still. Clear the area around the arms. Lower the speed with a longer
`hands_motion_duration`. To bring them down:

```bash
ros2 action send_goal /lower_hands surface_detector_interfaces/action/LowerHands "{}"
```

Try this away from the table first. Then measure `hand_retract_forward_offset`, and check the home pose before
lowering at the table: see [L20](known_limitations.md#l20-hands-raised-near-the-table-are-untested-on-a-robot).

**Step 3, full approach.** Stop `fake_nav` and remove the `navigate_to_pose_action` override.

In all steps, send the request with the command in [section 5](#5-send-a-request), and press `Ctrl+C` to
cancel it. Keep `hands_raise_min_distance` in mind: under 0.5 m from the contour, the request aborts.

---

## 9. Run the tests

```bash
python3 object_detection/test/test_reach.py
```

This integration test covers the action and navigation logic of the detector and coordinator. It
runs the real nodes with DINO and SAM stubbed, against a fake Nav2 server, a fake camera and fake
planner goals. Passing looks like this:

```text
PASS 1 goal accepted
...
PASS 6 detector accepts a new request afterwards
ALL PASSED
```

| Scenario | What it verifies |
|---|---|
| 1 | Success path, stale goal dropped, interval and hysteresis gates, preemption does not end the request, feedback relayed, goals after success ignored. |
| 2 | Cancelling a request cancels navigation. |
| 3 | A second concurrent request is rejected. |
| 4 | Timeout when navigation never starts. |
| 5 | Success through the coordinator. |
| 6 | Cancelling through the coordinator reaches navigation, and the detector accepts a new request. |

It uses `ROS_DOMAIN_ID=87` (override with `REACH_TEST_DOMAIN_ID`), so it can run next to a live
pipeline. `detector.py` imports `groundingdino` and `ultralytics` at module level, so these packages
must be importable even though they are never called.

`object_detection/test/test_hands.py` covers the hands stage, with the same stubs plus fake arm controllers
on a private YARP name server (`ROS_DOMAIN_ID=86`, YARP socket 10322, override with
`REACH_TEST_DOMAIN_ID` and `REACH_TEST_YARP_SOCKET`). It needs `yarp` and `yarpserver`.

There are no automated tests for `surface_detector` or `approach_planner`. The synthetic scene in
`docs/tools/` exercises them end to end, but it reports results and asserts nothing.
