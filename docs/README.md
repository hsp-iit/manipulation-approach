# manipulation-approach documentation

The repository [README](../README.md) is the quick reference. These pages explain how to run the
pipeline, how it works, and the theory behind each stage.

Given a text prompt such as `"red mug"`, the pipeline finds the object with an RGB-D camera, finds
the table or desk it stands on, computes a safe pose at the edge of that surface, and sends the
robot there with Nav2.

## Reading order

| Page | Read it to... |
|---|---|
| [Getting started](getting_started.md) | Install, build, run live or from a bag, send a request, run the tests. |
| [Architecture overview](architecture_overview.md) | Understand the nodes, the frames, and the design choices. Start here for the big picture. |
| [Object detection](object_detection.md) | Follow text prompt to 3D object cloud: Grounding DINO, SAM 2, back-projection. |
| [Surface detection](surface_detection.md) | Follow object cloud to surface contour: height slab, RANSAC, clustering, alpha shapes. |
| [Approach planning](approach_planning.md) | Follow contour to goal pose: candidates, orientation, costmap checks, selection. |
| [Request lifecycle](request_lifecycle.md) | Understand the action, goal forwarding, cancellation, failure modes and concurrency. |
| [Reference](reference.md) | Look up topics, interfaces, parameters, frames and hard-coded values. |
| [Tuning and troubleshooting](tuning_and_troubleshooting.md) | Fix a symptom, or decide which parameter to change. |
| [Known limitations](known_limitations.md) | See the known bugs, gaps and sharp edges, with evidence and workarounds. |

## How these pages were checked

The code is the authority. If a page disagrees with the source, the page has drifted.

| Claim | How it was checked |
|---|---|
| Surface detector and planner behaviour, measured numbers, figures | The compiled nodes ran on a synthetic scene: ray-cast depth, the real `project_depth_to_pc_torch`, a costmap built with the Nav2 inflation law. See `tools/`. |
| Hands raise stage | `object_detection/test/test_hands.py` passes, and `arm_interface.py` was run against a server generated from the arm controller's `service.thrift`. Never run on real arms. |
| Action, cancellation and goal-forwarding behaviour | `object_detection/test/test_reach.py` passes, all 21 checks. `groundingdino` and `ultralytics` were replaced by empty import shims, since the test stubs them anyway. |
| Nav2 cost values and inflation law, PCL plane model and alpha test, `ros2` CLI verbs | Read from the installed Jazzy headers and CLI. |

Not exercised while writing these pages: the real Grounding DINO and SAM 2 models (DINO and SAM were
stubbed), the Docker image, a full Nav2 stack, and the robot. Statements about model behaviour come
from the published papers and the calling code, and no inference timing is quoted. Measured timings
come from one workstation and a synthetic scene, so use them for comparison, not as a performance
specification.

## Regenerating the figures

Everything in `diagrams/` comes from `tools/`, so figures can be rebuilt when the code changes:

```bash
source ~/ws_ros2/install/setup.bash
docs/tools/run_synthetic_scene.sh /tmp/scene     # runs the real nodes, about 2 minutes
python3 docs/tools/make_figures.py /tmp/scene    # needs matplotlib, scipy and graphviz
```

`run_synthetic_scene.sh` uses an isolated ROS domain (`SCENE_DOMAIN_ID`, default 88), so it does not
touch a running pipeline.
