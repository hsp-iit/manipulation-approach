# Known limitations

Behaviour that may surprise you, with evidence and workarounds. Items are ordered by practical
impact. The numbers come from the synthetic scene in `tools/` unless stated otherwise. Item numbers
are stable identifiers, so resolved items leave a gap.

---

## Safety and correctness

### L2. Surface detection slows sharply close to the table

Time per frame went from 0.48 s at 1 m from the edge to 10.7 s at 0.3 m, which is when the goal is
being refined. Meanwhile, queued frames age. See
[performance](surface_detection.md#7-performance).

**Workaround.** `thicken_ransac: false` with `min_cluster_size: 20` gave 0.02 s with a nearly identical
contour on synthetic data. Verify on real data before relying on it.

### L3. Reach ignores height and the arms

The planner measures reach as a 2D distance from the base centre to the object's centroid, with one
limit (`dist_threshold`). A humanoid reaches from its shoulders, in 3D, with one arm or both. The
height difference between shoulder and table can use up much of the arm's length, and an object can
suit one arm and not the other, so a stance that passes this check is not guaranteed to be graspable.

Goal scoring keeps the robot close to the object, but the stance can only be as good as the geometry
allows: the object's depth behind the edge plus the 0.25 m offset. Objects near the 0.35 m depth limit
are still at the limit of the reach check.

**Workaround.** Set `dist_threshold` below the arm's true reach, to keep a margin. This also rejects
deeper objects.

### L4. Contour edges may be view limits, not surface edges

The contour comes from one frame, so the edge of the camera's view looks identical to a table edge.
Candidates generated from such a virtual edge stand *inside* the real table's footprint, or too close
to it.

This matters most near the table: at 0.3 m the true near edge was out of view, and the contour's
front edge lay 10 cm inside the table. Only the costmap protects against this, so its quality is a
safety dependency. If the table is not in the costmap, for instance because it is unseen, or the
costmap's obstacle band starts above the tabletop, nothing rejects these candidates.

In this scene the clearance check rejects every front candidate, so the planner publishes no goal. A
robot already driving keeps its last goal, but a request started this close to the table ends in the
navigation start timeout.

**Possible fix.** Accumulate contours in `map` over frames, or reject contour segments that lie along
the image border.

### L20. Hands raised near the table are untested on a robot

The stage ran against fake controllers and a generated Thrift server, never on real arms. Open points:

- **The offsets are placeholders.** `hand_forward_offset`, `hand_lateral_offset` and the clearance need
  measuring. A wrong forward offset invalidates the stance check.
- **The hands are in the camera view.** Hands were not part of the synthetic scene. They may occlude the
  object or surface. If a costmap uses the same camera without filtering, they can appear as obstacles in
  front of the robot. `visual-language-navigation` removes them with `sam_filter`; this repo does not.
- **The raise distance relies on the contour.** A virtual edge can make the surface look farther than it
  is, so `hands_raise_min_distance` can pass when the real edge is closer.
- **Objects on the table are not checked.** Only the surface height is known, so anything taller than
  `hand_surface_clearance` in the path of the hands can be hit.
- **Lowering at the table is not proven safe.** `/lower_hands` retracts to `hand_retract_forward_offset`, a
  placeholder that must be behind the surface edge. The second stage is still a straight line: if the home pose is
  ahead of the edge and below the surface, it can cross the edge. Check the home pose, or back away before lowering.

### L5. `reached` does not verify the object

`reached: true` means that Nav2 finished. Nothing re-detects the object, checks the final distance, or
confirms that the object is still where it was. Callers should verify it before starting a grasp.

---

## Perception

### L6. Nothing tracks object identity

Each frame takes the highest-scoring box, independently. With two matching objects, the target can
switch between frames, and the goal follows. Prompts are the only control.

### L7. The object and surface must be visible at the start

There is no search behaviour. If the object is not detected within `navigation_start_timeout`, the
request fails. The depth limit means that a surface beyond 3 m does not exist for this pipeline, even
if DINO detects the object.

### L8. Single-frame shape estimate

The centroid sits on the visible side of the object, shifted toward the camera
(2.7 cm for a 4 cm radius cup). The surface shape has occlusion gaps, which are bridged only below
0.4 m. Features narrower than about 0.4 m, such as a small indentation, are not represented.

### L9. Horizontal surfaces only

The plane model requires a normal within `ransac_eps` of Z, so tilted surfaces are not found. The
largest plane in the slab wins, which may not be the one touching the object.

### L10. Image encoding is assumed

`rgb8` input is accepted without warning and swaps the channels sent to DINO. `/sam_mask_img` is
labelled `rgb8` while containing BGR data, so its scene colors appear swapped. This is cosmetic.

---

## Planning

### L11. The robot is a circle

`robot_radius` has a `TODO` to read the footprint. An elongated robot gets one radius for all
directions, and the planner's radius (0.2) and the bag costmap's (0.1) are set independently.

### L12. Fallback positions keep the old heading

`findNearestFreeCell` moves a candidate by up to 14 cm, and the heading is not recomputed. Its name
also misleads: it picks the lowest cost in a 5×5 window, not the nearest free cell.

### L13. Selection ignores path length

The robot term of the goal score uses straight-line distance, which underestimates a drive around a
long table. The BFS already visits these cells, so it could supply path distances cheaply.

### L14. Hand height is not used yet

The object's height reaches the planner but is only used for markers. A `TODO` notes that a hand
position should be sent.

---

## Tooling

### L15. `use_sim_time` is ignored by two launch files

`approach_planner.launch.py` and `surface_detector.launch.py` declare the argument without passing it
to the node. Set it in the YAML.

### L16. Inactive lifecycle nodes keep working

Subscribers are not lifecycle-aware, so a deactivated node keeps computing. `approach_planner`'s
`on_cleanup` also leaves its subscribers in place, unlike `surface_detector`'s.

### L17. Parameters and values are scattered

The surface detector's compiled defaults differ from its YAML (see the
[reference](reference.md#surface_detector-1)), and several tuning values are
[hard-coded](reference.md#5-hard-coded-values), including the device, the depth limit and alpha.
`docker/run.sh` has a user-specific volume path.

### L18. No tests for the C++ nodes

`test_reach.py` covers the Python request logic only. The geometry in the planner and surface
detector has no assertions. `docs/tools/` could seed a regression test, since it already runs them
end to end.

### L19. Differences from the repository README

The [README](../README.md) is out of step with the code on these points:

| README says | Code does |
|---|---|
| `ros2 action call ...` | The verb does not exist in Jazzy. Use `ros2 action send_goal`. |
| The surface detector publishes the plane and marker | It also publishes `/surface_detector/results` and `/surface_detector/closer_cluster`. |
| The detector requires `/tf` | It creates a listener but performs no lookups. |
| The `surface_detector` "Default" column | Lists YAML values. Compiled defaults differ. |
| The build script needs only `TAG` and `DOCKER_NAME` | It also needs three arguments: git user, email and token. |
