# Object detection

From a text prompt and an RGB-D frame to a 3D point cloud of the object.
Code: `camera_callback` in `object_detection/detector.py`, `project_depth_to_pc_torch` in
`object_detection/utils.py`.

The real models were not run while writing this page. Model behaviour is described from the papers
and from how the code calls them.

---

## 1. Pipeline

```text
color + depth (approximately synchronised)
   │
   ├─ color ──> Grounding DINO + prompt ──> boxes with scores
   │                                           │ keep the highest score
   │                                           v
   ├─ color ──> SAM 2, box prompt ─────────> binary mask
   │                                           │
   └─ depth ──> back-projection <──────────────┘
                  ├─ masked pixels ──> object cloud
                  └─ all valid pixels ──> scene cloud
```

A frame with no box above the thresholds stops here: nothing is published, and the rest of the
pipeline stays idle until a later frame succeeds.

---

## 2. Input contract

The code unpacks image bytes directly, without `cv_bridge`, and without reading the encoding field.
A stream that breaks these assumptions either raises an exception or silently degrades.

| Assumption | Why | If broken |
|---|---|---|
| Color is `bgr8` | Converted with `COLOR_BGR2RGB` before DINO. | `rgb8` input swaps red and blue, which hurts color prompts such as "red mug". |
| Depth is `32FC1` in metres | Read as `float32`. | A `16UC1` stream raises a shape error at `reshape`. |
| No row padding | `reshape(height, width, ...)` ignores `step`. | Shape error. |
| Depth aligned to color, same size | The mask indexes the depth image directly. | Different sizes fail; misalignment shifts the object cloud. |
| Stamps within 0.1 s | `ApproximateTimeSynchronizer` with `slop=0.1`. | Pairs never form, and the callback never runs. |

Both image subscriptions keep only the latest message (depth 1, synchroniser queue 1). Inference is
slower than the camera, so a longer queue would process ever older frames.

The intrinsics come from the first `camera_info` message and are then frozen. With
`use_camera_info_topic:=false`, they come from `cam_calib_mat`.

---

## 3. Detection: Grounding DINO

**The problem.** Classic detectors recognise a closed set of classes and need labelled images for a
new one. Here, the object is named at runtime.

**The idea.** Grounding DINO (Liu et al., ECCV 2024) treats detection as matching image regions to
parts of a sentence. It extracts image features with a Swin Transformer and text features with
BERT, fuses them with cross-attention in both directions, and selects the image features most
related to the text as queries for a DETR-style decoder. Each predicted box gets a similarity score
against every text token.

**Inputs and thresholds.** The image is resized (shorter side 800, longer at most 1333) and
normalised with ImageNet statistics. Two thresholds apply:

| Parameter | Meaning |
|---|---|
| `box_threshold` (0.35) | A box survives if its best score against any token exceeds this. |
| `text_threshold` (0.35) | Within a box, tokens above this form the predicted phrase. |

**Selection.** The code keeps the box with the highest score. There is no check that the phrase
matches the prompt and no tracking across frames, so with two mugs on a table the winner can change
from frame to frame, and the goal can jump with it.

**Prompt writing.** Use a short noun phrase. A distinguishing attribute ("red mug on the left") helps
only as far as the model relates it to image regions, so check the result on `/annotated_dino_img`.

---

## 4. Segmentation: SAM 2

**Why a mask at all.** A box contains the object and the table behind it. Back-projecting the box
would mix table points into the object cloud and corrupt the height estimate, which later anchors
the surface search.

**The idea.** SAM 2 (Ravi et al., 2024) is a class-agnostic promptable segmenter. Prompted with a
box, it produces a mask of the dominant object inside it. It needs no knowledge of what the object
is, which is why it pairs well with a text detector: DINO answers *which*, SAM answers *exactly
where*.

The code calls `self.sam.predict(rgb, bboxes=xyxy, labels=[1])`, with `sam_model` selecting the
weights. If several masks return, `project_depth_to_pc_torch` uses the first.

SAM 2's video memory is unused: each frame is segmented independently.

---

## 5. Back-projection

**The pinhole model.** A 3D point `(X, Y, Z)` in the camera's optical frame projects to pixel
`(u, v)`:

```text
u = fx · X / Z + cx          fx, fy   focal lengths in pixels
v = fy · Y / Z + cy          cx, cy   principal point
```

A depth camera provides `Z` per pixel, so the projection can be inverted:

```text
X = (u - cx) · Z / fx
Y = (v - cy) · Z / fy
```

The RealSense `Z` is depth along the optical axis, not the distance along the ray, so these formulas
apply directly.

`project_depth_to_pc_torch` applies this to whole tensors on the GPU and builds two clouds from the
same frame:

| Cloud | Pixels used | Purpose |
|---|---|---|
| Object | Mask set and depth within range | Object position and height. |
| Scene | All pixels with depth within range | Surface search, and costmap input in the bag setup. |

The depth range is 0.2–3.0 m (`min_depth` default, `max_depth=3.0` at the call site). Beyond this
range, stereo depth error grows quickly, because it scales with the square of the distance. Most
importantly, it means a table more than 3 m away produces no points at all.

Points use the layout `x y z r g b` as `FLOAT32` with unit-range colors, in an unorganised cloud.
Color is carried for visualisation; later stages read only XYZ.

---

## 6. What the object cloud really measures

A depth camera sees only the near surface, so the object cloud is a shell. This has two consequences
later in the pipeline:

- **The centroid is biased toward the camera.** In the [synthetic baseline](approach_planning.md#8-worked-examples),
  the axis of the 4 cm radius cup was at x = 1.85 m and the centroid came out at x = 1.82 m. This
  error enters the reach check directly.
- **Mask errors become geometry errors.** A mask that bleeds onto the table lowers the measured
  object base. A bleed onto a distant wall has a larger effect: the centroid is an unweighted mean,
  so even a small fraction of far points shifts it.

---

## 7. References

- S. Liu et al., *Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object
  Detection*, ECCV 2024.
- A. Kirillov et al., *Segment Anything*, ICCV 2023.
- N. Ravi et al., *SAM 2: Segment Anything in Images and Videos*, 2024.
- R. Hartley and A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed., 2004.
