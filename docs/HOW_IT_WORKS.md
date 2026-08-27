# How the vision algorithm works

A walkthrough of what `ball_cluster_pipeline.py` does to each frame, and why.
Good to read with a student before they start tuning — every stage maps to a
function in the file.

## The pipeline, stage by stage

### 1. Color threshold → mask  (`threshold_color`)
Convert the frame to HSV and keep only pixels inside the target color range.
The result is a black-and-white **mask**: white where the ball color is, black
everywhere else. We clean it with morphological *open* (remove speckle) and
*close* (fill small holes in the ball). See `docs/HSV_TUNING.md`.

### 2. Blobs → ball detections  (`detect_blobs`)
Find the outlines (contours) of the white regions. Each contour is measured:
- **area** — how many pixels; rejects specks and giant washes.
- **min enclosing circle** — gives a center `(cx, cy)` and radius `r`.
- **circularity** = `4π·area / perimeter²` — 1.0 is a perfect circle. A lone
  ball is round; a merged/occluded blob is less round.

Contours passing the size/roundness filters become **detections**.

### 3. Estimate balls per blob  (`estimate_ball_count`, `reference_single_ball_area`)
When balls touch, their colors merge into one bigger blob — so "number of
blobs" would undercount. Instead we compare each blob's area to the area of one
typical ball and round:

```
balls_in_blob = round(blob_area / one_ball_area)   # at least 1
```

There are two ways to count, chosen by `Config.COUNT_METHOD`:

- **`"peaks"` (default):** run a distance transform on the mask — each pixel gets
  its distance to the nearest background pixel. Because all balls are the same
  size, this has **one local maximum per ball**, each peaking at about the ball's
  pixel radius. We find those peaks (non-max suppressed a ball-width apart) and
  count them. This counts a line, an L, or a 2-D pile correctly, and reads the
  ball radius straight off the peaks so no fixed fallback is needed. An area
  clamp rejects noise. See `find_ball_peaks` and `docs/BALL_COUNT_PLAN.md`.
- **`"area"`:** the simpler method — blob area ÷ one ball's area. "One ball's
  area" is the median area among clean, round blobs, falling back to a configured
  size. It over-counts a line of touching balls when that fallback is off, which
  is exactly why `"peaks"` is the default.

### 4. Group into clusters  (`cluster_detections`)
Balls that are close together belong to the same pile. We link two detections
when the gap between their centers is within
`CLUSTER_LINK_FACTOR × (r_i + r_j)` — i.e. "within a couple ball-widths".
Linking is transitive (A–B and B–C ⇒ one cluster of A, B, C), which we compute
with **union-find**. Each connected group is one cluster.

Scaling the link distance by the balls' own radii means it just works whether
the pile is close to the camera (big balls on screen) or far (small balls).

### 5. Summarize + rank  (`summarize_cluster`, `rank_clusters`)
Each cluster becomes: a ball-count-weighted center, a total ball estimate, a
radius (how big the pile looks), a distance, and a score. Clusters are sorted
**most balls first**; on a tie the **closer** pile wins (smaller known
distance), and if distance is unknown/uncalibrated it falls back to the tighter
(smaller) radius. So a 4-ball pile always outranks a 2-ball pile, but two
3-ball piles are ordered nearest-first.

Before ranking, clusters are **smoothed across frames** (`ClusterTracker`, when
`Config.SMOOTHING_ENABLED`): each cluster is matched to the previous frame's by
nearest center and its position/count/distance are exponentially averaged, so
the robot aims at a calm target instead of a value that jitters frame to frame.
A cluster that leaves view is dropped after a few missed frames rather than
lingering.

### 6. Send it to the robot  (`encode_llpython`, `best_cluster_contour`)
- The ranked clusters are packed into the 32-double `llpython` array (schema at
  the top of the pipeline file, mirrored in `BallClusterResult.java`).
- The best cluster's outline is returned as the pipeline's contour, so the
  Limelight's built-in crosshair `tx/ty` points straight at the densest pile —
  meaning you can aim with plain Limelight APIs, no math required.

### 7. Draw overlays  (`draw_overlays`)
Each ball gets a faint circle + its per-blob estimate; each cluster gets a
colored ring (green = best) and a label. This is your window into what the robot
"sees" — watch it in the dashboard while tuning.

## Why it's split into "pure" and "OpenCV" functions

The stages that are just math — estimation, clustering, ranking, encoding —
don't touch the camera or OpenCV. That's deliberate: they can be unit-tested on
a laptop (`tests/test_ball_clustering.py`) with made-up ball layouts, so you can
prove the logic is right before ever loading it onto the Limelight. Only the
image stages need real frames.

## Distance (optional, built in)

Each detection's pixel diameter is turned into an inch distance with the pinhole
model `distance ≈ focal_px · real_diameter / pixel_diameter`
(`estimate_distance_in`). A cluster's distance is the ball-count-weighted
average of its members' distances. It stays 0 ("unknown") until you set
`CAMERA_FOCAL_PX` — a one-time measurement described in `docs/DISTANCE.md`.

## Field position (robot-side, needs distance calibration)

`teamcode/vision/FieldLocalizer.java` turns a cluster's bearing + distance +
the robot's pose into a field (x, y) position, so you can plan a path to a pile
rather than just turn toward it. It runs on the robot (where the pose lives) and
returns `null` until distance is calibrated. The geometry is validated by
`tools/field_localization.py` and `tests/test_field_localization.py`.

## Already built on top of the basics

- **Temporal smoothing** (`ClusterTracker`): averages each cluster over recent
  frames for a steadier target.
- **Confidence in the score**: each detection's solidity feeds the ranking
  score so ragged, uncertain blobs rank below clean, solid ones on near-ties.
- **Distance** (above) and **field position** (above).

Further ideas: send the robot pose to the Limelight via `llrobot` so it can
report field positions directly; or fuse multiple frames for occlusion handling
(watershed, per the ball-count plan).
