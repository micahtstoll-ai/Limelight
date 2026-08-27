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

"One ball's area" is measured from the frame itself: the median area among the
clean, round blobs. If nothing round is visible, we fall back to a configured
ball size. This is what lets a clump read as "3 balls" instead of "1 big thing".

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
radius (how big the pile looks), and a score. Clusters are sorted **most balls
first**, ties broken by tighter radius (a compact pile beats a spread-out one).

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

## Ideas for building on this

- **Field position**: combine bearing (`tx`) + distance + robot pose to place
  each pile on the field.
- **Temporal smoothing**: average a cluster's position over a few frames to
  steady a jumpy target.
- **Confidence**: fold circularity / fill-ratio into the score so ragged,
  uncertain blobs rank below clean ones.
