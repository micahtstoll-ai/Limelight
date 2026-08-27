# Distance estimation & calibration

The pipeline can estimate how far away each ball cluster is, in inches, using a
**pinhole camera model**:

```
distance_in = CAMERA_FOCAL_PX * BALL_DIAMETER_IN / ball_pixel_diameter
```

A ball that looks bigger on screen (more pixels across) is closer; one that
looks smaller is farther. The only unknown is `CAMERA_FOCAL_PX` — the camera's
focal length in pixels — which you measure once.

Until you calibrate, distance is reported as **0 = unknown**, and everything
else (detection, clustering, ranking, aiming) still works.

## One-time calibration (5 minutes)

You need: one ball, a tape measure, and the Limelight dashboard.

1. Place the ball a **known distance** straight in front of the camera — 40 in
   is a good choice. Call this `known_distance_in`.
2. Read the ball's **pixel diameter** on screen. Two easy ways:
   - Set `Config.DEBUG_VIEW = "mask"` and eyeball the white blob's width, or
   - temporarily `print()` a detection's radius in `runPipeline` and read the
     Limelight console: `pixel_diameter = 2 * r`.
3. Compute the focal length. Either do the math:
   ```
   CAMERA_FOCAL_PX = pixel_diameter * known_distance_in / BALL_DIAMETER_IN
   ```
   or use the helper already in the pipeline:
   ```python
   calibrate_focal_px(pixel_diameter, known_distance_in, BALL_DIAMETER_IN)
   ```
   Example: a 3 in ball at 40 in shows 60 px across →
   `60 * 40 / 3 = 800`, so `CAMERA_FOCAL_PX = 800`.
4. Paste that number into `Config.CAMERA_FOCAL_PX` and Save.

## Verify it

Put the ball at a *different* known distance (say 60 in) and check the distance
the dashboard label / telemetry reports. It should be within a few inches. If
it's consistently off by a scale factor, your pixel-diameter reading in step 2
was off — remeasure.

## Notes & gotchas

- **Re-calibrate if you change resolution.** Focal length in *pixels* depends on
  the image size. A value found at 640×480 is wrong at 1280×960.
- **It measures the ball you can see.** For a merged clump, the per-ball pixel
  size still reflects distance, so the cluster distance (the ball-count-weighted
  average of its members) stays reasonable.
- **Accuracy falls off with distance** — a 1 px measurement error matters more
  far away. Treat far distances as approximate; they're plenty for "which pile
  is closer" decisions.
- The cluster distance ignores members whose distance is unknown (0), so a
  partially-calibrated frame still gives a sensible number.

## Where it shows up

- **Dashboard:** each cluster label gains a `…in` suffix once calibrated.
- **Robot (`llpython`):** field `+4` of each cluster block, decoded as
  `Cluster.distanceInches` (see `BallClusterResult.java`). Use
  `cluster.hasDistance()` to check it's real before trusting it.
