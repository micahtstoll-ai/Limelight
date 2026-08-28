# Limelight Ball Cluster Detection (FTC)

Vision for finding balls and going for the biggest pile.

The Limelight camera watches the field, detects the balls (yellow, ~3" outer
diameter by default), groups nearby balls into **clusters**, and **ranks the
clusters by how many balls they hold**. Your robot code reads that ranking and
can drive toward the densest group.

This is meant to be the foundation for the season's vision work — readable,
tunable, and testable — so the team can build on it instead of fighting it.

## How the pieces fit together

```
   Field                Limelight 3A                          Robot (Control Hub)
 ┌────────┐   frames   ┌────────────────────────────┐        ┌─────────────────────┐
 │ balls  │ ─────────► │ ball_cluster_pipeline.py   │        │ BallClusterResult   │
 │ (piles)│            │  color → blobs → estimate  │  llpython   │  .parse(...)   │
 └────────┘            │  → cluster → rank          │ ─────► │  best cluster, etc. │
                       │  aims tx/ty at best pile   │  tx/ty │  BallClusterVision  │
                       └────────────────────────────┘ ─────► │  OpMode (aim/drive) │
                                                              └─────────────────────┘
```

1. **`limelight/ball_cluster_pipeline.py`** runs *on* the Limelight. It does all
   the image processing and sends results back in two ways:
   - the `llpython` array (rich data: every cluster's position + ball count), and
   - the standard Limelight crosshair `tx/ty`, pointed at the best cluster so
     you can aim with plain Limelight APIs.
2. **`teamcode/vision/BallClusterResult.java`** decodes the `llpython` array into
   clean Java objects.
3. **`teamcode/vision/BallClusterVisionOpMode.java`** is a runnable sample that
   prints the clusters and shows how to turn toward the best pile.

## Quick start

### 1. Load the pipeline onto the Limelight
1. Open the Limelight web UI (its IP in a browser).
2. Create/select a pipeline, set its type to **Python SnapScript**.
3. Paste the entire contents of `limelight/ball_cluster_pipeline.py` into the
   Python editor and **Save**.
4. Open the dashboard preview, point the camera at some balls, and confirm you
   see blue circles on balls and colored circles around clusters.
5. **Tune the color** — see [`docs/HSV_TUNING.md`](docs/HSV_TUNING.md). This is
   the single most important step for reliable detection. (SnapScript has no
   built-in mask view, so the pipeline includes a `DEBUG_VIEW = "mask"` toggle
   for tuning.)
6. **(Optional) Calibrate distance** — see [`docs/DISTANCE.md`](docs/DISTANCE.md)
   to get real inch distances to each cluster. Skippable; everything else works
   without it.

### 2. Add the Java files to your robot
1. Copy `teamcode/vision/BallClusterResult.java` and
   `teamcode/vision/BallClusterVisionOpMode.java` into your `TeamCode` module
   under `org/firstinspires/ftc/teamcode/vision/`.
2. In the Robot Configuration, add a **Limelight3A** device named `limelight`.
3. Build, deploy, run the **"Ball Cluster Vision (sample)"** TeleOp, and watch
   the Driver Station telemetry.

### 3. Test the logic without hardware (optional but recommended)
```bash
python tests/test_ball_clustering.py
```
This exercises the clustering, ball-count estimation, ranking, and output
encoding with synthetic layouts — no camera or OpenCV required. Great for
verifying a change before you push it to the robot.

## Tuning cheat-sheet

All tunables live in the `Config` class at the top of
`ball_cluster_pipeline.py`:

| Setting | What it controls |
|---|---|
| `HSV_LOWER` / `HSV_UPPER` | The target color. **Tune these first.** |
| `DEBUG_VIEW` | Set to `"mask"` to see the color mask for HSV tuning (SnapScript has no built-in mask view); `"normal"` otherwise. |
| `BALL_DIAMETER_IN` | Real ball size (for distance math). |
| `CAMERA_FOCAL_PX` | Focal length in px for distance estimation; `0` = not calibrated (distance reported as unknown). See [`docs/DISTANCE.md`](docs/DISTANCE.md). |
| `MIN_AREA_PX` / `MAX_AREA_PX` | Reject specks and giant background blobs. |
| `MIN_CIRCULARITY` | How round a blob must be to count as ball(s). |
| `EROSION_ITERATIONS` | Shrinks the mask to pull apart touching balls and remove noise; 0 disables. Keep light (1). |
| `COUNT_METHOD` | `"peaks"` (distance-transform peak count, counts lines/piles correctly) or `"area"` (simpler area ratio). |
| `FALLBACK_SINGLE_BALL_RADIUS_PX` | On-screen size of one ball, used by the `"area"` method / as a peak-count fallback. |
| `CLUSTER_LINK_FACTOR` | How close balls must be to join one cluster. |
| `SMOOTHING_ENABLED` / `SMOOTHING_ALPHA` | Steady clusters across frames; lower alpha = smoother, laggier. |
| `MAX_CLUSTERS_REPORTED` | How many clusters to send to the robot (max 4). |

## Adapting to a different ball / game

- **Different color?** Change `HSV_LOWER`/`HSV_UPPER`. For colors that wrap the
  hue wheel (like red), see the note in `threshold_color()`.
- **Different size?** Update `BALL_DIAMETER_IN` and the pixel size settings.
- **Two colors at once (e.g. two alliance colors)?** Duplicate the color mask
  in `threshold_color()` and OR them, or run two pipelines and switch with
  `limelight.pipelineSwitch(...)`.

## Repo layout

```
limelight/ball_cluster_pipeline.py   # the SnapScript that runs on the camera
teamcode/vision/BallClusterResult.java        # llpython decoder
teamcode/vision/BallClusterVisionOpMode.java  # sample OpMode
teamcode/vision/FieldLocalizer.java           # cluster -> field position (needs distance calibration)
tools/field_localization.py           # validated reference for the localizer math
tests/test_ball_clustering.py         # off-hardware unit tests
tests/test_field_localization.py      # field-localization geometry tests
docs/HSV_TUNING.md                    # color calibration walkthrough
docs/DISTANCE.md                      # distance estimation + focal-length calibration
docs/HOW_IT_WORKS.md                  # the vision algorithm, explained
docs/BALL_COUNT_PLAN.md               # proposed: robust count for lines/piles
```
