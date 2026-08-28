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

## Why a Python pipeline instead of just an HSV color blob?

The Limelight's built-in color pipeline is essentially "HSV threshold -> find
the biggest blob -> hand back its `tx`, `ty`, and area." That is perfect for
aiming at *one* target, and it is all you need for a lot of games. But it can
only ever tell you about a single blob, and it has no idea what that blob *is*.

This project runs a **Python SnapScript** instead, which executes full OpenCV +
numpy on every frame. That lets the camera do things the fixed color pipeline
simply cannot express:

| Question | Built-in HSV blob | This Python pipeline |
|---|---|---|
| Where is the biggest blob? | yes (`tx`/`ty`) | yes (we still set `tx`/`ty`) |
| How many balls are in a touching clump? | no -- it is one blob | yes (distance-transform peak counting) |
| Which loose balls form a *pile*? | no | yes (spatial clustering) |
| Which pile should we go for? | no | yes (ranked by count, then distance) |
| How far away is it, in inches? | no | yes (calibrated pinhole model) |
| Is this a clean ball or a reflection? | no | yes (confidence from solidity) |
| Steady target across frames? | no | yes (temporal smoothing) |
| Report several targets at once? | no (one blob) | yes (up to 4, structured `llpython`) |

The cost is a little more CPU per frame and some code to maintain -- but you get
vision *logic*, not just a color filter. And because we still point the built-in
`tx`/`ty` crosshair at the best pile, simple aiming stays a one-line read on the
robot; the rich data in `llpython` is there when you want to do more.

HSV tuning still matters just as much here -- it is stage one of the pipeline.
Python does not replace tuning the color; it adds everything that happens *after*
the color is isolated.

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

## Tuning

All tunables live in the `Config` class at the top of
`ball_cluster_pipeline.py`. **[`docs/TUNING.md`](docs/TUNING.md) is the full
guide** -- it explains every variable, the order to tune them in, and a
symptom -> fix index. Start there. The table below is the quick reference.

Tune in pipeline order -- each stage feeds the next:

| Stage | Settings | What it controls |
|---|---|---|
| **1. Color** (tune first) | `HSV_LOWER` / `HSV_UPPER`, `DEBUG_VIEW="mask"` | Isolate the ball as a clean white blob. See [`docs/HSV_TUNING.md`](docs/HSV_TUNING.md). |
| **2. Mask cleanup** | `EROSION_ITERATIONS`, `EROSION_KERNEL_SIZE` | Shrink blobs to split touching balls and kill noise; 0 disables. |
| **3. Blob filter** | `MIN_AREA_PX`/`MAX_AREA_PX`, `MIN_RADIUS_PX`/`MAX_RADIUS_PX`, `MIN_CIRCULARITY` | Keep only real balls; reject specks, walls, and jagged shapes. |
| **4. Counting** | `COUNT_METHOD`, `PEAK_MIN_DISTANCE_FACTOR`, `PEAK_DT_THRESHOLD_FACTOR`, `COUNT_AREA_CLAMP_LOW`/`HIGH`, `MIN_BALL_RADIUS_FOR_PEAKS_PX` | How many balls a blob holds (`"peaks"` counts lines/piles correctly). |
| (area fallback) | `ROUND_ENOUGH_FOR_REFERENCE`, `FALLBACK_SINGLE_BALL_RADIUS_PX` | One-ball size for the simpler area method. |
| **5. Clustering** | `CLUSTER_LINK_FACTOR` | How close balls must be to join one pile. |
| **6. Distance** | `BALL_DIAMETER_IN`, `CAMERA_FOCAL_PX` | Inch distance to each pile (one-time calibration; see [`docs/DISTANCE.md`](docs/DISTANCE.md)). |
| **7. Smoothing** | `SMOOTHING_ENABLED`, `SMOOTHING_ALPHA`, `SMOOTHING_MATCH_FACTOR`, `SMOOTHING_MAX_MISSES` | Steady the target across frames (lower alpha = smoother, laggier). |
| **8. Ranking** | `SCORE_USE_CONFIDENCE`, `SCORE_CONFIDENCE_FLOOR` | Break count ties toward clean, solid piles. |
| **9. Output** | `MAX_CLUSTERS_REPORTED`, `DRAW_OVERLAYS`, `DEBUG_VIEW` | How many piles to send (max 4) and what the dashboard shows. |

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
docs/TUNING.md                        # full tuning guide: every Config variable
docs/HSV_TUNING.md                    # color calibration walkthrough
docs/DISTANCE.md                      # distance estimation + focal-length calibration
docs/HOW_IT_WORKS.md                  # the vision algorithm, explained
docs/BALL_COUNT_PLAN.md               # design notes for the peak-based ball counting
```
