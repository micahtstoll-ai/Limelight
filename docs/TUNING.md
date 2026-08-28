# Tuning guide: every knob, what it does, how to set it

Everything you tune lives in the `Config` class at the top of
`limelight/ball_cluster_pipeline.py`. This page walks each variable in the order
the pipeline uses it, tells you what it controls, and how to set it. You will not
touch most of these; the ones that matter most are marked **TUNE FIRST**.

## The tuning order (do it in this sequence)

Each stage feeds the next, so tune top-down. Fixing a later stage before the
mask is clean just chases symptoms.

1. **Color** so the ball is a clean white blob. (HSV)
2. **Mask cleanup** so blobs are solid and touching balls separate. (erosion)
3. **Blob filter** so only real balls survive. (area / radius / circularity)
4. **Counting** so a pile reads the right number. (peaks)
5. **Clustering** so piles group the way you want. (link factor)
6. **Distance** (one-time calibration) if you want inches.
7. **Polish**: smoothing, ranking, output count.

Work in the Limelight dashboard with the live camera. After each change, Save
and watch the overlay. Keep a ball / a small pile in view.

---

## 1. Color threshold (HSV) -- TUNE FIRST

```
HSV_LOWER = (18, 90, 90)      # (Hue, Saturation, Value) low end
HSV_UPPER = (38, 255, 255)    # high end
DEBUG_VIEW = "normal"         # set to "mask" while tuning color
```

**What it does:** keeps only pixels whose color falls in this HSV band. This is
the single most important thing to get right -- 90% of reliability is here.

**How to tune:** set `DEBUG_VIEW = "mask"`, Save, and the dashboard shows the
black-and-white mask (SnapScript has no built-in mask view). Adjust the two
tuples until the ball is **solid white** and the background is **solid black**,
then set `DEBUG_VIEW = "normal"`. OpenCV HSV ranges are H 0-179, S 0-255,
V 0-255 -- not the 0-360 / 0-100 you may expect.

- Ball flickers / has holes -> lower the `S` and `V` minimums.
- Background junk leaks in -> raise the `S` minimum; tighten the `H` range.
- Ball only seen in bright light -> lower the `V` minimum.

Full walkthrough and starter values for other colors: `docs/HSV_TUNING.md`.

---

## 2. Mask erosion

```
EROSION_ITERATIONS = 1        # 0 disables
EROSION_KERNEL_SIZE = 3       # px, odd numbers (3, 5, ...)
```

**What it does:** shrinks the white blobs after the built-in open/close cleanup.
This pulls apart balls that only touch at a thin bridge (so they become separate
contours) and eats leftover noise.

**How to tune:** leave at 1 to start. If two balls that clearly touch still read
as one blob, raise `EROSION_ITERATIONS` to 2, or `EROSION_KERNEL_SIZE` to 5. If
balls are vanishing or breaking into pieces, you have gone too far -- lower it,
or set iterations to 0.

**Watch out:** erosion also shrinks the real ball on screen, which feeds distance
and area. If you later calibrate distance, do it with erosion at its final
setting.

---

## 3. Blob filtering

```
MIN_AREA_PX = 150      MAX_AREA_PX = 200000     # blob area in pixels
MIN_RADIUS_PX = 6      MAX_RADIUS_PX = 400       # enclosing-circle radius
MIN_CIRCULARITY = 0.55                           # 1.0 = perfect circle
```

**What it does:** a blob must pass ALL of these to count as ball(s). Rejects
specks, giant background washes, and shapes too jagged to be a ball. Pixel
values assume a 640x480 stream -- scale them if you change resolution.

**How to tune:**
- **MIN_AREA_PX / MIN_RADIUS_PX** -- raise until noise specks stop being
  detected; lower if a real, far-away ball is being ignored.
- **MAX_AREA_PX / MAX_RADIUS_PX** -- lower if a big background object (a jersey,
  a wall) gets picked up; otherwise leave large.
- **MIN_CIRCULARITY** -- circularity is `4*pi*area / perimeter^2`; 1.0 is a
  perfect circle. Keep it **forgiving** (0.5-0.6): a single ball is round, but a
  merged pile or a partly-hidden ball is not, and you still want those. Raise it
  only if jagged reflections are sneaking through as false balls.

| Symptom | Fix |
|---|---|
| Noise specks detected as balls | raise `MIN_AREA_PX` / `MIN_RADIUS_PX` |
| Far ball ignored | lower `MIN_AREA_PX` / `MIN_RADIUS_PX` |
| Background object detected | lower `MAX_AREA_PX` / `MAX_RADIUS_PX` |
| Reflections read as balls | raise `MIN_CIRCULARITY` a little |
| A real pile gets rejected | lower `MIN_CIRCULARITY` |

---

## 4. Ball counting

```
COUNT_METHOD = "peaks"             # "peaks" | "area"
PEAK_MIN_DISTANCE_FACTOR = 0.9     # min gap between peaks, x ball radius
PEAK_DT_THRESHOLD_FACTOR = 0.5     # ignore weak peaks below this x radius
COUNT_AREA_CLAMP_LOW = 0.6         # sanity floor vs the area estimate
COUNT_AREA_CLAMP_HIGH = 1.4        # sanity ceiling vs the area estimate
MIN_BALL_RADIUS_FOR_PEAKS_PX = 10  # below this, fall back to the area method
```

**What it does:** decides how many balls a blob holds. `"peaks"` (default) runs a
distance transform and counts one peak per same-size ball, so a line or pile
counts correctly instead of over-counting from area. `"area"` is the simpler
blob-area / one-ball-area method. See `docs/HOW_IT_WORKS.md` and
`docs/BALL_COUNT_PLAN.md`.

**How to tune (peaks):** the defaults are good. If a tight pile **under-counts**
(two touching balls read as one), lower `PEAK_MIN_DISTANCE_FACTOR` toward 0.7 so
closer peaks are allowed. If a single ball **over-counts** (noise inside it makes
extra peaks), raise `PEAK_DT_THRESHOLD_FACTOR` toward 0.6-0.7 so only strong
peaks survive. The `COUNT_AREA_CLAMP_*` pair is a safety net -- it rejects peak
counts that disagree wildly with what the blob area could physically hold; widen
the range only if it is clamping legitimate counts. Very small, distant balls
(radius below `MIN_BALL_RADIUS_FOR_PEAKS_PX`) automatically use the area method.

**Area-method knobs** (only used when `COUNT_METHOD = "area"`, or as the
peak-method fallback):

```
ROUND_ENOUGH_FOR_REFERENCE = 0.80       # circularity to trust a blob as "1 ball"
FALLBACK_SINGLE_BALL_RADIUS_PX = 22     # on-screen ball radius if none is clean
```

- **ROUND_ENOUGH_FOR_REFERENCE** -- the area method sizes "one ball" from the
  median of blobs this round. Lower it if no blob ever qualifies; raise it if
  merged blobs are wrongly used as the reference.
- **FALLBACK_SINGLE_BALL_RADIUS_PX** -- used only when no clean single ball is in
  frame. Set it to the on-screen radius (pixels) of one ball at a typical
  detection distance. A wrong value here is the classic cause of a line of balls
  over-counting under the area method.

---

## 5. Clustering

```
CLUSTER_LINK_FACTOR = 2.0
```

**What it does:** groups nearby balls into one pile. Two balls link when the gap
between their centers is within `CLUSTER_LINK_FACTOR * (r_i + r_j)` -- i.e. within
this many ball-widths. Linking is transitive, so a chain becomes one cluster.

**How to tune:** raise it (2.5-3.0) to make looser, larger clusters that pull in
balls spread further apart; lower it (1.2-1.5) to only group balls that nearly
touch. Because the distance scales with the balls' own radii, one value works at
both near and far range -- you should rarely need to change it.

---

## 6. Distance (one-time calibration)

```
BALL_DIAMETER_IN = 3.0        # real outer diameter of the ball, inches
CAMERA_FOCAL_PX = 0.0         # 0 = uncalibrated (distance reported as unknown)
```

**What it does:** turns a ball's on-screen pixel size into an inch distance using
the pinhole model `distance = focal_px * ball_diameter / ball_pixel_diameter`.

**How to calibrate (about 5 minutes):**

1. Set `BALL_DIAMETER_IN` to your ball's real outer diameter.
2. Put one ball a **known distance** straight ahead -- 40 in is a good choice.
3. Read the ball's **pixel diameter** on screen. Easiest: set
   `DEBUG_VIEW = "mask"` and eyeball the white blob's width, or temporarily
   `print(2 * d["r"])` for a detection and read the Limelight console.
4. Compute the focal length and paste it into `CAMERA_FOCAL_PX`:
   ```
   CAMERA_FOCAL_PX = pixel_diameter * known_distance_in / BALL_DIAMETER_IN
   ```
   (or call the built-in helper `calibrate_focal_px(pixel_diameter,
   known_distance_in, BALL_DIAMETER_IN)`).
   Example: a 3 in ball at 40 in shows 60 px across -> `60 * 40 / 3 = 800`.
5. Verify at a **different** known distance; it should be within a few inches.

**Watch out:** focal length is in pixels, so it depends on resolution -- redo
this if you change the stream size, and do it with erosion at its final setting.
Distances stay `0` (unknown) until this is set, and everything else still works.
Full walkthrough: `docs/DISTANCE.md`.

---

## 7. Temporal smoothing

```
SMOOTHING_ENABLED = True
SMOOTHING_ALPHA = 0.5          # weight on the newest frame; 1.0 = no smoothing
SMOOTHING_MATCH_FACTOR = 1.5   # match a cluster to a prior one within this x radius
SMOOTHING_MAX_MISSES = 3       # forget a track after this many missed frames
```

**What it does:** steadies each cluster across frames so the robot aims at a calm
target instead of a jittery one. Clusters are matched to the previous frame by
nearest center and exponentially averaged.

**How to tune:**
- **SMOOTHING_ALPHA** -- lower (0.3) = smoother but laggier (slower to follow a
  moving pile); higher (0.8) = snappier but twitchier; 1.0 = off. 0.5 is a good
  middle.
- **SMOOTHING_MATCH_FACTOR** -- raise if a fast-moving cluster loses its identity
  frame to frame (gets treated as a new pile); lower if two nearby piles keep
  getting merged into one track.
- **SMOOTHING_MAX_MISSES** -- how many frames a pile can disappear (occlusion,
  flicker) before it is dropped. Raise for steadier persistence, lower to forget
  vanished piles faster.
- Set `SMOOTHING_ENABLED = False` to see raw per-frame clusters (useful while
  debugging detection).

---

## 8. Ranking score

```
SCORE_USE_CONFIDENCE = True
SCORE_CONFIDENCE_FLOOR = 0.5   # fraction of the raw count kept at confidence 0
```

**What it does:** clusters are ranked by ball count first; this blends a
**confidence** term in so that, when counts tie, a clean solid pile outranks a
ragged uncertain one. Confidence is each detection's *solidity* (contour area /
convex-hull area): a real ball or tight pile is solid, while noise and thin
reflections are not. Score is `est * (FLOOR + (1 - FLOOR) * confidence)`.

**How to tune:**
- **SCORE_CONFIDENCE_FLOOR** -- how much confidence is allowed to matter. 1.0
  ignores confidence (score = ball count). 0.5 lets a low-confidence pile lose up
  to half its score. Lower it (0.3) to punish ragged blobs harder; raise it
  toward 1.0 to make ball count almost the only thing that matters.
- Set `SCORE_USE_CONFIDENCE = False` to rank purely by ball count.

---

## 9. Output and overlays

```
MAX_CLUSTERS_REPORTED = 4      # clusters packed into the llpython array
DRAW_OVERLAYS = True           # draw circles/labels on the dashboard image
DEBUG_VIEW = "normal"          # "mask" shows the binary mask instead
```

**What it does:**
- **MAX_CLUSTERS_REPORTED** -- how many ranked piles to send to the robot. Capped
  at 4 by the 32-double `llpython` budget (3 header + 4 x 6 fields = 27). Lower it
  if you only ever act on the best one or two.
- **DRAW_OVERLAYS** -- turn off to send the clean camera image (marginally
  faster); leave on to see what the pipeline detects.
- **DEBUG_VIEW** -- `"mask"` while tuning color, `"normal"` otherwise.

---

## Quick "it's misbehaving" index

| Problem | First knob to try |
|---|---|
| Ball not detected at all | HSV band (use `DEBUG_VIEW="mask"`) |
| Ball detection flickers | HSV `S`/`V` minimums, then smoothing |
| Two touching balls read as one | `EROSION_ITERATIONS`, `PEAK_MIN_DISTANCE_FACTOR` |
| One ball reads as several | `PEAK_DT_THRESHOLD_FACTOR` up |
| A line of balls over-counts | make sure `COUNT_METHOD="peaks"`; else `FALLBACK_SINGLE_BALL_RADIUS_PX` |
| Piles group too loosely / tightly | `CLUSTER_LINK_FACTOR` |
| Distances are 0 | calibrate `CAMERA_FOCAL_PX` |
| Target jitters | `SMOOTHING_ALPHA` down |
| Wrong pile ranked first | check counts; `SCORE_CONFIDENCE_FLOOR` |
| Noise / reflections detected | `MIN_AREA_PX` up, `MIN_CIRCULARITY` up |
