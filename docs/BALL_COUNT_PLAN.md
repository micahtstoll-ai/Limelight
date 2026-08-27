# Plan: robust ball-count estimation (fix over-counting lines & piles)

Status: **implemented (P1 + P2)** — distance-transform peak counting is now the
default (`Config.COUNT_METHOD = "peaks"`), with the ball radius self-calibrated
from the peaks and an area clamp as a safety net. The area method remains
available as `"area"`. Remaining optional work: P3 real-frame tuning and P4
watershed for per-ball centroids (tracked in the issue). This document is kept
as the design rationale.

## The problem

Today `estimate_ball_count(area, single_ball_area) = round(area / single_ball_area)`.
That's a fragile proxy, and it breaks in exactly the case you hit:

- **A line of touching balls merges into one long blob.** Its count now depends
  entirely on `single_ball_area` being right. When no clean, round single ball
  is visible in the frame (a pure line has none), we fall back to the fixed
  `FALLBACK_SINGLE_BALL_RADIUS_PX`. If that fallback is smaller than the ball's
  real on-screen size, the division **over-counts** — a line of 4 can read as 8+.
- **Area error compounds quadratically.** Area ∝ radius², so a 20% error in the
  assumed ball radius becomes a ~44% error in the count.
- **It also under-counts** the other direction: balls overlap in projection
  (merged area < sum of areas) and glare/highlights punch holes in the mask.

Root cause: dividing area by a *guessed* single-ball area, with a fixed fallback
that a line of balls can never correct.

## Goal

A line of 4 → 4. An L of 5 → 5. A triangular pile of 6 → ~6. Robust to touching,
mild occlusion, glare, and lighting swings — within the Limelight's per-frame CPU
budget (must hold a usable frame rate at 640×480).

## Approach: distance-transform peak counting

All the balls are the **same known size**. That's the key we're not using yet.

Run a **distance transform** on the color mask: each pixel's value becomes its
distance to the nearest black pixel. For a blob of same-size circles, the
distance transform has **one local maximum per ball**, each peaking at ≈ the
ball's pixel radius. So:

1. **Distance transform** the mask (`cv2.distanceTransform`).
2. **Estimate ball pixel radius `R`** from the distribution of peak DT values
   (robust statistic — e.g. high-percentile/median of local maxima). This
   *replaces the fixed fallback*: the frame tells us the ball size even when no
   isolated ball is present.
3. **Non-max suppression**: keep local maxima that are ≥ ~`R` apart (balls can't
   be closer than a ball-width). Each surviving peak = one ball.
4. **Count = number of peaks** in the blob / cluster.

This counts lines, L-shapes, and 2-D piles the same way, and gives `R` for free.

### Defense in depth (cross-checks / clamps)

Peak counting alone can misfire on noise or very tight piles, so clamp it:

- **Area bound:** require `0.6·area/ballArea ≤ count ≤ 1.4·area/ballArea`; reject
  peak counts outside that as noise and fall back to the area estimate.
- **Round-single short-circuit:** a high-circularity blob whose area ≈ one ball
  is just 1 — skip the machinery.
- **Elongation check (optional):** for an elongated blob, `length/diameter` from
  `cv2.minAreaRect` should agree with the count within tolerance; a line of 4 has
  length ≈ 4 diameters.

## Alternatives considered

| Option | Verdict |
|---|---|
| Keep area ÷ area | Current; fragile fallback, quadratic error. Reject. |
| Hough circles (`HoughCircles`) | Needs a tuned radius range, double-detects in clusters, heavier CPU. Reject as primary. |
| Full watershed segmentation | Gives per-ball regions but more code + CPU; peak counting gets the *count* without full segmentation. Keep as an optional phase 4 if we need per-ball centroids for heavy occlusion. |
| Distance-transform peak count | Uses the same-size invariant, self-calibrates `R`, cheap, handles all layouts. **Chosen.** |

## Testability

Keep it testable off-hardware, like the rest of the pipeline. The peak-finding,
radius estimation, and clamp logic operate on a numpy distance-map / peak list —
write them as **pure functions** and unit-test with synthetic distance maps:

- single ball, line of 4 touching, line of 4 with gaps, L of 5, triangle of 6,
  a glare hole in a ball, two balls of different colors nearby.

Only `cv2.distanceTransform` stays on-device; everything scoring/counting is pure
numpy and runs in the existing `tests/` harness.

## Config additions (all tunable, defaults safe)

```python
COUNT_METHOD = "peaks"            # "peaks" | "area"  (area = today's behavior)
PEAK_MIN_DISTANCE_FACTOR = 0.9    # min peak spacing, × estimated ball radius R
PEAK_DT_THRESHOLD_FACTOR = 0.5    # ignore DT maxima below this × R
COUNT_AREA_CLAMP_LOW = 0.6        # reject peak counts below this × area estimate
COUNT_AREA_CLAMP_HIGH = 1.4       # ...or above this × area estimate
MIN_BALL_RADIUS_FOR_PEAKS_PX = 10 # below this, balls too small -> use area method
```

## Phasing

1. **P1 — self-calibrating ball radius (small, high value).** Derive `R` per
   frame from the distance transform and feed the existing area method. This
   alone kills the fixed-fallback over-count on lines. Low risk.
2. **P2 — peak counting as primary**, with the area clamp, behind
   `COUNT_METHOD` so we can A/B against today's behavior.
3. **P3 — validate & tune.** Synthetic tests above + a handful of real captured
   frames (line, pile) compared by hand; measure FPS on the Limelight.
4. **P4 (optional) — watershed** for per-ball centroids if heavy-occlusion
   accuracy is needed later.

## Validation & rollout

- Add the synthetic unit tests before flipping the default.
- Capture real frames with `DEBUG_VIEW`/logging for a line and a pile; confirm
  counts match ground truth.
- Watch CPU/frame rate on the device; distance transform + NMS at 640×480 is
  cheap but must be measured, not assumed.
- Ship P1 first (safe), then flip `COUNT_METHOD="peaks"` once P3 passes.

## Risks & mitigations

- **Very tight piles** where centers are < `R` apart merge peaks → under-count.
  Mitigate with slightly sub-`R` spacing and the area cross-check.
- **Distant, tiny balls**: small `R`, noisy peaks. Fall back to the area method
  below `MIN_BALL_RADIUS_FOR_PEAKS_PX`.
- **Performance**: if DT-per-frame is too slow, run it only on blobs whose area
  exceeds ~1.5 balls (a lone ball never needs it).
