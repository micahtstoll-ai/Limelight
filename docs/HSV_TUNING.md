# Tuning the color threshold (HSV)

Color detection is where 90% of vision reliability comes from. Spend time here.

## Why HSV instead of RGB

HSV separates **color** (Hue) from **brightness** (Value) and **richness**
(Saturation). That means a yellow ball reads as "yellow" whether it's in bright
sun or shade — the Hue barely moves, only the Value does. RGB mixes all three
together, so it falls apart the moment the lighting changes. Always threshold in
HSV for FTC.

OpenCV's HSV ranges (note these are **not** the 0–360 / 0–100 you may expect):

| Channel | Range | Meaning |
|---|---|---|
| H (hue) | 0–179 | the color itself (yellow ≈ 20–35) |
| S (saturation) | 0–255 | how vivid — low = washed-out/gray |
| V (value) | 0–255 | how bright — low = dark |

## The fast way (Limelight dashboard)

1. Open the Limelight web UI and select your SnapScript pipeline.
2. Switch the preview to show the **threshold / mask** view.
3. Point at your ball and adjust the HSV sliders until:
   - the ball is **solid white** in the mask, and
   - the background is **solid black**.
4. Copy those slider values into `Config.HSV_LOWER` and `Config.HSV_UPPER` in
   `ball_cluster_pipeline.py`.

## Good starting points

```python
# Yellow (default)
HSV_LOWER = (18, 90, 90)
HSV_UPPER = (38, 255, 255)

# Green
HSV_LOWER = (40, 80, 60)
HSV_UPPER = (85, 255, 255)

# Purple / violet
HSV_LOWER = (125, 60, 50)
HSV_UPPER = (160, 255, 255)

# Blue
HSV_LOWER = (95, 120, 60)
HSV_UPPER = (120, 255, 255)
```

Red is special: it lives at both ends of the hue wheel (near 0 and near 179), so
you threshold two ranges and OR them together — see the commented example inside
`threshold_color()`.

## How to adjust when it's not working

| Symptom | Fix |
|---|---|
| Ball has holes / flickers in the mask | Lower `S` and `V` minimums; the ball's highlights/shadows are falling outside the range. |
| Background junk shows up | Raise `S` minimum (kills washed-out grays); tighten the `H` range. |
| Ball detected only in bright light | Lower `V` minimum so shaded balls still pass. |
| Two colors bleed together | Narrow the `H` range so it only covers your color. |

## Tips for match reliability

- **Tune at the venue, under match lighting.** Gym lighting differs from your
  shop. Re-tune during practice/setup if you can.
- Prefer a **wider `S`/`V` range** and a **tight `H` range** — that survives
  lighting swings while still rejecting other colors.
- After the color is right, tune the shape/size filters (`MIN_AREA_PX`,
  `MIN_CIRCULARITY`, etc.) to reject reflections and non-ball objects.
- If detection is jumpy frame-to-frame, that's usually the mask, not the
  clustering — fix the color first.
