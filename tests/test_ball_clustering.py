"""Unit tests for the pure (no-OpenCV) logic in ball_cluster_pipeline.py.

These let you validate the clustering + ranking + output math on a laptop,
with no Limelight and no camera. Run from the repo root:

    python -m pytest tests/            # if you have pytest
    python tests/test_ball_clustering.py   # plain-python fallback (no pytest)

We import the pipeline module directly; its cv2 import is guarded, so this
works even though OpenCV isn't installed here.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "limelight"))

import ball_cluster_pipeline as blp  # noqa: E402


def _ball(cx, cy, r, est=1, circ=0.9):
    """Build a detection dict like detect_blobs() would."""
    import math
    area = math.pi * r * r
    return {"cx": cx, "cy": cy, "r": r, "area": area,
            "circularity": circ, "est": est}


def test_circularity_perfect_circle_is_one():
    import math
    r = 10.0
    area = math.pi * r * r
    perim = 2 * math.pi * r
    assert abs(blp.circularity(area, perim) - 1.0) < 1e-6


def test_estimate_ball_count_scales_with_area():
    single = 100.0
    assert blp.estimate_ball_count(100, single) == 1
    assert blp.estimate_ball_count(210, single) == 2      # ~2 balls merged
    assert blp.estimate_ball_count(320, single) == 3
    assert blp.estimate_ball_count(10, single) == 1       # never below 1


def test_reference_uses_median_of_round_blobs():
    dets = [
        {"area": 100.0, "circularity": 0.95},
        {"area": 120.0, "circularity": 0.90},
        {"area": 5000.0, "circularity": 0.30},   # merged blob, ignored
    ]
    # median of the two round ones (100, 120) -> 120 (upper-middle index)
    assert blp.reference_single_ball_area(dets, fallback_radius=22) == 120.0


def test_reference_falls_back_when_no_clean_ball():
    import math
    dets = [{"area": 9999.0, "circularity": 0.2}]
    expected = math.pi * 22 * 22
    assert abs(blp.reference_single_ball_area(dets, 22) - expected) < 1e-6


def test_two_far_apart_balls_make_two_clusters():
    dets = [_ball(50, 50, 10), _ball(600, 400, 10)]
    groups = blp.cluster_detections(dets, link_factor=2.0)
    assert len(groups) == 2


def test_three_touching_balls_make_one_cluster():
    # centers ~25px apart, radius 10 -> within 2.0*(10+10)=40 link distance
    dets = [_ball(100, 100, 10), _ball(125, 100, 10), _ball(150, 100, 10)]
    groups = blp.cluster_detections(dets, link_factor=2.0)
    assert len(groups) == 1
    assert len(groups[0]) == 3


def test_ranking_puts_biggest_cluster_first():
    # Cluster A: two balls near (100,100). Cluster B: one ball near (500,400).
    dets = [
        _ball(100, 100, 10), _ball(130, 100, 10),   # A -> est 2
        _ball(500, 400, 10),                          # B -> est 1
    ]
    groups = blp.cluster_detections(dets, link_factor=2.0)
    summaries = [blp.summarize_cluster([dets[i] for i in g]) for g in groups]
    ranked = blp.rank_clusters(summaries)
    assert ranked[0]["est"] == 2
    assert ranked[1]["est"] == 1


def test_merged_blob_counts_as_multiple_balls_in_cluster():
    single = _ball(0, 0, 10)["area"]
    # One big merged blob whose area ~= 3 balls, sitting alone.
    merged = _ball(300, 300, 18, est=blp.estimate_ball_count(3 * single, single))
    groups = blp.cluster_detections([merged], link_factor=2.0)
    summary = blp.summarize_cluster([merged])
    assert len(groups) == 1
    assert summary["est"] == 3


def test_encode_llpython_layout_and_normalization():
    # Best cluster dead-center-right, second cluster top-left.
    summaries = [
        {"cx": 480, "cy": 240, "est": 3, "radius": 64, "score": 3.0},
        {"cx": 160, "cy": 120, "est": 1, "radius": 20, "score": 1.0},
    ]
    out = blp.encode_llpython(summaries, total_balls=4, width=640,
                              height=480, max_clusters=5)
    assert len(out) == blp.LLPYTHON_SIZE
    assert out[0] == blp.SCHEMA_VERSION
    assert out[1] == 4.0                     # total balls
    assert out[2] == 2.0                     # clusters reported
    # cluster 0 center x: (480-320)/320 = 0.5 ; y: (240-240)/240 = 0
    assert abs(out[3] - 0.5) < 1e-6
    assert abs(out[4] - 0.0) < 1e-6
    assert out[5] == 3.0                     # est balls
    assert abs(out[6] - 64 / 640) < 1e-6     # radius normalized to width
    # cluster 1 center x: (160-320)/320 = -0.5 ; y: (120-240)/240 = -0.5
    assert abs(out[8] - (-0.5)) < 1e-6
    assert abs(out[9] - (-0.5)) < 1e-6


def test_encode_respects_max_clusters():
    summaries = [{"cx": 10 * i, "cy": 10, "est": 1, "radius": 5, "score": 1.0}
                 for i in range(8)]
    out = blp.encode_llpython(summaries, 8, 640, 480, max_clusters=5)
    assert out[2] == 5.0                      # capped at 5


# ---- plain-python runner (works without pytest) -----------------------------
if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    for t in tests:
        try:
            t()
            print("PASS", t.__name__)
            passed += 1
        except AssertionError as e:
            print("FAIL", t.__name__, "->", e)
        except Exception as e:  # noqa: BLE001
            print("ERROR", t.__name__, "->", repr(e))
    print("\n{}/{} tests passed".format(passed, len(tests)))
    sys.exit(0 if passed == len(tests) else 1)
