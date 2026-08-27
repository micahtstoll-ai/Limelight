"""Tests for the field-localization geometry (tools/field_localization.py).

Validates the math that FieldLocalizer.java mirrors. Run:
    python tests/test_field_localization.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

import field_localization as fl  # noqa: E402


def _approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


def test_straight_ahead_from_origin():
    # Ball dead center, 40in away, robot at origin facing +x -> (40, 0).
    x, y = fl.camera_target_to_field(0.0, 40, math.radians(60),
                                     0, 0, 0, 0, 0, 0)
    assert _approx(x, 40) and _approx(y, 0)


def test_straight_ahead_robot_facing_plus_y():
    # Same ball, robot facing +y (heading 90 deg) -> (0, 40).
    x, y = fl.camera_target_to_field(0.0, 40, math.radians(60),
                                     0, 0, 0, 0, 0, math.radians(90))
    assert _approx(x, 0, 1e-6) and _approx(y, 40)


def test_target_to_the_right_is_negative_y():
    # x_norm = +1 (full right) with a 90 deg FOV -> 45 deg clockwise.
    x, y = fl.camera_target_to_field(1.0, 10, math.radians(90),
                                     0, 0, 0, 0, 0, 0)
    assert _approx(x, 10 * math.cos(math.radians(-45)))
    assert _approx(y, 10 * math.sin(math.radians(-45)))   # to the right -> -y
    assert y < 0


def test_camera_forward_offset_adds_range():
    # Camera mounted 5in forward; ball 40in dead ahead -> 45in in x.
    x, y = fl.camera_target_to_field(0.0, 40, math.radians(60),
                                     5, 0, 0, 0, 0, 0)
    assert _approx(x, 45) and _approx(y, 0)


def test_robot_translation_offsets_result():
    x, y = fl.camera_target_to_field(0.0, 10, math.radians(60),
                                     0, 0, 0, 100, 50, 0)
    assert _approx(x, 110) and _approx(y, 50)


def test_unknown_distance_returns_none():
    assert fl.camera_target_to_field(0.0, 0, 1.0, 0, 0, 0, 0, 0, 0) is None
    assert fl.camera_target_to_field(0.0, None, 1.0, 0, 0, 0, 0, 0, 0) is None


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
