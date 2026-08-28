"""Reference implementation of ball-cluster field localization.

This is the validated geometry that `teamcode/vision/FieldLocalizer.java`
mirrors. It turns a cluster's bearing + distance + the robot's pose into a
field (x, y) position. It runs off the robot (for analysis and to unit-test the
math); the real-time version lives in Java on the robot, which is where the
robot pose is available.

Conventions (standard FTC-style, CCW-positive):
  - Field frame: x, y in inches; heading measured CCW from field +x.
  - Robot frame: x forward, y to the robot's left.
  - Cluster x_norm in [-1, 1]: positive is to the RIGHT of the image, which is
    a clockwise (negative) angle in the CCW convention.
  - Camera mount: cam_forward_in / cam_left_in are the camera's position in the
    robot frame; cam_yaw_rad is the camera's yaw relative to robot forward.

Needs a real distance (inches), so it returns None until distance estimation is
calibrated (see docs/DISTANCE.md, issue #7).
"""

import math


def camera_target_to_field(x_norm, distance_in, hfov_rad,
                           cam_forward_in, cam_left_in, cam_yaw_rad,
                           robot_x, robot_y, robot_heading_rad):
    """Return (field_x, field_y) in inches, or None if distance is unknown."""
    if distance_in is None or distance_in <= 0:
        return None
    # Bearing of the target off the camera axis (CCW positive).
    angle = -x_norm * (hfov_rad / 2.0)
    direction = cam_yaw_rad + angle
    # Target position in the robot frame (x forward, y left).
    rx = cam_forward_in + distance_in * math.cos(direction)
    ry = cam_left_in + distance_in * math.sin(direction)
    # Rotate into the field frame by the robot heading, then translate.
    cos_h, sin_h = math.cos(robot_heading_rad), math.sin(robot_heading_rad)
    field_x = robot_x + rx * cos_h - ry * sin_h
    field_y = robot_y + rx * sin_h + ry * cos_h
    return (field_x, field_y)
