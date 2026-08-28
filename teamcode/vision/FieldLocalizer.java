package org.firstinspires.ftc.teamcode.vision;

/**
 * Turns a detected ball cluster into a field position (inches), given the
 * cluster's bearing + distance and the robot's current pose.
 *
 * <p>This runs on the robot because it needs the robot pose (from odometry or a
 * pose estimator). The geometry mirrors, and is unit-tested by,
 * {@code tools/field_localization.py}.
 *
 * <p>Conventions (standard FTC-style, CCW-positive):
 * <ul>
 *   <li>Field frame: x, y in inches; heading CCW from field +x.</li>
 *   <li>Robot frame: x forward, y to the robot's left.</li>
 *   <li>Cluster {@code xNorm} in [-1, 1]: positive is to the RIGHT of the
 *       image, a clockwise (negative) angle in the CCW convention.</li>
 *   <li>Camera mount: {@code camForwardIn}/{@code camLeftIn} are the camera's
 *       position in the robot frame; {@code camYawRad} is its yaw vs robot
 *       forward.</li>
 * </ul>
 *
 * <p>Needs a real distance, so it returns {@code null} until distance
 * estimation is calibrated (see {@code docs/DISTANCE.md}). Use
 * {@link BallClusterResult.Cluster#hasDistance()} to check first.
 */
public final class FieldLocalizer {

    private FieldLocalizer() { }

    /** A field position in inches. */
    public static final class FieldPosition {
        public final double x;
        public final double y;

        public FieldPosition(double x, double y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public String toString() {
            return String.format("Field(%.1f, %.1f)", x, y);
        }
    }

    /**
     * @param cluster            the cluster to locate
     * @param hfovRad            camera horizontal field of view, radians
     * @param camForwardIn       camera position forward of robot center, inches
     * @param camLeftIn          camera position left of robot center, inches
     * @param camYawRad          camera yaw vs robot forward, radians (CCW)
     * @param robotX             robot field x, inches
     * @param robotY             robot field y, inches
     * @param robotHeadingRad    robot heading, radians (CCW from field +x)
     * @return the cluster's field position, or {@code null} if its distance is
     *         unknown (camera not calibrated).
     */
    public static FieldPosition estimate(
            BallClusterResult.Cluster cluster, double hfovRad,
            double camForwardIn, double camLeftIn, double camYawRad,
            double robotX, double robotY, double robotHeadingRad) {
        if (cluster == null || !cluster.hasDistance()) {
            return null;
        }
        double distance = cluster.distanceInches;
        // Bearing of the target off the camera axis (CCW positive).
        double angle = -cluster.xNorm * (hfovRad / 2.0);
        double direction = camYawRad + angle;
        // Target position in the robot frame (x forward, y left).
        double rx = camForwardIn + distance * Math.cos(direction);
        double ry = camLeftIn + distance * Math.sin(direction);
        // Rotate into the field frame by robot heading, then translate.
        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);
        double fieldX = robotX + rx * cosH - ry * sinH;
        double fieldY = robotY + rx * sinH + ry * cosH;
        return new FieldPosition(fieldX, fieldY);
    }
}
