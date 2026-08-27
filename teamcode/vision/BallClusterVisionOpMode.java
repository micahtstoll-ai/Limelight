package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Sample TeleOp that reads ball clusters from the Limelight and shows how to
 * use them. Drop this file (and {@link BallClusterResult}) into your TeamCode
 * module under {@code org.firstinspires.ftc.teamcode.vision}.
 *
 * <p>Hardware setup: add a "Limelight3A" device named {@code "limelight"} in
 * your robot configuration, and make sure the Limelight is running the pipeline
 * that has {@code ball_cluster_pipeline.py} loaded (index 0 below).
 *
 * <p>What it demonstrates:
 * <ul>
 *   <li>Starting the Limelight and reading a result each loop.</li>
 *   <li>Decoding the cluster data with {@link BallClusterResult}.</li>
 *   <li>Reading the best (densest) cluster and its bearing.</li>
 *   <li>Using the Limelight's built-in {@code tx} -- which the pipeline aims
 *       at the best cluster -- as a ready-made "turn toward the pile" signal.</li>
 * </ul>
 */
@TeleOp(name = "Ball Cluster Vision (sample)", group = "vision")
public class BallClusterVisionOpMode extends LinearOpMode {

    /** Pipeline index on the Limelight that runs the ball-cluster SnapScript. */
    private static final int PIPELINE_INDEX = 0;

    /** Horizontal FOV of the Limelight 3A, degrees. Used to turn the
     *  normalized cluster X into an approximate bearing. Adjust to your lens. */
    private static final double HORIZONTAL_FOV_DEG = 54.0;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.setPollRateHz(50);   // ask the LL for up to 50 results/sec
        limelight.start();

        telemetry.addLine("Ready. Point the camera at some balls, then press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addLine("No valid Limelight result");
                telemetry.update();
                continue;
            }

            BallClusterResult clusters =
                BallClusterResult.parse(result.getPythonOutput());

            telemetry.addData("Total balls in view", clusters.getTotalBalls());
            telemetry.addData("Clusters", clusters.getClusters().size());

            BallClusterResult.Cluster best = clusters.getBestCluster();
            if (best != null) {
                // Two equivalent ways to get the bearing to the best pile:
                //  1) from our normalized X + FOV, or
                //  2) directly from result.getTx() -- the pipeline points the
                //     Limelight crosshair at the best cluster, so getTx() is
                //     already the bearing to it (usually the cleaner choice).
                double bearingFromNorm = best.xNorm * (HORIZONTAL_FOV_DEG / 2.0);

                telemetry.addData("Best cluster balls", best.estimatedBalls);
                telemetry.addData("Best cluster bearing (from norm)",
                    "%.1f deg", bearingFromNorm);
                telemetry.addData("Best cluster bearing (Limelight tx)",
                    "%.1f deg", result.getTx());
                telemetry.addData("Best cluster distance",
                    best.hasDistance()
                        ? String.format("%.0f in", best.distanceInches)
                        : "unknown (calibrate CAMERA_FOCAL_PX)");

                // Example: a simple proportional "turn toward the pile" value
                // you could feed to a drivetrain. Left stick etc. omitted.
                double turnPower = clamp(result.getTx() / 25.0, -0.5, 0.5);
                telemetry.addData("Suggested turn power", "%.2f", turnPower);

                // Rank the rest for strategy (e.g. plan a collection order).
                int rank = 1;
                for (BallClusterResult.Cluster c : clusters.getClusters()) {
                    telemetry.addData("  #" + rank++, c.toString());
                }
            } else {
                telemetry.addLine("No cluster targeted");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
