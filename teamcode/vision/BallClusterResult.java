package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.List;

/**
 * Decodes the {@code llpython} array produced by the Limelight
 * {@code ball_cluster_pipeline.py} SnapScript into clean, ranked cluster data.
 *
 * <p>Read the raw array on the robot with
 * {@code result.getPythonOutput()} and pass it to {@link #parse(double[])}.
 *
 * <p>The array layout MUST stay in sync with the schema documented at the top
 * of {@code ball_cluster_pipeline.py}:
 * <pre>
 *   [0] schema version (expected {@value #SCHEMA_VERSION})
 *   [1] total estimated balls in frame
 *   [2] number of clusters reported (K)
 *   then K blocks of 5 doubles, best cluster first:
 *       +0 center X, normalized [-1..1]  (left -1, right +1)
 *       +1 center Y, normalized [-1..1]  (top  -1, bottom +1)
 *       +2 estimated ball count
 *       +3 cluster radius, normalized to image width [0..1]
 *       +4 score
 * </pre>
 */
public class BallClusterResult {

    public static final int SCHEMA_VERSION = 1;
    private static final int HEADER_FIELDS = 3;
    private static final int FIELDS_PER_CLUSTER = 5;

    /** One detected group of balls. */
    public static class Cluster {
        /** Center X, normalized [-1..1]; negative = left of center. */
        public final double xNorm;
        /** Center Y, normalized [-1..1]; negative = above center. */
        public final double yNorm;
        /** Estimated number of balls in this cluster. */
        public final int estimatedBalls;
        /** Cluster radius as a fraction of image width [0..1]. */
        public final double radiusNorm;
        /** Ranking score (currently equals estimatedBalls). */
        public final double score;

        Cluster(double xNorm, double yNorm, int estimatedBalls,
                double radiusNorm, double score) {
            this.xNorm = xNorm;
            this.yNorm = yNorm;
            this.estimatedBalls = estimatedBalls;
            this.radiusNorm = radiusNorm;
            this.score = score;
        }

        @Override
        public String toString() {
            return String.format(
                "Cluster[balls=%d x=%.2f y=%.2f r=%.2f score=%.1f]",
                estimatedBalls, xNorm, yNorm, radiusNorm, score);
        }
    }

    private final int totalBalls;
    private final List<Cluster> clusters;

    private BallClusterResult(int totalBalls, List<Cluster> clusters) {
        this.totalBalls = totalBalls;
        this.clusters = clusters;
    }

    /**
     * Parse a raw llpython array. Returns an empty result (no clusters) if the
     * array is null, too short, or from an unexpected schema version -- so
     * callers never have to null-check.
     */
    public static BallClusterResult parse(double[] py) {
        List<Cluster> clusters = new ArrayList<>();
        if (py == null || py.length < HEADER_FIELDS) {
            return new BallClusterResult(0, clusters);
        }
        if ((int) Math.round(py[0]) != SCHEMA_VERSION) {
            return new BallClusterResult(0, clusters);
        }
        int totalBalls = (int) Math.round(py[1]);
        int count = (int) Math.round(py[2]);
        for (int k = 0; k < count; k++) {
            int base = HEADER_FIELDS + k * FIELDS_PER_CLUSTER;
            if (base + FIELDS_PER_CLUSTER > py.length) {
                break;
            }
            clusters.add(new Cluster(
                py[base],
                py[base + 1],
                (int) Math.round(py[base + 2]),
                py[base + 3],
                py[base + 4]));
        }
        return new BallClusterResult(totalBalls, clusters);
    }

    /** Total balls estimated across the whole frame. */
    public int getTotalBalls() {
        return totalBalls;
    }

    /** All reported clusters, best (most balls) first. */
    public List<Cluster> getClusters() {
        return clusters;
    }

    public boolean hasTarget() {
        return !clusters.isEmpty();
    }

    /** The highest-ranked cluster, or {@code null} if none were seen. */
    public Cluster getBestCluster() {
        return clusters.isEmpty() ? null : clusters.get(0);
    }
}
