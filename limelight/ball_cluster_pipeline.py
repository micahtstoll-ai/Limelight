# =============================================================================
#  FTC Ball Cluster Detection  -  Limelight SnapScript pipeline
# =============================================================================
#
#  WHAT THIS DOES
#  --------------
#  Runs ON the Limelight 3A. Every camera frame it:
#    1. Finds yellow balls by color (HSV threshold).
#    2. Filters blobs by size / roundness so only real balls survive.
#    3. Estimates how many balls each blob represents (handles balls that
#       touch / partially hide each other and merge into one blob).
#    4. Groups nearby balls into CLUSTERS.
#    5. RANKS the clusters by estimated ball count (biggest pile first).
#    6. Sends the results back to the robot in the `llpython` array, and
#       aims the Limelight crosshair (tx/ty) at the best cluster.
#    7. Draws overlays so you can watch it work in the Limelight dashboard.
#
#  HOW TO USE IT
#  -------------
#    - Limelight web UI  ->  a pipeline  ->  set type to "Python SnapScript"
#    - Paste this whole file into the Python editor and Save.
#    - Point the camera at some balls and open the dashboard preview.
#    - Tune the CONFIG block below until detection looks clean.
#
#  This whole file is self-contained on purpose: the Limelight editor takes a
#  single script, so everything lives here. The functions that don't need
#  OpenCV are written so they can be unit-tested off the robot (see
#  ../tests/test_ball_clustering.py).
# =============================================================================

import math

# OpenCV + numpy are always present on the Limelight. Off the device (e.g. in
# unit tests on a laptop) cv2 may be missing -- we guard the import so the pure
# math below can still be imported and tested without a camera.
try:
    import cv2
except ImportError:                     # pragma: no cover - only off-device
    cv2 = None

import numpy as np


# =============================================================================
#  CONFIG  --  this is the block you tune. You should rarely need to touch
#  anything below it.
# =============================================================================
class Config:
    # ---- Color threshold (HSV) ----------------------------------------------
    # OpenCV HSV ranges: H 0-179, S 0-255, V 0-255.
    # These defaults target a saturated YELLOW ball. Use the Limelight
    # dashboard's "eyedropper" / HSV tuner, or docs/HSV_TUNING.md, to dial
    # these in for YOUR lighting and YOUR ball.
    #
    # To detect a different color, just change these two rows. To detect a
    # color that wraps around the hue wheel (like red), see the note in
    # threshold_color() below.
    HSV_LOWER = (18, 90, 90)
    HSV_UPPER = (38, 255, 255)

    # ---- Real-world ball size -----------------------------------------------
    # Outer diameter of the ball in inches. Used for distance estimation.
    BALL_DIAMETER_IN = 3.0

    # ---- Distance estimation -------------------------------------------------
    # Camera focal length in PIXELS, for the pinhole model
    #     distance_in = focal_px * ball_diameter_in / ball_pixel_diameter
    # This is a ONE-TIME calibration per camera + resolution. Put one ball at a
    # known distance, read its on-screen pixel diameter, then:
    #     CAMERA_FOCAL_PX = pixel_diameter * known_distance_in / BALL_DIAMETER_IN
    # (helper: calibrate_focal_px() below; walkthrough in docs/DISTANCE.md).
    # Leave at 0.0 until calibrated -- distances are then reported as 0 (unknown)
    # and everything else still works.
    CAMERA_FOCAL_PX = 0.0

    # ---- Blob filtering ------------------------------------------------------
    # A blob must pass ALL of these to count as ball(s). Values are in pixels
    # for a 640x480 stream -- scale them if you change resolution.
    MIN_AREA_PX = 150          # ignore tiny specks / noise
    MAX_AREA_PX = 200000       # ignore huge background washes
    MIN_RADIUS_PX = 6          # min enclosing-circle radius of a blob
    MAX_RADIUS_PX = 400        # max enclosing-circle radius of a blob
    MIN_CIRCULARITY = 0.55     # 1.0 = perfect circle. A single ball is round;
    #                            merged balls / occluded balls are less round,
    #                            so keep this fairly forgiving.

    # ---- Ball-count estimation ----------------------------------------------
    # When balls touch, their color blobs merge into one bigger blob. We
    # estimate how many balls a blob holds by comparing its area to the area
    # of one typical ball. A round, isolated blob is treated as the reference.
    ROUND_ENOUGH_FOR_REFERENCE = 0.80   # circularity above this -> it's a clean
    #                                     single ball we can measure from.
    # Fallback single-ball radius (px) if no clean reference ball is visible in
    # the frame. Set this to the on-screen radius of one ball at a typical
    # detection distance.
    FALLBACK_SINGLE_BALL_RADIUS_PX = 22

    # ---- Clustering ----------------------------------------------------------
    # Two balls join the same cluster when the gap between their centers is
    # within this factor times the sum of their radii. 2.0 means "centers
    # within ~2 ball-widths link up". Bigger = looser groups.
    CLUSTER_LINK_FACTOR = 2.0

    # ---- Output --------------------------------------------------------------
    # 4 clusters * 6 fields + 3 header = 27 doubles, within the 32 llpython cap.
    MAX_CLUSTERS_REPORTED = 4  # how many clusters to pack into llpython

    # ---- Overlay drawing -----------------------------------------------------
    DRAW_OVERLAYS = True

    # ---- Debug view ----------------------------------------------------------
    # SnapScript has no separate built-in threshold view -- the dashboard shows
    # whatever image this pipeline returns. Set to "mask" to send the binary
    # color mask to the dashboard for HSV tuning (ball = white, background =
    # black); set back to "normal" for the annotated camera image.
    DEBUG_VIEW = "normal"      # "normal" | "mask"


# =============================================================================
#  llpython OUTPUT SCHEMA  (32 doubles, read on the robot with
#  result.getPythonOutput())
#
#    index 0 : SCHEMA_VERSION (currently 2)
#    index 1 : total estimated balls in the whole frame
#    index 2 : number of clusters reported (K, 0..MAX_CLUSTERS_REPORTED)
#    then K blocks of 6 doubles, best cluster first:
#        +0 : center X, normalized [-1..1]  (left -1, right +1)
#        +1 : center Y, normalized [-1..1]  (top  -1, bottom +1)
#        +2 : estimated ball count in this cluster
#        +3 : cluster radius, normalized to image width [0..1]
#        +4 : distance to cluster, inches (0 = unknown / not calibrated)
#        +5 : score (currently == estimated ball count)
#    unused trailing entries are 0.
#
#  Keep this table identical to BallClusterResult.java on the robot side.
# =============================================================================
SCHEMA_VERSION = 2
HEADER_FIELDS = 3
FIELDS_PER_CLUSTER = 6
LLPYTHON_SIZE = 32


# -----------------------------------------------------------------------------
#  PURE HELPERS  (no OpenCV -- unit-testable off the robot)
# -----------------------------------------------------------------------------
def circularity(area, perimeter):
    """How round a blob is: 1.0 == perfect circle, lower == more ragged."""
    if perimeter <= 0:
        return 0.0
    return float(4.0 * math.pi * area / (perimeter * perimeter))


def estimate_distance_in(pixel_diameter, focal_px, real_diameter_in):
    """Distance to a ball via the pinhole model.

    distance = focal_px * real_diameter / pixel_diameter

    A bigger ball on screen (larger pixel_diameter) means it's closer. Returns
    0.0 (unknown) if we can't compute it -- no calibration, or a zero-size blob.
    """
    if focal_px <= 0 or pixel_diameter <= 0:
        return 0.0
    return float(focal_px * real_diameter_in / pixel_diameter)


def calibrate_focal_px(pixel_diameter, known_distance_in, real_diameter_in):
    """One-time calibration helper (inverse of estimate_distance_in).

    Put a ball at a known distance, read its on-screen pixel diameter, and pass
    all three here to get the CAMERA_FOCAL_PX value to paste into Config.
    """
    if real_diameter_in <= 0:
        return 0.0
    return float(pixel_diameter * known_distance_in / real_diameter_in)


def estimate_ball_count(area, single_ball_area):
    """Estimate how many balls a blob of `area` represents.

    A single ball -> 1. A blob roughly twice a ball's area -> 2, etc. Always
    at least 1 (we already know it passed the filters, so it's >= one ball).
    """
    if single_ball_area <= 0:
        return 1
    return max(1, int(round(area / single_ball_area)))


def reference_single_ball_area(detections, fallback_radius):
    """Pick the area of one 'typical' ball from this frame.

    We use the median area among blobs round enough to be a lone ball. If none
    are that clean, fall back to a circle of the configured fallback radius.
    `detections` is a list of dicts with keys 'area' and 'circularity'.
    """
    clean = [
        d["area"]
        for d in detections
        if d["circularity"] >= Config.ROUND_ENOUGH_FOR_REFERENCE
    ]
    if clean:
        clean.sort()
        return float(clean[len(clean) // 2])          # median
    return float(math.pi * fallback_radius * fallback_radius)


def cluster_detections(detections, link_factor):
    """Group detections whose circles are close together (union-find).

    Two detections i, j link when the distance between centers is within
    link_factor * (r_i + r_j). Returns a list of clusters; each cluster is a
    list of indices into `detections`. Each detection is a dict with keys
    'cx', 'cy', 'r'.
    """
    n = len(detections)
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]              # path halving
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for i in range(n):
        for j in range(i + 1, n):
            di, dj = detections[i], detections[j]
            dist = math.hypot(di["cx"] - dj["cx"], di["cy"] - dj["cy"])
            if dist <= link_factor * (di["r"] + dj["r"]):
                union(i, j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


def summarize_cluster(members):
    """Reduce a cluster (list of detection dicts) to a summary dict.

    center = ball-count-weighted centroid; est = sum of member estimates;
    radius = distance from center to the farthest member edge; score = est;
    distance = ball-count-weighted mean of member distances (0 if none known).
    Members without a 'distance' key are treated as unknown (0).
    """
    total_est = sum(m["est"] for m in members)
    weight = float(total_est) if total_est > 0 else float(len(members))
    cx = sum(m["cx"] * m["est"] for m in members) / weight
    cy = sum(m["cy"] * m["est"] for m in members) / weight
    radius = 0.0
    for m in members:
        radius = max(radius, math.hypot(m["cx"] - cx, m["cy"] - cy) + m["r"])
    # Average only over members with a known (>0) distance, weighted by est.
    dist_weight = sum(m["est"] for m in members if m.get("distance", 0) > 0)
    if dist_weight > 0:
        distance = sum(
            m.get("distance", 0) * m["est"]
            for m in members if m.get("distance", 0) > 0
        ) / dist_weight
    else:
        distance = 0.0
    return {
        "cx": cx,
        "cy": cy,
        "est": total_est,
        "radius": radius,
        "distance": distance,
        "score": float(total_est),
    }


def rank_clusters(summaries):
    """Best cluster first: most balls, ties broken by tighter (smaller) radius."""
    return sorted(summaries, key=lambda c: (-c["score"], c["radius"]))


def encode_llpython(summaries, total_balls, width, height, max_clusters):
    """Pack ranked cluster summaries into the fixed 32-double llpython array."""
    out = [0.0] * LLPYTHON_SIZE
    reported = min(len(summaries), max_clusters)
    out[0] = float(SCHEMA_VERSION)
    out[1] = float(total_balls)
    out[2] = float(reported)

    half_w = width / 2.0 if width else 1.0
    half_h = height / 2.0 if height else 1.0
    for k in range(reported):
        c = summaries[k]
        base = HEADER_FIELDS + k * FIELDS_PER_CLUSTER
        out[base + 0] = (c["cx"] - half_w) / half_w         # x norm [-1..1]
        out[base + 1] = (c["cy"] - half_h) / half_h         # y norm [-1..1]
        out[base + 2] = float(c["est"])                     # ball count
        out[base + 3] = c["radius"] / width if width else 0.0
        out[base + 4] = c.get("distance", 0.0)              # inches (0=unknown)
        out[base + 5] = c["score"]
    return out


# -----------------------------------------------------------------------------
#  OpenCV STAGES  (need the camera / cv2 -- run on the Limelight)
# -----------------------------------------------------------------------------
def threshold_color(image):
    """BGR frame -> cleaned binary mask of the target color."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array(Config.HSV_LOWER, dtype=np.uint8)
    upper = np.array(Config.HSV_UPPER, dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    # For a color that wraps hue (e.g. red near H=0 and H=179) you would
    # threshold two ranges and OR them:
    #   mask = cv2.inRange(hsv, low1, high1) | cv2.inRange(hsv, low2, high2)

    # Clean up salt-and-pepper noise and close small gaps inside a ball.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_blobs(mask):
    """Binary mask -> list of ball detections (dicts), plus their contours.

    Each detection: {cx, cy, r, area, circularity, contour}. Estimation and
    clustering happen later on these dicts.
    """
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    detections = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < Config.MIN_AREA_PX or area > Config.MAX_AREA_PX:
            continue
        (x, y), r = cv2.minEnclosingCircle(c)
        if r < Config.MIN_RADIUS_PX or r > Config.MAX_RADIUS_PX:
            continue
        perim = cv2.arcLength(c, True)
        detections.append(
            {
                "cx": float(x),
                "cy": float(y),
                "r": float(r),
                "area": float(area),
                "circularity": circularity(area, perim),
                "contour": c,
            }
        )
    return detections


def draw_overlays(image, detections, ranked, width, height):
    """Draw detections + ranked clusters onto the frame for the dashboard."""
    # Rank colors: best cluster green, then yellow, orange, red-ish, gray.
    rank_colors = [
        (0, 255, 0),
        (0, 255, 255),
        (0, 165, 255),
        (0, 80, 255),
        (160, 160, 160),
    ]
    # Individual ball detections in faint blue with their per-blob estimate.
    for d in detections:
        center = (int(d["cx"]), int(d["cy"]))
        cv2.circle(image, center, int(d["r"]), (255, 160, 0), 1)
        cv2.putText(
            image, str(d["est"]), center,
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 160, 0), 1,
        )
    # Clusters, best first.
    for i, c in enumerate(ranked[: Config.MAX_CLUSTERS_REPORTED]):
        color = rank_colors[min(i, len(rank_colors) - 1)]
        center = (int(c["cx"]), int(c["cy"]))
        cv2.circle(image, center, int(c["radius"]), color, 2)
        label = "#{} balls~{}".format(i + 1, c["est"])
        if c.get("distance", 0) > 0:
            label += " {:.0f}in".format(c["distance"])
        cv2.putText(
            image, label, (center[0] - 40, center[1] - int(c["radius"]) - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
        )
    return image


def best_cluster_contour(ranked, detections_by_index):
    """Return a contour for the best cluster so Limelight's tx/ty locks onto it.

    We hand back the convex hull of the best cluster's member contours. If
    there are no clusters, return an empty array (Limelight treats that as
    'no target').
    """
    if not ranked:
        return np.array([[]])
    best = ranked[0]
    pts = best.get("_contour_points")
    if pts is None or len(pts) == 0:
        return np.array([[]])
    return cv2.convexHull(pts)


# -----------------------------------------------------------------------------
#  ENTRY POINT  --  Limelight calls this once per frame.
# -----------------------------------------------------------------------------
def runPipeline(image, llrobot):
    """image: BGR numpy frame. llrobot: doubles sent from the robot.

    Returns (largestContour, image, llpython) as Limelight expects.
    """
    height, width = image.shape[:2]

    mask = threshold_color(image)
    detections = detect_blobs(mask)

    # Estimate ball count per blob using this frame's reference ball size.
    single_area = reference_single_ball_area(
        detections, Config.FALLBACK_SINGLE_BALL_RADIUS_PX
    )
    for d in detections:
        d["est"] = estimate_ball_count(d["area"], single_area)
        d["distance"] = estimate_distance_in(
            2.0 * d["r"], Config.CAMERA_FOCAL_PX, Config.BALL_DIAMETER_IN
        )

    # Cluster, summarize, rank.
    groups = cluster_detections(detections, Config.CLUSTER_LINK_FACTOR)
    summaries = []
    for g in groups:
        members = [detections[i] for i in g]
        s = summarize_cluster(members)
        # stash the raw contour points so we can build a hull for tx/ty
        s["_contour_points"] = (
            np.vstack([m["contour"] for m in members]) if members else None
        )
        summaries.append(s)
    ranked = rank_clusters(summaries)

    total_balls = int(sum(d["est"] for d in detections))
    llpython = encode_llpython(
        ranked, total_balls, width, height, Config.MAX_CLUSTERS_REPORTED
    )

    # In SnapScript mode the dashboard shows whatever image we return -- there
    # is no separate built-in threshold/mask view. So when tuning color, flip
    # Config.DEBUG_VIEW to "mask" to send the black-and-white mask back to the
    # dashboard instead of the camera image. Tune HSV until the ball is solid
    # white and the background is solid black, then set it back to "normal".
    if Config.DEBUG_VIEW == "mask":
        output_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    else:
        output_image = image
        if Config.DRAW_OVERLAYS:
            output_image = draw_overlays(
                output_image, detections, ranked, width, height
            )

    largest = best_cluster_contour(ranked, detections)
    return largest, output_image, llpython
