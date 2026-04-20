# -*- coding: utf-8 -*-
import cv2
import numpy as np

FRAME_W = 320
FRAME_H = 240
CENTER_X = FRAME_W // 2  # 160, used only for debug line

ROI_TOP = 132
BOTTOM_IGNORE_HEIGHT = 6

#  w w Camera offset compensation  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
# Camera is mounted offset to the right.
# When car drives correctly (yellow line on right, ~295px), error should = 0.
# Measure: drive straight along the line and read line_cx. Put that value here.
LANE_TARGET_CX = 295   # line_cx when car is positioned correctly

# How sensitive steering is.
# error = line_cx - LANE_TARGET_CX
#   negative error -> line drifted left  -> car steers left
#   positive error -> line drifted right -> car steers right
# Deadband: ignore tiny wobbles
LANE_DEADBAND = 12

#  w w Component filter  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
MIN_COMPONENT_AREA    = 200
MIN_COMPONENT_WIDTH   = 6
MIN_COMPONENT_HEIGHT  = 20
MIN_ELONGATION_RATIO  = 1.3
MIN_SOLIDITY          = 0.50

#  w w Morphology  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
OPEN_KERNEL  = (3, 3)
CLOSE_KERNEL = (7, 5)

#  w w Hough (debug only)  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
HOUGH_THRESHOLD = 25
MIN_LINE_LENGTH = 30
MAX_LINE_GAP    = 10

#  w w Row sampling  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
SAMPLE_ROWS    = [232, 220, 208, 196, 184, 172, 160, 148, 136]
MIN_ROW_PIXELS = 8
MAX_ROW_JUMP   = 50
EDGE_MARGIN    = 8


#  w w ROI  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w

def apply_roi(mask):
    roi = np.zeros_like(mask)
    cv2.fillPoly(roi, [np.array([
        (0, FRAME_H), (0, ROI_TOP), (FRAME_W, ROI_TOP), (FRAME_W, FRAME_H)
    ], dtype=np.int32)], 255)
    out = cv2.bitwise_and(mask, roi)
    if BOTTOM_IGNORE_HEIGHT > 0:
        out[FRAME_H - BOTTOM_IGNORE_HEIGHT:FRAME_H, :] = 0
    return out


#  w w Colour mask (white + yellow)  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w

def build_lane_mask(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # White / tinted-white
    white = cv2.inRange(hsv,
                        np.array([15,  60, 155], dtype=np.uint8),
                        np.array([110, 185, 255], dtype=np.uint8))
    glare = cv2.inRange(hsv,
                        np.array([0,   0, 175], dtype=np.uint8),
                        np.array([180, 45, 255], dtype=np.uint8))
    white = cv2.bitwise_and(white, cv2.bitwise_not(glare))

    # Yellow
    yellow = cv2.inRange(hsv,
                         np.array([15, 100, 100], dtype=np.uint8),
                         np.array([35, 255, 255], dtype=np.uint8))

    mask = cv2.bitwise_or(white, yellow)
    mask = apply_roi(mask)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones(OPEN_KERNEL,  np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones(CLOSE_KERNEL, np.uint8))
    return mask

# Alias
def build_white_mask(frame_bgr):
    return build_lane_mask(frame_bgr)


#  w w Component filter  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w

def filter_components(mask):
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    filtered = np.zeros_like(mask)

    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        w    = stats[i, cv2.CC_STAT_WIDTH]
        h    = stats[i, cv2.CC_STAT_HEIGHT]

        if area < MIN_COMPONENT_AREA: continue
        if w < MIN_COMPONENT_WIDTH or h < MIN_COMPONENT_HEIGHT: continue

        comp = np.uint8(labels == i) * 255
        contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: continue
        cnt = contours[0]

        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        if hull_area < 1: continue
        if float(area) / hull_area < MIN_SOLIDITY: continue

        if len(cnt) >= 5:
            _, (mw, mh), _ = cv2.minAreaRect(cnt)
            long_side  = max(mw, mh)
            short_side = min(mw, mh)
            elongation = long_side / short_side if short_side >= 1 else long_side
        else:
            elongation = max(w, h) / float(max(1, min(w, h)))

        if elongation < MIN_ELONGATION_RATIO: continue
        filtered[labels == i] = 255

    return filtered


#  w w Hough (debug only)  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w

def detect_hough_lines(mask):
    edges = cv2.Canny(mask, 40, 120)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180,
                            threshold=HOUGH_THRESHOLD,
                            minLineLength=MIN_LINE_LENGTH,
                            maxLineGap=MAX_LINE_GAP)
    return edges, lines


#  w w Row sampling: find the line spine  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w

def find_segments_on_row(mask, y):
    xs = np.where(mask[y] > 0)[0]
    xs = xs[(xs >= EDGE_MARGIN) & (xs < FRAME_W - EDGE_MARGIN)]
    if len(xs) == 0:
        return []
    segments, start, prev = [], xs[0], xs[0]
    for x in xs[1:]:
        if x == prev + 1:
            prev = x
        else:
            segments.append((start, prev))
            start = prev = x
    segments.append((start, prev))
    return segments


def collect_line_center_points(mask):
    """Pick the widest segment on each row (the lane line spine)."""
    points = []
    for y in SAMPLE_ROWS:
        if y < ROI_TOP or y >= FRAME_H:
            continue
        segs = find_segments_on_row(mask, y)
        if not segs:
            continue
        # Filter segments wide enough, then pick closest to LANE_TARGET_CX
        valid = [s for s in segs if s[1] - s[0] + 1 >= MIN_ROW_PIXELS]
        if not valid:
            continue
        best = min(valid, key=lambda s: abs((s[0] + s[1]) // 2 - LANE_TARGET_CX))
        points.append(((best[0] + best[1]) // 2, y))

    # Smooth: drop jumps
    if not points:
        return []
    points = sorted(points, key=lambda p: p[1], reverse=True)
    smoothed = [points[0]]
    for x, y in points[1:]:
        if abs(x - smoothed[-1][0]) <= MAX_ROW_JUMP:
            smoothed.append((x, y))
    return smoothed


def fit_line_cx(points, y_query):
    if len(points) == 0: return None
    if len(points) == 1: return int(points[0][0])
    ys = np.array([p[1] for p in points], dtype=np.float32)
    xs = np.array([p[0] for p in points], dtype=np.float32)
    coeff = np.polyfit(ys, xs, 1)
    return int(round(coeff[0] * y_query + coeff[1]))


#  w w Core: compute error  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w

def compute_steering(line_points):
    """
    Returns (found, mode, error, line_cx).

    error > 0  -> line is RIGHT of target  -> car steers RIGHT  (car drifted left)
    error < 0  -> line is LEFT  of target  -> car steers LEFT   (car drifted right)
    error = 0  -> line is exactly where it should be -> go straight
    """
    if not line_points:
        return False, "none", 0, None

    # Use bottommost actual point; use fit only if stable and in-frame
    bottom_x = int(max(line_points, key=lambda p: p[1])[0])
    line_cx = bottom_x
    if len(line_points) >= 3:
        fitted = fit_line_cx(line_points, SAMPLE_ROWS[0])
        if fitted is not None and 0 <= fitted < FRAME_W:
            line_cx = fitted

    line_cx = max(0, min(FRAME_W - 1, line_cx))

    error = line_cx - LANE_TARGET_CX   # 0 when driving correctly

    # ── Curve look-ahead: detect slope (line heading left or right) ──────────
    # Compare topmost point x vs bottommost point x.
    # For a right turn: the line curves toward RIGHT side as it goes farther up
    # (smaller y = higher in image). So top_x > bottom_x means curving right.
    CURVE_SLOPE_THRESHOLD = 18   # px difference to count as a curve
    CURVE_ERROR_BOOST     = 28   # extra error added when curve detected

    slope_bias = 0
    if len(line_points) >= 3:
        sorted_pts = sorted(line_points, key=lambda p: p[1])  # sort by y ascending (top first)
        top_x    = sorted_pts[0][0]
        bot_x    = sorted_pts[-1][0]
        slope_dx = top_x - bot_x   # positive = line tilts right (right turn ahead)

        if slope_dx >= CURVE_SLOPE_THRESHOLD:
            slope_bias = CURVE_ERROR_BOOST   # push error right → steer right
        elif slope_dx <= -CURVE_SLOPE_THRESHOLD:
            slope_bias = -CURVE_ERROR_BOOST  # push error left → steer left

    combined_error = error + slope_bias

    if abs(combined_error) <= LANE_DEADBAND and slope_bias == 0:
        mode = "straight"
        error = 0
    elif combined_error < 0:
        mode = "line_left"
        error = combined_error
    else:
        mode = "line_right"
        error = combined_error

    return True, mode, error, line_cx


#  w w Drawing helpers  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w

def draw_polyline(debug, points, color):
    if len(points) < 2: return
    cv2.polylines(debug, [np.array(points, dtype=np.int32).reshape(-1,1,2)], False, color, 2)

def draw_sample_points(debug, points, color):
    for x, y in points:
        cv2.circle(debug, (int(x), int(y)), 4, color, -1)


#  w w Main entry  w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w

def detect_white_lane(frame_bgr):
    debug = frame_bgr.copy()

    white_mask    = build_lane_mask(frame_bgr)
    filtered_mask = filter_components(white_mask)
    edges, hough_lines = detect_hough_lines(filtered_mask)

    line_points = collect_line_center_points(filtered_mask)

    # Hough fallback
    if len(line_points) == 0 and hough_lines is not None:
        best = max(hough_lines, key=lambda l: np.hypot(l[0][2]-l[0][0], l[0][3]-l[0][1]))
        x1, y1, x2, y2 = best[0]
        line_points = [((x1+x2)//2, (y1+y2)//2)]

    found, mode, error, line_cx = compute_steering(line_points)

    # target_x for display (what app.py uses as "target_x")
    target_x = (line_cx if line_cx is not None else CENTER_X)

    #  w w Debug overlay  w w
    # ROI box + camera center (red) + lane target (orange dashed)
    cv2.rectangle(debug, (0, ROI_TOP), (FRAME_W, FRAME_H), (255, 255, 0), 1)
    cv2.line(debug, (CENTER_X, ROI_TOP), (CENTER_X, FRAME_H), (0, 0, 255), 2)

    # Lane target reference line (where line should be)
    for y in range(ROI_TOP, FRAME_H, 8):
        cv2.line(debug, (LANE_TARGET_CX, y), (LANE_TARGET_CX, min(y+4, FRAME_H)), (0, 165, 255), 1)

    # Hough lines (green, thin)
    if hough_lines is not None:
        for line in hough_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(debug, (x1,y1), (x2,y2), (0,200,0), 1)

    # Line spine: cyan dots + polyline
    draw_polyline(debug, line_points, (0, 255, 255))
    draw_sample_points(debug, line_points, (0, 255, 255))

    # Detected line_cx (yellow vertical)
    if line_cx is not None:
        cv2.line(debug, (line_cx, ROI_TOP), (line_cx, FRAME_H), (0, 255, 255), 2)

    # HUD
    color = (0, 255, 0) if mode == "straight" else (0, 100, 255)
    cv2.putText(debug, f"mode={mode}",          (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.putText(debug, f"error={error}",         (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.putText(debug, f"line_pts={len(line_points)}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200,255,120), 1)
    if line_cx is not None:
        cv2.putText(debug, f"line_cx={line_cx} target={LANE_TARGET_CX}", (10, 88),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,165,255), 1)
    # Show slope info
    if len(line_points) >= 3:
        sorted_pts2 = sorted(line_points, key=lambda p: p[1])
        sdx = sorted_pts2[0][0] - sorted_pts2[-1][0]
        slope_txt = f"slope={sdx:+d}"
        slope_col = (0,200,255) if abs(sdx) >= 18 else (120,120,120)
        cv2.putText(debug, slope_txt, (10, 106), cv2.FONT_HERSHEY_SIMPLEX, 0.42, slope_col, 1)

    return {
        "found":       found,
        "mode":        mode,
        "error":       error,
        "target_x":    target_x,
        "center_x":    CENTER_X,
        "line_cx":     line_cx,
        "debug_frame": debug,
        "mask":        filtered_mask,
        "edges":       edges,
    }# -*- coding: utf-8 -*-
