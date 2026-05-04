#!/usr/bin/env python3
"""
live_tag_viewer.py
------------------
Live detection viewer for the overhead scene camera.
Detects AprilTags AND black squares on white paper (zone corner markers).

Press 'q' to quit, 's' to save a snapshot.

Usage:
  python3 live_tag_viewer.py [--camera 4]
"""

import cv2
import numpy as np
import dt_apriltags
import argparse
import time
import sys

# Tag role assignments
TOOL_TAGS = {2: 'phillips', 3: 'hammer', 4: 'flathead'}
GRIPPER_TAG = 22

# Zone dimensions (inches): x = depth from robot, y = lateral width
# Both zones share 15.5in depth; width distinguishes them (14.5 vs 9)
PICKUP_DIMS = (15.5, 14.5)   # (x_depth, y_width)
DROPOFF_DIMS = (15.5, 9.0)   # (x_depth, y_width)


def get_tag_label(tag_id):
    if tag_id in TOOL_TAGS:
        return f"TOOL:{TOOL_TAGS[tag_id]}", (0, 0, 255)
    elif tag_id == GRIPPER_TAG:
        return f"GRIP:{tag_id}", (0, 255, 255)
    else:
        return f"ID:{tag_id}", (255, 255, 255)


def detect_black_squares(gray, min_area=400, max_area=80000):
    """Detect black squares on white paper in a grayscale image.

    Returns a list of dicts with keys: center, contour, area, corners.
    """
    # Adaptive threshold handles uneven lighting from the angled camera
    binary = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 51, 15)

    # Clean up noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    squares = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area > max_area:
            continue

        # Approximate to polygon
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

        # Must be a quadrilateral
        if len(approx) != 4:
            continue

        # Check roughly square aspect ratio (allow perspective skew)
        x, y, w, h = cv2.boundingRect(approx)
        aspect = float(w) / h if h > 0 else 0
        if aspect < 0.5 or aspect > 2.0:
            continue

        # Solidity check — filled square, not a frame or hollow shape
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area > 0 else 0
        if solidity < 0.80:
            continue

        # Check that interior is dark (mean intensity inside contour < threshold)
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [approx], -1, 255, -1)
        mean_val = cv2.mean(gray, mask=mask)[0]
        if mean_val > 100:  # not dark enough to be a black square
            continue

        M = cv2.moments(contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            squares.append({
                'center': (cx, cy),
                'contour': approx,
                'area': area,
                'corners': approx.reshape(-1, 2),
            })

    return squares


def group_squares_into_zones(squares):
    """Group detected squares into two zones using distance-based clustering.

    Returns a list of zone dicts with keys: squares, centroid, bbox_dims.
    Returns up to 2 zones (pickup and dropoff).
    """
    if len(squares) < 4:
        # Not enough squares for even one full zone
        if len(squares) >= 2:
            centroid = np.mean([s['center'] for s in squares], axis=0).astype(int)
            w = max(s['center'][0] for s in squares) - min(s['center'][0] for s in squares)
            h = max(s['center'][1] for s in squares) - min(s['center'][1] for s in squares)
            return [{'squares': squares, 'centroid': tuple(centroid), 'bbox_dims': (w, h)}]
        return []

    centers = np.array([s['center'] for s in squares], dtype=np.float32)

    if len(squares) < 8:
        # Only enough for one zone
        centroid = np.mean(centers, axis=0).astype(int)
        w = centers[:, 0].max() - centers[:, 0].min()
        h = centers[:, 1].max() - centers[:, 1].min()
        return [{'squares': squares, 'centroid': tuple(centroid), 'bbox_dims': (w, h)}]

    # K-means with k=2
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1.0)
    _, labels, _ = cv2.kmeans(centers, 2, None, criteria, 10, cv2.KMEANS_PP_CENTERS)
    labels = labels.flatten()

    zones = []
    for cluster_id in range(2):
        cluster_indices = np.where(labels == cluster_id)[0]
        if len(cluster_indices) < 2:
            continue
        cluster_squares = [squares[i] for i in cluster_indices]
        cluster_centers = centers[cluster_indices]
        centroid = np.mean(cluster_centers, axis=0).astype(int)
        w = cluster_centers[:, 0].max() - cluster_centers[:, 0].min()
        h = cluster_centers[:, 1].max() - cluster_centers[:, 1].min()
        zones.append({
            'squares': cluster_squares,
            'centroid': tuple(centroid),
            'bbox_dims': (w, h),
        })

    return zones


def identify_zones(zones):
    """Label zones as PICKUP or DROPOFF.

    Both zones share the same 15.5in depth (x). The y-width differs:
      Pickup:  14.5in wide → ratio ~0.94 (nearly square)
      Dropoff:  9.0in wide → ratio ~0.58 (rectangular)

    In the camera frame: pickup appears on the RIGHT, dropoff on the LEFT.
    We use the pixel bounding-box width as the primary cue (wider = pickup),
    with camera-frame horizontal position as a tiebreaker.
    """
    if len(zones) == 0:
        return zones

    for zone in zones:
        w, h = zone['bbox_dims']
        # Pixel-space bounding width of the zone's corner spread
        zone['pixel_spread'] = max(w, h)

    if len(zones) == 1:
        # Single zone — use aspect ratio to guess
        w, h = zones[0]['bbox_dims']
        short = min(w, h) if min(w, h) > 0 else 1
        long_ = max(w, h) if max(w, h) > 0 else 1
        ratio = short / long_
        zones[0]['label'] = 'PICKUP' if ratio > 0.75 else 'DROPOFF'
        return zones

    # Two zones: the wider pixel spread is pickup (14.5in vs 9in lateral width)
    if zones[0]['pixel_spread'] > zones[1]['pixel_spread']:
        zones[0]['label'] = 'PICKUP'
        zones[1]['label'] = 'DROPOFF'
    elif zones[0]['pixel_spread'] < zones[1]['pixel_spread']:
        zones[0]['label'] = 'DROPOFF'
        zones[1]['label'] = 'PICKUP'
    else:
        # Tiebreak: pickup is to the RIGHT in camera frame (higher x pixel)
        if zones[0]['centroid'][0] >= zones[1]['centroid'][0]:
            zones[0]['label'] = 'PICKUP'
            zones[1]['label'] = 'DROPOFF'
        else:
            zones[0]['label'] = 'DROPOFF'
            zones[1]['label'] = 'PICKUP'

    return zones


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', type=int, default=4)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Cannot open camera {args.camera}")
        sys.exit(1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera opened: {actual_w}x{actual_h}", flush=True)

    detector = dt_apriltags.Detector(
        families='tag36h11',
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0)

    cv2.startWindowThread()
    cv2.namedWindow("Scene Camera Live", cv2.WINDOW_AUTOSIZE)
    print("Scene camera viewer running. Press 'q' to quit, 's' to save snapshot.", flush=True)

    frame_count = 0
    fps_start = time.time()
    fps_display = 0.0

    # Colors for zones
    PICKUP_COLOR = (0, 200, 0)    # green
    DROPOFF_COLOR = (0, 165, 255)  # orange

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- AprilTag detection ---
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        gray_enhanced = clahe.apply(gray)

        try:
            tags = detector.detect(gray_enhanced, estimate_tag_pose=False)
        except Exception:
            tags = []

        for tag in tags:
            corners = tag.corners.astype(int)
            label, color = get_tag_label(tag.tag_id)

            for j in range(4):
                cv2.line(frame, tuple(corners[j]), tuple(corners[(j + 1) % 4]), color, 2)

            ctr_x, ctr_y = int(tag.center[0]), int(tag.center[1])
            cv2.circle(frame, (ctr_x, ctr_y), 4, color, -1)
            cv2.putText(frame, label, (ctr_x - 40, ctr_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # --- Black square detection ---
        squares = detect_black_squares(gray)
        zones = group_squares_into_zones(squares)
        zones = identify_zones(zones)

        # Draw each detected square
        for sq in squares:
            cv2.drawContours(frame, [sq['contour']], -1, (255, 255, 0), 2)
            cx, cy = sq['center']
            cv2.circle(frame, (cx, cy), 4, (255, 255, 0), -1)

        # Draw zone overlays
        for zone in zones:
            label = zone.get('label', '???')
            color = PICKUP_COLOR if label == 'PICKUP' else DROPOFF_COLOR
            cx, cy = zone['centroid']

            # Draw convex hull around all squares in this zone
            all_pts = np.vstack([s['corners'] for s in zone['squares']])
            hull = cv2.convexHull(all_pts)
            cv2.polylines(frame, [hull], True, color, 3)

            # Zone label at centroid
            cv2.putText(frame, f"{label} ZONE", (cx - 60, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(frame, f"({len(zone['squares'])} corners)",
                        (cx - 50, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # --- HUD ---
        frame_count += 1
        elapsed = time.time() - fps_start
        if elapsed > 1.0:
            fps_display = frame_count / elapsed
            frame_count = 0
            fps_start = time.time()

        cv2.putText(frame, f"Tags:{len(tags)}  Squares:{len(squares)}  FPS:{fps_display:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Scene Camera Live", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            path = f"/tmp/snapshot_{int(time.time())}.jpg"
            cv2.imwrite(path, frame)
            print(f"Saved: {path}", flush=True)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
