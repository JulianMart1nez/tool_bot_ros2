#!/usr/bin/env python3
"""
RealSense D435i preview with zone marker + AprilTag detection.
Uses top-hat morphology + ring/center contrast for marker detection,
spatial clustering to separate zones, and temporal smoothing for stability.

Press 'q' to quit, 's' to save a snapshot.
Usage: python3 realsense_preview.py [--camera 10]
"""

import cv2
import numpy as np
import dt_apriltags
import sys
import time
import argparse
from collections import deque


# AprilTag assignments
TOOL_TAGS = {2: 'phillips', 3: 'hammer', 4: 'flathead'}
GRIPPER_TAG = 22

# Colors
PICKUP_COLOR = (0, 200, 0)
DROPOFF_COLOR = (0, 165, 255)
MARKER_COLOR = (0, 255, 255)
TAG_TOOL_COLOR = (0, 0, 255)
TAG_GRIP_COLOR = (255, 255, 0)

# Frame dimensions (set on startup)
FRAME_W = 640
FRAME_H = 480


def detect_zone_markers(gray):
    """Detect black-square-on-white-paper zone markers using top-hat."""
    tophat_k = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
    tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, tophat_k)

    _, bright = cv2.threshold(tophat, 25, 255, cv2.THRESH_BINARY)

    clean_k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, clean_k, iterations=2)
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN, clean_k, iterations=1)

    contours, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    markers = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 100 or area > 5000:
            continue

        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.05 * peri, True)

        x, y, w, h = cv2.boundingRect(c)
        aspect = float(w) / h if h > 0 else 0
        if aspect < 0.4 or aspect > 2.5:
            continue

        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        r_inner = max(w, h) // 4
        r_outer = max(w, h) // 2
        if r_inner < 2 or r_outer < 4:
            continue

        ys = np.arange(max(0, cy - r_outer), min(gray.shape[0], cy + r_outer + 1))
        xs = np.arange(max(0, cx - r_outer), min(gray.shape[1], cx + r_outer + 1))
        if len(ys) == 0 or len(xs) == 0:
            continue
        yg, xg = np.meshgrid(ys, xs, indexing='ij')
        dist = np.sqrt((xg - cx) ** 2 + (yg - cy) ** 2)

        center_mask = dist < r_inner
        ring_mask = (dist >= r_inner) & (dist < r_outer)

        roi = gray[max(0, cy - r_outer):min(gray.shape[0], cy + r_outer + 1),
                    max(0, cx - r_outer):min(gray.shape[1], cx + r_outer + 1)]

        if roi.shape != dist.shape:
            continue

        center_pixels = roi[center_mask]
        ring_pixels = roi[ring_mask]
        if len(center_pixels) < 3 or len(ring_pixels) < 3:
            continue

        if ring_pixels.mean() - center_pixels.mean() > 15:
            markers.append({
                'center': (cx, cy),
                'contour': approx,
                'area': area,
                'corners': approx.reshape(-1, 2),
            })

    return markers


def group_markers_into_zones(markers, frame_w):
    """Group markers into zones.

    < 6 markers: one zone (hovering over a single zone).
    >= 6 markers: k-means k=2 (bird's-eye, both zones visible).
    """
    if len(markers) < 2:
        return []

    centers = np.array([m['center'] for m in markers], dtype=np.float32)

    def _make_zone(group):
        gc = np.array([m['center'] for m in group], dtype=np.float32)
        centroid = np.mean(gc, axis=0).astype(int)
        w = gc[:, 0].max() - gc[:, 0].min()
        h = gc[:, 1].max() - gc[:, 1].min()
        return {'markers': group, 'centroid': tuple(centroid), 'bbox_dims': (w, h)}

    if len(markers) < 6:
        return [_make_zone(markers)]

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1.0)
    _, labels, _ = cv2.kmeans(centers, 2, None, criteria, 10, cv2.KMEANS_PP_CENTERS)
    labels = labels.flatten()

    zones = []
    for cid in range(2):
        idx = np.where(labels == cid)[0]
        if len(idx) < 2:
            continue
        zones.append(_make_zone([markers[i] for i in idx]))

    return zones if zones else [_make_zone(markers)]


def identify_zones(zones):
    """Label zones: wider pixel spread = PICKUP (14.5in vs 9in width)."""
    if not zones:
        return zones

    for zone in zones:
        w, h = zone['bbox_dims']
        zone['pixel_spread'] = max(w, h)

    if len(zones) == 1:
        w, h = zones[0]['bbox_dims']
        short = min(w, h) if min(w, h) > 0 else 1
        long_ = max(w, h) if max(w, h) > 0 else 1
        ratio = short / long_
        zones[0]['label'] = 'PICKUP' if ratio > 0.75 else 'DROPOFF'
        return zones

    if zones[0]['pixel_spread'] > zones[1]['pixel_spread']:
        zones[0]['label'] = 'PICKUP'
        zones[1]['label'] = 'DROPOFF'
    elif zones[0]['pixel_spread'] < zones[1]['pixel_spread']:
        zones[0]['label'] = 'DROPOFF'
        zones[1]['label'] = 'PICKUP'
    else:
        if zones[0]['centroid'][0] >= zones[1]['centroid'][0]:
            zones[0]['label'] = 'PICKUP'
            zones[1]['label'] = 'DROPOFF'
        else:
            zones[0]['label'] = 'DROPOFF'
            zones[1]['label'] = 'PICKUP'

    return zones


class TemporalSmoother:
    """Smooths zone centroid positions over N frames to prevent jitter."""

    def __init__(self, window=8):
        self.window = window
        self.history = {}  # label -> deque of (cx, cy)

    def update(self, zones):
        """Update with current frame detections, return smoothed zones."""
        seen = set()
        smoothed = []

        for zone in zones:
            label = zone.get('label', '???')
            seen.add(label)

            if label not in self.history:
                self.history[label] = deque(maxlen=self.window)

            self.history[label].append(zone['centroid'])

            # Smoothed centroid = average of recent detections
            pts = np.array(list(self.history[label]))
            smooth_cx = int(pts[:, 0].mean())
            smooth_cy = int(pts[:, 1].mean())

            smoothed_zone = dict(zone)
            smoothed_zone['centroid'] = (smooth_cx, smooth_cy)
            smoothed_zone['raw_centroid'] = zone['centroid']
            smoothed.append(smoothed_zone)

        return smoothed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', type=int, default=10)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Cannot open camera {args.camera}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, 166)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4600)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"RealSense opened: {actual_w}x{actual_h}", flush=True)
    print("Press 'q' to quit, 's' to save snapshot.", flush=True)

    detector = dt_apriltags.Detector(
        families='tag36h11', nthreads=2, quad_decimate=1.0,
        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)

    smoother = TemporalSmoother(window=8)

    frame_count = 0
    fps_start = time.time()
    fps_display = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- AprilTag detection ---
        try:
            tags = detector.detect(gray, estimate_tag_pose=False)
        except Exception:
            tags = []

        for tag in tags:
            corners = tag.corners.astype(int)
            tid = tag.tag_id
            if tid in TOOL_TAGS:
                color, label = TAG_TOOL_COLOR, f"TOOL:{TOOL_TAGS[tid]}"
            elif tid == GRIPPER_TAG:
                color, label = TAG_GRIP_COLOR, f"GRIP:{tid}"
            else:
                color, label = (255, 255, 255), f"TAG:{tid}"

            for j in range(4):
                cv2.line(frame, tuple(corners[j]), tuple(corners[(j + 1) % 4]), color, 2)
            ctr_x, ctr_y = int(tag.center[0]), int(tag.center[1])
            cv2.circle(frame, (ctr_x, ctr_y), 4, color, -1)
            cv2.putText(frame, label, (ctr_x - 40, ctr_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # --- Zone marker detection ---
        markers = detect_zone_markers(gray)
        zones = group_markers_into_zones(markers, actual_w)
        zones = identify_zones(zones)
        zones = smoother.update(zones)

        # Draw markers
        for m in markers:
            cv2.drawContours(frame, [m['contour']], -1, MARKER_COLOR, 2)
            cx, cy = m['center']
            cv2.circle(frame, (cx, cy), 3, MARKER_COLOR, -1)

        # Draw zones
        for zone in zones:
            label = zone.get('label', '???')
            color = PICKUP_COLOR if label == 'PICKUP' else DROPOFF_COLOR
            cx, cy = zone['centroid']

            all_pts = np.vstack([m['corners'] for m in zone['markers']])
            hull = cv2.convexHull(all_pts)
            cv2.polylines(frame, [hull], True, color, 3)
            cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 30, 2)
            cv2.putText(frame, f"{label} ZONE", (cx - 60, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(frame, f"({cx},{cy}) {len(zone['markers'])}corners",
                        (cx - 70, cy + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Draw frame center crosshair (target for centering)
        fcx, fcy = actual_w // 2, actual_h // 2
        cv2.drawMarker(frame, (fcx, fcy), (255, 255, 255),
                       cv2.MARKER_CROSSHAIR, 40, 2)

        # HUD
        frame_count += 1
        elapsed = time.time() - fps_start
        if elapsed > 1.0:
            fps_display = frame_count / elapsed
            frame_count = 0
            fps_start = time.time()

        cv2.putText(frame,
                    f"Tags:{len(tags)} Markers:{len(markers)} Zones:{len(zones)} FPS:{fps_display:.1f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        cv2.imshow("Detect Zone Test", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            path = f"/tmp/rs_snapshot_{int(time.time())}.jpg"
            cv2.imwrite(path, frame)
            print(f"Saved: {path}", flush=True)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
