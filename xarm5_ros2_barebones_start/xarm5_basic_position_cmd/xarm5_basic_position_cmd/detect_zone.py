#!/usr/bin/env python3
"""
detect_zone.py
--------------
ROS2 node that orchestrates zone detection using the gripper-mounted
RealSense D435i camera. Eliminates the need for a separate scene camera.

Flow:
  1. Subscribes to /voice_command/tool_request
  2. On request → moves arm to Detect Zone pose (tall, bird's-eye view)
  3. Camera detects black-square-on-white-paper zone corner markers
  4. Identifies pickup zone by width (wider = pickup)
  5. Adjusts J1 to center the pickup zone in the camera frame
  6. Moves arm to Hover pose above pickup zone
  7. Hands off to fine_localization for AprilTag-based tool alignment

Subscribes:
  /voice_command/tool_request  (std_msgs/String) — "phrase=...|tool=...|fiducial=..."

Publishes:
  /tool_pose  (geometry_msgs/PoseStamped) — zone centroid for grasp_pose_generator

Requires:
  - Robot stack running (MoveIt2 + controllers)
  - Depth camera launch (depth_camera.launch.py)
"""

import math
import os
import time
import threading
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import dt_apriltags

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
)
from rclpy.action import ActionClient
from cv_bridge import CvBridge


# ── Constants ────────────────────────────────────────────────────────────

PLANNING_GROUP = 'xarm5'
MOVE_ACTION = '/move_action'
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

# Detect Zone pose — tall bird's-eye position (degrees → radians)
DETECT_ZONE_JOINTS = {
    'joint1': math.radians(0),
    'joint2': math.radians(0),
    'joint3': math.radians(-160),
    'joint4': math.radians(145),
    'joint5': math.radians(0),
}

# Hover over pickup zone pose (degrees → radians)
HOVER_PICKUP_JOINTS = {
    'joint1': math.radians(23),
    'joint2': math.radians(23),
    'joint3': math.radians(-130),
    'joint4': math.radians(107),
    'joint5': math.radians(23),
}

# Camera frame dimensions (RealSense @ 640x480)
FRAME_W = 640
FRAME_H = 480

# Cart surface z in link_base frame (meters)
CART_SURFACE_Z = -4.35 * 0.0254   # -0.11049 m — TODO: user will re-measure

# Tool tags
TOOL_TAGS = {2: 'phillips_screwdriver', 3: 'hammer', 4: 'flathead_screwdriver'}
GRIPPER_TAG = 22


# ── Zone marker detection ────────────────────────────────────────────────

def detect_zone_markers(gray):
    """Detect black-square-on-white-paper markers using morphological top-hat.

    Top-hat isolates bright features (white paper) against the dark cart
    surface. Each candidate is verified by checking that its center is
    darker than its surrounding ring (black square inside white paper).
    """
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

        # Ring/center contrast check
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

        roi = gray[max(0, cy - r_outer):min(gray.shape[0], cy + r_outer + 1),
                    max(0, cx - r_outer):min(gray.shape[1], cx + r_outer + 1)]
        if roi.shape != dist.shape:
            continue

        center_pixels = roi[dist < r_inner]
        ring_pixels = roi[(dist >= r_inner) & (dist < r_outer)]
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


def _tag_vertical_angle(corners):
    """Signed delta (deg) between the tag's most-vertical edge pair and the
    image vertical axis. Mirrors debug_overlay.vertical_edge — rotating
    J5 by -ang brings the tag's vertical edge parallel to image vertical.
    """
    c = corners
    pairs = [(c[0], c[1], c[3], c[2]), (c[1], c[2], c[0], c[3])]
    best = None
    for p1a, p1b, p2a, p2b in pairs:
        d1 = p1b - p1a
        d2 = p2b - p2a
        n1 = np.linalg.norm(d1)
        n2 = np.linalg.norm(d2)
        if n1 < 1e-3 or n2 < 1e-3:
            continue
        avg = d1 / n1 + d2 / n2
        an = np.linalg.norm(avg)
        if an < 1e-3:
            continue
        avg /= an
        ang = math.degrees(math.atan2(avg[0], avg[1]))
        if ang > 90.0:
            ang -= 180.0
        elif ang < -90.0:
            ang += 180.0
        if best is None or abs(ang) < abs(best):
            best = ang
    return float(best) if best is not None else 0.0


# ── ROS2 Node ────────────────────────────────────────────────────────────

class DetectZoneNode(Node):

    def __init__(self):
        super().__init__('detect_zone')

        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self.busy = False
        self.debug_dir = None
        self.step_counter = 0

        # Camera state
        self.latest_image = None
        self.frame_w = FRAME_W
        self.frame_h = FRAME_H

        # Temporal smoothing for zone centroid
        self.centroid_history = deque(maxlen=8)

        # AprilTag detector
        self.detector = dt_apriltags.Detector(
            families='tag36h11', nthreads=2, quad_decimate=1.0,
            quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(
            String, '/voice_command/tool_request',
            self.tool_request_callback, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            Image, '/gripper_cam/depth_camera/color/image_raw',
            self._image_cb, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            CameraInfo, '/gripper_cam/depth_camera/color/camera_info',
            self._info_cb, 10,
            callback_group=self.cb_group)

        # ── Publishers ───────────────────────────────────────────────
        self.tool_pose_pub = self.create_publisher(PoseStamped, '/tool_pose', 10)
        self.complete_pub = self.create_publisher(String, '/detect_zone/complete', 10)

        # ── Action client for MoveGroup ──────────────────────────────
        self.move_client = ActionClient(
            self, MoveGroup, MOVE_ACTION, callback_group=self.cb_group)

        self.get_logger().info(
            'Detect Zone node ready. Waiting for /voice_command/tool_request...')

    # ── Debug image capture ──────────────────────────────────────────

    def _start_debug_session(self, tool_name):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.debug_dir = f'/tmp/detect_zone_debug/{ts}_{tool_name}'
        os.makedirs(self.debug_dir, exist_ok=True)
        self.step_counter = 0
        self.get_logger().info(f'Debug images → {self.debug_dir}')

    def _save_debug(self, label, markers=None, zones=None, target_label=None,
                    offset_px=None, extra_text=None):
        """Save raw + annotated current frame to the debug folder."""
        if self.debug_dir is None or self.latest_image is None:
            return
        self.step_counter += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(
                self.latest_image, desired_encoding='bgr8').copy()
        except Exception as e:
            self.get_logger().warn(f'Debug capture failed: {e}')
            return

        raw_path = os.path.join(
            self.debug_dir, f'{self.step_counter:02d}_{label}_raw.jpg')
        cv2.imwrite(raw_path, frame)

        annotated = frame.copy()
        h, w = annotated.shape[:2]

        # Frame center crosshair (white)
        cv2.drawMarker(annotated, (w // 2, h // 2), (255, 255, 255),
                       cv2.MARKER_CROSS, 30, 2)

        if markers:
            for m in markers:
                cx, cy = m['center']
                cv2.drawContours(annotated, [m['contour']], -1, (0, 255, 255), 2)
                cv2.circle(annotated, (cx, cy), 3, (0, 255, 255), -1)

        if zones:
            for z in zones:
                color = (0, 255, 0) if z.get('label') == 'PICKUP' else (0, 165, 255)
                pts = np.array([m['center'] for m in z['markers']], dtype=np.int32)
                if len(pts) >= 3:
                    hull = cv2.convexHull(pts)
                    cv2.polylines(annotated, [hull], True, color, 2)
                cx, cy = z['centroid']
                cv2.drawMarker(annotated, (cx, cy), color,
                               cv2.MARKER_TILTED_CROSS, 20, 2)
                cv2.putText(annotated, z.get('label', '?'), (cx - 25, cy - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if offset_px is not None:
            cv2.putText(annotated, f'offset: {offset_px}px', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        hdr = label
        if target_label:
            hdr += f' | target={target_label}'
        if markers is not None:
            hdr += f' | {len(markers)} markers'
        if zones is not None:
            hdr += f' | {len(zones)} zones'
        cv2.putText(annotated, hdr, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        if extra_text:
            cv2.putText(annotated, extra_text, (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        ann_path = os.path.join(
            self.debug_dir, f'{self.step_counter:02d}_{label}_annotated.jpg')
        cv2.imwrite(ann_path, annotated)

    # ── Camera callbacks ─────────────────────────────────────────────

    def _image_cb(self, msg):
        self.latest_image = msg

    def _info_cb(self, msg):
        if self.frame_w == FRAME_W:
            self.frame_w = msg.width
            self.frame_h = msg.height
            self.get_logger().info(
                f'Camera: {self.frame_w}x{self.frame_h}')

    # ── Voice command handler ────────────────────────────────────────

    def tool_request_callback(self, msg):
        if self.busy:
            self.get_logger().warn('Already executing a request. Ignoring.')
            return

        parts = {}
        for segment in msg.data.split('|'):
            if '=' in segment:
                key, val = segment.split('=', 1)
                parts[key.strip()] = val.strip()

        tool_name = parts.get('tool', 'unknown')
        fiducial_id = int(parts.get('fiducial', '-1'))

        self.get_logger().info(
            f'Tool requested: {tool_name} (AprilTag ID {fiducial_id})')

        self.busy = True
        threading.Thread(
            target=self._run_detect_sequence,
            args=(tool_name, fiducial_id),
            daemon=True).start()

    # ── Main detection sequence ──────────────────────────────────────

    def _run_detect_sequence(self, tool_name, fiducial_id):
        try:
            self._start_debug_session(tool_name)

            # Step 1: Move to bird's-eye Detect Zone pose
            self.get_logger().info('=== DETECT ZONE: Moving to bird\'s-eye pose ===')
            if not self._move_to_joint_pose(DETECT_ZONE_JOINTS, 'DETECT-POSE'):
                self.get_logger().error('Failed to reach Detect Zone pose. Aborting.')
                self._save_debug('birdseye_FAIL_move')
                return

            time.sleep(1.0)  # let camera settle
            self._save_debug('birdseye_arrived')

            # Step 2: Scan for pickup zone
            self.get_logger().info('Scanning for pickup zone markers...')
            self.centroid_history.clear()
            pickup_centroid = self._scan_for_zone('PICKUP', save_label='birdseye_scan')

            if pickup_centroid is None:
                self.get_logger().error('Could not detect pickup zone. Aborting.')
                self._save_debug('birdseye_FAIL_no_pickup')
                return

            self.get_logger().info(
                f'Pickup zone at pixel ({pickup_centroid[0]}, {pickup_centroid[1]})')

            # Step 3: Move to hover pose above pickup zone
            self.get_logger().info('=== Moving to hover position over pickup zone ===')
            if not self._move_to_joint_pose(HOVER_PICKUP_JOINTS, 'HOVER-PICKUP'):
                self.get_logger().error('Failed to reach hover pose. Aborting.')
                self._save_debug('hover_FAIL_move')
                return

            time.sleep(0.5)
            self._save_debug('hover_arrived')

            # Step 4: Center the zone in the camera frame by adjusting J1
            self.get_logger().info('Centering pickup zone in camera frame...')
            zone_centered = self._center_zone_in_frame('PICKUP')

            if not zone_centered:
                self.get_logger().warn(
                    'Could not fully center zone, proceeding to tag-centering.')
                self._save_debug('center_FAIL_or_timeout')
            else:
                self._save_debug('center_success')

            # Step 4b: Acquire (verify visibility of) the REQUESTED tool's
            # AprilTag. We don't try to nudge joints to center it — the
            # J1/J5 → pixel mapping isn't reliable enough open-loop, and
            # go_to already does 3D-pose-based XY alignment via
            # fine_localization (which IS reliable, since poses arrive
            # in link_base through the calibrated TF chain).
            self.get_logger().info(
                f'=== Acquiring tool tag {fiducial_id} ({tool_name}) ===')
            tag_locked = self._acquire_tag(fiducial_id)
            if tag_locked:
                self.get_logger().info(
                    f'=== Tag {fiducial_id} in frame — handing off to go_to ===')
            else:
                self.get_logger().warn(
                    f'Tag {fiducial_id} not acquired. Handing off anyway — '
                    'go_to will wait for a pose.')

            # Step 5: Hand off to go_to. Payload carries tool + fiducial
            # so the downstream node knows which /fine_loc/tag_<id> to track.
            status = 'ok' if tag_locked else (
                'zone_only' if zone_centered else 'degraded')
            complete_msg = String()
            complete_msg.data = (
                f'tool={tool_name}|fiducial={fiducial_id}|status={status}')
            self.complete_pub.publish(complete_msg)

        except Exception as e:
            self.get_logger().error(f'Detect sequence failed: {e}')
        finally:
            self.busy = False

    # ── Zone scanning ────────────────────────────────────────────────

    def _get_frame_gray(self):
        """Get current camera frame as grayscale, or None."""
        if self.latest_image is None:
            return None
        frame = self.bridge.imgmsg_to_cv2(
            self.latest_image, desired_encoding='bgr8')
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _scan_for_zone(self, target_label, max_attempts=15, save_label=None):
        """Scan for a zone and return its temporally-smoothed pixel centroid."""
        last_markers = None
        last_zones = None
        for attempt in range(max_attempts):
            gray = self._get_frame_gray()
            if gray is None:
                self.get_logger().warn('No camera image yet.',
                                       throttle_duration_sec=1.0)
                time.sleep(0.5)
                continue

            markers = detect_zone_markers(gray)
            zones = group_markers_into_zones(markers, self.frame_w)
            zones = identify_zones(zones)
            last_markers, last_zones = markers, zones

            self.get_logger().info(
                f'Scan {attempt + 1}/{max_attempts}: '
                f'{len(markers)} markers, {len(zones)} zones')

            for zone in zones:
                if zone.get('label') == target_label:
                    self.centroid_history.append(zone['centroid'])

                    if len(self.centroid_history) >= 3:
                        pts = np.array(list(self.centroid_history))
                        smooth = (int(pts[:, 0].mean()), int(pts[:, 1].mean()))
                        if save_label:
                            self._save_debug(
                                f'{save_label}_hit', markers=markers,
                                zones=zones, target_label=target_label)
                        return smooth

            time.sleep(0.2)

        if save_label:
            self._save_debug(
                f'{save_label}_timeout', markers=last_markers,
                zones=last_zones, target_label=target_label)

        # If we got at least one detection, return what we have
        if len(self.centroid_history) > 0:
            pts = np.array(list(self.centroid_history))
            return (int(pts[:, 0].mean()), int(pts[:, 1].mean()))

        return None

    def _center_zone_in_frame(self, target_label, max_iterations=5,
                               pixel_tolerance=40):
        """Iteratively adjust J1 to center the target zone in the camera frame.

        Compares the zone centroid x-pixel to the frame center x-pixel.
        If the zone is left of center, decrease J1 (rotate CW from above).
        If the zone is right of center, increase J1 (rotate CCW from above).
        """
        # Degrees of J1 adjustment per pixel of offset (rough calibration)
        # At hover height, ~1 degree of J1 ≈ several pixels of shift
        # Start conservative, the loop will converge
        deg_per_pixel = 0.05

        frame_cx = self.frame_w // 2

        for iteration in range(max_iterations):
            self.centroid_history.clear()
            centroid = self._scan_for_zone(
                target_label, max_attempts=5,
                save_label=f'center_iter{iteration}_scan')

            if centroid is None:
                self.get_logger().warn(
                    f'[CENTER iter {iteration}] Lost zone detection')
                self._save_debug(f'center_iter{iteration}_LOST',
                                 target_label=target_label)
                return False

            zone_cx = centroid[0]
            offset_px = zone_cx - frame_cx

            self.get_logger().info(
                f'[CENTER iter {iteration}] zone_cx={zone_cx} '
                f'frame_cx={frame_cx} offset={offset_px}px')

            if abs(offset_px) < pixel_tolerance:
                self.get_logger().info(
                    f'[CENTER] Zone centered within {pixel_tolerance}px tolerance')
                self._save_debug(
                    f'center_iter{iteration}_DONE',
                    target_label=target_label, offset_px=offset_px,
                    extra_text=f'within {pixel_tolerance}px tolerance')
                return True

            self._save_debug(
                f'center_iter{iteration}_adjust',
                target_label=target_label, offset_px=offset_px,
                extra_text=f'will adjust J1')

            # Compute J1 adjustment
            j1_adjust_deg = offset_px * deg_per_pixel

            # Get current J1 from the hover pose and adjust
            current_j1 = HOVER_PICKUP_JOINTS['joint1']
            # In camera frame: zone right of center → increase J1 (rotate toward it)
            new_j1 = current_j1 + math.radians(j1_adjust_deg)

            adjusted_joints = dict(HOVER_PICKUP_JOINTS)
            adjusted_joints['joint1'] = new_j1

            self.get_logger().info(
                f'[CENTER] Adjusting J1 by {j1_adjust_deg:.1f}deg '
                f'→ {math.degrees(new_j1):.1f}deg')

            if not self._move_to_joint_pose(adjusted_joints, f'CENTER-{iteration}'):
                return False

            # Update the reference for next iteration
            HOVER_PICKUP_JOINTS['joint1'] = new_j1
            time.sleep(0.3)

        return False

    # ── Per-tool tag acquisition (visibility gate, no joint nudging) ─

    def _acquire_tag(self, fiducial_id):
        """Ensure the requested tool's AprilTag is visible to the camera
        before handoff.

        Pixel-based joint nudges were giving unreliable results (J1/J5
        directions depend on wrist pose in a way that's hard to calibrate
        open-loop), so this function does NOT try to fine-align with
        joint moves. It only:

          1. Checks if tag_<fid> is visible in the current frame.
          2. If not, sweeps J1 by ±15 deg in 3-deg steps looking for it.
          3. If found, logs pixel location and returns True.
          4. If never found, returns False (hand-off happens in degraded
             mode and go_to will wait for a pose).

        go_to's continuous tracker then uses /fine_loc/tag_<fid>'s 3D
        pose in link_base to move the TCP over the tag — that path
        is fully TF-calibrated, unlike open-loop pixel→joint nudges.
        """
        # Quick check: is it already visible?
        found = self._find_tag(fiducial_id, max_attempts=10)
        if found is not None:
            tag_cx, tag_cy, ang_deg = found
            self.get_logger().info(
                f'[ACQUIRE] tag {fiducial_id} visible at '
                f'({tag_cx:.0f},{tag_cy:.0f})px  ang={ang_deg:+.1f}deg. '
                f'Handing off — go_to will track via 3D pose.')
            self._save_debug(
                f'acquire_tag{fiducial_id}_FOUND',
                extra_text=f'@({tag_cx:.0f},{tag_cy:.0f}) ang={ang_deg:+.1f}')
            return True

        # Not visible — do a small J1 sweep to try to bring it into view.
        self.get_logger().warn(
            f'[ACQUIRE] tag {fiducial_id} not in initial frame — sweeping J1.')
        sweep = dict(HOVER_PICKUP_JOINTS)
        base_j1 = sweep['joint1']
        for delta_deg in (-5, +5, -10, +10, -15, +15):
            sweep['joint1'] = base_j1 + math.radians(delta_deg)
            if not self._move_to_joint_pose(
                    sweep, f'ACQUIRE-sweep-{delta_deg:+d}'):
                continue
            time.sleep(0.3)
            found = self._find_tag(fiducial_id, max_attempts=8)
            if found is not None:
                tag_cx, tag_cy, ang_deg = found
                self.get_logger().info(
                    f'[ACQUIRE] tag {fiducial_id} FOUND after J1{delta_deg:+d}deg '
                    f'@({tag_cx:.0f},{tag_cy:.0f})px. Handing off.')
                HOVER_PICKUP_JOINTS['joint1'] = sweep['joint1']
                self._save_debug(
                    f'acquire_tag{fiducial_id}_FOUND_after_sweep',
                    extra_text=f'J1{delta_deg:+d}deg')
                return True

        # Restore original hover pose and bail.
        sweep['joint1'] = base_j1
        self._move_to_joint_pose(sweep, 'ACQUIRE-restore')
        self.get_logger().warn(
            f'[ACQUIRE] tag {fiducial_id} NOT FOUND after ±15deg J1 sweep. '
            'Handing off in degraded mode.')
        self._save_debug(f'acquire_tag{fiducial_id}_NOT_FOUND')
        return False

    def _find_tag(self, fiducial_id, max_attempts=10):
        """Scan up to max_attempts camera frames for tag_id==fiducial_id.
        Returns (cx, cy, ang_deg) on first hit, else None."""
        for _ in range(max_attempts):
            gray = self._get_frame_gray()
            if gray is None:
                time.sleep(0.1)
                continue
            tags = self.detector.detect(gray, estimate_tag_pose=False)
            for t in tags:
                if int(t.tag_id) != fiducial_id:
                    continue
                corners = np.asarray(t.corners, dtype=np.float32)
                center = corners.mean(axis=0)
                ang = _tag_vertical_angle(corners)
                return float(center[0]), float(center[1]), ang
            time.sleep(0.1)
        return None

    # ── Joint-space motion ───────────────────────────────────────────

    def _move_to_joint_pose(self, joint_targets, label="move"):
        """Plan and execute a joint-space move."""
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'[{label}] {MOVE_ACTION} not available')
            return False

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = PLANNING_GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        constraints = Constraints()
        for name in JOINT_NAMES:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = joint_targets[name]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        event = threading.Event()
        result_holder = [None]
        accepted = [False]

        def on_goal(future):
            gh = future.result()
            if gh is None or not gh.accepted:
                self.get_logger().error(f'[{label}] Goal rejected')
                event.set()
                return
            accepted[0] = True
            self.get_logger().info(f'[{label}] Executing...')
            gh.get_result_async().add_done_callback(on_result)

        def on_result(future):
            result_holder[0] = future.result()
            event.set()

        self.move_client.send_goal_async(goal).add_done_callback(on_goal)
        if not event.wait(timeout=60.0):
            self.get_logger().error(f'[{label}] Timed out')
            return False

        if not accepted[0] or result_holder[0] is None:
            return False

        if result_holder[0].result.error_code.val == 1:
            self.get_logger().info(f'[{label}] Success')
            return True
        else:
            self.get_logger().error(
                f'[{label}] Failed (error_code='
                f'{result_holder[0].result.error_code.val})')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = DetectZoneNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
