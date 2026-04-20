#!/usr/bin/env python3
"""
debug_overlay.py
----------------
Read-only diagnostic viewer. Subscribes to the gripper RGB stream and
overlays EXACTLY what the perception stack is trying to recognize:
  - Zone markers (black-square-on-white-paper, morph top-hat + ring contrast).
  - AprilTags (tag36h11, IDs 2/3/4 for tools, 22 for gripper).
  - HUD text with detection counts.

Publishes an annotated BGR image on /debug/overlay so rqt_image_view
on that topic shows live what the detector sees. Does NOT send motion
goals, does NOT modify detect_zone or fine_localization.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import dt_apriltags


TOOL_TAGS = {2: 'phillips', 3: 'hammer', 4: 'flathead'}
GRIPPER_TAG = 22
IMG_TOPIC = '/gripper_cam/depth_camera/color/image_raw'
OUT_TOPIC = '/debug/overlay'


def detect_zone_markers(gray):
    """Mirror of detect_zone.py:detect_zone_markers (read-only copy)."""
    tophat_k = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
    tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, tophat_k)
    _, bright = cv2.threshold(tophat, 25, 255, cv2.THRESH_BINARY)
    clean_k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, clean_k, iterations=2)
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN, clean_k, iterations=1)

    contours, _ = cv2.findContours(
        bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
                'corners': approx.reshape(-1, 2),
            })
    return markers


class DebugOverlay(Node):

    def __init__(self):
        super().__init__('debug_overlay')
        self.bridge = CvBridge()
        self.detector = dt_apriltags.Detector(
            families='tag36h11', nthreads=2, quad_decimate=1.0,
            quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)

        self.sub = self.create_subscription(Image, IMG_TOPIC, self._cb, 10)
        self.pub = self.create_publisher(Image, OUT_TOPIC, 5)
        self.get_logger().info(
            f'debug_overlay: subscribed {IMG_TOPIC} → publishing {OUT_TOPIC}')

    def _cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # Zone markers (yellow outlines + center dot)
        markers = detect_zone_markers(gray)
        for m in markers:
            cv2.polylines(bgr, [m['corners'].astype(np.int32)], True,
                          (0, 255, 255), 2)
            cx, cy = m['center']
            cv2.circle(bgr, (cx, cy), 3, (0, 255, 255), -1)

        # AprilTags (green box + big ID label, red for unknown ID)
        tags = self.detector.detect(gray, estimate_tag_pose=False)
        for t in tags:
            pts = np.int32(t.corners).reshape(-1, 2)
            tid = int(t.tag_id)
            known = tid in TOOL_TAGS or tid == GRIPPER_TAG
            color = (0, 255, 0) if known else (0, 0, 255)
            cv2.polylines(bgr, [pts], True, color, 2)
            cx, cy = int(t.center[0]), int(t.center[1])
            cv2.circle(bgr, (cx, cy), 4, color, -1)
            label = f'{TOOL_TAGS.get(tid, "gripper" if tid == GRIPPER_TAG else "?")}:{tid}'
            cv2.putText(bgr, label, (cx + 6, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # HUD
        h, w = bgr.shape[:2]
        cv2.drawMarker(bgr, (w // 2, h // 2), (255, 255, 255),
                       cv2.MARKER_CROSS, 14, 1)
        tool_hits = [t.tag_id for t in tags if t.tag_id in TOOL_TAGS]
        hud = (f'zones_markers:{len(markers)}  apriltags:{len(tags)}  '
               f'tool_hits:{tool_hits}')
        cv2.putText(bgr, hud, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 0), 3)
        cv2.putText(bgr, hud, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1)

        out = self.bridge.cv2_to_imgmsg(bgr, 'bgr8')
        out.header = msg.header
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DebugOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
