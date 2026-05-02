#!/usr/bin/env python3
"""
debug_overlay.py
----------------
Read-only diagnostic viewer. Two image-only safety signals — no depth.

  1) SIZE   — measured tag edge length in px vs the per-tool grabbable
              sweet size (+/- tol). Phillips/hammer/flathead each have
              their own sweet size because the tools sit at slightly
              different heights on the cart.
  2) ALIGN  — angle (deg) between the tag's most-vertical edge pair and
              the image vertical axis. Drives a J5 rotation correction:
              rotate J5 by ALIGN to bring the tag's vertical edge
              parallel to the camera-frame vertical axis.

Both must agree (SIZE in band AND |ALIGN| <= tol) for IN BAND.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import dt_apriltags


# Per-tool sweet size in px at the grabbable range, +/- per-tool tol.
# Mirror of go_to.SWEET_PX / SWEET_TOL_PX_PER_TID — keep these two
# tables in sync so the IN BAND overlay matches the grab-arming check.
SWEET_PX = {
    2: 98.0,    # phillips  (96-100 px)
    3: 107.5,   # hammer    (105-110 px, strict)
    4: 98.0,    # flathead  (96-100 px)
}
# Per-tool tolerance. Falls back to the sweet_tol_px node parameter
# for any tid not listed.
SWEET_TOL_PX_PER_TID = {2: 2.0, 3: 2.5, 4: 2.0}
TOOL_NAMES = {2: 'phillips', 3: 'hammer', 4: 'flathead'}
GRIPPER_TAG = 22
RGB_TOPIC = '/gripper_cam/depth_camera/color/image_raw'
OUT_TOPIC = '/debug/overlay'


def vertical_edge(corners):
    """Pick the tag's most-vertical pair of parallel edges and return a line
    that runs THROUGH the tag center, PARALLEL to those edges (not the line
    connecting their midpoints — that one is perpendicular).

    Returns:
        angle_deg  — signed delta from image vertical, in [-90, 90]
        center     — (cx, cy) tag centroid in image
        dir_unit   — unit vector along the line direction
    """
    c = corners.astype(np.float32)
    pairs = [
        (c[0], c[1], c[3], c[2]),  # bottom + top edges
        (c[1], c[2], c[0], c[3]),  # right + left edges
    ]
    center = c.mean(axis=0)
    best = None
    for p1a, p1b, p2a, p2b in pairs:
        d1 = p1b - p1a
        d2 = p2b - p2a
        n1 = np.linalg.norm(d1)
        n2 = np.linalg.norm(d2)
        if n1 < 1e-3 or n2 < 1e-3:
            continue
        # Average direction of the two parallel edges (handles slight
        # perspective skew). Then normalise.
        avg = d1 / n1 + d2 / n2
        an = np.linalg.norm(avg)
        if an < 1e-3:
            continue
        avg /= an
        # Signed angle from image vertical axis (image y-axis points down).
        ang = np.degrees(np.arctan2(avg[0], avg[1]))
        # Fold to [-90, 90] — line orientation, not edge direction.
        if ang > 90.0:
            ang -= 180.0
        elif ang < -90.0:
            ang += 180.0
        if best is None or abs(ang) < abs(best[0]):
            best = (float(ang), center, avg)
    return best


def extend_line(center, dir_unit, length):
    """Return endpoints of a segment of `length`, centered on `center`,
    along `dir_unit`."""
    half = (length / 2.0) * dir_unit
    return center - half, center + half


class DebugOverlay(Node):

    def __init__(self):
        super().__init__('debug_overlay')
        self.declare_parameter('sweet_tol_px', 5.0)
        self.declare_parameter('align_tol_deg', 3.0)
        self.tol_px = float(self.get_parameter('sweet_tol_px').value)
        self.tol_deg = float(self.get_parameter('align_tol_deg').value)

        self.bridge = CvBridge()
        self.detector = dt_apriltags.Detector(
            families='tag36h11', nthreads=2, quad_decimate=1.0,
            quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)

        self.create_subscription(Image, RGB_TOPIC, self._cb, 10)
        self.pub = self.create_publisher(Image, OUT_TOPIC, 5)

        self.get_logger().info(
            f'debug_overlay: sweet_px={SWEET_PX}  '
            f'per_tool_tol={SWEET_TOL_PX_PER_TID}  '
            f'default_tol={self.tol_px}px  align_tol={self.tol_deg}deg')

    def _cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        h, w = bgr.shape[:2]
        cx_img, cy_img = w // 2, h // 2

        tags = self.detector.detect(gray, estimate_tag_pose=False)

        # Pick first detected tool tag (priority: phillips, hammer, flathead).
        tool_pick = None
        for prefer in (2, 3, 4):
            for t in tags:
                if int(t.tag_id) == prefer:
                    tool_pick = t
                    break
            if tool_pick is not None:
                break

        # Outline every detected tag. Colour by status if it's the picked tool.
        size_ok = False
        align_ok = False
        edge_px = None
        ang_deg = None
        for t in tags:
            tid = int(t.tag_id)
            pts = np.int32(t.corners).reshape(-1, 2)
            if tool_pick is not None and t is tool_pick:
                e = float(np.mean([
                    np.linalg.norm(t.corners[i] - t.corners[(i + 1) % 4])
                    for i in range(4)]))
                edge_px = e
                sweet = SWEET_PX[tid]
                tol = SWEET_TOL_PX_PER_TID.get(tid, self.tol_px)
                size_ok = abs(e - sweet) <= tol
                ve = vertical_edge(t.corners)
                if ve is not None:
                    ang_deg = ve[0]
                    align_ok = abs(ang_deg) <= self.tol_deg
                col = (0, 255, 0) if (size_ok and align_ok) else (0, 165, 255)
            elif tid == GRIPPER_TAG:
                col = (255, 200, 0)
            elif tid in TOOL_NAMES:
                col = (0, 200, 200)
            else:
                col = (0, 0, 255)
            cv2.polylines(bgr, [pts], True, col, 2)

        # --- ALIGN guide: image vertical reference + tag-vertical line ---
        # Image vertical reference (full height, dashed white).
        for y in range(0, h, 14):
            cv2.line(bgr, (cx_img, y), (cx_img, min(h, y + 7)),
                     (255, 255, 255), 1)

        if tool_pick is not None and ang_deg is not None:
            ve = vertical_edge(tool_pick.corners)
            if ve is not None:
                _, ctr, dir_unit = ve
                a, b = extend_line(ctr, dir_unit, length=max(h, w))
                col = (0, 255, 0) if align_ok else (0, 165, 255)
                cv2.line(bgr,
                         (int(round(a[0])), int(round(a[1]))),
                         (int(round(b[0])), int(round(b[1]))),
                         col, 2)
                tcx = int(tool_pick.center[0])
                tcy = int(tool_pick.center[1])
                cv2.circle(bgr, (tcx, tcy), 4, col, -1)

        # --- HUD readouts (top-left big plate) -------------------------
        cv2.rectangle(bgr, (0, 0), (w, 70), (0, 0, 0), -1)
        if tool_pick is None:
            line1 = 'SIZE : --- px  (no tool tag in view)'
            line2 = 'ALIGN: --- deg'
            c1 = c2 = (0, 0, 255)
        else:
            tid = int(tool_pick.tag_id)
            sweet = SWEET_PX[tid]
            delta = edge_px - sweet
            sign = '+' if delta >= 0 else '-'
            line1 = (f'SIZE : {TOOL_NAMES[tid]}:{tid}  '
                     f'{edge_px:5.1f} px  (sweet {sweet:.1f} '
                     f'{sign}{abs(delta):4.1f} / tol {self.tol_px:.1f})')
            c1 = (0, 255, 0) if size_ok else (0, 165, 255)
            line2 = (f'ALIGN: {ang_deg:+5.1f} deg  '
                     f'(rotate J5 by {-ang_deg:+5.1f} deg / tol {self.tol_deg:.1f})')
            c2 = (0, 255, 0) if align_ok else (0, 165, 255)

        cv2.putText(bgr, line1, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, c1, 2)
        cv2.putText(bgr, line2, (10, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, c2, 2)

        # GO/NO-GO badge (upper-right)
        ok = size_ok and align_ok and tool_pick is not None
        badge = 'IN BAND' if ok else 'WAIT'
        bcol = (0, 255, 0) if ok else (0, 165, 255)
        cv2.rectangle(bgr, (w - 130, 10), (w - 10, 60), (0, 0, 0), -1)
        cv2.putText(bgr, badge, (w - 122, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, bcol, 2)

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
