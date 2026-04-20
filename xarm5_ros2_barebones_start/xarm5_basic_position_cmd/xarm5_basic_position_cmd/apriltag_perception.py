#!/usr/bin/env python3
"""
apriltag_perception.py
----------------------
ROS2 node for overhead scene camera perception. Detects:
  - Black squares on white paper (zone corner markers)
  - AprilTags (gripper tag 22, tool tags 2/3/4)

Publishes:
  - /pickup_zone  (PoseStamped) — centroid of pickup zone in link_base
  - /dropoff_zone (PoseStamped) — centroid of dropoff zone in link_base
  - /tool_pose    (PoseStamped) — tool position (if visible to scene camera)

Zone corners are identified by detecting black squares, grouping them by
proximity, and distinguishing pickup from dropoff by bounding rectangle
aspect ratio (pickup is nearly square, dropoff is more rectangular).

The scene camera provides coarse zone localization only — the gripper-mounted
Intel RealSense D435i handles fine tool localization via AprilTags.

Tag assignments (tag36h11):
  Gripper: 22
  Hammer: 3
  Phillips screwdriver: 2
  Flathead screwdriver: 4
"""

import cv2
import numpy as np
import dt_apriltags

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


# AprilTag assignments (zone corners are now black squares, not tags)
TOOL_TAGS = {2: 'phillips_screwdriver', 3: 'hammer', 4: 'flathead_screwdriver'}
GRIPPER_TAG = 22

# Camera intrinsics (from Matthew's calibration — same N5 webcam)
FX = 969.20915798
FY = 966.76406593
CX = 588.80142754
CY = 283.34318455

TAG_SIZE = 0.0508  # 2 inches in meters (tool + gripper tags)

# All measurements in link_base frame (meters)
# 1 inch = 0.0254 m
CART_SURFACE_Z = -4.35 * 0.0254   # -0.11049 m — TODO: user will re-measure

# Zone dimensions (inches): x = depth from robot, y = lateral width
PICKUP_DIMS_IN = (15.5, 14.5)   # (x_depth, y_width)
DROPOFF_DIMS_IN = (15.5, 9.0)   # (x_depth, y_width)

# Known zone centroid positions in link_base frame
# TODO: Update near_edge_x and inner_edge_y once user measures exact distances.
# Placeholder offsets based on old AprilTag positions.
_NEAR_EDGE_X = 26.0   # inches from base to near edge of zones — PLACEHOLDER
_INNER_EDGE_Y = 4.0    # inches from centerline to inner edge — PLACEHOLDER

# Pickup zone: LEFT of robot (+y), appears RIGHT in camera frame
PICKUP_CENTROID_BASE = np.array([
    (_NEAR_EDGE_X + PICKUP_DIMS_IN[0] / 2) * 0.0254,
    (_INNER_EDGE_Y + PICKUP_DIMS_IN[1] / 2) * 0.0254,
    CART_SURFACE_Z
])

# Dropoff zone: RIGHT of robot (-y), appears LEFT in camera frame
DROPOFF_CENTROID_BASE = np.array([
    (_NEAR_EDGE_X + DROPOFF_DIMS_IN[0] / 2) * 0.0254,
    -(_INNER_EDGE_Y + DROPOFF_DIMS_IN[1] / 2) * 0.0254,
    CART_SURFACE_Z
])

# Gripper tag known position (for display/verification)
GRIPPER_HOME_POS = np.array([9.5 * 0.0254, 0.0, 3.5 * 0.0254])

# Tool tags sit slightly above cart surface
TOOL_Z = CART_SURFACE_Z + 0.02  # ~2cm above cart


def detect_black_squares(gray, min_area=400, max_area=80000):
    """Detect black squares on white paper in a grayscale image.

    Returns a list of dicts with keys: center, contour, area, corners.
    """
    binary = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 51, 15)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    squares = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area > max_area:
            continue

        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

        if len(approx) != 4:
            continue

        x, y, w, h = cv2.boundingRect(approx)
        aspect = float(w) / h if h > 0 else 0
        if aspect < 0.5 or aspect > 2.0:
            continue

        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area > 0 else 0
        if solidity < 0.80:
            continue

        # Verify interior is dark
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [approx], -1, 255, -1)
        mean_val = cv2.mean(gray, mask=mask)[0]
        if mean_val > 100:
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
    """Group detected squares into up to two zones using k-means clustering."""
    if len(squares) < 2:
        return []

    centers = np.array([s['center'] for s in squares], dtype=np.float32)

    if len(squares) < 6:
        # Likely only one zone visible
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
        idx = np.where(labels == cluster_id)[0]
        if len(idx) < 2:
            continue
        cluster_sq = [squares[i] for i in idx]
        cluster_centers = centers[idx]
        centroid = np.mean(cluster_centers, axis=0).astype(int)
        w = cluster_centers[:, 0].max() - cluster_centers[:, 0].min()
        h = cluster_centers[:, 1].max() - cluster_centers[:, 1].min()
        zones.append({
            'squares': cluster_sq,
            'centroid': tuple(centroid),
            'bbox_dims': (w, h),
        })

    return zones


def identify_zones(zones):
    """Label zones as PICKUP or DROPOFF.

    Both zones share 15.5in depth (x). The y-width differs:
      Pickup:  14.5in wide → ratio ~0.94 (nearly square)
      Dropoff:  9.0in wide → ratio ~0.58 (rectangular)

    In camera frame: pickup is RIGHT, dropoff is LEFT.
    Primary cue: wider pixel spread = pickup (14.5 vs 9 lateral width).
    Tiebreaker: horizontal position in camera frame.
    """
    if len(zones) == 0:
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
        # Tiebreak: pickup is RIGHT in camera frame (higher x pixel)
        if zones[0]['centroid'][0] >= zones[1]['centroid'][0]:
            zones[0]['label'] = 'PICKUP'
            zones[1]['label'] = 'DROPOFF'
        else:
            zones[0]['label'] = 'DROPOFF'
            zones[1]['label'] = 'PICKUP'

    return zones


class AprilTagPerception(Node):

    def __init__(self):
        super().__init__('apriltag_perception')

        self.bridge = CvBridge()

        # AprilTag detector (for gripper + tool tags only)
        self.detector = dt_apriltags.Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0)

        # Publishers
        self.tool_pose_pub = self.create_publisher(PoseStamped, '/tool_pose', 10)
        self.pickup_zone_pub = self.create_publisher(PoseStamped, '/pickup_zone', 10)
        self.dropoff_zone_pub = self.create_publisher(PoseStamped, '/dropoff_zone', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        self.show_preview = True

        self.get_logger().info(
            'Scene camera perception ready. '
            'Detecting black square zone corners + AprilTags (gripper/tools).')

    def compute_zone_centroid(self, positions):
        if len(positions) < 2:
            return None
        return np.mean(positions, axis=0)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- AprilTag detection (gripper + tools) ---
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[FX, FY, CX, CY],
            tag_size=TAG_SIZE)

        for tag in tags:
            tag_id = tag.tag_id

            if self.show_preview:
                corners = tag.corners.astype(int)
                for j in range(4):
                    cv2.line(frame, tuple(corners[j]), tuple(corners[(j+1) % 4]),
                             (0, 255, 255), 2)
                cx, cy = int(tag.center[0]), int(tag.center[1])

                if tag_id == GRIPPER_TAG:
                    label = f"GRIP:{tag_id}"
                    color = (0, 255, 255)
                elif tag_id in TOOL_TAGS:
                    label = f"{TOOL_TAGS[tag_id]}:{tag_id}"
                    color = (0, 0, 255)
                else:
                    label = f"ID:{tag_id}"
                    color = (255, 255, 255)

                cv2.putText(frame, label, (cx - 30, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            if tag_id == GRIPPER_TAG:
                self.get_logger().info(
                    f'Gripper tag {tag_id} visible',
                    throttle_duration_sec=5.0)
            elif tag_id in TOOL_TAGS:
                self.get_logger().info(
                    f'Tool {TOOL_TAGS[tag_id]} (tag {tag_id}) visible in scene camera',
                    throttle_duration_sec=2.0)

        # --- Black square detection (zone corners) ---
        squares = detect_black_squares(gray)
        zones = group_squares_into_zones(squares)
        zones = identify_zones(zones)

        # Draw detected squares
        if self.show_preview:
            for sq in squares:
                cv2.drawContours(frame, [sq['contour']], -1, (255, 255, 0), 2)
                cx, cy = sq['center']
                cv2.circle(frame, (cx, cy), 3, (255, 255, 0), -1)

        # Process and publish zones
        for zone in zones:
            label = zone.get('label', '???')

            if label == 'PICKUP':
                color = (0, 200, 0)
                self.publish_pose(self.pickup_zone_pub, PICKUP_CENTROID_BASE)
                self.get_logger().info(
                    f'Pickup zone detected ({len(zone["squares"])} corners), '
                    f'publishing centroid ({PICKUP_CENTROID_BASE[0]:.3f}, '
                    f'{PICKUP_CENTROID_BASE[1]:.3f}, {PICKUP_CENTROID_BASE[2]:.3f})',
                    throttle_duration_sec=2.0)
            elif label == 'DROPOFF':
                color = (0, 165, 255)
                self.publish_pose(self.dropoff_zone_pub, DROPOFF_CENTROID_BASE)
                self.get_logger().info(
                    f'Dropoff zone detected ({len(zone["squares"])} corners), '
                    f'publishing centroid ({DROPOFF_CENTROID_BASE[0]:.3f}, '
                    f'{DROPOFF_CENTROID_BASE[1]:.3f}, {DROPOFF_CENTROID_BASE[2]:.3f})',
                    throttle_duration_sec=2.0)
            else:
                color = (128, 128, 128)

            # Draw zone overlay on preview
            if self.show_preview:
                cx, cy = zone['centroid']
                all_pts = np.vstack([s['corners'] for s in zone['squares']])
                hull = cv2.convexHull(all_pts)
                cv2.polylines(frame, [hull], True, color, 3)
                cv2.putText(frame, f"{label} ZONE", (cx - 60, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.putText(frame, f"({len(zone['squares'])} corners)",
                            (cx - 50, cy + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # HUD
        if self.show_preview:
            status = f"Tags:{len(tags)} Squares:{len(squares)} Zones:{len(zones)}"
            cv2.putText(frame, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("Scene Camera Perception", frame)
            cv2.waitKey(1)

    def publish_pose(self, publisher, position):
        msg = PoseStamped()
        msg.header.frame_id = 'link_base'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = 0.0
        msg.pose.orientation.x = 1.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
