#!/usr/bin/env python3
"""
apriltag_perception.py
----------------------
ROS2 node that detects AprilTags via webcam, transforms their poses from
camera frame to robot base frame, and publishes:
  - /tool_pose (PoseStamped) — grip pose for detected tools
  - /pickup_zone (PoseStamped) — centroid of pickup cart
  - /dropoff_zone (PoseStamped) — centroid of dropoff cart

Auto-calibrates using 9 known tag positions (gripper + 8 zone corners)
via Procrustes alignment (SVD-based least squares).

Tag assignments (tag36h11):
  Pickup zone corners: 10, 12, 16, 24
  Drop-off zone corners: 15, 17, 18, 19
  Hammer: 3
  Phillips screwdriver: 2
  Flathead screwdriver: 4
  Gripper: 23
"""

import cv2
import numpy as np
import dt_apriltags

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


# Tag role assignments
PICKUP_ZONE_TAGS = {10, 12, 16, 24}
DROPOFF_ZONE_TAGS = {15, 17, 18, 19}
ZONE_TAGS = PICKUP_ZONE_TAGS | DROPOFF_ZONE_TAGS
TOOL_TAGS = {2: 'phillips_screwdriver', 3: 'hammer', 4: 'flathead_screwdriver'}
GRIPPER_TAG = 23

# Camera intrinsics (from Matthew's calibration — same N5 webcam)
FX = 969.20915798
FY = 966.76406593
CX = 588.80142754
CY = 283.34318455

TAG_SIZE = 0.0508  # 2 inches in meters

# All measurements in link_base frame (meters)
# Julian's measurements converted: 1 inch = 0.0254 m
CART_SURFACE_Z = -4.35 * 0.0254   # -0.11049 m

# Known tag positions in link_base frame (from Julian's physical measurements)
KNOWN_TAG_POSITIONS = {
    # Gripper tag at home pose: 9.5in forward, centered, 3.5in above base
    23: np.array([9.5 * 0.0254, 0.0, 3.5 * 0.0254]),

    # Pickup zone (LEFT of robot from behind, +y)
    # Tags 12, 24: x = 26in from base
    # Tags 10, 16: x = 41in from base
    # Tags 12, 10: y = 4in to the left (+y)
    # Tags 24, 16: y = 18in to the left (+y)
    12: np.array([26.0 * 0.0254, 4.0 * 0.0254, CART_SURFACE_Z]),
    24: np.array([26.0 * 0.0254, 18.0 * 0.0254, CART_SURFACE_Z]),
    10: np.array([41.0 * 0.0254, 4.0 * 0.0254, CART_SURFACE_Z]),
    16: np.array([41.0 * 0.0254, 18.0 * 0.0254, CART_SURFACE_Z]),

    # Drop-off zone (RIGHT of robot from behind, -y)
    # Tags 15, 17: x = 26in from base
    # Tags 18, 19: x = 41in from base
    # Tags 15, 18: y = 4in to the right (-y)
    # Tags 17, 19: y = 18in to the right (-y)
    15: np.array([26.0 * 0.0254, -4.0 * 0.0254, CART_SURFACE_Z]),
    17: np.array([26.0 * 0.0254, -18.0 * 0.0254, CART_SURFACE_Z]),
    18: np.array([41.0 * 0.0254, -4.0 * 0.0254, CART_SURFACE_Z]),
    19: np.array([41.0 * 0.0254, -18.0 * 0.0254, CART_SURFACE_Z]),
}

# Tool tags sit slightly above cart surface
TOOL_Z = CART_SURFACE_Z + 0.02  # ~2cm above cart


class AprilTagPerception(Node):

    def __init__(self):
        super().__init__('apriltag_perception')

        self.bridge = CvBridge()

        # AprilTag detector
        self.detector = dt_apriltags.Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0)

        # Transform: p_base = scale * R @ p_cam_corrected + t
        self.R = np.eye(3)
        self.t = np.zeros(3)
        self.scale = 1.0

        # Calibration state
        self.calibrated = False
        self.calib_samples = {}  # tag_id -> list of [x, y, z] cam coords
        self.calib_frame_count = 0
        self.calib_frames_needed = 20

        # Publishers
        self.tool_pose_pub = self.create_publisher(PoseStamped, '/tool_pose', 10)
        self.pickup_zone_pub = self.create_publisher(PoseStamped, '/pickup_zone', 10)
        self.dropoff_zone_pub = self.create_publisher(PoseStamped, '/dropoff_zone', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        self.show_preview = True

        self.get_logger().info(
            f'AprilTag perception ready. Will calibrate from {len(KNOWN_TAG_POSITIONS)} '
            f'known tag positions using Procrustes alignment.')

    def correct_cam(self, pose_t):
        """Apply y-flip to raw camera coordinates (image is vertically flipped)."""
        p = np.array(pose_t).flatten()
        p[1] = -p[1]
        return p

    def cam_to_base(self, pose_t):
        """Transform camera coordinates to link_base frame."""
        p = self.correct_cam(pose_t)
        return self.scale * (self.R @ p) + self.t

    def calibrate(self):
        """Procrustes alignment using known tag positions."""
        # Average camera samples for each known tag
        cam_points = []
        base_points = []
        tag_ids_used = []

        for tag_id, known_pos in KNOWN_TAG_POSITIONS.items():
            if tag_id in self.calib_samples and len(self.calib_samples[tag_id]) >= 3:
                avg_cam = np.mean(self.calib_samples[tag_id], axis=0)
                avg_cam[1] = -avg_cam[1]  # flip y
                cam_points.append(avg_cam)
                base_points.append(known_pos)
                tag_ids_used.append(tag_id)

        n = len(cam_points)
        if n < 4:
            self.get_logger().error(f'Only {n} known tags detected, need at least 4')
            return

        cam_pts = np.array(cam_points)   # N x 3
        base_pts = np.array(base_points) # N x 3

        self.get_logger().info(f'Calibrating with {n} known tags: {tag_ids_used}')

        # Procrustes: find scale, R, t such that base = scale * R @ cam + t
        # Step 1: Center both point sets
        cam_mean = cam_pts.mean(axis=0)
        base_mean = base_pts.mean(axis=0)
        cam_centered = cam_pts - cam_mean
        base_centered = base_pts - base_mean

        # Step 2: Compute scale
        cam_norm = np.sqrt(np.sum(cam_centered ** 2))
        base_norm = np.sqrt(np.sum(base_centered ** 2))
        self.scale = base_norm / cam_norm

        # Step 3: SVD for optimal rotation
        H = cam_centered.T @ base_centered  # 3x3
        U, S, Vt = np.linalg.svd(H)
        # Ensure proper rotation (det = +1)
        d = np.linalg.det(Vt.T @ U.T)
        sign_matrix = np.diag([1, 1, d])
        self.R = (Vt.T @ sign_matrix @ U.T)

        # Step 4: Compute translation
        self.t = base_mean - self.scale * (self.R @ cam_mean)

        # Verify all known tags
        self.get_logger().info('CALIBRATED — verification:')
        total_err = 0.0
        for i, tag_id in enumerate(tag_ids_used):
            p_computed = self.scale * (self.R @ cam_pts[i]) + self.t
            p_known = base_pts[i]
            err = np.linalg.norm(p_computed - p_known)
            total_err += err
            label = "GRIP" if tag_id == GRIPPER_TAG else \
                    "PICK" if tag_id in PICKUP_ZONE_TAGS else "DROP"
            self.get_logger().info(
                f'  {label}:{tag_id} computed=({p_computed[0]:.3f},{p_computed[1]:.3f},{p_computed[2]:.3f}) '
                f'known=({p_known[0]:.3f},{p_known[1]:.3f},{p_known[2]:.3f}) err={err:.4f}m')

        avg_err = total_err / n
        self.get_logger().info(
            f'  scale={self.scale:.4f}, avg_error={avg_err:.4f}m')

        self.calibrated = True

    def get_z_for_tag(self, tag_id):
        """Return the known z height for a tag if available."""
        if tag_id in KNOWN_TAG_POSITIONS:
            return KNOWN_TAG_POSITIONS[tag_id][2]
        elif tag_id in TOOL_TAGS:
            return TOOL_Z
        return 0.0

    def compute_zone_centroid(self, positions):
        if len(positions) < 2:
            return None
        return np.mean(positions, axis=0)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[FX, FY, CX, CY],
            tag_size=TAG_SIZE)

        # --- Calibration phase ---
        if not self.calibrated:
            has_known = False
            for tag in tags:
                tid = tag.tag_id
                if tid in KNOWN_TAG_POSITIONS:
                    if tid not in self.calib_samples:
                        self.calib_samples[tid] = []
                    self.calib_samples[tid].append(np.array(tag.pose_t).flatten())
                    has_known = True

            if has_known:
                self.calib_frame_count += 1

            n_detected = len(self.calib_samples)
            self.get_logger().info(
                f'Calibration {self.calib_frame_count}/{self.calib_frames_needed} '
                f'({n_detected}/{len(KNOWN_TAG_POSITIONS)} known tags seen)',
                throttle_duration_sec=0.5)

            if self.calib_frame_count >= self.calib_frames_needed:
                self.calibrate()

            if self.show_preview:
                for tag in tags:
                    corners = tag.corners.astype(int)
                    for j in range(4):
                        cv2.line(frame, tuple(corners[j]), tuple(corners[(j+1)%4]), (0,255,0), 2)
                    cx, cy = int(tag.center[0]), int(tag.center[1])
                    cv2.putText(frame, f"ID:{tag.tag_id}", (cx-20, cy-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                cv2.putText(frame,
                            f"CALIBRATING {self.calib_frame_count}/{self.calib_frames_needed} "
                            f"({len(self.calib_samples)}/{len(KNOWN_TAG_POSITIONS)} tags)",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                cv2.imshow("AprilTag Perception", frame)
                cv2.waitKey(1)
            return

        # --- Normal operation ---
        pickup_positions = []
        dropoff_positions = []
        tool_detections = []

        for tag in tags:
            tag_id = tag.tag_id
            p_base = self.cam_to_base(tag.pose_t)

            # For known tags and tools, override z with measured height
            if tag_id in KNOWN_TAG_POSITIONS:
                p_base[2] = KNOWN_TAG_POSITIONS[tag_id][2]
            elif tag_id in TOOL_TAGS:
                p_base[2] = TOOL_Z

            # Draw on preview
            if self.show_preview:
                corners = tag.corners.astype(int)
                for j in range(4):
                    cv2.line(frame, tuple(corners[j]), tuple(corners[(j+1)%4]), (0,255,0), 2)

                cx, cy = int(tag.center[0]), int(tag.center[1])

                if tag_id == GRIPPER_TAG:
                    label = f"GRIP:{tag_id}"
                    color = (0, 255, 255)
                elif tag_id in PICKUP_ZONE_TAGS:
                    label = f"PICK:{tag_id}"
                    color = (255, 165, 0)
                elif tag_id in DROPOFF_ZONE_TAGS:
                    label = f"DROP:{tag_id}"
                    color = (255, 0, 255)
                elif tag_id in TOOL_TAGS:
                    label = f"{TOOL_TAGS[tag_id]}:{tag_id}"
                    color = (0, 0, 255)
                else:
                    label = f"ID:{tag_id}"
                    color = (128, 128, 128)

                cv2.putText(frame, label, (cx-30, cy-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                coord_label = f"({p_base[0]:.2f},{p_base[1]:.2f},{p_base[2]:.2f})"
                cv2.putText(frame, coord_label, (cx-40, cy+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # Categorize
            if tag_id == GRIPPER_TAG:
                self.get_logger().info(
                    f'Gripper tag {tag_id}: ({p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f})',
                    throttle_duration_sec=5.0)
            elif tag_id in PICKUP_ZONE_TAGS:
                pickup_positions.append(p_base)
            elif tag_id in DROPOFF_ZONE_TAGS:
                dropoff_positions.append(p_base)
            elif tag_id in TOOL_TAGS:
                tool_detections.append((tag_id, p_base))
                self.get_logger().info(
                    f'Tool {TOOL_TAGS[tag_id]}: ({p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f})',
                    throttle_duration_sec=2.0)

        # Publish zone centroids
        if len(pickup_positions) >= 2:
            centroid = self.compute_zone_centroid(pickup_positions)
            if centroid is not None:
                self.publish_pose(self.pickup_zone_pub, centroid)

        if len(dropoff_positions) >= 2:
            centroid = self.compute_zone_centroid(dropoff_positions)
            if centroid is not None:
                self.publish_pose(self.dropoff_zone_pub, centroid)

        # Publish tool poses
        for tag_id, p_base in tool_detections:
            self.publish_pose(self.tool_pose_pub, p_base)

        # Show preview
        if self.show_preview:
            cv2.putText(frame, "CALIBRATED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("AprilTag Perception", frame)
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
