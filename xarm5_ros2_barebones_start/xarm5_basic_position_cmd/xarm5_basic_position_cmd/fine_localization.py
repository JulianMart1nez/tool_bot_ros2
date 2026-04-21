#!/usr/bin/env python3
"""
fine_localization.py
--------------------
Continuously detects AprilTags in the gripper-mounted RealSense RGB stream
and publishes each detected tool tag's full 6-DOF pose in the robot's
`link_base` frame. Orientation comes from dt_apriltags' solvePnP, so the
tag's local +Z axis transforms to the surface-normal direction in
link_base — downstream consumers (tool_approach) use that to align the
gripper perpendicular to the tag and trace down along it.

Subscribes:
  color_image  (sensor_msgs/Image)       — RGB from gripper camera
  color_info   (sensor_msgs/CameraInfo)  — intrinsics

Publishes (while any tool tag is in view):
  /fine_loc/result        (geometry_msgs/PoseStamped)
      closest tool tag's full 6-DOF pose in link_base
  /fine_loc/tag_<id>      (geometry_msgs/PoseStamped)
      per-tag pose in link_base (ids 2/3/4)
  /fine_loc/tag_detected  (std_msgs/Bool)
      true while any tool tag is visible
"""

import numpy as np
import dt_apriltags
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge

import tf2_ros
from tf2_geometry_msgs import do_transform_pose


TOOL_TAGS = {2: 'phillips_screwdriver', 3: 'hammer', 4: 'flathead_screwdriver'}
TAG_SIZE = 0.0254  # 25.4 mm (1 inch) physical tag
TARGET_FRAME = 'link_base'


def _rot_matrix_to_quat(R):
    """3x3 rotation matrix -> [x, y, z, w] quaternion (Shepperd's method)."""
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        return np.array([
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
            0.25 * s,
        ])
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return np.array([
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[2, 1] - R[1, 2]) / s,
        ])
    if R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return np.array([
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
            (R[0, 2] - R[2, 0]) / s,
        ])
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return np.array([
        (R[0, 2] + R[2, 0]) / s,
        (R[1, 2] + R[2, 1]) / s,
        0.25 * s,
        (R[1, 0] - R[0, 1]) / s,
    ])


class FineLocalization(Node):

    def __init__(self):
        super().__init__('fine_localization')

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.latest_image = None

        self.detector = dt_apriltags.Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0)

        self.declare_parameter('detect_rate_hz', 10.0)
        self.declare_parameter('log_every_n', 10)
        # When true, subscribes to an aligned depth image and publishes a
        # per-tag depth reading (m) sampled at the tag center.
        self.declare_parameter('enable_depth', False)
        self.declare_parameter('depth_sample_radius_px', 3)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_result = self.create_publisher(PoseStamped, '/fine_loc/result', 10)
        self.pub_detected = self.create_publisher(Bool, '/fine_loc/tag_detected', 10)
        self.pub_per_tag = {
            tag_id: self.create_publisher(PoseStamped, f'/fine_loc/tag_{tag_id}', 10)
            for tag_id in TOOL_TAGS
        }

        self.latest_depth = None
        self._depth_enabled = bool(self.get_parameter('enable_depth').value)
        if self._depth_enabled:
            self.create_subscription(Image, 'aligned_depth', self._depth_cb, 10)
            self.pub_per_tag_depth = {
                tag_id: self.create_publisher(
                    Float32, f'/fine_loc/tag_{tag_id}/depth', 10)
                for tag_id in TOOL_TAGS
            }
            self.get_logger().info(
                'Depth sampling enabled — subscribing to `aligned_depth` and '
                'publishing /fine_loc/tag_<id>/depth (m).')
        else:
            self.pub_per_tag_depth = {}

        # Tag pixel size (mean edge length in px) — the primary distance
        # signal for the initial approach, before we close in enough for
        # the D435i depth reading to be the authority. Always published.
        self.pub_per_tag_pixel = {
            tag_id: self.create_publisher(
                Float32, f'/fine_loc/tag_{tag_id}/pixel_size', 10)
            for tag_id in TOOL_TAGS
        }

        self.create_subscription(Image, 'color_image', self._image_cb, 10)
        self.create_subscription(CameraInfo, 'color_info', self._info_cb, 10)

        rate = self.get_parameter('detect_rate_hz').value
        self.create_timer(1.0 / rate, self._tick)

        self._tick_count = 0
        self._last_detected = False

        self.get_logger().info(
            f'Fine localization running at {rate:.1f} Hz. '
            f'Publishing 6-DOF tag pose in {TARGET_FRAME} on '
            f'/fine_loc/result and /fine_loc/tag_<id>.')

    def _info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _image_cb(self, msg):
        self.latest_image = msg

    def _depth_cb(self, msg):
        self.latest_depth = msg

    def _sample_tag_depth_m(self, tag):
        """Median depth (meters) in a small ROI around the tag center, or NaN."""
        if self.latest_depth is None:
            return float('nan')
        try:
            depth = self.bridge.imgmsg_to_cv2(
                self.latest_depth, desired_encoding='passthrough')
        except Exception:
            return float('nan')
        u = int(round(float(tag.center[0])))
        v = int(round(float(tag.center[1])))
        h, w = depth.shape[:2]
        r = int(self.get_parameter('depth_sample_radius_px').value)
        y0, y1 = max(0, v - r), min(h, v + r + 1)
        x0, x1 = max(0, u - r), min(w, u + r + 1)
        roi = depth[y0:y1, x0:x1]
        if roi.size == 0:
            return float('nan')
        valid = roi[roi > 0]
        if valid.size == 0:
            # Everything is zero — usually "below D435i min range" or occluded.
            return float('nan')
        if depth.dtype == np.uint16:
            return float(np.median(valid)) / 1000.0  # 16UC1 encoded in mm
        return float(np.median(valid))  # 32FC1 already in meters

    def _detect_tags(self):
        if self.latest_image is None or self.fx is None:
            return None, []
        frame = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=TAG_SIZE)
        tool_tags = [t for t in tags if t.tag_id in TOOL_TAGS]
        return self.latest_image.header, tool_tags

    def _tag_pose_in_cam(self, tag):
        pose = Pose()
        t = np.array(tag.pose_t).flatten()
        pose.position.x = float(t[0])
        pose.position.y = float(t[1])
        pose.position.z = float(t[2])
        q = _rot_matrix_to_quat(np.array(tag.pose_R))
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        return pose

    def _tick(self):
        self._tick_count += 1
        log_n = self.get_parameter('log_every_n').value

        header, tool_tags = self._detect_tags()

        detected_msg = Bool()
        if not tool_tags:
            detected_msg.data = False
            self.pub_detected.publish(detected_msg)
            if self._last_detected:
                self.get_logger().info('No tool tag in view.')
            self._last_detected = False
            return

        # Prefer the image's own timestamp so the transform matches the
        # arm pose when the frame was captured. During continuous
        # tracking the arm can move meaningfully between capture and
        # TF lookup, and using rclpy.time.Time() (= latest) produces
        # poses that are inconsistent with the image.
        tf = None
        try:
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME, header.frame_id, header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e_stamped:
            # Fall back to latest if the buffer doesn't have that exact
            # timestamp yet (rare, usually on first frames or after
            # clock hiccups). Log both failures the first time.
            try:
                tf = self.tf_buffer.lookup_transform(
                    TARGET_FRAME, header.frame_id, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                self.get_logger().warn(
                    f'TF at image stamp unavailable ({e_stamped}); '
                    f'used latest instead.',
                    throttle_duration_sec=5.0)
            except Exception as e_latest:
                self.get_logger().warn(
                    f'TF lookup {TARGET_FRAME} <- {header.frame_id} failed: '
                    f'{e_latest}',
                    throttle_duration_sec=2.0)
                detected_msg.data = False
                self.pub_detected.publish(detected_msg)
                return

        detected_msg.data = True
        self.pub_detected.publish(detected_msg)

        stamp = self.get_clock().now().to_msg()
        closest_tag = None
        closest_pose_base = None
        closest_cam_z = float('inf')
        closest_depth_m = float('nan')
        closest_pixel_size = float('nan')

        for tag in tool_tags:
            pose_cam = self._tag_pose_in_cam(tag)
            pose_base = do_transform_pose(pose_cam, tf)

            ps = PoseStamped()
            ps.header.frame_id = TARGET_FRAME
            ps.header.stamp = stamp
            ps.pose = pose_base
            self.pub_per_tag[tag.tag_id].publish(ps)

            # Tag pixel size: mean of the four edge lengths in pixels.
            corners = np.asarray(tag.corners, dtype=np.float64)
            edges = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)
            pixel_size = float(np.mean(edges))
            self.pub_per_tag_pixel[tag.tag_id].publish(Float32(data=pixel_size))

            depth_m = float('nan')
            if self._depth_enabled:
                depth_m = self._sample_tag_depth_m(tag)
                if tag.tag_id in self.pub_per_tag_depth and not np.isnan(depth_m):
                    self.pub_per_tag_depth[tag.tag_id].publish(Float32(data=depth_m))

            cam_z = float(np.array(tag.pose_t).flatten()[2])
            if cam_z < closest_cam_z:
                closest_cam_z = cam_z
                closest_tag = tag
                closest_pose_base = pose_base
                closest_depth_m = depth_m
                closest_pixel_size = pixel_size

        result = PoseStamped()
        result.header.frame_id = TARGET_FRAME
        result.header.stamp = stamp
        result.pose = closest_pose_base
        self.pub_result.publish(result)

        if not self._last_detected or (self._tick_count % log_n == 0):
            p = closest_pose_base.position
            q = closest_pose_base.orientation
            # Compute the tag's +Z axis in link_base for a quick sanity check.
            # Rotate [0,0,1] by the pose quaternion.
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            nx = 2.0 * (qx * qz + qw * qy)
            ny = 2.0 * (qy * qz - qw * qx)
            nz = 1.0 - 2.0 * (qx * qx + qy * qy)
            depth_str = (
                f'depth={closest_depth_m*1000:.0f}mm'
                if self._depth_enabled and not np.isnan(closest_depth_m)
                else ('depth=n/a' if self._depth_enabled else 'depth=disabled'))
            # Raw optical-frame translation from solvePnP. Useful for
            # sanity-checking the TF chain: if optical (x,y) is near 0
            # and z>0, the camera is directly above the tag; if the
            # transformed link_base (x,y) is then wildly off from the
            # current TCP (x,y), the static TF link_tcp→camera_link is
            # suspect. See pose_check (ros2 run ... pose_check).
            t_opt = np.array(closest_tag.pose_t).flatten()
            self.get_logger().info(
                f'{TOOL_TAGS[closest_tag.tag_id]} (id={closest_tag.tag_id})  '
                f'optical(x,y,z)=({t_opt[0]*1000:+.1f},'
                f'{t_opt[1]*1000:+.1f},{t_opt[2]*1000:+.1f})mm  '
                f'→ link_base pos=({p.x*1000:+.1f},{p.y*1000:+.1f},'
                f'{p.z*1000:+.1f})mm  '
                f'normal=({nx:+.2f},{ny:+.2f},{nz:+.2f})  '
                f'pixel={closest_pixel_size:.0f}px  {depth_str}')
        self._last_detected = True


def main(args=None):
    rclpy.init(args=args)
    node = FineLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
