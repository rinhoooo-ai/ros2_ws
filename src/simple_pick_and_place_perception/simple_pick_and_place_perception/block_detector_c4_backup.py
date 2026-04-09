#!/usr/bin/env python3
"""
Block Detector — Challenge 4
- Detect block màu (red/yellow/blue/green) từ wrist camera
- Filter bin ra khỏi block bằng:
  1. Depth filter: block gần camera hơn bin (bin có thành cao)
  2. Area filter: block nhỏ hơn bin
  3. Aspect ratio filter: block gần vuông, bin có thể dài/rộng hơn
- Publish PoseStamped lên /block_pose, frame_id = màu block ("red", "blue", ...)
- Publish debug image lên /block_detector/debug_image
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401

# ─── HSV Color Ranges ──────────────────────────────────────────────────────────
# Format: {color: [(lo, hi), ...]}  — hỗ trợ nhiều range cho red (wrap around 180)
COLOR_RANGES = {
    'red': [
        (np.array([0,   140, 100]), np.array([10,  255, 255])),
        (np.array([170, 140, 100]), np.array([179, 255, 255])),
    ],
    'yellow': [
        (np.array([20, 120, 100]), np.array([35, 255, 255])),
    ],
    'blue': [
        (np.array([100, 120,  80]), np.array([130, 255, 255])),
    ],
    'green': [
        (np.array([40,  80,  80]), np.array([85,  255, 255])),
    ],
}

# Debug colors (BGR) cho từng màu block
DEBUG_COLORS = {
    'red':    (0, 0, 255),
    'yellow': (0, 255, 255),
    'blue':   (255, 100, 0),
    'green':  (0, 255, 0),
}

# ─── Detection Parameters ──────────────────────────────────────────────────────
MIN_CONTOUR_AREA  = 300      # bỏ noise nhỏ
MAX_CONTOUR_AREA  = 15000    # bỏ bin lớn (bin >> block)
MIN_DEPTH_M       = 0.10     # bỏ artifact quá gần
MAX_DEPTH_M       = 0.90     # bỏ object quá xa (bin thường xa hơn block khi arm ở home)
MAX_ASPECT_RATIO  = 3.0      # bỏ contour dài bất thường (edge của bin)

# Target: detect tất cả màu (để pick đúng bin), hoặc chỉ 1 màu cụ thể
# None = auto-detect màu nào thấy trước
# 'red' / 'yellow' / 'blue' / 'green' = chỉ detect màu đó
TARGET_COLOR = None

# ─── Bin exclusion zones (world frame) ─────────────────────────────────────────
# Skip contours whose transformed world position lands near the 4 bin centers.
# Tune BIN_EXCLUSION_RADIUS if needed.
BIN_POSITIONS = {
    'red':    {'x':  0.55, 'y': -0.45, 'z': 0.05},
    'yellow': {'x': -0.55, 'y': -0.45, 'z': 0.05},
    'blue':   {'x': -0.55, 'y':  0.45, 'z': 0.05},
    'green':  {'x':  0.55, 'y':  0.45, 'z': 0.05},
}
BIN_EXCLUSION_RADIUS = 0.18   # meters
BIN_EXCLUSION_Z_TOL   = 0.25   # only skip objects near the table surface


class BlockDetector(Node):
    def __init__(self):
        super().__init__('block_detector')
        self.bridge = CvBridge()

        # Camera intrinsics (cập nhật từ /camera_info)
        self.fx, self.fy = 554.38, 554.38
        self.cx, self.cy = 320.0,  240.0
        self.camera_frame = 'fr3_cam_optical_frame'

        self.latest_rgb   = None
        self.latest_depth = None

        # TF2
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.sub_info = self.create_subscription(
            CameraInfo, '/fr3_cam/camera_info', self.camera_info_cb, 1)
        self.create_subscription(
            Image, '/fr3_cam/image',       self.rgb_cb,   10)
        self.create_subscription(
            Image, '/fr3_cam/depth_image', self.depth_cb, 10)

        # Publishers
        self.pub_pose  = self.create_publisher(PoseStamped, '/block_pose', 10)
        self.pub_debug = self.create_publisher(
            Image, '/block_detector/debug_image', 10)

        self.create_timer(0.1, self.detect)
        target_str = TARGET_COLOR if TARGET_COLOR else 'AUTO (all colors)'
        self.get_logger().info(f'BlockDetector started — target: {target_str}')

    # ── Callbacks ──────────────────────────────────────────────────────────────
    def camera_info_cb(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.camera_frame = msg.header.frame_id
        self.destroy_subscription(self.sub_info)
        self.get_logger().info(
            f'Camera intrinsics: fx={self.fx:.1f} cx={self.cx:.1f} '
            f'cy={self.cy:.1f} frame={self.camera_frame}')

    def rgb_cb(self,   msg: Image): self.latest_rgb   = msg
    def depth_cb(self, msg: Image): self.latest_depth = msg

    # ── Main detect loop ───────────────────────────────────────────────────────
    def detect(self):
        if self.latest_rgb is None or self.latest_depth is None:
            return

        try:
            bgr   = self.bridge.imgmsg_to_cv2(self.latest_rgb, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(
                self.latest_depth, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        depth = np.array(depth, dtype=np.float32)
        debug = bgr.copy()

        # Colors để detect
        colors_to_check = [TARGET_COLOR] if TARGET_COLOR else list(COLOR_RANGES.keys())

        best_block = None   # (color, contour, u, v, Z)
        best_area  = 0

        for color in colors_to_check:
            result = self._detect_color(bgr, depth, color, debug)
            if result is None:
                continue
            contour, u, v, Z, area = result

            # Convert to world and ignore anything that lands near a bin.
            X_cam = (u - self.cx) * Z / self.fx
            Y_cam = (v - self.cy) * Z / self.fy
            pt = PointStamped()
            pt.header.stamp = self.latest_rgb.header.stamp
            pt.header.frame_id = self.camera_frame
            pt.point.x = X_cam
            pt.point.y = Y_cam
            pt.point.z = Z
            try:
                pt_world = self.tf_buffer.transform(
                    pt, 'world',
                    timeout=rclpy.duration.Duration(seconds=0.1))
            except Exception as e:
                self.get_logger().warn(f'TF2 transform failed during pre-filter: {e}')
                continue

            skip_bin, bin_name = self._is_near_bin(
                pt_world.point.x, pt_world.point.y, pt_world.point.z)
            if skip_bin:
                self.get_logger().info(
                    f'[{color}] Skip: near {bin_name} bin at '
                    f'({pt_world.point.x:.3f}, {pt_world.point.y:.3f}, {pt_world.point.z:.3f})')
                continue

            # Ưu tiên block lớn nhất (gần camera nhất)
            if area > best_area:
                best_area  = area
                best_block = (color, contour, u, v, Z)

        if best_block is None:
            self._pub_debug(debug)
            return

        color, contour, u, v, Z = best_block
        self._publish_pose(color, u, v, Z, contour, debug)

    # ── Detect 1 màu ──────────────────────────────────────────────────────────
    def _detect_color(self, bgr, depth, color, debug):
        """
        Trả về (contour, u, v, Z, area) nếu tìm thấy block màu `color`,
        None nếu không tìm thấy.
        """
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in COLOR_RANGES[color]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

        # Morphology: bỏ noise nhỏ
        k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.erode(mask,  k, iterations=2)
        mask = cv2.dilate(mask, k, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Filter theo area
        valid = [c for c in contours
                 if MIN_CONTOUR_AREA < cv2.contourArea(c) < MAX_CONTOUR_AREA]
        if not valid:
            return None

        h_img, w_img = depth.shape[:2]

        for c in sorted(valid, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(c)

            # Aspect ratio filter — loại edge/thành của bin
            x, y, w, h = cv2.boundingRect(c)
            aspect = max(w, h) / (min(w, h) + 1e-5)
            if aspect > MAX_ASPECT_RATIO:
                self.get_logger().debug(
                    f'[{color}] Skip: aspect={aspect:.1f} > {MAX_ASPECT_RATIO}')
                continue

            # Centroid
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            u = int(M['m10'] / M['m00'])
            v = int(M['m01'] / M['m00'])

            # Depth tại centroid (patch 5x5)
            patch = depth[max(0, v-2):min(h_img, v+3),
                          max(0, u-2):min(w_img, u+3)]
            valid_d = patch[np.isfinite(patch) & (patch > 0.01)]
            if valid_d.size == 0:
                self.get_logger().debug(f'[{color}] Skip: no valid depth at ({u},{v})')
                continue

            Z = float(np.median(valid_d))

            # Depth range filter — block phải ở khoảng gần hơn bin
            if not (MIN_DEPTH_M < Z < MAX_DEPTH_M):
                self.get_logger().info(
                    f'[{color}] Skip: depth={Z:.3f}m out of range '
                    f'[{MIN_DEPTH_M}, {MAX_DEPTH_M}]')
                continue

            self.get_logger().info(
                f'[{color}] Candidate: area={area:.0f} '
                f'centroid=({u},{v}) depth={Z:.3f}m aspect={aspect:.1f}')

            # Vẽ tất cả candidate contours lên debug (màu mờ)
            cv2.drawContours(debug, [c], -1, DEBUG_COLORS.get(color, (128,128,128)), 1)

            return (c, u, v, Z, area)

        return None

    def _is_near_bin(self, x, y, z) -> tuple[bool, str | None]:
        """Return (True, bin_name) if world point is near any bin center."""
        for bin_name, p in BIN_POSITIONS.items():
            dx = x - p['x']
            dy = y - p['y']
            dz = abs(z - p['z'])
            if (dx * dx + dy * dy) ** 0.5 <= BIN_EXCLUSION_RADIUS and dz <= BIN_EXCLUSION_Z_TOL:
                return True, bin_name
        return False, None

    # ── Publish pose ──────────────────────────────────────────────────────────
    def _publish_pose(self, color, u, v, Z, contour, debug):
        X_cam = (u - self.cx) * Z / self.fx
        Y_cam = (v - self.cy) * Z / self.fy

        pt = PointStamped()
        pt.header.stamp    = self.latest_rgb.header.stamp
        pt.header.frame_id = self.camera_frame
        pt.point.x = X_cam
        pt.point.y = Y_cam
        pt.point.z = Z

        try:
            pt_world = self.tf_buffer.transform(
                pt, 'world',
                timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f'TF2 transform failed: {e}')
            self._pub_debug(debug)
            return

        skip_bin, bin_name = self._is_near_bin(
            pt_world.point.x, pt_world.point.y, pt_world.point.z)
        if skip_bin:
            self.get_logger().info(
                f'[{color}] Skip publish: near {bin_name} bin at '
                f'({pt_world.point.x:.3f}, {pt_world.point.y:.3f}, {pt_world.point.z:.3f})')
            self._pub_debug(debug)
            return

        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        # frame_id encode màu block để pick_and_place.py biết cần đặt vào bin nào
        pose.header.frame_id = color
        pose.pose.position.x = pt_world.point.x
        pose.pose.position.y = pt_world.point.y
        pose.pose.position.z = pt_world.point.z
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

        self.get_logger().info(
            f'✓ [{color}] world: '
            f'x={pt_world.point.x:.3f} '
            f'y={pt_world.point.y:.3f} '
            f'z={pt_world.point.z:.3f}  '
            f'depth={Z:.3f}m')

        # Debug image
        dbg_color = DEBUG_COLORS.get(color, (0, 255, 0))
        cv2.drawContours(debug, [contour], -1, dbg_color, 3)
        cv2.circle(debug, (u, v), 6, (255, 255, 255), -1)
        cv2.circle(debug, (u, v), 4, dbg_color, -1)
        cv2.putText(debug,
                    f'{color} {Z:.2f}m',
                    (u + 10, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, dbg_color, 2)
        self._pub_debug(debug)

    def _pub_debug(self, bgr_img):
        try:
            self.pub_debug.publish(
                self.bridge.cv2_to_imgmsg(bgr_img, 'bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = BlockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
