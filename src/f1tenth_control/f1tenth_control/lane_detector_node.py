#!/usr/bin/env python3

"""Camera-based lane detector that publishes a local path for pure pursuit."""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf_transformations import quaternion_from_euler


class LaneDetectorNode(Node):
    """Detects a yellow lane line and publishes a nav_msgs/Path in odom frame."""

    def __init__(self) -> None:
        super().__init__('lane_detector')

        # Interface parameters -------------------------------------------------
        self.declare_parameter('image_topic', '/car1/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/car1/camera/color/camera_info')
        self.declare_parameter('odom_topic', '/car1/odom')
        self.declare_parameter('path_topic', 'lane_center_path')
        self.declare_parameter('confidence_topic', 'lane_confidence')
        self.declare_parameter('debug_image_topic', 'lane_detector/debug_image')
        self.declare_parameter('publish_debug_image', True)

        # Image-processing parameters -----------------------------------------
        self.declare_parameter('roi_row_start', 200)
        self.declare_parameter('roi_row_end', -1)
        self.declare_parameter('roi_col_min', 0)
        self.declare_parameter('roi_col_max', -1)
        self.declare_parameter('hsv_lower', [15, 60, 70])
        self.declare_parameter('hsv_upper', [40, 255, 255])
        self.declare_parameter('lab_b_min', 150)
        self.declare_parameter('min_row_pixels', 40)
        self.declare_parameter('row_step', 15)
        self.declare_parameter('max_rows', 12)
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('min_points', 4)
        self.declare_parameter('pixel_to_ground_homography', [1.0, 0.0, 0.0,
                                                              0.0, 1.0, 0.0,
                                                              0.0, 0.0, 1.0])
        self.declare_parameter('output_frame', 'odom')

        self.bridge = CvBridge()
        self.latest_odom: Odometry | None = None
        self.latest_camera_info: CameraInfo | None = None
        self._missing_odom_warned = False

        self.pixel_to_ground_h = self._load_homography()

        # Publishers -----------------------------------------------------------
        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter('path_topic').value,
            10,
        )
        self.confidence_pub = self.create_publisher(
            Float32,
            self.get_parameter('confidence_topic').value,
            10,
        )

        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        debug_topic = self.get_parameter('debug_image_topic').value
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, debug_topic, 10)

        # Subscribers ----------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )

        self.get_logger().info('Lane detector node initialized')

    # ------------------------------------------------------------------
    # Callbacks
    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def odom_callback(self, msg: Odometry) -> None:
        if self.latest_odom is None:
            self.get_logger().info('First odometry message received!')
        self.latest_odom = msg

    def image_callback(self, msg: Image) -> None:
        if self.latest_odom is None:
            if not self._missing_odom_warned:
                self.get_logger().warn('No odometry yet; skipping frame')
                self._missing_odom_warned = True
            return

        self._missing_odom_warned = False

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        roi, offsets = self.crop_roi(frame)
        if roi.size == 0:
            self.get_logger().warn('ROI is empty; adjust roi parameters')
            return

        mask = self.generate_mask(roi)
        lane_points_px = self.extract_lane_points(mask, offsets)
        confidence = self.compute_confidence(mask)
        self.publish_confidence(confidence)

        if len(lane_points_px) < int(self.get_parameter('min_points').value):
            return

        ground_points = self.project_pixels_to_ground(lane_points_px)
        if not ground_points:
            return

        world_points = self.transform_to_world_frame(ground_points)
        if not world_points:
            return

        smoothed = self.smooth_path(world_points)
        path_msg = self.points_to_path(smoothed, msg)
        self.path_pub.publish(path_msg)

        if self.publish_debug_image and self.debug_pub is not None:
            debug_img = self.create_debug_overlay(roi, mask, lane_points_px, offsets)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    # ------------------------------------------------------------------
    def crop_roi(self, frame: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
        row_start = int(self.get_parameter('roi_row_start').value)
        row_end_param = int(self.get_parameter('roi_row_end').value)
        col_min = int(self.get_parameter('roi_col_min').value)
        col_max_param = int(self.get_parameter('roi_col_max').value)

        height, width = frame.shape[:2]
        row_end = height if row_end_param < 0 else min(row_end_param, height)
        col_max = width if col_max_param < 0 else min(col_max_param, width)

        roi = frame[row_start:row_end, col_min:col_max]
        return roi, (col_min, row_start)

    def generate_mask(self, roi: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        mask_hsv = cv2.inRange(hsv, lower, upper)

        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        b_channel = lab[:, :, 2]
        lab_low = int(self.get_parameter('lab_b_min').value)
        lab_mask = cv2.inRange(b_channel, lab_low, 255)

        mask = cv2.bitwise_and(mask_hsv, lab_mask)

        kernel_size = max(1, int(self.get_parameter('morph_kernel_size').value))
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def extract_lane_points(self, mask: np.ndarray, offsets: Tuple[int, int]) -> List[Tuple[float, float]]:
        col_offset, row_offset = offsets
        height, _ = mask.shape
        row_step = max(1, int(self.get_parameter('row_step').value))
        max_rows = max(1, int(self.get_parameter('max_rows').value))
        min_pixels = max(1, int(self.get_parameter('min_row_pixels').value))

        samples: List[Tuple[float, float]] = []
        rows_processed = 0
        for row in range(height - 1, -1, -row_step):
            if rows_processed >= max_rows:
                break
            rows_processed += 1

            row_indices = np.where(mask[row, :] > 0)[0]
            if row_indices.size < min_pixels:
                continue

            x_mean = float(np.mean(row_indices) + col_offset)
            y_pix = float(row + row_offset)
            samples.append((x_mean, y_pix))

        return samples

    def compute_confidence(self, mask: np.ndarray) -> float:
        total = float(mask.size)
        if total < 1e-6:
            return 0.0
        coverage = float(cv2.countNonZero(mask)) / total
        return float(np.clip(coverage, 0.0, 1.0))

    def publish_confidence(self, confidence: float) -> None:
        msg = Float32()
        msg.data = confidence
        self.confidence_pub.publish(msg)

    def project_pixels_to_ground(self, pixels: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if not pixels:
            return []
        pts = np.array(pixels, dtype=np.float32).reshape(-1, 1, 2)
        projected = cv2.perspectiveTransform(pts, self.pixel_to_ground_h)
        return [(float(p[0][0]), float(p[0][1])) for p in projected]

    def smooth_path(self, points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        window = max(1, int(self.get_parameter('smoothing_window').value))
        if window <= 1 or len(points) < 3:
            return list(points)

        smoothed: List[Tuple[float, float]] = []
        for idx in range(len(points)):
            start = max(0, idx - window + 1)
            chunk = points[start:idx + 1]
            xs = [pt[0] for pt in chunk]
            ys = [pt[1] for pt in chunk]
            smoothed.append((float(np.mean(xs)), float(np.mean(ys))))
        return smoothed

    def transform_to_world_frame(self, points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if self.latest_odom is None:
            return []

        pose = self.latest_odom.pose.pose
        yaw = self.extract_yaw(self.latest_odom)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        origin_x = pose.position.x
        origin_y = pose.position.y

        transformed: List[Tuple[float, float]] = []
        for x_local, y_local in points:
            world_x = origin_x + cos_yaw * x_local - sin_yaw * y_local
            world_y = origin_y + sin_yaw * x_local + cos_yaw * y_local
            transformed.append((world_x, world_y))
        return transformed

    def points_to_path(self, points: Sequence[Tuple[float, float]], image_msg: Image) -> Path:
        path = Path()
        path.header.frame_id = self.get_parameter('output_frame').value
        path.header.stamp = image_msg.header.stamp

        for idx, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            yaw = self.estimate_segment_yaw(points, idx)
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        return path

    @staticmethod
    def estimate_segment_yaw(points: Sequence[Tuple[float, float]], idx: int) -> float:
        if len(points) <= 1:
            return 0.0
        if idx < len(points) - 1:
            curr = points[idx]
            nxt = points[idx + 1]
        else:
            curr = points[idx - 1]
            nxt = points[idx]
        dy = nxt[1] - curr[1]
        dx = nxt[0] - curr[0]
        return math.atan2(dy, dx)

    def create_debug_overlay(self, roi: np.ndarray, mask: np.ndarray,
                             points: Sequence[Tuple[float, float]], offsets: Tuple[int, int]) -> np.ndarray:
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        overlay = cv2.addWeighted(roi, 0.6, mask_color, 0.4, 0)
        for x_pix, y_pix in points:
            cv2.circle(
                overlay,
                (int(x_pix - offsets[0]), int(y_pix - offsets[1])),
                4,
                (0, 0, 255),
                -1,
            )
        return overlay

    def _load_homography(self) -> np.ndarray:
        values = self.get_parameter('pixel_to_ground_homography').value
        if len(values) != 9:
            raise ValueError('pixel_to_ground_homography must contain 9 values')
        return np.array(values, dtype=np.float32).reshape(3, 3)

    @staticmethod
    def extract_yaw(odom_msg: Odometry) -> float:
        q = odom_msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
