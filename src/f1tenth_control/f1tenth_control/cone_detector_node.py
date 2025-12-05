#!/usr/bin/env python3

"""Camera-based cone detector that publishes a corridor centerline path for pure pursuit."""

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
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from tf_transformations import quaternion_from_euler


class ConeDetectorNode(Node):
    """Detects orange cones and publishes a corridor centerline nav_msgs/Path."""

    def __init__(self) -> None:
        super().__init__('cone_detector')

        # Interface parameters -------------------------------------------------
        self.declare_parameter('image_topic', '/car1/D435i/color/image_raw')
        self.declare_parameter('path_topic', 'cone_corridor_path')
        self.declare_parameter('confidence_topic', 'cone_confidence')
        self.declare_parameter('debug_image_topic', 'cone_detector/debug_image')
        self.declare_parameter('publish_debug_image', True)

        # Image-processing parameters -----------------------------------------
        self.declare_parameter('roi_row_start', 220)
        self.declare_parameter('roi_row_end', -1)
        self.declare_parameter('roi_col_min', 0)
        self.declare_parameter('roi_col_max', -1)
        self.declare_parameter('hsv_lower', [5, 100, 100])
        self.declare_parameter('hsv_upper', [25, 255, 255])
        self.declare_parameter('min_contour_area', 100)
        self.declare_parameter('max_contour_area', 10000)
        self.declare_parameter('min_aspect_ratio', 0.3)
        self.declare_parameter('max_aspect_ratio', 3.0)
        self.declare_parameter('morph_kernel_size', 5)

        # Corridor computation parameters -------------------------------------
        self.declare_parameter('min_cones_per_side', 1)
        self.declare_parameter('max_cone_pair_distance', 200)
        self.declare_parameter('max_y_diff_for_pairing', 50)
        self.declare_parameter('smoothing_window', 3)
        self.declare_parameter('min_corridor_points', 2)

        # Homography parameters -----------------------------------------------
        self.declare_parameter('pixel_to_ground_homography', [1.0, 0.0, 0.0,
                                                              0.0, 1.0, 0.0,
                                                              0.0, 0.0, 1.0])
        self.declare_parameter('ground_x_offset', 0.6)
        self.declare_parameter('homography_swap_xy', False)
        self.declare_parameter('homography_negate_x', False)
        self.declare_parameter('homography_negate_y', False)
        self.declare_parameter('output_frame', 'base_link')

        self.bridge = CvBridge()
        self.pixel_to_ground_h = self._load_homography()

        x_offset = float(self.get_parameter('ground_x_offset').value)
        self.get_logger().info(f"Ground X offset: {x_offset}m")

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

        self.get_logger().info('Cone detector node initialized')

    # ------------------------------------------------------------------
    # Callbacks
    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        roi, offsets = self.crop_roi(frame)
        if roi.size == 0:
            self.get_logger().warn('ROI is empty; adjust roi parameters')
            return

        mask = self.generate_cone_mask(roi)
        left_cones, right_cones = self.detect_and_classify_cones(mask, offsets)

        confidence = self.compute_confidence(left_cones, right_cones)
        self.publish_confidence(confidence)

        corridor_points_px = self.compute_corridor_centerline(left_cones, right_cones)

        if len(corridor_points_px) < int(self.get_parameter('min_corridor_points').value):
            self.get_logger().debug('Not enough corridor points detected', throttle_duration_sec=1.0)
            # Publish empty path when no corridor detected
            empty_path = Path()
            empty_path.header.frame_id = self.get_parameter('output_frame').value
            empty_path.header.stamp = msg.header.stamp
            self.path_pub.publish(empty_path)

            if self.publish_debug_image and self.debug_pub is not None:
                debug_img = self.create_debug_overlay(roi, mask, left_cones, right_cones, [], offsets)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            return

        ground_points = self.project_pixels_to_ground(corridor_points_px)
        if not ground_points:
            self.get_logger().debug('Failed to project corridor points to ground', throttle_duration_sec=1.0)
            return

        smoothed = self.smooth_path(ground_points)
        path_msg = self.points_to_path(smoothed, msg)
        self.path_pub.publish(path_msg)

        if self.publish_debug_image and self.debug_pub is not None:
            debug_img = self.create_debug_overlay(roi, mask, left_cones, right_cones, corridor_points_px, offsets)
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

    def generate_cone_mask(self, roi: np.ndarray) -> np.ndarray:
        """Generate binary mask for orange cones using HSV color filtering."""
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel_size = max(1, int(self.get_parameter('morph_kernel_size').value))
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def detect_and_classify_cones(self, mask: np.ndarray, offsets: Tuple[int, int]) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
        """Detect cone contours and classify as left or right of image center.

        Returns:
            Tuple of (left_cones, right_cones) where each is a list of (x, y) pixel coordinates.
        """
        col_offset, row_offset = offsets

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = float(self.get_parameter('min_contour_area').value)
        max_area = float(self.get_parameter('max_contour_area').value)
        min_aspect = float(self.get_parameter('min_aspect_ratio').value)
        max_aspect = float(self.get_parameter('max_aspect_ratio').value)

        # Filter contours by area and aspect ratio
        valid_cones: List[Tuple[float, float]] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue

            # Check aspect ratio
            x, y, w, h = cv2.boundingRect(contour)
            if h == 0:
                continue
            aspect_ratio = float(w) / float(h)
            if aspect_ratio < min_aspect or aspect_ratio > max_aspect:
                continue

            # Use centroid of contour
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            cx = float(M['m10'] / M['m00']) + col_offset
            cy = float(M['m01'] / M['m00']) + row_offset
            valid_cones.append((cx, cy))

        # Classify left/right based on image center
        image_center_x = mask.shape[1] / 2.0 + col_offset
        left_cones = [(x, y) for x, y in valid_cones if x < image_center_x]
        right_cones = [(x, y) for x, y in valid_cones if x >= image_center_x]

        # Sort cones by y-coordinate (top to bottom)
        left_cones.sort(key=lambda p: p[1])
        right_cones.sort(key=lambda p: p[1])

        return left_cones, right_cones

    def compute_corridor_centerline(self, left_cones: List[Tuple[float, float]],
                                    right_cones: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Compute centerline path by pairing left and right cones and finding midpoints.

        Args:
            left_cones: List of (x, y) pixel coordinates for left cones
            right_cones: List of (x, y) pixel coordinates for right cones

        Returns:
            List of (x, y) pixel coordinates for corridor centerline
        """
        min_cones = int(self.get_parameter('min_cones_per_side').value)
        max_pair_dist = float(self.get_parameter('max_cone_pair_distance').value)
        max_y_diff = float(self.get_parameter('max_y_diff_for_pairing').value)

        # Need at least min_cones on each side
        if len(left_cones) < min_cones or len(right_cones) < min_cones:
            return []

        centerline: List[Tuple[float, float]] = []

        # For each left cone, find the best matching right cone at similar y-level
        for left_x, left_y in left_cones:
            best_right = None
            best_y_diff = float('inf')

            for right_x, right_y in right_cones:
                y_diff = abs(left_y - right_y)
                lateral_dist = abs(right_x - left_x)

                # Check if cones are at similar y-level and not too far apart
                if y_diff < max_y_diff and lateral_dist < max_pair_dist:
                    if y_diff < best_y_diff:
                        best_y_diff = y_diff
                        best_right = (right_x, right_y)

            if best_right is not None:
                # Compute midpoint
                mid_x = (left_x + best_right[0]) / 2.0
                mid_y = (left_y + best_right[1]) / 2.0
                centerline.append((mid_x, mid_y))

        # Also check for right cones that might not have been matched
        for right_x, right_y in right_cones:
            best_left = None
            best_y_diff = float('inf')

            for left_x, left_y in left_cones:
                y_diff = abs(left_y - right_y)
                lateral_dist = abs(right_x - left_x)

                if y_diff < max_y_diff and lateral_dist < max_pair_dist:
                    if y_diff < best_y_diff:
                        best_y_diff = y_diff
                        best_left = (left_x, left_y)

            if best_left is not None:
                mid_x = (best_left[0] + right_x) / 2.0
                mid_y = (best_left[1] + right_y) / 2.0
                # Avoid duplicates
                if not any(abs(p[0] - mid_x) < 1.0 and abs(p[1] - mid_y) < 1.0 for p in centerline):
                    centerline.append((mid_x, mid_y))

        # Sort centerline points by y-coordinate (bottom to top for consistency with lane detector)
        centerline.sort(key=lambda p: -p[1])

        return centerline

    def compute_confidence(self, left_cones: List[Tuple[float, float]],
                          right_cones: List[Tuple[float, float]]) -> float:
        """Compute detection confidence based on number of cone pairs.

        Returns:
            Confidence value from 0.0 to 1.0
        """
        min_cones = int(self.get_parameter('min_cones_per_side').value)

        # Low confidence if not enough cones on each side
        if len(left_cones) < min_cones or len(right_cones) < min_cones:
            return 0.2

        # Confidence based on number of cone pairs detected
        num_pairs = min(len(left_cones), len(right_cones))

        if num_pairs >= 3:
            return 0.9  # High confidence
        elif num_pairs >= 2:
            return 0.7  # Medium-high confidence
        elif num_pairs >= 1:
            return 0.5  # Medium confidence
        else:
            return 0.3  # Low confidence

    def publish_confidence(self, confidence: float) -> None:
        msg = Float32()
        msg.data = confidence
        self.confidence_pub.publish(msg)

    def project_pixels_to_ground(self, pixels: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Project pixel coordinates to ground plane in robot frame (x=forward, y=left).

        Uses the same homography approach as lane_detector_node.
        """
        if not pixels:
            return []
        pts = np.array(pixels, dtype=np.float32).reshape(-1, 1, 2)
        projected = cv2.perspectiveTransform(pts, self.pixel_to_ground_h)

        swap_xy = bool(self.get_parameter('homography_swap_xy').value)
        negate_x = bool(self.get_parameter('homography_negate_x').value)
        negate_y = bool(self.get_parameter('homography_negate_y').value)
        x_offset = float(self.get_parameter('ground_x_offset').value)

        result = []
        for p in projected:
            x, y = float(p[0][0]), float(p[0][1])
            if swap_xy:
                x, y = y, x
            if negate_x:
                x = -x
            if negate_y:
                y = -y
            # Apply offset after all transformations
            x += x_offset
            result.append((x, y))

        if len(result) > 0:
            self.get_logger().info(f"First corridor point: raw=({projected[0][0][0]:.2f}, {projected[0][0][1]:.2f}), final=({result[0][0]:.2f}, {result[0][1]:.2f})", throttle_duration_sec=2.0)
        return result

    def smooth_path(self, points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth path using moving average window."""
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

    def points_to_path(self, points: Sequence[Tuple[float, float]], image_msg: Image) -> Path:
        """Convert list of (x, y) points to nav_msgs/Path message."""
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
        """Estimate yaw angle for path segment."""
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
                            left_cones: List[Tuple[float, float]],
                            right_cones: List[Tuple[float, float]],
                            centerline: List[Tuple[float, float]],
                            offsets: Tuple[int, int]) -> np.ndarray:
        """Create debug visualization with detected cones and centerline.

        - Left cones: blue circles
        - Right cones: red circles
        - Centerline: green circles
        - Orange mask overlay
        """
        # Create colored overlay of mask
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # Make orange overlay
        mask_color[:, :, 0] = 0  # Zero out blue channel
        overlay = cv2.addWeighted(roi, 0.7, mask_color, 0.3, 0)

        col_offset, row_offset = offsets

        # Draw left cones (blue)
        for x_pix, y_pix in left_cones:
            cv2.circle(
                overlay,
                (int(x_pix - col_offset), int(y_pix - row_offset)),
                6,
                (255, 0, 0),  # Blue
                -1,
            )

        # Draw right cones (red)
        for x_pix, y_pix in right_cones:
            cv2.circle(
                overlay,
                (int(x_pix - col_offset), int(y_pix - row_offset)),
                6,
                (0, 0, 255),  # Red
                -1,
            )

        # Draw centerline (green)
        for x_pix, y_pix in centerline:
            cv2.circle(
                overlay,
                (int(x_pix - col_offset), int(y_pix - row_offset)),
                4,
                (0, 255, 0),  # Green
                -1,
            )

        # Draw lines connecting centerline points
        if len(centerline) > 1:
            for i in range(len(centerline) - 1):
                pt1 = (int(centerline[i][0] - col_offset), int(centerline[i][1] - row_offset))
                pt2 = (int(centerline[i+1][0] - col_offset), int(centerline[i+1][1] - row_offset))
                cv2.line(overlay, pt1, pt2, (0, 255, 0), 2)

        return overlay

    def _load_homography(self) -> np.ndarray:
        """Load homography matrix from parameters."""
        values = self.get_parameter('pixel_to_ground_homography').value
        if len(values) != 9:
            raise ValueError('pixel_to_ground_homography must contain 9 values')
        return np.array(values, dtype=np.float32).reshape(3, 3)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
