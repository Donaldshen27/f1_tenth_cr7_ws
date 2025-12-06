#!/usr/bin/env python3

"""Camera-based cone detector that publishes a corridor centerline path for pure pursuit.

Simple approach: Find the two closest cones (one left, one right) at the bottom of the image
and navigate to their midpoint. More robust than line fitting, especially in curves.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

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
        self.declare_parameter('min_cone_separation', 50)  # Min x-distance between left and right cone (pixels)
        self.declare_parameter('max_cone_separation', 400)  # Max x-distance between left and right cone (pixels)

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
        all_cones = self.detect_cones(mask, offsets)

        # Find the closest left and right cones
        left_cone, right_cone = self.find_closest_cone_pair(all_cones, offsets, mask.shape)

        # Compute confidence based on whether we found a valid pair
        confidence = self.compute_confidence(left_cone, right_cone)
        self.publish_confidence(confidence)

        # Compute target point (midpoint between the two cones)
        if left_cone is None or right_cone is None:
            self.get_logger().debug('Could not find valid cone pair', throttle_duration_sec=1.0)
            empty_path = Path()
            empty_path.header.frame_id = self.get_parameter('output_frame').value
            empty_path.header.stamp = msg.header.stamp
            self.path_pub.publish(empty_path)

            if self.publish_debug_image and self.debug_pub is not None:
                debug_img = self.create_debug_overlay(roi, mask, left_cone, right_cone, None, offsets)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            return

        # Compute midpoint in pixel coordinates
        mid_x = (left_cone[0] + right_cone[0]) / 2.0
        mid_y = (left_cone[1] + right_cone[1]) / 2.0
        target_px = [(mid_x, mid_y)]

        # Project to ground plane
        ground_points = self.project_pixels_to_ground(target_px)
        if not ground_points:
            self.get_logger().debug('Failed to project target to ground', throttle_duration_sec=1.0)
            return

        path_msg = self.points_to_path(ground_points, msg)
        self.path_pub.publish(path_msg)

        if self.publish_debug_image and self.debug_pub is not None:
            debug_img = self.create_debug_overlay(roi, mask, left_cone, right_cone, (mid_x, mid_y), offsets)
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

    def detect_cones(self, mask: np.ndarray, offsets: Tuple[int, int]) -> List[Tuple[float, float]]:
        """Detect all cone contours and return their centroids.

        Returns:
            List of (x, y) pixel coordinates for all detected cones.
        """
        col_offset, row_offset = offsets

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = float(self.get_parameter('min_contour_area').value)
        max_area = float(self.get_parameter('max_contour_area').value)
        min_aspect = float(self.get_parameter('min_aspect_ratio').value)
        max_aspect = float(self.get_parameter('max_aspect_ratio').value)

        cones: List[Tuple[float, float]] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if h == 0:
                continue
            aspect_ratio = float(w) / float(h)
            if aspect_ratio < min_aspect or aspect_ratio > max_aspect:
                continue

            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            cx = float(M['m10'] / M['m00']) + col_offset
            cy = float(M['m01'] / M['m00']) + row_offset
            cones.append((cx, cy))

        return cones

    def find_closest_cone_pair(self, cones: List[Tuple[float, float]],
                                offsets: Tuple[int, int],
                                mask_shape: Tuple[int, ...]) -> Tuple[Optional[Tuple[float, float]], Optional[Tuple[float, float]]]:
        """Find the closest left and right cones.

        Strategy:
        1. Find the cone with the largest y-coordinate (closest to camera, bottom of image)
        2. That cone is either left or right based on image center
        3. Find the closest cone on the opposite side

        Returns:
            Tuple of (left_cone, right_cone) or (None, None) if no valid pair found.
        """
        if len(cones) < 2:
            return None, None

        col_offset, row_offset = offsets
        image_center_x = mask_shape[1] / 2.0 + col_offset
        min_sep = float(self.get_parameter('min_cone_separation').value)
        max_sep = float(self.get_parameter('max_cone_separation').value)

        # Sort cones by y-coordinate descending (bottom of image = closest = first)
        sorted_cones = sorted(cones, key=lambda c: -c[1])

        # Find the closest cone (largest y)
        closest_cone = sorted_cones[0]

        # Determine if it's left or right of center
        if closest_cone[0] < image_center_x:
            # Closest cone is on the left, find closest right cone
            left_cone = closest_cone
            right_cone = None

            # Look for the closest cone on the right side with valid separation
            for cone in sorted_cones[1:]:
                if cone[0] >= image_center_x:
                    separation = cone[0] - left_cone[0]
                    if min_sep <= separation <= max_sep:
                        right_cone = cone
                        break
        else:
            # Closest cone is on the right, find closest left cone
            right_cone = closest_cone
            left_cone = None

            # Look for the closest cone on the left side with valid separation
            for cone in sorted_cones[1:]:
                if cone[0] < image_center_x:
                    separation = right_cone[0] - cone[0]
                    if min_sep <= separation <= max_sep:
                        left_cone = cone
                        break

        # Validate we have both
        if left_cone is None or right_cone is None:
            # Try alternative: find the two closest cones that are on opposite sides
            for i, cone1 in enumerate(sorted_cones):
                for cone2 in sorted_cones[i+1:]:
                    # Check they're on opposite sides
                    if (cone1[0] < image_center_x) != (cone2[0] < image_center_x):
                        separation = abs(cone1[0] - cone2[0])
                        if min_sep <= separation <= max_sep:
                            if cone1[0] < cone2[0]:
                                return cone1, cone2
                            else:
                                return cone2, cone1
            return None, None

        return left_cone, right_cone

    def compute_confidence(self, left_cone: Optional[Tuple[float, float]],
                          right_cone: Optional[Tuple[float, float]]) -> float:
        """Compute detection confidence based on whether we found a valid cone pair.

        Returns:
            Confidence value from 0.0 to 1.0
        """
        if left_cone is not None and right_cone is not None:
            return 0.9  # High confidence - we have a valid pair
        elif left_cone is not None or right_cone is not None:
            return 0.3  # Low confidence - only one cone
        else:
            return 0.1  # Very low confidence - no cones

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
                            left_cone: Optional[Tuple[float, float]],
                            right_cone: Optional[Tuple[float, float]],
                            target_point: Optional[Tuple[float, float]],
                            offsets: Tuple[int, int]) -> np.ndarray:
        """Create debug visualization with detected cone pair and target.

        - Left cone: blue circle with label
        - Right cone: red circle with label
        - Target (midpoint): green circle
        - Line connecting the cones through the target
        - Min/max separation zone visualization
        - Orange mask overlay
        """
        # Create colored overlay of mask
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_color[:, :, 0] = 0  # Zero out blue channel for orange tint
        overlay = cv2.addWeighted(roi, 0.7, mask_color, 0.3, 0)

        col_offset, row_offset = offsets
        img_height, img_width = roi.shape[:2]
        img_center_x = img_width // 2

        # Get separation parameters
        min_sep = int(self.get_parameter('min_cone_separation').value)
        max_sep = int(self.get_parameter('max_cone_separation').value)

        # Draw min/max separation zone at bottom of image
        # This shows the valid detection zone
        zone_y = img_height - 30  # Near bottom of ROI

        # Draw max separation (outer bounds) - yellow dashed
        max_left = img_center_x - max_sep // 2
        max_right = img_center_x + max_sep // 2
        cv2.line(overlay, (max_left, zone_y - 10), (max_left, zone_y + 10), (0, 255, 255), 2)
        cv2.line(overlay, (max_right, zone_y - 10), (max_right, zone_y + 10), (0, 255, 255), 2)
        cv2.line(overlay, (max_left, zone_y), (max_right, zone_y), (0, 255, 255), 1)

        # Draw min separation (inner bounds) - cyan dashed
        min_left = img_center_x - min_sep // 2
        min_right = img_center_x + min_sep // 2
        cv2.line(overlay, (min_left, zone_y - 10), (min_left, zone_y + 10), (255, 255, 0), 2)
        cv2.line(overlay, (min_right, zone_y - 10), (min_right, zone_y + 10), (255, 255, 0), 2)

        # Draw center line
        cv2.line(overlay, (img_center_x, zone_y - 15), (img_center_x, zone_y + 15), (255, 255, 255), 1)

        # Add labels
        cv2.putText(overlay, f"min:{min_sep}", (min_left - 15, zone_y + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
        cv2.putText(overlay, f"max:{max_sep}", (max_right - 20, zone_y + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

        # Draw left cone (blue) with larger circle
        if left_cone is not None:
            pt = (int(left_cone[0] - col_offset), int(left_cone[1] - row_offset))
            cv2.circle(overlay, pt, 12, (255, 0, 0), 3)  # Blue outline
            cv2.putText(overlay, "L", (pt[0] - 5, pt[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Draw right cone (red) with larger circle
        if right_cone is not None:
            pt = (int(right_cone[0] - col_offset), int(right_cone[1] - row_offset))
            cv2.circle(overlay, pt, 12, (0, 0, 255), 3)  # Red outline
            cv2.putText(overlay, "R", (pt[0] - 5, pt[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Draw target point (green) and line connecting cones
        if target_point is not None and left_cone is not None and right_cone is not None:
            target_pt = (int(target_point[0] - col_offset), int(target_point[1] - row_offset))
            left_pt = (int(left_cone[0] - col_offset), int(left_cone[1] - row_offset))
            right_pt = (int(right_cone[0] - col_offset), int(right_cone[1] - row_offset))

            # Draw line from left to right cone
            cv2.line(overlay, left_pt, right_pt, (0, 255, 0), 2)

            # Draw target point
            cv2.circle(overlay, target_pt, 8, (0, 255, 0), -1)  # Filled green
            cv2.putText(overlay, "TARGET", (target_pt[0] - 25, target_pt[1] - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            # Show actual separation
            actual_sep = int(abs(right_cone[0] - left_cone[0]))
            cv2.putText(overlay, f"sep:{actual_sep}px", (target_pt[0] - 25, target_pt[1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

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
