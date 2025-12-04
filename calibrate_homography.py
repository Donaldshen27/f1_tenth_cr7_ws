#!/usr/bin/env python3
"""
Interactive homography calibration for lane following.

This tool computes the pixel-to-ground transformation matrix using 4-point correspondence.

Usage:
1. Run this script while the camera is running
2. Click 4 corners of a rectangle on the ground (e.g., tape markers)
3. Enter the real-world dimensions
4. Copy the resulting matrix to lane_detector.yaml
"""

import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class HomographyCalibrator(Node):
    def __init__(self):
        super().__init__('homography_calibrator')

        self.declare_parameter('image_topic', '/car1/camera/color/image_raw')

        self.bridge = CvBridge()
        self.latest_image = None
        self.points = []

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            10
        )

        self.get_logger().info('Homography calibrator started')
        self.get_logger().info(f'Waiting for images on {self.get_parameter("image_topic").value}...')

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.points) < 4:
            self.points.append((x, y))
            print(f"Point {len(self.points)}/4: ({x}, {y})")

    def run_calibration(self):
        print("\n" + "="*60)
        print("HOMOGRAPHY CALIBRATION TOOL")
        print("="*60)
        print("\nINSTRUCTIONS:")
        print("1. Place 4 tape markers on the ground forming a rectangle")
        print("2. Measure the rectangle dimensions (width and length in meters)")
        print("3. Click the 4 corners in this order:")
        print("   - Top-left (farthest, left)")
        print("   - Top-right (farthest, right)")
        print("   - Bottom-right (closest, right)")
        print("   - Bottom-left (closest, left)")
        print("\nWaiting for camera image...")

        # Wait for image
        rate = self.create_rate(10)
        while self.latest_image is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_image is None:
            print("ERROR: No camera image received!")
            return None

        print("\nImage received! Click 4 corners of your rectangle...")

        # Display image and collect points
        display_img = self.latest_image.copy()
        cv2.namedWindow('Calibration')
        cv2.setMouseCallback('Calibration', self.mouse_callback)

        while len(self.points) < 4:
            temp_img = display_img.copy()

            # Draw points
            for i, pt in enumerate(self.points):
                cv2.circle(temp_img, pt, 5, (0, 255, 0), -1)
                cv2.putText(temp_img, f"{i+1}", (pt[0]+10, pt[1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Draw lines between points
            if len(self.points) > 1:
                for i in range(len(self.points)-1):
                    cv2.line(temp_img, self.points[i], self.points[i+1], (0, 255, 0), 2)

            cv2.imshow('Calibration', temp_img)
            if cv2.waitKey(50) == 27:  # ESC to quit
                print("\nCalibration cancelled")
                cv2.destroyAllWindows()
                return None

        cv2.destroyAllWindows()

        # Get real-world measurements
        print("\n" + "-"*60)
        print("Now enter the REAL-WORLD dimensions of your rectangle:")
        try:
            width = float(input("Width (horizontal, in meters): "))
            length = float(input("Length (depth/forward, in meters): "))
        except ValueError:
            print("ERROR: Invalid input")
            return None

        # Define real-world coordinates
        # Origin at bottom-left, X forward (along camera optical axis), Y left
        world_points = np.array([
            [length, width/2],   # Top-left (far, left)
            [length, -width/2],  # Top-right (far, right)
            [0, -width/2],       # Bottom-right (close, right)
            [0, width/2],        # Bottom-left (close, left)
        ], dtype=np.float32)

        # Compute homography
        pixel_points = np.array(self.points, dtype=np.float32)
        H, status = cv2.findHomography(pixel_points, world_points)

        if H is None:
            print("ERROR: Failed to compute homography")
            return None

        # Display results
        print("\n" + "="*60)
        print("CALIBRATION SUCCESSFUL!")
        print("="*60)
        print("\nPixel points (image coordinates):")
        for i, pt in enumerate(self.points):
            print(f"  {i+1}. ({pt[0]:4d}, {pt[1]:4d})")

        print(f"\nWorld points (ground plane in meters):")
        for i, pt in enumerate(world_points):
            print(f"  {i+1}. X={pt[0]:5.2f}m (forward), Y={pt[1]:+6.2f}m (lateral)")

        print("\n" + "-"*60)
        print("HOMOGRAPHY MATRIX:")
        print("-"*60)
        print(H)

        print("\n" + "-"*60)
        print("YAML FORMAT (copy to lane_detector.yaml):")
        print("-"*60)
        print("pixel_to_ground_homography: [")
        print(f"  {H[0,0]:8.5f}, {H[0,1]:8.5f}, {H[0,2]:8.5f},")
        print(f"  {H[1,0]:8.5f}, {H[1,1]:8.5f}, {H[1,2]:8.5f},")
        print(f"  {H[2,0]:8.5f}, {H[2,1]:8.5f}, {H[2,2]:8.5f}")
        print("]")
        print("-"*60)

        # Verify transformation
        print("\n" + "-"*60)
        print("VERIFICATION: Pixel → Ground transformation")
        print("-"*60)
        test_pixels = np.array(self.points, dtype=np.float32).reshape(-1, 1, 2)
        transformed = cv2.perspectiveTransform(test_pixels, H)

        for i, (orig, trans) in enumerate(zip(pixel_points, transformed)):
            trans_x, trans_y = trans[0]
            expected_x, expected_y = world_points[i]
            print(f"Point {i+1}: Pixel ({orig[0]:4.0f},{orig[1]:4.0f}) → "
                  f"Ground ({trans_x:5.2f}, {trans_y:+6.2f})m  "
                  f"[Expected: ({expected_x:5.2f}, {expected_y:+6.2f})m]")

        return H


def main(args=None):
    rclpy.init(args=args)
    calibrator = HomographyCalibrator()

    try:
        H = calibrator.run_calibration()
        if H is not None:
            print("\n" + "="*60)
            print("NEXT STEPS:")
            print("="*60)
            print("1. Copy the YAML format matrix above")
            print("2. Edit: src/f1tenth_control/config/lane_detector.yaml")
            print("3. Replace the pixel_to_ground_homography values")
            print("4. Restart the lane detector node")
            print("5. Test lane following!")
    except KeyboardInterrupt:
        print("\nCalibration interrupted")
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
