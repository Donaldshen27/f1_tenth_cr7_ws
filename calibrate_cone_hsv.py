#!/usr/bin/env python3
"""
Cone HSV Calibration Tool

Click on orange cones in the camera feed to extract HSV values.
The tool will collect samples and automatically update cone_detector.yaml.

Usage:
    python3 calibrate_cone_hsv.py

Controls:
    - Left click: Sample HSV at that pixel (samples a 5x5 region)
    - 'c': Clear all samples
    - 's': Save and UPDATE cone_detector.yaml with new HSV range
    - 'q': Quit

Requirements:
    - Camera must be running: ros2 launch realsense2_camera rs_launch.py camera_namespace:=car1 camera_name:=D435i
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import re


class ConeHSVCalibrator(Node):
    # Path to cone_detector.yaml config file
    CONFIG_PATH = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'src/f1tenth_control/config/cone_detector.yaml'
    )

    def __init__(self):
        super().__init__('cone_hsv_calibrator')

        self.bridge = CvBridge()
        self.current_frame = None
        self.hsv_samples = []

        # Find config file path
        self.config_path = self._find_config_path()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/car1/D435i/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info('Cone HSV Calibrator started')
        self.get_logger().info(f'Config file: {self.config_path}')
        self.get_logger().info('Click on orange cones to sample HSV values')
        self.get_logger().info('Press "s" to save and UPDATE cone_detector.yaml')
        self.get_logger().info('Press "c" to clear samples')
        self.get_logger().info('Press "q" to quit')

    def _find_config_path(self):
        """Find the cone_detector.yaml config file."""
        # Try relative to script location
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.join(script_dir, 'src/f1tenth_control/config/cone_detector.yaml'),
            os.path.join(script_dir, 'config/cone_detector.yaml'),
            '/home/orin/CR7_ws/f1_tenth_cr7_ws/src/f1tenth_control/config/cone_detector.yaml',
        ]
        for path in candidates:
            if os.path.exists(path):
                return path
        # Default fallback
        return candidates[0]

    def image_callback(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.current_frame is not None:
            # Sample a 5x5 region around the click
            hsv_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)

            h, w = hsv_frame.shape[:2]
            x_start = max(0, x - 2)
            x_end = min(w, x + 3)
            y_start = max(0, y - 2)
            y_end = min(h, y + 3)

            region = hsv_frame[y_start:y_end, x_start:x_end]

            # Get all HSV values in the region
            for row in range(region.shape[0]):
                for col in range(region.shape[1]):
                    h_val, s_val, v_val = region[row, col]
                    self.hsv_samples.append((int(h_val), int(s_val), int(v_val)))

            # Also get the center pixel
            center_hsv = hsv_frame[y, x]

            # Get BGR value for reference
            bgr = self.current_frame[y, x]

            print(f"\n--- Sample at ({x}, {y}) ---")
            print(f"  BGR: ({bgr[0]}, {bgr[1]}, {bgr[2]})")
            print(f"  HSV: ({center_hsv[0]}, {center_hsv[1]}, {center_hsv[2]})")
            print(f"  Total samples: {len(self.hsv_samples)}")

    def compute_hsv_range(self):
        if not self.hsv_samples:
            print("No samples collected!")
            return None, None

        samples = np.array(self.hsv_samples)

        # Compute statistics
        h_vals = samples[:, 0]
        s_vals = samples[:, 1]
        v_vals = samples[:, 2]

        # Use percentiles to handle outliers
        h_min, h_max = np.percentile(h_vals, 5), np.percentile(h_vals, 95)
        s_min, s_max = np.percentile(s_vals, 5), np.percentile(s_vals, 95)
        v_min, v_max = np.percentile(v_vals, 5), np.percentile(v_vals, 95)

        # Add some margin
        margin_h = 5
        margin_s = 20
        margin_v = 30

        lower = [
            max(0, int(h_min - margin_h)),
            max(0, int(s_min - margin_s)),
            max(0, int(v_min - margin_v)),
        ]
        upper = [
            min(179, int(h_max + margin_h)),
            min(255, int(s_max + margin_s)),
            min(255, int(v_max + margin_v)),
        ]

        return lower, upper

    def update_config_file(self, lower, upper):
        """Update cone_detector.yaml with new HSV values."""
        if not os.path.exists(self.config_path):
            print(f"ERROR: Config file not found: {self.config_path}")
            return False

        with open(self.config_path, 'r') as f:
            content = f.read()

        # Update hsv_lower line
        content = re.sub(
            r'hsv_lower:\s*\[[^\]]+\]',
            f'hsv_lower: [{lower[0]}, {lower[1]}, {lower[2]}]',
            content
        )

        # Update hsv_upper line
        content = re.sub(
            r'hsv_upper:\s*\[[^\]]+\]',
            f'hsv_upper: [{upper[0]}, {upper[1]}, {upper[2]}]',
            content
        )

        with open(self.config_path, 'w') as f:
            f.write(content)

        return True

    def save_and_show_range(self):
        lower, upper = self.compute_hsv_range()
        if lower is None:
            return

        samples = np.array(self.hsv_samples)

        print("\n" + "="*60)
        print("HSV CALIBRATION RESULTS")
        print("="*60)
        print(f"\nTotal samples: {len(self.hsv_samples)}")
        print(f"\nHue range:        {samples[:,0].min()} - {samples[:,0].max()}")
        print(f"Saturation range: {samples[:,1].min()} - {samples[:,1].max()}")
        print(f"Value range:      {samples[:,2].min()} - {samples[:,2].max()}")
        print("\n" + "-"*60)
        print("NEW HSV VALUES:")
        print("-"*60)
        print(f"\nhsv_lower: [{lower[0]}, {lower[1]}, {lower[2]}]")
        print(f"hsv_upper: [{upper[0]}, {upper[1]}, {upper[2]}]")
        print("\n" + "="*60)

        # Update cone_detector.yaml directly
        if self.update_config_file(lower, upper):
            print(f"\n>>> UPDATED: {self.config_path}")
            print(">>> Rebuild to apply: colcon build --packages-select f1tenth_control")
        else:
            print("\n>>> FAILED to update config file")

        # Also save to backup text file
        with open('cone_hsv_calibration.txt', 'w') as f:
            f.write("Cone HSV Calibration Results\n")
            f.write("="*40 + "\n\n")
            f.write(f"Total samples: {len(self.hsv_samples)}\n\n")
            f.write("Raw ranges:\n")
            f.write(f"  Hue: {samples[:,0].min()} - {samples[:,0].max()}\n")
            f.write(f"  Sat: {samples[:,1].min()} - {samples[:,1].max()}\n")
            f.write(f"  Val: {samples[:,2].min()} - {samples[:,2].max()}\n\n")
            f.write("Applied to cone_detector.yaml:\n")
            f.write(f"  hsv_lower: [{lower[0]}, {lower[1]}, {lower[2]}]\n")
            f.write(f"  hsv_upper: [{upper[0]}, {upper[1]}, {upper[2]}]\n\n")
            f.write("All samples (H, S, V):\n")
            for s in self.hsv_samples:
                f.write(f"  {s}\n")

        print(f"Backup saved to: cone_hsv_calibration.txt")

    def run(self):
        cv2.namedWindow('Cone HSV Calibrator')
        cv2.setMouseCallback('Cone HSV Calibrator', self.mouse_callback)

        print("\nWaiting for camera feed...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.current_frame is not None:
                # Create display frame with info overlay
                display = self.current_frame.copy()

                # Add instructions
                cv2.putText(display, "Click on ORANGE CONES to sample", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display, f"Samples: {len(self.hsv_samples)}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display, "s=save  c=clear  q=quit", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # If we have samples, show current preview mask
                if self.hsv_samples:
                    lower, upper = self.compute_hsv_range()
                    if lower and upper:
                        hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
                        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

                        # Show mask as overlay
                        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                        mask_colored[:,:,0] = 0  # Remove blue
                        mask_colored[:,:,1] = mask_colored[:,:,1] // 2  # Reduce green
                        # Keep red for orange appearance

                        display = cv2.addWeighted(display, 0.7, mask_colored, 0.3, 0)

                        # Show current range
                        cv2.putText(display, f"Lower: {lower}", (10, 120),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                        cv2.putText(display, f"Upper: {upper}", (10, 140),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                cv2.imshow('Cone HSV Calibrator', display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                self.hsv_samples = []
                print("\nSamples cleared!")
            elif key == ord('s'):
                self.save_and_show_range()

        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = ConeHSVCalibrator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
