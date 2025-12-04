#!/usr/bin/env python3
"""
Diagnostic tool to verify homography matrix issue.
Shows what pixel coordinates become in 'ground' frame.
"""

import numpy as np

# Current identity matrix from config
identity_h = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

# Example pixel coordinates from lane detection
# (these would be typical lane line pixels in a 1280x720 image)
pixel_coords = np.array([
    [320.0, 700.0],  # Bottom of image (closest to car)
    [340.0, 600.0],  # Middle
    [360.0, 500.0],  # Middle-far
    [380.0, 400.0],  # Far
], dtype=np.float32).reshape(-1, 1, 2)

# Transform using identity matrix (what currently happens)
import cv2
ground_coords_wrong = cv2.perspectiveTransform(pixel_coords, identity_h)

print("=" * 60)
print("HOMOGRAPHY DIAGNOSTIC")
print("=" * 60)
print("\nCurrent matrix (IDENTITY):")
print(identity_h)
print("\nPixel coords → 'Ground' coords (WRONG - just pixels!):")
print("-" * 60)
for i, (px, gnd) in enumerate(zip(pixel_coords, ground_coords_wrong)):
    px_x, px_y = px[0]
    gnd_x, gnd_y = gnd[0]
    print(f"Pixel ({px_x:6.1f}, {px_y:6.1f}) → Ground ({gnd_x:6.1f}, {gnd_y:6.1f})")

print("\n" + "=" * 60)
print("PROBLEM: Ground coordinates should be in METERS!")
print("Expected range: X: 0.5-2.0m forward, Y: -0.3 to +0.3m lateral")
print("Actual values: X: 320-380 (pixels!), Y: 400-700 (pixels!)")
print("=" * 60)

print("\nEXAMPLE of what a proper homography SHOULD produce:")
print("-" * 60)
# Example of reasonable ground plane coordinates
proper_coords = np.array([
    [0.5, 0.0],   # 0.5m forward, on centerline
    [1.0, -0.05], # 1.0m forward, slightly left
    [1.5, -0.08], # 1.5m forward, more left
    [2.0, -0.10], # 2.0m forward, curved left
])
for i, coord in enumerate(proper_coords):
    print(f"Point {i+1}: X={coord[0]:4.2f}m forward, Y={coord[1]:+5.2f}m lateral")

print("\n" + "=" * 60)
print("TO FIX: Calibrate homography matrix to map pixels → meters")
print("=" * 60)
