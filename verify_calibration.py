#!/usr/bin/env python3
"""Quick verification of calibrated homography matrix."""

import numpy as np
import cv2

# Your calibrated matrix
H_calibrated = np.array([
    [0.00003,  0.00175, -0.97424],
    [0.00060,  0.00020, -0.47260],
    [-0.00026, -0.00325,  1.00000]
], dtype=np.float32)

print("="*60)
print("CALIBRATION VERIFICATION")
print("="*60)

# Test with typical lane pixels (bottom-middle of 1280x720 image)
test_pixels = np.array([
    [640, 700],  # Center-bottom (very close to car)
    [640, 600],  # Center-middle
    [640, 500],  # Center-upper middle
    [640, 400],  # Center-far
], dtype=np.float32).reshape(-1, 1, 2)

transformed = cv2.perspectiveTransform(test_pixels, H_calibrated)

print("\nTypical lane centerline pixels → Ground coordinates:")
print("-"*60)
for px, gnd in zip(test_pixels, transformed):
    px_x, px_y = px[0]
    gnd_x, gnd_y = gnd[0]
    print(f"Pixel ({px_x:4.0f}, {px_y:4.0f}) → Ground X={gnd_x:5.2f}m forward, Y={gnd_y:+6.2f}m lateral")

print("\n" + "="*60)
print("CHECKS:")
print("="*60)

# Sanity checks
all_good = True

# Check 1: Forward distances should increase as we go up in image
forward_distances = [gnd[0][0] for gnd in transformed]
if forward_distances[0] < forward_distances[-1]:
    print("✓ Forward distance increases correctly (pixels higher in image = farther away)")
else:
    print("✗ WARNING: Forward distance trend is wrong!")
    all_good = False

# Check 2: Reasonable ranges
max_forward = max(forward_distances)
if 0.3 < max_forward < 3.0:
    print(f"✓ Forward distances are reasonable (max: {max_forward:.2f}m)")
else:
    print(f"✗ WARNING: Forward distance seems off (max: {max_forward:.2f}m)")
    all_good = False

# Check 3: Center pixels should have near-zero lateral offset
lateral_offsets = [abs(gnd[0][1]) for gnd in transformed]
if all(offset < 0.2 for offset in lateral_offsets):
    print(f"✓ Center pixels map near centerline (max offset: {max(lateral_offsets):.3f}m)")
else:
    print(f"✗ WARNING: Center pixels have large lateral offset (max: {max(lateral_offsets):.3f}m)")
    all_good = False

print("\n" + "="*60)
if all_good:
    print("✅ CALIBRATION LOOKS GOOD - SAFE TO APPLY!")
else:
    print("⚠️  CALIBRATION MAY NEED ADJUSTMENT - REVIEW WARNINGS")
print("="*60)
