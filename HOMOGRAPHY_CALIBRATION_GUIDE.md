# Homography Calibration Guide

## Why You Need This

Your camera lane following isn't working because the `pixel_to_ground_homography` matrix is set to **identity**, which means:
- Pixel coordinates are treated as "ground" coordinates directly
- A pixel at (400, 500) becomes "ground position" (400, 500) - **complete nonsense!**
- Should be: Pixel (400, 500) → Ground (1.2m forward, 0.05m left)

Pure pursuit can't follow lanes when coordinates are off by 100x!

---

## Method 1: Quick 4-Point Calibration (RECOMMENDED)

**Time:** 15 minutes
**Accuracy:** Good enough for lane following
**Requirements:** 4 tape markers, ruler/measuring tape

### Step 1: Prepare Calibration Target

1. **Place 4 tape markers on the ground** forming a rectangle in the camera's view:
   ```
   Camera view:

         [1]------[2]    ← Far from camera (1-2m ahead)
          |        |
          |        |
         [4]------[3]    ← Close to camera (0.5m ahead)
   ```

2. **Measure the rectangle:**
   - Width (horizontal): _____ meters (e.g., 0.50m)
   - Length (depth): _____ meters (e.g., 1.00m)

3. **Make it visible:**
   - Use bright colored tape (yellow, red, or white)
   - Ensure all 4 corners are clearly visible in the camera image

### Step 2: Run Calibration Tool

1. **Start the camera** (if not already running):
   ```bash
   ros2 launch realsense2_camera rs_launch.py \
       camera_name:=car1 \
       camera_namespace:=camera
   ```

2. **In another terminal, run calibration:**
   ```bash
   cd ~/CR7_ws/f1_tenth_cr7_ws
   python3 calibrate_homography.py
   ```

3. **Click the 4 corners** in order:
   - Top-left (farthest, left) - marker [1]
   - Top-right (farthest, right) - marker [2]
   - Bottom-right (closest, right) - marker [3]
   - Bottom-left (closest, left) - marker [4]

4. **Enter measurements** when prompted:
   ```
   Width (horizontal, in meters): 0.50
   Length (depth/forward, in meters): 1.00
   ```

5. **Copy the output** - you'll see something like:
   ```yaml
   pixel_to_ground_homography: [
      0.00234, -0.00156,  1.24567,
     -0.00089,  0.00298, -0.15432,
      0.00001, -0.00234,  1.00000
   ]
   ```

### Step 3: Update Configuration

1. **Edit the config file:**
   ```bash
   nano src/f1tenth_control/config/lane_detector.yaml
   ```

2. **Replace lines 23-27** (the identity matrix) with your calibrated values:
   ```yaml
   pixel_to_ground_homography: [
      0.00234, -0.00156,  1.24567,  # ← Use YOUR values
     -0.00089,  0.00298, -0.15432,
      0.00001, -0.00234,  1.00000
   ]
   ```

3. **Save** (Ctrl+O, Enter, Ctrl+X)

### Step 4: Test the Calibration

1. **Restart your lane following system:**
   ```bash
   # Kill existing launch (Ctrl+C in launch terminal)
   ros2 launch f1tenth_control vision_lane_follow.launch.py \
       car_name:=car1 \
       start_visualization:=true
   ```

2. **Check the published path coordinates:**
   ```bash
   ros2 topic echo /car1/lane_center_path --field poses[0].pose.position
   ```

3. **Verify reasonable values:**
   - `x` should be 0.5 - 2.0 (meters forward)
   - `y` should be -0.5 to +0.5 (meters lateral)
   - NOT 100s or 1000s!

4. **Test driving:**
   - Place car on lane
   - Enable autonomous mode
   - Vehicle should now follow the lane!

---

## Method 2: Quick Estimation (For Testing Only)

If you don't have time for proper calibration but want to test, here's a **rough approximation**:

### Assumptions
- Camera mounted ~0.15m high
- Tilted down ~20 degrees
- 1280x720 resolution
- FOV ~69° horizontal

### Quick Formula
For a camera at height `h` meters, tilt angle `θ` degrees, the approximate homography is:

```python
# Rough approximation - NOT ACCURATE!
import numpy as np
h = 0.15  # camera height in meters
focal_px = 640  # approximate focal length in pixels
scale = h / focal_px

H_approx = np.array([
    [scale,  0.0,    0.0],
    [0.0,    scale,  -h],
    [0.0,    0.0,    1.0]
])
```

This will give you **better** results than identity, but **NOT as good** as proper calibration.

**Resulting YAML (for h=0.15m, focal=640px):**
```yaml
pixel_to_ground_homography: [
   0.000234,  0.0,       0.0,
   0.0,       0.000234, -0.15,
   0.0,       0.0,       1.0
]
```

---

## Method 3: Full Camera Calibration (Most Accurate)

**Time:** 1-2 hours
**When to use:** Production systems, precise positioning needed

This uses OpenCV's full camera calibration with checkerboard patterns.

See: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

---

## Troubleshooting

### Issue: "No camera image received"
**Fix:**
```bash
ros2 topic list | grep image_raw
ros2 topic hz /car1/camera/color/image_raw
```
Ensure camera is publishing before running calibration.

### Issue: Homography values look weird (very large or very small)
**Causes:**
- Clicked points in wrong order
- Entered wrong measurements (cm instead of m?)
- Rectangle too small or too distorted

**Fix:** Re-run calibration, double-check:
- Click order: top-left, top-right, bottom-right, bottom-left
- Measurements in METERS
- Rectangle has clear 90° corners

### Issue: After calibration, path coordinates still look wrong
**Verify:**
```bash
ros2 topic echo /car1/lane_center_path --once
```

**Expected:**
- poses[0].pose.position.x: 0.5 to 2.0 (forward distance)
- poses[0].pose.position.y: -0.3 to +0.3 (lateral offset)

**If still wrong:**
- Double-check you copied ALL 9 matrix values correctly
- Ensure no extra spaces or missing commas in YAML
- Restart the lane_detector node

### Issue: Path looks mirrored or flipped
**Cause:** Coordinate system convention mismatch

**Fix:**
- Try swapping width signs in calibration
- Or adjust point click order

---

## Understanding the Matrix

The 3x3 homography matrix maps pixels → ground coordinates:

```
[x_ground]   [h11  h12  h13]   [x_pixel]
[y_ground] = [h21  h22  h23] × [y_pixel]
[   1    ]   [h31  h32  h33]   [   1   ]
```

**What each element does:**
- `h11, h22`: Scale factors (pixels to meters)
- `h12, h21`: Rotation/skew
- `h13, h23`: Translation
- `h31, h32`: Perspective correction
- `h33`: Normalization (usually 1.0)

**Identity matrix** means no transformation - pixel coords pass through unchanged!

---

## Validation Checklist

After calibration, verify:

- [ ] Debug image shows detected lane
- [ ] Path coordinates are in meters (0.5-2.0 range)
- [ ] Path shape matches physical lane in RViz
- [ ] Vehicle follows lane when driving
- [ ] Confidence > 0.35 when on lane
- [ ] Multiplexer uses vision path (not waypoints)

---

## Next Steps After Calibration

1. **Fine-tune pure pursuit lookahead:**
   - Edit `src/f1tenth_control/config/vision_pp.yaml`
   - Adjust `look_ahead` parameter (default: 0.35m)
   - Shorter = more responsive, longer = smoother

2. **Tune detection parameters** if needed:
   - HSV ranges for yellow detection
   - ROI (region of interest)
   - Sampling density

3. **Test in various conditions:**
   - Different lighting
   - Curves vs straight sections
   - Different lane widths

4. **Document your calibration values:**
   - Save matrix for your specific camera setup
   - Note camera height and tilt angle
   - Can reuse if camera doesn't move

---

## Quick Reference

**Calibration command:**
```bash
python3 calibrate_homography.py
```

**Config file location:**
```
src/f1tenth_control/config/lane_detector.yaml
```

**Test path coordinates:**
```bash
ros2 topic echo /car1/lane_center_path --field poses[0].pose.position
```

**Expected coordinate ranges:**
- X (forward): 0.5 to 2.0 meters
- Y (lateral): -0.5 to +0.5 meters

---

**Remember:** Identity matrix = broken! Calibrate first, then tune everything else.
