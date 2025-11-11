# Camera Lane Following Testing - Implementation Plan

**Last Updated:** 2025-11-11

---

## Executive Summary

This plan provides a systematic approach to testing the camera-enabled lane following system for the F1Tenth autonomous racing platform. The system uses an Intel RealSense D435i camera to detect yellow lane lines in real-time, generating navigation paths that are fed to a pure pursuit controller for autonomous lane keeping with automatic fallback to pre-recorded waypoints.

**Objectives:**
- Enable reliable testing of camera-based lane detection
- Visualize lane detection performance in real-time
- Validate path selection logic between vision and waypoints
- Tune detection parameters for optimal performance
- Document testing procedures for team collaboration

**Timeline Estimate:** 2-3 hours for initial setup and validation

---

## Current State Analysis

### System Architecture

The camera lane following stack consists of three core nodes:

1. **lane_detector_node** (`src/f1tenth_control/f1tenth_control/lane_detector_node.py:25`)
   - Subscribes to: `/car1/camera/color/image_raw`, `/car1/odom`, `/car1/camera/color/camera_info`
   - Publishes: `lane_center_path`, `lane_confidence`, `lane_detector/debug_image`
   - Detects yellow lanes using HSV+LAB color filtering
   - Projects 2D pixels to ground plane using homography
   - Transforms to world frame using odometry

2. **lane_path_mux** (`src/f1tenth_control/f1tenth_control/lane_path_mux.py:24`)
   - Subscribes to: `lane_center_path`, `lane_confidence`, `waypoints_path`
   - Publishes: `selected_path`, `vision_path_active`
   - Selects best path based on confidence (>0.35), freshness (<250ms), and point count (≥4)
   - Falls back to waypoints when vision unreliable

3. **pure_pursuit_control** (`src/f1tenth_control/f1tenth_control/pure_pursuit_control.py`)
   - Subscribes to: `selected_path`, `/car1/odom`
   - Publishes: `/drive` (AckermannDriveStamped)
   - Follows selected path using pure pursuit algorithm
   - Look-ahead: 0.35m, speeds: 0.4-1.0 m/s

### Supporting Infrastructure

- **RealSense D435i**: Provides 1280x720@30fps RGB images
- **Motion Capture System**: Provides precise odometry for localization
- **VESC Motor Controller**: Executes drive commands

### Current Challenges

1. **Multi-user Environment**: Multiple developers may run conflicting ROS2 nodes simultaneously
2. **Topic Namespace Conflicts**: Different lane detector nodes may publish to different topics
3. **Visualization Access**: Team members need clear guidance on viewing detection output
4. **Parameter Tuning**: HSV color ranges may need adjustment for lighting conditions
5. **Homography Calibration**: Currently using identity matrix, needs proper camera calibration

---

## Proposed Future State

### Immediate Goals (Session 1)

1. **Isolated Testing Environment**
   - Clear procedure for stopping conflicting nodes
   - Verified camera access and exclusivity
   - All expected nodes running correctly

2. **Real-time Visualization**
   - Debug image showing detected lane pixels
   - RViz displaying detected and selected paths
   - Confidence monitoring for quality assessment

3. **Validated System Operation**
   - Lane detection responding to physical lanes
   - Multiplexer switching between vision/waypoints correctly
   - Pure pursuit following selected path

### Long-term Goals (Future Sessions)

4. **Optimized Detection**
   - Calibrated homography matrix for accurate projection
   - Tuned HSV parameters for various lighting conditions
   - Improved confidence metrics

5. **Testing Documentation**
   - Standard operating procedures
   - Troubleshooting guide
   - Parameter tuning guide

---

## Implementation Phases

### Phase 1: Environment Preparation (15 minutes)

**Objective:** Ensure exclusive access to robot resources and clear any conflicts

#### Task 1.1: Identify and Stop Conflicting Nodes
**Effort:** S
**Dependencies:** None
**Acceptance Criteria:**
- No `/dl_lane_detector` or other conflicting lane detection nodes running
- Camera device not in use by other processes
- Motion capture node available

**Steps:**
1. List all running ROS2 nodes: `ros2 node list`
2. Identify conflicts (any other lane detectors, camera nodes)
3. Coordinate with team to stop conflicting processes
4. Verify camera availability: `rs-enumerate-devices`

**Commands:**
```bash
# Check for conflicting nodes
ros2 node list | grep -E "(lane|camera|detector)"

# Kill specific nodes if needed (coordinate with team first!)
# Option: Go to running terminal and Ctrl+C
# Option: pkill -9 -f "ros2 launch" (nuclear option)

# Verify camera access
rs-enumerate-devices
```

#### Task 1.2: Verify Prerequisites
**Effort:** S
**Dependencies:** Task 1.1
**Acceptance Criteria:**
- ROS2 workspace sourced correctly
- RealSense camera connected and detected
- Motion capture system providing odometry

**Steps:**
1. Navigate to workspace: `cd ~/CR7_ws/f1_tenth_cr7_ws`
2. Source setup: `source install/setup.bash`
3. Check camera: `rs-enumerate-devices` should show D435i
4. Verify mocap: Check if `/car1/odom` topic exists (from other sessions)

**Commands:**
```bash
cd ~/CR7_ws/f1_tenth_cr7_ws
source install/setup.bash

# Verify camera
rs-enumerate-devices | grep -i d435

# Check if odometry is available
ros2 topic list | grep odom
```

---

### Phase 2: System Launch (10 minutes)

**Objective:** Start the vision lane following stack and verify all nodes initialize correctly

#### Task 2.1: Launch Vision Lane Following
**Effort:** M
**Dependencies:** Phase 1 complete
**Acceptance Criteria:**
- Launch command executes without errors
- All expected nodes appear in node list
- No error messages in launch terminal output

**Launch Command:**
```bash
ros2 launch f1tenth_control vision_lane_follow.launch.py \
  car_name:=car1 \
  start_cam:=true \
  start_visualization:=true
```

**Expected Output:**
```
[INFO] [lane_detector]: Lane detector node initialized
[INFO] [lane_path_mux]: Lane path multiplexer ready
[INFO] [pure_pursuit_control]: Pure pursuit controller initialized
[INFO] [D435i]: RealSense node started
[INFO] [motion_capture_tracking]: Motion capture tracking ready
```

**Troubleshooting:**
- **"Camera not found"**: Camera disconnected or in use → reconnect, check conflicts
- **"No config file"**: Wrong workspace or missing install → rebuild workspace
- **Python errors**: Code syntax issues → check recent changes

#### Task 2.2: Verify Node Status
**Effort:** S
**Dependencies:** Task 2.1
**Acceptance Criteria:**
- `/lane_detector` node running
- `/lane_path_mux` node running
- `/pure_pursuit_control` node running
- `/car1/D435i` camera node running
- `/motion_capture_tracking` node running

**Commands:**
```bash
# Open NEW terminal (keep launch terminal running)
cd ~/CR7_ws/f1_tenth_cr7_ws
source install/setup.bash

# Check nodes
ros2 node list

# Expected nodes:
# /lane_detector
# /lane_path_mux
# /pure_pursuit_control
# /car1/D435i
# /motion_capture_tracking
# /world_to_map
# /joy_teleop
```

**If nodes missing:**
- Check launch terminal for errors
- Verify config files exist
- Check Python node executables are installed

#### Task 2.3: Verify Topic Publication
**Effort:** S
**Dependencies:** Task 2.2
**Acceptance Criteria:**
- Camera image publishing at ~30Hz
- Lane paths publishing at ~10-20Hz
- Confidence updating in real-time
- Selected path publishing at 20Hz

**Commands:**
```bash
# Check critical topics exist
ros2 topic list | grep -E "(image|lane|path)"

# Expected topics:
# /car1/camera/color/image_raw
# /lane_detector/debug_image
# /lane_center_path
# /lane_confidence
# /selected_path
# /vision_path_active
# /waypoints_path

# Check publication rates
ros2 topic hz /car1/camera/color/image_raw  # Should be ~30 Hz
ros2 topic hz /lane_detector/debug_image    # Should match camera
ros2 topic hz /lane_center_path             # 10-20 Hz
ros2 topic hz /selected_path                # 20 Hz
```

**If topics not publishing:**
- **No camera image**: Check camera connection, restart node
- **No debug image**: Check `publish_debug_image: true` in config
- **No lane path**: Check if camera is pointing at a lane

---

### Phase 3: Visualization Setup (15 minutes)

**Objective:** Configure visual feedback for lane detection performance monitoring

#### Task 3.1: Verify Raw Camera Feed
**Effort:** S
**Dependencies:** Task 2.3
**Acceptance Criteria:**
- rqt_image_view displays live camera feed
- Image is clear, properly exposed
- Frame rate is smooth (~30fps)

**Commands:**
```bash
# View raw camera feed
ros2 run rqt_image_view rqt_image_view /car1/camera/color/image_raw
```

**What to check:**
- Image appears and updates smoothly
- Exposure appropriate for environment
- Yellow lane visible in frame
- No significant motion blur

**Troubleshooting:**
- **No image**: Camera node not running → check Task 2.2
- **Black screen**: Camera permissions → check `rs-enumerate-devices`
- **Frozen frame**: Topic not updating → check `ros2 topic hz`

#### Task 3.2: View Lane Detection Debug Image
**Effort:** M
**Dependencies:** Task 3.1
**Acceptance Criteria:**
- Debug image displays camera view with lane overlay
- Red circles appear on detected lane points
- Lane mask overlay shows yellow detection regions
- Image updates in real-time

**Commands:**
```bash
# View debug image with lane detection overlay
ros2 run rqt_image_view rqt_image_view /lane_detector/debug_image
```

**What to look for:**
- **Red circles**: Mark sampled lane center points (should be ~12 points max)
- **Colored overlay**: Shows where yellow pixels detected (blended with original)
- **Distribution**: Points should follow lane centerline from bottom to top

**Troubleshooting:**
- **No image**: lane_detector not publishing → check node status
- **No red circles**: No lane detected → adjust camera angle or parameters
- **Circles not on lane**: HSV tuning needed or homography incorrect
- **Very few points**: Increase ROI size or adjust color thresholds

#### Task 3.3: Configure RViz Path Visualization
**Effort:** M
**Dependencies:** Task 2.3
**Acceptance Criteria:**
- RViz displays detected lane path
- Selected path visible and updating
- Vision active status can be monitored
- Coordinate frames properly aligned

**RViz Setup:**

1. **RViz should auto-launch** if `start_visualization:=true` was used

2. **Add Lane Center Path Display:**
   - Click "Add" button (bottom left)
   - Select "By topic"
   - Find `/lane_center_path` → Select "Path"
   - Click "OK"
   - In left panel, expand "Path" display
   - Change "Color" to cyan (0, 255, 255)
   - Set "Line Style" → "Lines" or "Billboards"
   - Set "Line Width" to 0.02

3. **Add Selected Path Display:**
   - Add → By topic → `/selected_path` → Path
   - Color: Yellow (255, 255, 0)
   - Line Width: 0.03

4. **Add Debug Image to RViz (Optional):**
   - Add → By topic → `/lane_detector/debug_image` → Image
   - Resize panel as needed

5. **Set Fixed Frame:**
   - In "Global Options", set "Fixed Frame" to `odom` or `world`

**What you should see:**
- **Cyan path**: Detected lane centerline from camera
- **Yellow path**: Active path being followed (switches between lane/waypoints)
- **Path orientation**: Should align with physical lane direction

**Troubleshooting:**
- **No path visible**: Check if topics publishing, verify frame_id matches fixed frame
- **Path in wrong location**: Odometry or homography issue
- **Path jumpy**: Normal if confidence fluctuates, may need smoothing

---

### Phase 4: Functional Validation (20 minutes)

**Objective:** Verify system responds correctly to lane visibility and operates as designed

#### Task 4.1: Monitor Lane Confidence
**Effort:** S
**Dependencies:** Phase 3 complete
**Acceptance Criteria:**
- Confidence value updates in real-time
- Pointing at lane increases confidence
- Moving away from lane decreases confidence
- Threshold behavior at 0.35 is observable

**Commands:**
```bash
# Monitor confidence in real-time
ros2 topic echo /lane_confidence
```

**Testing procedure:**
1. Point camera at yellow lane → confidence should rise (>0.35 ideal)
2. Move camera away from lane → confidence should drop
3. Partially obscure lane → confidence decreases proportionally
4. Return to lane → confidence recovers

**Expected behavior:**
- **On lane**: Confidence 0.4 - 0.8 (depends on lane quality, lighting)
- **Off lane**: Confidence < 0.2
- **Threshold**: 0.35 is the multiplexer decision point

**Record observations:**
- Typical on-lane confidence: ___
- Confidence drop rate when leaving lane: ___
- Recovery time when returning to lane: ___

#### Task 4.2: Verify Path Multiplexer Behavior
**Effort:** M
**Dependencies:** Task 4.1
**Acceptance Criteria:**
- System uses vision path when confidence > 0.35
- Falls back to waypoints when confidence < 0.35
- Transition is smooth and logged
- `vision_path_active` boolean reflects actual state

**Commands:**
```bash
# Terminal 1: Monitor vision active status
ros2 topic echo /vision_path_active

# Terminal 2: Monitor confidence
ros2 topic echo /lane_confidence

# Terminal 3: Monitor path switches (advanced)
ros2 topic echo /selected_path --field header.stamp
```

**Testing procedure:**
1. **Vision mode**: Point at lane, verify `vision_path_active: true`
2. **Waypoint mode**: Point away, verify `vision_path_active: false`
3. **Hysteresis test**: Hover around 0.35 confidence, observe switching
4. **Freshness test**: Cover camera, path should timeout after 250ms

**Switching logic verification:**
- Vision active requires: confidence ≥ 0.35 AND points ≥ 4 AND age ≤ 250ms
- Otherwise: Falls back to waypoints

**Expected behavior:**
```
# When pointing at lane:
vision_path_active: true
lane_confidence: 0.52

# When pointing away:
vision_path_active: false
lane_confidence: 0.12
```

#### Task 4.3: Validate Pure Pursuit Response
**Effort:** M
**Dependencies:** Task 4.2
**Acceptance Criteria:**
- Drive commands generated when path available
- Steering responds to path curvature
- Speed adjusts based on path geometry
- No commands when no valid path

**Commands:**
```bash
# Monitor drive commands
ros2 topic echo /drive
```

**Testing procedure (stationary):**
1. Point camera at straight lane → expect small steering angles
2. Point at curved section → expect larger steering angles
3. Remove all paths → drive commands should stop or maintain safe defaults

**Safety note:** This is stationary testing. Actual motion testing requires:
- Clear testing area
- Safety observer
- Emergency stop ready
- Validated waypoint fallback path

**Expected /drive output:**
```yaml
drive:
  speed: 1.0           # fast_speed when path is good
  steering_angle: 0.12  # varies with path curvature
  # OR
  speed: 0.4           # slow_speed when uncertain
```

---

### Phase 5: Performance Tuning (30-60 minutes)

**Objective:** Optimize detection parameters for current environment and lane characteristics

#### Task 5.1: Analyze Detection Quality
**Effort:** M
**Dependencies:** Phase 4 complete
**Acceptance Criteria:**
- Lane pixels correctly identified in various lighting
- Minimal false positives from non-lane yellow objects
- Consistent detection across lane distance (near to far)
- Red sampling points well-distributed along lane

**Analysis procedure:**

1. **Visual inspection** of `/lane_detector/debug_image`:
   - Are all yellow lane pixels captured in mask?
   - Are non-lane yellow objects rejected?
   - Do red circles follow lane centerline accurately?
   - Are points sampled evenly from bottom to top?

2. **Quantitative measurement**:
   ```bash
   # Record confidence for various conditions
   # On clean lane in good light: _____
   # On worn lane: _____
   # With shadows: _____
   # With reflections/glare: _____
   ```

3. **Common issues:**
   - **Under-detection**: Increase HSV upper bounds, decrease lab_b_min
   - **Over-detection**: Tighten HSV ranges, increase min_row_pixels
   - **Noisy points**: Increase morph_kernel_size, smoothing_window
   - **Points off-center**: Need homography calibration

#### Task 5.2: Tune HSV Color Parameters (If Needed)
**Effort:** L
**Dependencies:** Task 5.1
**Acceptance Criteria:**
- Yellow lane reliably detected in target conditions
- Non-yellow objects rejected
- Parameter changes documented in config file

**Current parameters** (`src/f1tenth_control/config/lane_detector.yaml:14-16`):
```yaml
hsv_lower: [15, 60, 70]   # [Hue, Saturation, Value]
hsv_upper: [40, 255, 255]
lab_b_min: 150             # LAB color space b-channel (yellow boost)
```

**Tuning guide:**

1. **Hue range [15-40]**: Defines yellow color
   - Too narrow → misses some yellow tones
   - Too wide → captures orange or green

2. **Saturation [60-255]**: Color purity
   - Low min → accepts washed-out/pale yellows
   - High min → requires vivid yellows only

3. **Value [70-255]**: Brightness
   - Low min → works in shadows
   - High min → requires well-lit areas

4. **LAB b-channel [150-255]**: Yellow reinforcement
   - High min (150) → very selective for yellow
   - Lower min → more permissive

**Iterative tuning:**
```bash
# Edit config file
nano src/f1tenth_control/config/lane_detector.yaml

# Restart lane_detector node (in launch terminal: Ctrl+C, then relaunch)
ros2 launch f1tenth_control vision_lane_follow.launch.py ...

# Test with debug image
ros2 run rqt_image_view rqt_image_view /lane_detector/debug_image
```

**Document changes:**
Create `dev/active/camera-lane-following-testing/parameter-tuning-log.md`:
```markdown
# Parameter Tuning Log

## 2025-11-11 - Initial Conditions
- Lighting: Indoor fluorescent
- Lane: Yellow tape on gray carpet
- Original HSV: [15,60,70] to [40,255,255]
- Result: Confidence ~0.52, some false positives from yellow posters

## Changes Made
- Increased saturation min: 60 → 80
- Result: False positives eliminated, confidence now 0.48
```

#### Task 5.3: Adjust ROI and Sampling Parameters (If Needed)
**Effort:** M
**Dependencies:** Task 5.1
**Acceptance Criteria:**
- ROI covers appropriate portion of image
- Sampling density sufficient for path accuracy
- No performance degradation from excessive sampling

**Current parameters** (`src/f1tenth_control/config/lane_detector.yaml:10-19`):
```yaml
roi_row_start: 220      # Start from row 220 (lower portion of image)
roi_row_end: 719        # End at row 719 (bottom of 720p image)
roi_col_min: 0          # Full width
roi_col_max: -1         # -1 means full width
row_step: 15            # Sample every 15 pixels vertically
max_rows: 12            # Maximum 12 sample rows
min_row_pixels: 40      # Need 40 yellow pixels to consider row valid
```

**ROI tuning:**
- `roi_row_start`: Lower value → sees farther ahead, but more distortion
- `roi_row_end`: Should be near image bottom for closest lane view
- **Recommendation**: Keep ROI focused on ground plane (bottom 60-70% of image)

**Sampling tuning:**
- `row_step`: Smaller → more points, higher accuracy, more computation
- `max_rows`: More rows → longer lookahead distance
- `min_row_pixels`: Higher → more strict, rejects sparse detections

**Typical adjustments:**
```yaml
# For longer lookahead:
roi_row_start: 180      # See farther
max_rows: 15            # More sample rows

# For higher accuracy:
row_step: 10            # Denser sampling
smoothing_window: 7     # More smoothing
```

#### Task 5.4: Homography Calibration (Advanced, Optional)
**Effort:** XL
**Dependencies:** Task 5.1-5.3
**Acceptance Criteria:**
- Pixel coordinates accurately map to ground plane meters
- Path position in world frame matches physical lane
- Distances along path are geometrically correct

**Current state:**
```yaml
pixel_to_ground_homography: [
  1.0, 0.0, 0.0,
  0.0, 1.0, 0.0,
  0.0, 0.0, 1.0
]
```
This is an **identity matrix** - no projection, just pixel coordinates!

**Why calibration matters:**
- Converts 2D image pixels → 3D ground plane coordinates
- Accounts for camera tilt, height, field-of-view
- Essential for accurate path distances and positions

**Calibration procedure** (requires additional tools):
1. Print calibration checkerboard pattern
2. Place on ground in camera view
3. Use OpenCV calibration tools to compute homography
4. Measure camera height and tilt angle
5. Compute perspective transform matrix
6. Update config with 9 matrix values

**Resources:**
- OpenCV camera calibration tutorial
- ROS camera_calibration package
- Sample homography for F1Tenth (if available from team)

**Note:** This is a complex task best done in a dedicated session. For initial testing, identity matrix may work if:
- Camera mounted nearly horizontal
- Only relative positions matter (not absolute distances)
- Path following robust to scale errors

---

### Phase 6: Documentation and Knowledge Transfer (20 minutes)

**Objective:** Create persistent documentation for team members to replicate testing

#### Task 6.1: Create Quick Start Guide
**Effort:** M
**Dependencies:** Phase 4 complete
**Acceptance Criteria:**
- Concise checklist for launching system
- Common troubleshooting issues covered
- Shared with team in accessible location

**Deliverable:** `dev/active/camera-lane-following-testing/QUICK_START.md`

**Contents:**
- Pre-launch checklist
- Launch commands
- Verification steps
- Visualization commands
- Common errors and fixes

#### Task 6.2: Document Testing Observations
**Effort:** S
**Dependencies:** Phase 5 complete
**Acceptance Criteria:**
- Parameter values for current environment documented
- Performance metrics recorded
- Known issues and workarounds listed

**Deliverable:** Update `camera-lane-following-testing-context.md`

**Record:**
- Optimal HSV parameters for lab environment
- Typical confidence values achieved
- Path accuracy observations
- System limitations discovered

#### Task 6.3: Create Troubleshooting Guide
**Effort:** M
**Dependencies:** All phases
**Acceptance Criteria:**
- Common errors documented with solutions
- Diagnostic commands provided
- Decision tree for debugging

**Deliverable:** `dev/active/camera-lane-following-testing/TROUBLESHOOTING.md`

**Sections:**
- Camera not found
- No lane detection
- Poor confidence
- Path not following lane
- Multiplexer not switching
- Drive commands incorrect

---

## Risk Assessment and Mitigation

### Risk 1: Multi-User Conflicts
**Probability:** High
**Impact:** High (blocks testing entirely)

**Mitigation:**
- Create team communication protocol for robot access
- Use shared calendar or Slack channel for scheduling
- Implement namespace isolation (future enhancement)
- Document conflict resolution procedure

**Contingency:**
- Test on different robot if available
- Use recorded bag files for development
- Simulate camera input for algorithm testing

### Risk 2: Poor Lane Detection in Real Conditions
**Probability:** Medium
**Impact:** High (system won't work in practice)

**Mitigation:**
- Test in multiple lighting conditions during Phase 5
- Create parameter profiles for different environments
- Consider adaptive thresholding algorithms (future)
- Have waypoint fallback well-tuned

**Contingency:**
- Operate in controlled lighting initially
- Add LED strip lighting to car for consistent illumination
- Explore alternative detection methods (deep learning)

### Risk 3: Uncalibrated Homography Causes Geometric Errors
**Probability:** High (currently using identity matrix)
**Impact:** Medium (paths may be distorted but still followable)

**Mitigation:**
- Plan dedicated calibration session (Phase 5, Task 5.4)
- Use relative path following initially (less sensitive to scale)
- Validate against known test track dimensions

**Contingency:**
- Operate at low speeds where errors are less critical
- Use pure vision relative to current position
- Rely on waypoint mode for precision tasks

### Risk 4: Camera Disconnection or Failure
**Probability:** Low
**Impact:** High (system stops working)

**Mitigation:**
- Secure USB cable with strain relief
- Monitor camera health with watchdog
- Automatic fallback to waypoints on camera loss

**Contingency:**
- Carry spare RealSense camera
- Implement automatic restart on camera reconnect
- Operate in waypoint-only mode

### Risk 5: Real-time Performance Issues
**Probability:** Low (Jetson Orin is powerful)
**Impact:** Medium (delayed responses, missed frames)

**Mitigation:**
- Profile node performance with `ros2 topic hz`
- Reduce image resolution if needed (currently 1280x720)
- Optimize ROI size to minimum necessary
- Disable depth/IMU streams (already done)

**Contingency:**
- Reduce camera framerate (30 → 15 fps)
- Simplify detection algorithm
- Use GPU acceleration for image processing

---

## Success Metrics

### Phase 1-3: Setup Success
- [ ] All nodes launch without errors
- [ ] All expected topics publishing at correct rates
- [ ] Debug image displays lane overlay in real-time
- [ ] RViz shows detected paths

### Phase 4: Functional Success
- [ ] Lane confidence > 0.35 when viewing lane
- [ ] Confidence < 0.20 when not viewing lane
- [ ] Multiplexer switches correctly based on confidence
- [ ] Drive commands generated with reasonable values
- [ ] System runs continuously for 5+ minutes without crashes

### Phase 5: Performance Success
- [ ] Lane detection consistent across 3+ lighting conditions
- [ ] False positive rate < 10% (subjective assessment)
- [ ] Path following accuracy: within 10cm of lane center (if testable)
- [ ] Confidence stable (no rapid fluctuations) during steady operation

### Phase 6: Documentation Success
- [ ] Team member can replicate setup from documentation alone
- [ ] Common issues documented with working solutions
- [ ] Parameter tuning guide enables others to optimize for new environments

---

## Required Resources and Dependencies

### Hardware
- [x] F1Tenth race car with VESC motor controller
- [x] Intel RealSense D435i camera (mounted on car)
- [x] Jetson Orin or equivalent compute platform
- [x] Motion capture system (Vicon/OptiTrack) for odometry
- [ ] Yellow lane markers (tape, paint, or track boundaries)
- [ ] Adequate lighting (fluorescent or natural light preferred)

### Software
- [x] ROS2 (Humble or later)
- [x] f1tenth_control package (contains lane following stack)
- [x] realsense-ros package
- [x] motion_capture_tracking package
- [x] OpenCV (for image processing)
- [x] cv_bridge (ROS-OpenCV interface)
- [ ] rqt_image_view (for visualization)
- [ ] RViz2 (for path visualization)

### Access and Permissions
- [ ] Physical access to F1Tenth robot
- [ ] USB camera device permissions (`/dev/video*`)
- [ ] Motion capture system network access
- [ ] Coordination with other team members

### Knowledge Prerequisites
- Basic ROS2 concepts (nodes, topics, launch files)
- Command-line terminal usage
- Understanding of coordinate frames (odom, camera, base_link)
- Basic computer vision concepts (helpful but not required)

---

## Timeline Estimates

**Optimistic (Experienced user, no issues): 1.5 hours**
- Phase 1: 10 min
- Phase 2: 5 min
- Phase 3: 10 min
- Phase 4: 15 min
- Phase 5: 30 min
- Phase 6: 20 min

**Realistic (First-time user, minor troubleshooting): 2.5 hours**
- Phase 1: 20 min (coordination with team, camera troubleshooting)
- Phase 2: 15 min (launch errors, config issues)
- Phase 3: 20 min (learning tools, RViz setup)
- Phase 4: 30 min (understanding behavior, multiple test scenarios)
- Phase 5: 45 min (iterative parameter tuning)
- Phase 6: 20 min (documentation)

**Pessimistic (Multiple issues, calibration needed): 4+ hours**
- Phase 1: 30 min (significant conflicts, hardware issues)
- Phase 2: 30 min (code bugs, missing dependencies)
- Phase 3: 30 min (visualization tool issues)
- Phase 4: 45 min (unexpected behavior, debugging)
- Phase 5: 90+ min (full homography calibration, extensive tuning)
- Phase 6: 30 min (detailed documentation of issues found)

**Recommendation:** Plan for 3 hours for first session, with option to continue in second session if calibration needed.

---

## Next Steps After This Plan

1. **Session 1**: Execute Phases 1-4, validate basic functionality
2. **Session 2**: Complete Phase 5 tuning for current environment
3. **Session 3** (optional): Advanced calibration, alternative lighting, stress testing
4. **Future Work**:
   - Deep learning-based lane detection for robustness
   - Multi-lane handling for passing/overtaking
   - Dynamic obstacle avoidance integration
   - Closed-loop control validation on physical track

---

## References

**Code Files:**
- Launch: `src/f1tenth_control/launch/vision_lane_follow.launch.py`
- Lane Detector: `src/f1tenth_control/f1tenth_control/lane_detector_node.py`
- Multiplexer: `src/f1tenth_control/f1tenth_control/lane_path_mux.py`
- Config: `src/f1tenth_control/config/lane_detector.yaml`
- Pure Pursuit Config: `src/f1tenth_control/config/vision_pp.yaml`

**Documentation:**
- This Plan: `dev/active/camera-lane-following-testing/camera-lane-following-testing-plan.md`
- Context: `dev/active/camera-lane-following-testing/camera-lane-following-testing-context.md`
- Tasks: `dev/active/camera-lane-following-testing/camera-lane-following-testing-tasks.md`

**External Resources:**
- RealSense ROS: https://github.com/IntelRealSense/realsense-ros
- OpenCV Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Pure Pursuit: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
