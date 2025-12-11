# Camera Lane Following Testing - Task Checklist

**Last Updated:** 2025-11-11

---

## Progress Overview

- **Phase 1**: ⏳ NOT STARTED - Environment Preparation
- **Phase 2**: ⏳ NOT STARTED - System Launch
- **Phase 3**: ⏳ NOT STARTED - Visualization Setup
- **Phase 4**: ⏳ NOT STARTED - Functional Validation
- **Phase 5**: ⏳ NOT STARTED - Performance Tuning
- **Phase 6**: ⏳ NOT STARTED - Documentation

---

## Phase 1: Environment Preparation ⏳ NOT STARTED

**Objective:** Clear conflicts and ensure exclusive robot access

**Estimated Time:** 15 minutes

### Task 1.1: Identify and Stop Conflicting Nodes
- [ ] Run `ros2 node list` to see all active nodes
- [ ] Check for `/dl_lane_detector` or other lane detection nodes
- [ ] Coordinate with team members using the robot
- [ ] Stop conflicting nodes (Ctrl+C in their terminal, or coordinate shutdown)
- [ ] Verify camera availability with `rs-enumerate-devices`
- [ ] Confirm no other processes using `/dev/video*`

**Acceptance:**
- No conflicting lane detector nodes in `ros2 node list`
- Camera device accessible
- Team coordination complete

### Task 1.2: Verify Prerequisites
- [ ] Navigate to workspace: `cd ~/CR7_ws/f1_tenth_cr7_ws`
- [ ] Source ROS2 setup: `source install/setup.bash`
- [ ] Verify RealSense camera detected: `rs-enumerate-devices | grep D435`
- [ ] Check motion capture odometry available: `ros2 topic list | grep odom`
- [ ] Verify USB cable secure and connected

**Acceptance:**
- Workspace sourced successfully
- RealSense D435i appears in device list
- Odometry topic exists (from motion capture)

---

## Phase 2: System Launch ⏳ NOT STARTED

**Objective:** Start vision lane following stack and validate initialization

**Estimated Time:** 10 minutes

### Task 2.1: Launch Vision Lane Following
- [ ] Open dedicated terminal for launch
- [ ] Execute launch command:
  ```bash
  ros2 launch f1tenth_control vision_lane_follow.launch.py \
    car_name:=car1 \
    start_cam:=true \
    start_visualization:=true
  ```
- [ ] Watch terminal output for initialization messages
- [ ] Verify no error messages appear
- [ ] Keep terminal open and visible for monitoring

**Acceptance:**
- Launch completes without errors
- All nodes report "initialized" or "ready"
- No Python exceptions or stack traces

### Task 2.2: Verify Node Status
- [ ] Open new terminal (keep launch terminal running)
- [ ] Source workspace: `cd ~/CR7_ws/f1_tenth_cr7_ws && source install/setup.bash`
- [ ] List running nodes: `ros2 node list`
- [ ] Confirm `/lane_detector` is running
- [ ] Confirm `/lane_path_mux` is running
- [ ] Confirm `/pure_pursuit_control` is running
- [ ] Confirm `/car1/D435i` is running (camera)
- [ ] Confirm `/motion_capture_tracking` is running

**Acceptance:**
- All 5 critical nodes appear in node list
- No unexpected nodes present
- Launch terminal shows no new errors

### Task 2.3: Verify Topic Publication
- [ ] Check topics exist: `ros2 topic list | grep -E "(image|lane|path)"`
- [ ] Verify camera image: `ros2 topic hz /car1/camera/color/image_raw`
  - Should be ~30 Hz
- [ ] Check debug image: `ros2 topic hz /lane_detector/debug_image`
  - Should match camera rate
- [ ] Check lane path: `ros2 topic hz /lane_center_path`
  - May be 10-20 Hz or lower if no lane visible
- [ ] Check selected path: `ros2 topic hz /selected_path`
  - Should be 20 Hz (multiplexer timer rate)

**Acceptance:**
- `/car1/camera/color/image_raw` publishing at ~30 Hz
- `/lane_detector/debug_image` publishing (rate depends on camera)
- `/selected_path` publishing at 20 Hz
- All expected topics present

---

## Phase 3: Visualization Setup ⏳ NOT STARTED

**Objective:** Configure visual feedback for lane detection monitoring

**Estimated Time:** 15 minutes

### Task 3.1: Verify Raw Camera Feed
- [ ] Open new terminal, source workspace
- [ ] Launch image viewer: `ros2 run rqt_image_view rqt_image_view /car1/camera/color/image_raw`
- [ ] Verify image appears and updates smoothly
- [ ] Check image quality (exposure, focus, clarity)
- [ ] Point camera at test area with yellow lane
- [ ] Verify lane is visible in image

**Acceptance:**
- Live camera feed displays in rqt_image_view
- Image updates at ~30fps (smooth, no stuttering)
- Yellow lane clearly visible when camera pointed at it
- Image well-exposed (not too dark or blown out)

### Task 3.2: View Lane Detection Debug Image
- [ ] Open new terminal, source workspace
- [ ] Launch debug viewer: `ros2 run rqt_image_view rqt_image_view /lane_detector/debug_image`
- [ ] Point camera at yellow lane
- [ ] Observe colored mask overlay on image
- [ ] Look for red circles marking detected lane points
- [ ] Count number of red circles (should be up to 12)
- [ ] Verify circles follow lane centerline from bottom to top

**Acceptance:**
- Debug image displays with lane overlay
- Red circles appear when camera sees lane
- Circles positioned along lane centerline (roughly)
- Mask overlay highlights yellow lane pixels
- Image updates in real-time as camera moves

### Task 3.3: Configure RViz Path Visualization
- [ ] Verify RViz launched automatically (check windows)
- [ ] If not, launch manually: `ros2 run rviz2 rviz2`
- [ ] In RViz, set "Fixed Frame" to `odom` (top left, Global Options)
- [ ] Click "Add" button (bottom left)
- [ ] Add lane center path:
  - [ ] By topic → `/lane_center_path` → Path → OK
  - [ ] Set color to cyan (0, 255, 255)
  - [ ] Set Line Width to 0.02
- [ ] Add selected path:
  - [ ] By topic → `/selected_path` → Path → OK
  - [ ] Set color to yellow (255, 255, 0)
  - [ ] Set Line Width to 0.03
- [ ] Optional: Add debug image display
  - [ ] Add → By topic → `/lane_detector/debug_image` → Image
- [ ] Point camera at lane and observe paths in RViz

**Acceptance:**
- RViz displays coordinate grid
- Cyan path visible when camera sees lane
- Yellow path shows active path (may be waypoints initially)
- Paths update in real-time
- Fixed frame set correctly (no TF errors)

---

## Phase 4: Functional Validation ⏳ NOT STARTED

**Objective:** Verify system responds correctly to lane presence/absence

**Estimated Time:** 20 minutes

### Task 4.1: Monitor Lane Confidence
- [ ] Open new terminal, source workspace
- [ ] Start confidence monitor: `ros2 topic echo /lane_confidence`
- [ ] Point camera directly at yellow lane
- [ ] Record confidence value: _____ (should be >0.35)
- [ ] Move camera away from lane
- [ ] Record confidence value: _____ (should be <0.20)
- [ ] Partially obscure lane (cover half)
- [ ] Observe confidence decrease
- [ ] Return to full lane view
- [ ] Observe confidence increase

**Acceptance:**
- On-lane confidence consistently >0.35 (ideally >0.5)
- Off-lane confidence <0.20
- Confidence changes smoothly with lane visibility
- No erratic jumping between high/low values

**Observations:**
- Typical on-lane confidence: _____
- Typical off-lane confidence: _____
- Transition speed (fast/medium/slow): _____

### Task 4.2: Verify Path Multiplexer Behavior
- [ ] Open two terminals, source both
- [ ] Terminal 1: Monitor vision status: `ros2 topic echo /vision_path_active`
- [ ] Terminal 2: Monitor confidence: `ros2 topic echo /lane_confidence`
- [ ] Point camera at lane with confidence >0.35
- [ ] Verify `vision_path_active: true` in Terminal 1
- [ ] Point camera away (confidence <0.35)
- [ ] Verify `vision_path_active: false` in Terminal 1
- [ ] Hover camera at edge of lane (confidence ~0.35)
- [ ] Observe switching behavior
- [ ] Cover camera completely for 1 second
- [ ] Verify switches to waypoints after 250ms timeout

**Acceptance:**
- Vision active = true when confidence ≥0.35 and fresh path
- Vision active = false when confidence <0.35 or path stale
- Switching occurs reliably within 1 frame (50ms)
- Path timeout works correctly after 250ms

**Observations:**
- Switching lag time: _____ ms
- Hysteresis observed: yes/no
- Timeout behavior: correct/incorrect

### Task 4.3: Validate Pure Pursuit Response
- [ ] Open new terminal, source workspace
- [ ] Monitor drive commands: `ros2 topic echo /drive`
- [ ] Point camera at straight lane section
- [ ] Observe `steering_angle` (should be near 0)
- [ ] Point camera at curved section (if available)
- [ ] Observe `steering_angle` increase
- [ ] Point camera away from lane completely
- [ ] Observe behavior (should use waypoint path or stop)
- [ ] Record typical values:
  - Speed on good path: _____ m/s
  - Steering on straight: _____ rad
  - Steering on curve: _____ rad

**Acceptance:**
- Drive commands publishing continuously
- Steering responds to path curvature
- Speed appropriate (0.4 - 1.0 m/s range)
- No extreme/erratic steering commands
- System safe when no path available

**⚠️ SAFETY NOTE:** This is stationary testing only. Do NOT drive the car without:
- Clear testing area
- Safety observer
- Emergency stop ready
- Validated waypoint fallback

---

## Phase 5: Performance Tuning ⏳ NOT STARTED

**Objective:** Optimize detection for current environment (Optional, do if time permits)

**Estimated Time:** 30-60 minutes

### Task 5.1: Analyze Detection Quality
- [ ] Open `/lane_detector/debug_image` viewer
- [ ] Test various scenarios:
  - [ ] Lane in bright light: confidence = _____
  - [ ] Lane in shadow: confidence = _____
  - [ ] Lane with reflections: confidence = _____
  - [ ] Worn/faded lane: confidence = _____
- [ ] Evaluate mask quality:
  - [ ] All lane pixels captured? yes/no
  - [ ] False positives from other objects? yes/no
  - [ ] Points well-distributed bottom to top? yes/no
- [ ] Identify issues:
  - [ ] Under-detection (missing lane pixels)
  - [ ] Over-detection (non-lane objects included)
  - [ ] Noisy points (scattered, not smooth)
  - [ ] Off-center points (not on lane middle)

**Acceptance:**
- Detection performance documented for 4+ scenarios
- Issues identified with specific symptoms
- Baseline confidence values recorded

### Task 5.2: Tune HSV Color Parameters (If Needed)
**Skip if Task 5.1 shows good performance**

- [ ] Note current HSV values from `src/f1tenth_control/config/lane_detector.yaml`:
  - hsv_lower: [___, ___, ___]
  - hsv_upper: [___, ___, ___]
  - lab_b_min: _____
- [ ] Identify tuning need:
  - [ ] Under-detection → widen ranges
  - [ ] Over-detection → narrow ranges
  - [ ] Shadow issues → lower value minimum
  - [ ] Saturation issues → adjust saturation bounds
- [ ] Make incremental changes to config file
- [ ] Restart system to test changes
- [ ] Document changes in `parameter-tuning-log.md`:
  ```markdown
  ## 2025-11-11 - [Your Name]
  - Issue: [describe problem]
  - Change: hsv_lower [15,60,70] → [new values]
  - Result: confidence improved from X to Y
  ```
- [ ] Repeat until satisfactory performance

**Acceptance:**
- Parameters tuned for target environment
- All changes documented with rationale
- Confidence meets target thresholds
- False positive rate acceptable

### Task 5.3: Adjust ROI and Sampling (If Needed)
**Skip if detection quality is good**

- [ ] Note current ROI from config:
  - roi_row_start: _____
  - roi_row_end: _____
  - row_step: _____
  - max_rows: _____
- [ ] Identify adjustment need:
  - [ ] Need longer lookahead → decrease roi_row_start
  - [ ] Too much non-lane area → tighten ROI vertically
  - [ ] Need more path points → decrease row_step, increase max_rows
  - [ ] Performance issues → increase row_step (fewer samples)
- [ ] Make changes to config file
- [ ] Restart and test
- [ ] Document changes and results

**Acceptance:**
- ROI appropriate for test environment
- Sampling density sufficient for smooth paths
- No performance degradation
- Changes documented

### Task 5.4: Homography Calibration (Advanced, Time Permitting)
**This is a multi-hour task - only start if time available**

**Prerequisites:**
- [ ] Calibration checkerboard pattern printed
- [ ] Measuring tape or known distance markers
- [ ] OpenCV calibration tools or scripts available

**Steps:**
- [ ] Place checkerboard on ground in camera view
- [ ] Capture calibration images (20+ poses)
- [ ] Run OpenCV camera calibration
- [ ] Measure camera height and tilt angle
- [ ] Compute homography matrix
- [ ] Update `pixel_to_ground_homography` in config
- [ ] Test with real lane: verify distances in RViz match physical
- [ ] Document calibration procedure and results

**Acceptance:**
- Homography matrix computed from calibration
- Paths in RViz align with physical lane geometry
- Distances along path accurate to within 10%
- Calibration procedure documented for future use

**Note:** If time is limited, defer this to a dedicated calibration session.

---

## Phase 6: Documentation ⏳ NOT STARTED

**Objective:** Create reusable documentation for team

**Estimated Time:** 20 minutes

### Task 6.1: Create Quick Start Guide
- [ ] Create file: `dev/active/camera-lane-following-testing/QUICK_START.md`
- [ ] Include sections:
  - [ ] Pre-launch checklist (stop conflicts, verify camera)
  - [ ] Launch command with all parameters
  - [ ] Verification steps (check nodes, topics)
  - [ ] Visualization commands (rqt_image_view, RViz)
  - [ ] Expected behavior summary
  - [ ] Common errors and quick fixes
- [ ] Test guide: have team member follow it independently
- [ ] Revise based on feedback

**Acceptance:**
- Quick start guide exists and is readable
- Team member can follow it successfully
- Covers all essential steps from Phases 1-4
- Formatted with clear commands and examples

### Task 6.2: Document Testing Observations
- [ ] Update `camera-lane-following-testing-context.md`:
  - [ ] Fill in "Performance Baseline" section with measured values
  - [ ] Complete "Lessons Learned" section with insights
  - [ ] Update "Testing Environments" with actual lab details
  - [ ] Add any parameter changes to "Important Decisions" section
- [ ] Include:
  - [ ] Optimal HSV parameters found
  - [ ] Typical confidence values in lab conditions
  - [ ] System limitations discovered
  - [ ] Recommended improvements

**Acceptance:**
- Context file updated with real test data
- All TBD sections filled in with actual observations
- "Last Updated" date reflects current session
- SESSION PROGRESS section accurately reflects completion status

### Task 6.3: Create Troubleshooting Guide
- [ ] Create file: `dev/active/camera-lane-following-testing/TROUBLESHOOTING.md`
- [ ] Include sections:
  - [ ] **Camera Issues**: Not found, no image, wrong device
  - [ ] **Node Issues**: Not launching, crashing, missing topics
  - [ ] **Detection Issues**: No debug image, no red circles, low confidence
  - [ ] **Path Issues**: No path, wrong location, not following lane
  - [ ] **Multiplexer Issues**: Not switching, stuck in one mode
  - [ ] **Drive Issues**: No commands, wrong steering, safety concerns
- [ ] For each issue, provide:
  - Symptom description
  - Diagnostic commands
  - Root cause explanation
  - Step-by-step fix
  - Prevention tips
- [ ] Include decision tree diagram (if time permits)

**Acceptance:**
- Troubleshooting guide created
- Covers 10+ common issues encountered
- Each issue has clear diagnostic and fix
- Guide referenced in Quick Start for easy access

---

## Completion Criteria

### Minimum Viable Testing (Phase 1-4)
- [x] All Phase 1 tasks complete (environment ready)
- [x] All Phase 2 tasks complete (system launched and verified)
- [x] All Phase 3 tasks complete (visualization working)
- [x] All Phase 4 tasks complete (functional behavior validated)

**Outcome:** System tested and operational, ready for basic use

### Full Testing with Optimization (Phase 5)
- [x] Minimum Viable Testing complete
- [x] At least Task 5.1 complete (quality analyzed)
- [x] At least one tuning task complete (5.2, 5.3, or 5.4)

**Outcome:** System optimized for lab environment

### Complete Documentation (Phase 6)
- [x] All Phase 6 tasks complete
- [x] Quick Start guide usable by team
- [x] Troubleshooting guide comprehensive
- [x] Context file updated with findings

**Outcome:** Team can replicate testing independently

---

## Post-Session Checklist

After completing testing session:

- [ ] Update SESSION PROGRESS in `camera-lane-following-testing-context.md`
- [ ] Mark completed tasks in this file
- [ ] Note any blockers or issues encountered
- [ ] Commit parameter changes if made
- [ ] Share documentation with team (Slack, wiki, etc.)
- [ ] Schedule follow-up session if needed for incomplete phases
- [ ] Clean up: Stop all ROS2 nodes gracefully (Ctrl+C in launch terminal)

---

## Notes and Observations

_Use this space for quick notes during testing:_

**Session Date:** 2025-11-11

**Participants:** _____

**Key Findings:**
-

**Issues Encountered:**
-

**Next Steps:**
-

---

**End of Task Checklist**

_Remember: Check off tasks as you complete them, and update context.md frequently!_
