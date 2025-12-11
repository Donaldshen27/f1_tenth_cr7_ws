# Camera Lane Following Testing - Context

**Last Updated:** 2025-11-11

---

## SESSION PROGRESS (2025-11-11)

### ✅ COMPLETED
- Created comprehensive testing plan with 6 phases
- Documented system architecture and data flow
- Identified current challenges (multi-user conflicts, visualization access)
- Created dev docs structure for knowledge persistence

### 🟡 IN PROGRESS
- Initial testing session not yet started
- Awaiting coordination with team to clear conflicting nodes

### ⚠️ BLOCKERS
- Colleague running `/dl_lane_detector` node on same robot
  - Publishes to different topics: `/lane_debug`, `/lane_waypoints`
  - Blocks testing of our `vision_lane_follow.launch.py` system
  - **Resolution**: Coordinate testing time slots with team

### 📋 NEXT ACTIONS
1. Schedule exclusive robot access with team
2. Execute Phase 1: Stop conflicting nodes
3. Launch vision_lane_follow.launch.py
4. Begin visualization and validation (Phases 2-4)

---

## System Overview

### Architecture Summary

The camera lane following system is a **three-node pipeline** that enables autonomous lane keeping with intelligent fallback:

```
Camera (RGB) → Lane Detector → Multiplexer → Pure Pursuit → Drive Commands
                     ↓              ↑
                 Confidence    Waypoints (fallback)
```

**Key Innovation:** Automatic switching between vision and waypoints based on detection quality, ensuring robustness.

---

## Key Files and Their Roles

### Launch Files

**`src/f1tenth_control/launch/vision_lane_follow.launch.py`**
- **Purpose**: Main entry point - launches entire stack
- **What it does**:
  - Starts lane_detector_node, lane_path_mux, pure_pursuit_control
  - Launches RealSense camera driver (D435i)
  - Starts motion capture node for odometry
  - Optional: RViz, LIDAR, teleop
- **Key parameters**:
  - `car_name`: Namespace (default: car1)
  - `start_cam`: Enable camera (default: true)
  - `start_visualization`: Launch RViz (default: false)
- **When to use**: Primary command for all testing

### Core Nodes

**`src/f1tenth_control/f1tenth_control/lane_detector_node.py`**
- **Purpose**: Computer vision pipeline for yellow lane detection
- **Line count**: 338 lines
- **Key methods**:
  - `image_callback()`: Main processing loop
  - `generate_mask()`: HSV + LAB color filtering
  - `extract_lane_points()`: Samples lane centerline
  - `transform_to_world_frame()`: Converts to odometry frame
  - `create_debug_overlay()`: Generates visualization
- **Subscribes to**:
  - `/car1/camera/color/image_raw` (sensor_msgs/Image)
  - `/car1/odom` (nav_msgs/Odometry)
- **Publishes**:
  - `lane_center_path` (nav_msgs/Path) - Detected lane
  - `lane_confidence` (std_msgs/Float32) - Quality metric
  - `lane_detector/debug_image` (sensor_msgs/Image) - Visualization
- **Critical for**: All vision-based functionality

**`src/f1tenth_control/f1tenth_control/lane_path_mux.py`**
- **Purpose**: Intelligent path selector with fallback logic
- **Line count**: 134 lines
- **Selection logic** (line 100-119):
  - Uses vision if: confidence ≥ 0.35 AND points ≥ 4 AND age ≤ 250ms
  - Otherwise: Falls back to static waypoints
- **Subscribes to**:
  - `lane_center_path` - Vision path
  - `lane_confidence` - Quality score
  - `waypoints_path` - Fallback path
- **Publishes**:
  - `selected_path` (nav_msgs/Path) - Active path for controller
  - `vision_path_active` (std_msgs/Bool) - Mode indicator
- **Runs at**: 20 Hz timer (configurable)
- **Critical for**: Robust autonomous operation

**`src/f1tenth_control/f1tenth_control/pure_pursuit_control.py`**
- **Purpose**: Path following controller
- **Algorithm**: Pure pursuit (lookahead-based steering)
- **Subscribes to**: `selected_path`, `/car1/odom`
- **Publishes**: `/drive` (ackermann_msgs/AckermannDriveStamped)
- **Note**: Not lane-following specific, works with any path

### Configuration Files

**`src/f1tenth_control/config/lane_detector.yaml`**
- **Purpose**: All vision processing parameters
- **Key sections**:
  - **Topics**: Input/output topic names
  - **ROI**: Region of interest (rows 220-719, full width)
  - **Color filtering**:
    - `hsv_lower: [15, 60, 70]` - Yellow lower bound
    - `hsv_upper: [40, 255, 255]` - Yellow upper bound
    - `lab_b_min: 150` - LAB color reinforcement
  - **Sampling**:
    - `row_step: 15` - Vertical spacing between samples
    - `max_rows: 12` - Maximum points to extract
    - `min_row_pixels: 40` - Minimum lane width
  - **Homography**: Pixel-to-ground transformation (currently identity)
  - **Debug**: `publish_debug_image: true`
- **Most likely to tune**: HSV ranges, ROI, sampling parameters

**`src/f1tenth_control/config/vision_pp.yaml`**
- **Purpose**: Pure pursuit controller parameters
- **Key settings**:
  - `look_ahead: 0.35` m - Lookahead distance
  - `wheelbase: 0.325` m - Car geometry
  - `fast_speed: 1.0` m/s, `slow_speed: 0.4` m/s
  - `dynamic_path_timeout: 0.25` s
  - `selected_path_topic: "selected_path"` - Input topic
- **When to edit**: Speed tuning, stability adjustments

**`src/f1tenth_control/config/camera.yaml`**
- **Purpose**: RealSense D435i configuration
- **Settings**:
  - Color: 1280x720 @ 30fps, RGB8
  - Depth: Disabled (not needed for 2D lane detection)
  - IMU: Disabled
- **When to edit**: Performance optimization (reduce resolution if needed)

---

## Important Decisions Made

### Design Decision 1: Multiplexer Architecture
**Decision**: Use separate multiplexer node instead of integrated logic in controller

**Rationale**:
- **Separation of concerns**: Vision quality assessment vs. path following
- **Reusability**: Pure pursuit controller remains generic
- **Testability**: Can test multiplexer logic independently
- **Flexibility**: Easy to add more path sources (e.g., planner, remote control)

**Trade-off**: Extra node overhead (~20ms latency), but negligible for control loop

### Design Decision 2: HSV + LAB Color Filtering
**Decision**: Combine HSV and LAB color spaces for yellow detection

**Rationale**:
- **HSV alone**: Sensitive to lighting variations
- **LAB b-channel**: Robust yellow indicator (positive b = yellow/blue axis)
- **Combined**: Intersection of both masks = high precision
- **Simple**: No ML training required, real-time on CPU

**Trade-off**: Fixed thresholds may need tuning per environment

### Design Decision 3: Bottom-Up Lane Sampling
**Decision**: Sample lane from bottom of image upward with fixed row spacing

**Rationale**:
- **Closest data first**: Bottom of image = closest to car = most reliable
- **Fixed spacing**: Consistent path density
- **Early termination**: Can stop after max_rows reached
- **Robust**: Works even if top of lane is occluded

**Trade-off**: Misses farther lookahead if lane curves out of view

### Design Decision 4: Identity Homography (Temporary)
**Decision**: Use identity matrix for pixel-to-ground transformation initially

**Rationale**:
- **Quick start**: Testing path following logic without calibration delay
- **Relative positioning**: Pure pursuit works with relative path shape
- **Deferred complexity**: Can calibrate later when needed

**Trade-off**: Absolute positions and distances are incorrect, scale is off

**Future work**: Proper camera calibration for geometric accuracy

### Design Decision 5: Confidence-Based Fallback
**Decision**: Use coverage-based confidence (ratio of yellow pixels in ROI)

**Rationale**:
- **Simple metric**: Easy to compute and interpret
- **Direct indicator**: More yellow pixels = more confident detection
- **No ML needed**: Works immediately without training

**Trade-off**: Doesn't account for lane geometry quality (e.g., straight vs. curved)

**Future enhancement**: Multi-factor confidence (coverage + consistency + geometry)

---

## Technical Constraints

### Hardware Limitations
1. **Camera FOV**: RealSense D435i has ~69° HFOV
   - Can't see very sharp turns until already entering them
   - Lookahead limited to ~1-2 meters depending on mount height

2. **Motion Capture Range**: Robot must stay within mocap coverage
   - Odometry degrades if tracking is lost
   - Limits testing area to mocap-equipped zone

3. **Compute**: Jetson Orin (sufficient, but consider profiling)
   - 1280x720 @ 30fps processing
   - Multiple ROS2 nodes running
   - Should be fine, but monitor CPU usage if adding features

### Software Constraints
1. **Frame Synchronization**: Lane detector uses latest odometry
   - If odometry stops, lane detection pauses (by design, line 117-121)
   - Ensures transforms are valid

2. **Topic Namespaces**: Camera topics under `/car1/` namespace
   - Multiplexer and detector use global namespace
   - Mixed pattern, works but slightly inconsistent

3. **No Persistence**: All state resets on node restart
   - No learned parameters or history
   - Must re-detect lane from scratch each launch

### Algorithmic Constraints
1. **Yellow Lane Only**: Hardcoded for yellow detection
   - Won't work with white, red, or other lane colors
   - Would need separate color profiles for each

2. **Single Lane**: Detects one centerline
   - Doesn't distinguish left/right boundaries
   - Can't handle multi-lane scenarios

3. **Ground Plane Assumption**: Assumes flat ground
   - Hills or ramps would violate homography assumptions
   - Would need 3D mapping for non-flat terrain

---

## Known Issues and Workarounds

### Issue 1: Conflicting Nodes in Multi-User Environment
**Symptom**: Multiple lane detector nodes running simultaneously
- Example: `/dl_lane_detector` vs. `/lane_detector`
- Publish to different topics, use same camera

**Impact**: Can't test own system, camera may be locked

**Workaround**: Coordinate with team, schedule exclusive access

**Future solution**: Implement namespace isolation, or use ROS2 daemon separation

### Issue 2: Identity Homography Causes Distorted Paths
**Symptom**: Paths in RViz don't align with physical lane geometry

**Impact**: Absolute positions wrong, but relative shape preserved

**Workaround**: Rely on relative path following, don't trust absolute coordinates

**Future solution**: Complete camera calibration (see plan Phase 5, Task 5.4)

### Issue 3: No Debug Image When Camera Not Found
**Symptom**: `rqt_image_view /lane_detector/debug_image` shows nothing

**Possible causes**:
1. `/lane_detector` node not running → check `ros2 node list`
2. Camera not publishing → check `/car1/camera/color/image_raw`
3. Odometry missing → node paused (check launch terminal logs)

**Diagnostic**:
```bash
ros2 node list | grep lane
ros2 topic hz /car1/camera/color/image_raw
ros2 topic echo /lane_detector/debug_image --once
```

**Fix**: See troubleshooting guide (to be created in Phase 6)

### Issue 4: Confidence Too Low in Good Lighting
**Symptom**: Viewing clear yellow lane but confidence < 0.35

**Possible causes**:
1. HSV thresholds too strict for your specific yellow
2. ROI too large (includes lots of non-lane area)
3. Lane too narrow (< 40 pixels minimum)

**Workaround**:
- Widen HSV ranges temporarily: `hsv_lower: [10, 40, 50]` to `hsv_upper: [45, 255, 255]`
- Reduce `min_row_pixels` to 20-30 for narrow lanes
- Tighten ROI to lane area only

**Proper solution**: Systematic tuning in Phase 5

---

## Quick Resume Instructions

### If Continuing This Session

1. **Check current state**:
   ```bash
   ros2 node list  # What's running?
   ros2 topic list | grep lane  # What topics exist?
   ```

2. **Identify blockers**:
   - Is colleague's node still running?
   - Is camera available?

3. **Resume from plan**:
   - Read `camera-lane-following-testing-plan.md`
   - Start at Phase 1 if not yet begun
   - Continue from current phase if in progress

### If Starting Fresh After Context Reset

1. **Read these files in order**:
   - `camera-lane-following-testing-context.md` (this file) - Current state
   - `camera-lane-following-testing-tasks.md` - What's done/pending
   - `camera-lane-following-testing-plan.md` - Detailed instructions

2. **Check SESSION PROGRESS** (top of this file) to see:
   - What's already completed
   - What's in progress
   - Any blockers

3. **Execute next phase** from plan

---

## Testing Environments

### Current Lab Setup
- **Location**: [Fill in actual lab name/room]
- **Lighting**: Fluorescent overhead (consistent)
- **Lane type**: Yellow tape on gray floor (assumed)
- **Lane width**: [Measure and fill in] cm
- **Motion capture**: [System name/type] providing odometry at [Hz] Hz

### Alternative Environments (Future)
- Outdoor track (natural lighting variations)
- Gymnasium (different flooring, echoes)
- Mixed lighting (windows + artificial)

---

## Team Coordination

### Current Testing Conflict
- **Colleague**: Running `/dl_lane_detector` (deep learning version?)
- **Their topics**: `/lane_debug`, `/lane_waypoints`
- **Our topics**: `/lane_detector/debug_image`, `/lane_center_path`

**Resolution strategy**:
1. Check team calendar for scheduled testing
2. Use Slack/chat to coordinate
3. Agree on time slots: Person A (9-11am), Person B (1-3pm), etc.

### Shared Resources
- **F1Tenth car**: One physical robot (shared)
- **RealSense camera**: Attached to shared robot
- **Motion capture**: Shared system (multiple objects trackable)

**Best practice**:
- Always check `ros2 node list` before launching
- Communicate when starting/stopping tests
- Don't kill other people's nodes without asking

---

## Performance Baseline (To Be Filled During Testing)

### Detection Quality
- **On-lane confidence**: ___ (target: >0.50)
- **Off-lane confidence**: ___ (target: <0.15)
- **Transition time**: ___ ms (lane to no-lane)

### Path Quality
- **Points per path**: ___ (max: 12)
- **Path update rate**: ___ Hz (expected: 10-20)
- **Smoothness**: [subjective: smooth / jittery / unstable]

### System Performance
- **Camera framerate**: ___ fps (target: 30)
- **Debug image rate**: ___ fps (should match camera)
- **Lane detector CPU**: ___% (measure with `top`)
- **Total system CPU**: ___% (all nodes combined)

---

## Dependencies for Testing

### Hardware
- [x] F1Tenth car with mounted RealSense D435i
- [x] Jetson Orin or compute platform
- [x] Motion capture system operational
- [ ] Yellow lane markers deployed in test area
- [ ] USB cable securely connected

### Software
- [x] ROS2 workspace built (`colcon build`)
- [x] RealSense SDK and ROS wrapper installed
- [x] Motion capture tracking package built
- [x] OpenCV and cv_bridge available
- [ ] rqt_image_view installed: `sudo apt install ros-humble-rqt-image-view`
- [ ] RViz2 installed: `sudo apt install ros-humble-rviz2`

### Access
- [ ] Physical access to lab
- [ ] Scheduled time slot (coordinate with team)
- [ ] SSH access to robot if needed
- [ ] Network connectivity to motion capture

---

## Lessons Learned (Updated During Testing)

_This section will be populated as testing progresses._

### What Worked Well
- TBD

### What Didn't Work
- TBD

### Unexpected Issues
- TBD

### Parameter Changes Made
- TBD

---

## Future Enhancements

### Short-term (Next Few Sessions)
1. Complete homography calibration for geometric accuracy
2. Create parameter profiles for different lighting conditions
3. Add logging for confidence, path quality metrics
4. Implement visualization dashboard (rqt plugin?)

### Medium-term (Next Few Weeks)
5. Multi-lane detection (left and right boundaries)
6. Adaptive thresholding for varying lighting
7. Integration with obstacle avoidance
8. Closed-loop testing on full track

### Long-term (Future Milestones)
9. Deep learning-based lane detection (more robust)
10. Semantic segmentation for complex environments
11. Multi-car coordination (passing, overtaking)
12. SLAM integration for GPS-denied operation

---

## Related Documentation

**In this project:**
- Plan: `dev/active/camera-lane-following-testing/camera-lane-following-testing-plan.md`
- Tasks: `dev/active/camera-lane-following-testing/camera-lane-following-testing-tasks.md`
- Quick Start: _(to be created in Phase 6)_
- Troubleshooting: _(to be created in Phase 6)_

**External resources:**
- F1Tenth docs: https://f1tenth.org/
- RealSense ROS: https://github.com/IntelRealSense/realsense-ros
- Pure Pursuit: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
- OpenCV color spaces: https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html

---

**End of Context Document**

_Remember to update SESSION PROGRESS section after each major milestone!_
