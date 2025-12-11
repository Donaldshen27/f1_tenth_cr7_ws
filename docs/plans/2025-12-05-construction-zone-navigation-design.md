# Construction Zone Navigation Design

**Date:** 2025-12-05
**Status:** Approved

## Overview

Add cone detection to enable construction zone navigation. The car follows yellow lanes normally, switches to cone-corridor following when entering a construction zone (two rows of orange cones), and returns to lane following upon exit.

## Architecture

```
Camera (RealSense D435i)
        ↓
   ┌────┴────┐
   ↓         ↓
[Lane Detector]  [Cone Detector]    ← Two parallel vision nodes
   ↓              ↓
lane_center_path  cone_corridor_path
lane_confidence   cone_confidence
   ↓              ↓
   └──────┬───────┘
          ↓
  [Construction Zone Mux]           ← New mux replaces lane_path_mux
          ↓
     selected_path
     zone_mode (LANE / CONE)
          ↓
  [Pure Pursuit Control]            ← Unchanged
          ↓
        drive
```

### Components

| Component | File | Purpose |
|-----------|------|---------|
| Cone Detector | `cone_detector_node.py` | Detects orange cones, classifies left/right, outputs corridor centerline |
| Construction Zone Mux | `construction_zone_mux.py` | Hysteresis-based mode switching between lane and cone following |
| Config | `cone_detector.yaml` | HSV tuning, contour filtering params |
| Config | `construction_zone_mux.yaml` | Transition thresholds |
| Launch | `vision_lane_follow.launch.py` | Updated to include new nodes |

## Cone Detector Node

**File:** `src/f1tenth_control/f1tenth_control/cone_detector_node.py`

### Subscriptions
- `/car1/D435i/color/image_raw` (Image) - Same camera feed as lane detector

### Publications
- `cone_corridor_path` (nav_msgs/Path) - Centerline between cone rows
- `cone_confidence` (std_msgs/Float32) - Detection quality (0.0-1.0)
- `cone_detector/debug_image` (Image) - Visualization with detected cones

### Processing Pipeline

1. **ROI Crop** - Same region as lane detector (bottom portion of image)

2. **Orange Color Mask** - HSV filtering for orange:
   - Lower: [5, 100, 100] (orange-red hue)
   - Upper: [25, 255, 255]

3. **Contour Detection** - Find blobs, filter by:
   - Minimum area (reject noise)
   - Aspect ratio ~0.5-2.0 (roughly cone-shaped)
   - Position in image (reject sky-level detections)

4. **Left/Right Classification** - Split detected cones by image x-coordinate:
   - Left of center → left_cones[]
   - Right of center → right_cones[]

5. **Corridor Centerline** - For each row (bottom to top):
   - Find closest left cone and right cone at similar y-level
   - Midpoint = (left_x + right_x) / 2
   - Project to ground plane using same homography as lane detector

6. **Confidence Calculation:**
   - High (0.8+): Multiple cone pairs detected
   - Medium (0.4-0.8): At least one cone pair
   - Low (<0.4): Cones on only one side or none

## Construction Zone Mux

**File:** `src/f1tenth_control/f1tenth_control/construction_zone_mux.py`

### Subscriptions
- `lane_center_path` (Path) - From lane detector
- `lane_confidence` (Float32) - From lane detector
- `cone_corridor_path` (Path) - From cone detector
- `cone_confidence` (Float32) - From cone detector

### Publications
- `selected_path` (Path) - Active path for pure pursuit
- `zone_mode` (std_msgs/String) - "LANE" or "CONE" for debugging

### State Machine

```
         ┌─────────────────────────────────┐
         ↓                                 │
   [LANE_FOLLOWING]                        │
         │                                 │
         │ cone_confidence > 0.4           │
         │ for 5+ consecutive frames       │
         ↓                                 │
   [CONE_CORRIDOR]                         │
         │                                 │
         │ cone_confidence < 0.2           │
         │ for 5+ consecutive frames       │
         │ AND lane_confidence > 0.3       │
         └─────────────────────────────────┘
```

### Logic
- **In LANE mode:** Publish `lane_center_path`, watch for cone entry
- **In CONE mode:** Publish `cone_corridor_path`, watch for cone exit + lane return
- **Safety:** If in CONE mode and lose both signals → stop (like current behavior)

## Configuration

### cone_detector.yaml

```yaml
# Orange cone HSV range
hsv_lower: [5, 100, 100]
hsv_upper: [25, 255, 255]

# Contour filtering
min_contour_area: 100        # pixels, reject noise
max_contour_area: 10000      # pixels, reject large blobs
min_aspect_ratio: 0.3
max_aspect_ratio: 3.0

# ROI (same as lane detector)
roi_row_start: 220
roi_row_end: 719

# Corridor computation
min_cones_per_side: 1        # minimum to compute corridor
max_cone_pair_distance: 200  # pixels, max lateral separation

# Uses same homography as lane detector
pixel_to_ground_homography: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
ground_x_offset: 0.6
```

### construction_zone_mux.yaml

```yaml
entry_threshold: 0.4
exit_threshold: 0.2
hysteresis_frames: 5
require_lane_on_exit: true
publish_rate_hz: 50.0
```

## Testing Strategy

### Phase 1: Cone Detection Only
- Run `cone_detector_node` standalone
- View `cone_detector/debug_image` in rqt_image_view
- Tune HSV range until orange cones are reliably masked
- Verify left/right classification with debug overlay

### Phase 2: Corridor Path Generation
- Place cones in a simple corridor (2 rows, 3-4 cones each)
- Verify `cone_corridor_path` publishes sensible waypoints
- Check path is centered between cone rows

### Phase 3: Mode Switching
- Run full system with lane + cones
- Test entry: drive toward cones on yellow lane, verify switch to CONE mode
- Test exit: drive through corridor, verify return to LANE mode
- Tune `hysteresis_frames` if switching is too slow/fast

### Phase 4: Integration
- Full construction zone course
- Entry → corridor → exit → back to lane
- Tune speeds if needed (maybe slower in cone corridor)

### Debug Topics
- `cone_detector/debug_image` - See detected cones, classifications
- `lane_detector/debug_image` - See lane detection
- `zone_mode` - Monitor current mode (LANE/CONE)

## Physical Setup

- **Cones:** Small orange cones (15-20cm tall)
- **Spacing:** ~50-80cm apart in each row
- **Corridor width:** ~40-60cm between rows
- **Detection method:** Color-based (HSV filtering) with shape validation

## Key Design Decisions

1. **Color-based detection** - Matches existing lane detector pipeline, fast on Jetson Orin
2. **Separate nodes** - Clean architecture, easier to test/debug independently
3. **Hysteresis-based transitions** - Prevents mode flickering, 5 frames (~170ms at 30fps)
4. **Same homography** - Both detectors use identical pixel-to-ground projection
5. **Safety-first exit** - Must see lane before exiting cone mode
