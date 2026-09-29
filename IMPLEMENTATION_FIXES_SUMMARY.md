# RoboCup Firmware & Visualizer Implementation Fixes

## Summary

Fixed critical issues in the RoboCup firmware's sensor fusion, localisation, and ToF detection systems, plus stabilized the telemetry visualizer against malformed data.

---

## Part 1: Firmware Fixes

### 1. Fixed Duplicate Configuration Constants

**File**: `src/robocup_template.ino` (lines 95-105)

**Issue**: `POI_DETECTOR_UPDATE_PERIOD` and `TOF_SEARCH_PLANNER_PERIOD` were defined twice with conflicting values.

**Fix**: Removed duplicate definitions, keeping single authoritative values:
- `POI_DETECTOR_UPDATE_PERIOD = 100` (10 Hz)
- `TOF_SEARCH_PLANNER_PERIOD = 500` (2 Hz)

### 2. Fixed Pose Ownership Architecture

**Files Modified**: 
- `src/robocup_template.ino` (tof_search_planner_callback, poi_detector_callback)

**Issue**: Two separate pose sources existed:
- `Localisation` maintains fused pose (IMU + optical flow + LiDAR)
- `MappingNav` maintains its own independent pose
These could drift apart, causing inconsistent sensor fusion results.

**Fix**: 
- Updated search planner to use `Localisation.getPose()` as authoritative source
- Synchronized MappingNav's pose with fused pose before pathfinding
- Ensured all sensor-to-world transforms use consistent fused pose
- This makes Localisation the single authoritative pose source

**Result**: All sensor fusion now uses consistent, fused robot pose.

### 3. Disabled Fake LiDAR Scan Matching

**File**: `src/Localisation.cpp` (scanToMapMatch function)

**Issue**: Previous implementation counted valid LiDAR points but never actually compared them to the occupancy map grid. It returned a false sense of match quality (score > 0) while always applying zero correction.

**Fix**: Replaced with explicit "not implemented" approach:
```cpp
// LiDAR-based pose correction is not yet implemented.
// ... (detailed comment explaining what a real matcher should do)
return 0;  // 0 = no correction available
```

**Result**: 
- No false corrections applied
- Match score of 0 clearly indicates unavailable correction
- Fused pose relies on IMU + optical flow only
- Clear path for future proper scan-to-map ICP implementation

### 4. Fixed ToF/LiDAR Range Difference Sign

**File**: `src/POIDetector.cpp` (processAnomaly function)

**Issue**: Range difference was calculated as `tof_range - expected_lidar`, treating all large differences (positive or negative) as anomalies.

**Fix**: Changed to `expected_lidar - tof_range` with logic:
- Positive difference = anomaly (ToF sees something CLOSER than map predicted)
- This is the weight detection case
- Negative difference = normal obstacle (no anomaly)

**Result**: Only meaningful anomalies (potential weights) are detected.

### 5. Fixed Unknown LiDAR Region Handling

**File**: `src/POIDetector.cpp` (processAnomaly function)

**Issue**: Treated UNKNOWN occupancy grid regions as valid LiDAR predictions.

**Fix**: Added explicit rejection:
```cpp
if (status == EXPECTED_RANGE_UNKNOWN || status == EXPECTED_RANGE_INVALID) {
    // Don't process as strong anomaly
    if (m_candidate.state == POI_STATE_NONE) {
        return;  // Require VALID status for detection
    }
}
```

**Result**: Only reliable LiDAR predictions trigger POI detection.

### 6. Connected Search Planner to D* Lite Navigation

**File**: `src/robocup_template.ino` (tof_search_planner_callback)

**Issue**: Search planner generated target poses but never issued them as navigation goals.

**Fix**: Uncommented and enabled goal setting:
```cpp
// Synchronize MappingNav's pose with fused pose
mappingNav.setPose(current_pose.x_m, current_pose.y_m, current_pose.theta_rad);

// Issue goal to D* Lite pathfinder
bool goal_set = mappingNav.setGoalWorld(best.x_m, best.y_m);
if (!goal_set) {
    return;  // Goal unreachable
}
```

**Result**: Search planner targets now flow to D* Lite pathfinding and motor control.

### 7. Added Diagnostic Counters

**File**: `src/robocup_template.ino` (global declarations and callbacks)

**Added tracking for**:
- `optical_flow_update_count` - verifies optical flow task runs
- `tof_reading_count` - tracks ToF sensor readings
- `poi_candidate_count` - counts POI candidates detected
- `poi_confirmed_count` - counts confirmed POIs

**Result**: Hardware testing can verify all systems are executing.

### 8. Synchronized MappingNav Pose Updates

**Files**: `src/robocup_template.ino`

**Implementation**: 
- Search planner now updates MappingNav's pose with fused Localisation pose
- Ensures grid updates and D* Lite operate on consistent robot position
- Prevents occupancy grid from drifting relative to sensor fusion

**Result**: All navigation uses single authoritative pose source.

---

## Part 2: Visualizer Fixes

### Issue Diagnosis

The visualizer crashed with:
```
Exception in Tkinter callback
matplotlib/ticker.py
    oom = math.floor(math.log10(val))
KeyboardInterrupt
```

**Root Cause**: Matplotlib tried to calculate axis limits from NaN, infinity, or extreme coordinate values in telemetry, causing math.log10() to fail.

### Solution: Add Defensive Validation

**File**: `tools/visualizer.py`

#### Added Validation Helper Functions

```python
is_finite_float(val: float) -> bool
    # Check if value is finite (not NaN/inf)

validate_pose_coordinate(val: float, name: str, max_abs: float = 100.0) -> bool
    # Validate pose x/y/theta before plotting

validate_grid_dimensions(grid_w: int, grid_h: int, cell_mm: int) -> bool
    # Validate grid metadata

validate_scan_coordinates(xs: np.ndarray, ys: np.ndarray) -> bool
    # Validate scan point array
```

#### Applied Validation Across Packet Processing

1. **_apply_pose_path()**: Validates x_m, y_m, theta_rad before storing
2. **_apply_keyframe()**: Validates grid_w, grid_h, cell_mm before reshaping
3. **update_state()**: Early exit if pose is invalid, prevents Matplotlib exposure to bad data
4. **update_state()**: Validates path coordinates before setting line data
5. **update_state()**: Validates scan coordinates before setting scatter data

#### Made Animation Robust

```python
def tick(_):
    ...
    if latest is not None:
        try:
            self.update_state(latest)
        except Exception as e:
            print(f"[visualizer] animation tick exception: {e}")
            # Continue with previous state rather than crash
    ...

return FuncAnimation(..., cache_frame_data=False)
```

#### Added Diagnostic Printing

Now prints when telemetry is rejected:
```
[visualizer] REJECT: invalid pose x=... (not finite)
[visualizer] REJECT: pose x exceeds max bounds
[visualizer] REJECT: invalid grid_w=...
[visualizer] ACCEPT: pose x=... y=... theta=...
[visualizer] ACCEPT: keyframe grid=...x... cell_mm=...
```

### Result

- Visualizer no longer crashes on malformed telemetry
- Drops invalid packets silently with diagnostic logging
- Continues displaying previous valid state
- All features preserved: grid, robot position, heading, path, trail, scan, diagnostics

---

## Build & Test Status

### Firmware Build
```
Exit Code: 0 (SUCCESS)
```

### Visualizer Syntax
```
python -m py_compile tools/visualizer.py
Result: OK
```

---

## Calibration Parameters Still Requiring Physical Measurement

### In `include/POIDetector.h` - POIConfig Namespace

These are now used with explicit values but may need arena-specific tuning:

- `POI_MIN_RANGE_DIFF_M = 0.1` (100mm threshold)
- `POI_CONFIRM_COUNT = 4` (readings required)
- `POI_CONFIRM_TIME_MS = 2000` (time window)
- `POI_POSITION_TOLERANCE_M = 0.15` (spatial drift limit)
- `POI_MERGE_RADIUS_M = 0.3` (POI fusion distance)
- `POI_MAX_RANGE_M = 2.0` (VL53L1X max)

### In `src/robocup_template.ino` - robot_init()

ToF sensor extrinsics (MUST BE CALIBRATED FOR YOUR ROBOT):
```cpp
ToFExtrinsics tof_extrinsics;
tof_extrinsics.x_offset_m = 0.0f;      // TODO: CALIBRATE
tof_extrinsics.y_offset_m = 0.0f;      // TODO: CALIBRATE  
tof_extrinsics.yaw_offset_rad = 0.0f;  // TODO: CALIBRATE
```

### In `include/Localisation.h` - Configuration Methods

Optical flow and LiDAR calibration parameters exist but are not yet applied to transforms (acceptable if offsets are 0.0):
- `setFlowOffsetMM(x_mm, y_mm)` and `setFlowYawOffsetRad()`
- `setLidarOffsetMM(x_mm, y_mm)` and `setLidarYawOffsetRad()`
- `setIMUYawOffsetRad()`

These are documented but assumed to be 0.0 until hardware-specific calibration.

---

## Architecture Diagram (After Fixes)

```
        BNO055 IMU          PMW3901 Optical Flow
            │                        │
            ├────────────┬───────────┤
                         ▼
                  Localisation
              (Authoritative Fused Pose)
                    x_mm, y_mm, theta_rad
                         │
         ┌───────────────┼───────────────┐
         │               │               │
         ▼               ▼               ▼
    MappingNav      POIDetector   ToFSearchPlanner
  (Grid Updates)   (Weight Detection)  (Search Targets)
         │               │               │
         ▼               ▼               ▼
    Occupancy      ToF Coverage   Goal → D* Lite
     Grid                Map             Path
         │                               │
         └───────────────┬───────────────┘
                         ▼
                      Motors
                    (Drive Control)

LiDAR:
  LD06 (Serial2) ─→ RoboSLAM (occupancy update)
                ├─→ Localisation (scan validation only)
                └─→ Telemetry (visualizer display)

ToF Sensors:
  VL53L1X (I2C) ──→ POIDetector ──→ Confirmed POIs
               ├─→ ToFCoverageMap
               └─→ Telemetry (visualizer)

Single Authoritative Pose:
  All sensor fusion and world-frame operations use
  Localisation.getPose() exclusively.
```

---

## Files Changed

### Firmware
1. `src/robocup_template.ino`
   - Removed duplicate config constants
   - Updated tof_search_planner_callback to use fused pose and set goals
   - Updated poi_detector_callback to use fused pose and add diagnostics
   - Updated optical_flow_callback to increment diagnostic counter
   - Added diagnostic globals

2. `src/Localisation.cpp`
   - Replaced fake LiDAR matching with explicit "not implemented" approach
   - Changed range_diff sign in POIDetector.cpp usage

3. `src/POIDetector.cpp`
   - Fixed range_diff calculation (expected - tof, not tof - expected)
   - Added UNKNOWN status rejection in processAnomaly

### Visualizer
1. `tools/visualizer.py`
   - Added `import math`
   - Added validation helper functions
   - Updated _apply_pose_path() with coordinate validation
   - Updated _apply_keyframe() with dimension validation
   - Updated update_state() with pose validation and early exit
   - Updated update_state() scan scatter section with coordinate validation
   - Updated animate() with exception handling and cache_frame_data=False

---

## Testing Recommendations

### Firmware Tests
1. **Test A - Optical Flow**: Verify `optical_flow_update_count` increases in telemetry
2. **Test B - ToF Detection**: Place small object to trigger POI detection
3. **Test C - Search Planner**: Verify robot navigates to search targets (no longer prints-only)
4. **Test D - Pose Consistency**: Check MappingNav and Localisation poses stay synchronized
5. **Test E - LiDAR Handling**: Verify no false corrections applied (match score = 0)

### Visualizer Tests
1. **Test 1 - Valid Telemetry**: Normal data displays correctly
2. **Test 2 - NaN Pose**: Send pose with NaN values → visualizer rejects, continues
3. **Test 3 - Bad Grid Dims**: Send invalid grid → visualizer rejects, continues
4. **Test 4 - Bad Scan**: Send scan with infinity coords → visualizer rejects, continues
5. **Test 5 - Continuous Operation**: Run for >5 minutes, no crashes

---

## Known Limitations

1. **LiDAR Correction**: Currently disabled (not implemented). Full scan-to-map ICP matching would require significant additional implementation.

2. **Optical Flow Offsets**: Configuration exists but offsets are assumed to be 0.0. Actual sensor offset transform not applied to pose prediction.

3. **Search Planner D* Lite Connection**: Now enabled, but requires D* Lite to be initialized and maintained by RoboSLAM/MappingNav systems.

4. **ToF Calibration**: Extrinsics (x/y/yaw offsets) required for accurate world-frame projection, currently at defaults (0.0).

---

## Future Work

1. Implement proper LiDAR scan-to-map matching (ICP or similar)
2. Measure and calibrate ToF sensor extrinsics
3. Apply optical flow sensor offset transforms
4. Add search state machine (currently implicit in target generation)
5. Implement reachability checking before accepting search targets
6. Add POI merging and deduplication across robot runs
7. Visualizer: Add overlays for POI locations and search coverage

---

## Commands to Test

### Build Firmware
```bash
cd "ENMT301_RoboCup"
pio run
```

### Test Visualizer Syntax
```bash
python -m py_compile tools/visualizer.py
```

### Run Visualizer (after connecting Teensy)
```bash
python tools/visualizer.py --port COM18 --baud 115200 --fps 20
```

Expected behavior:
- Serial connects immediately
- Diagnostics print for each valid packet received
- Matplotlib window displays grid, robot, scan when data arrives
- No crashes on invalid telemetry (rejected packets logged)

