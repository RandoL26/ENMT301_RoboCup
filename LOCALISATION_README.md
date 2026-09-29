# Localisation System Documentation

## Overview

The Localisation module provides robust 2D pose estimation for the ENMT301 RoboCup robot by fusing data from three sensors:

1. **LD06 2D LiDAR** - Provides scan-based position/heading correction (5-10 Hz)
2. **BNO055 IMU** - Provides heading rate via gyroscope for rotational prediction (100-200 Hz)
3. **PMW3901 Optical Flow** - Provides short-term XY displacement in robot frame (50-100 Hz)

The system produces a single fused pose estimate:
```
x_mm          // Position X in mm (world frame)
y_mm          // Position Y in mm (world frame)
theta_rad     // Heading in radians [-pi, pi] (world frame)
```

## Architecture

### High-Level Data Flow

```
BNO055 IMU                    OpticalFlow PMW3901       LD06 LiDAR
    |                                 |                      |
    v                                 v                      v
updateIMU()               updateOpticalFlow()        correctFromLiDAR()
    |                                 |                      |
    +------> Localisation <-----------+                      |
                 |                                            |
                 v                                            |
            updateFromSensors() --------- Prediction ---------+
                 |                      (IMU gyro + flow)     |
                 |                                            |
                 +-----> Correction <------------------------+
                         (LiDAR scan-to-map)
                 |
                 v
            getPose() --> fused x, y, theta
```

### Timing

- **IMU Task**: 10 ms period (100 Hz)
- **Optical Flow Task**: 40 ms period (25 Hz nominal)
- **Localisation Update**: 10 ms period (100 Hz)
- **LiDAR Task**: 1 ms polling (reads when scan ready, ~5-10 Hz)
- **LiDAR Correction**: Triggered with each complete scan (~5-10 Hz)

## Coordinate Frames

### Robot Frame (Body Frame)

- **+X**: Forward (direction of robot motion)
- **+Y**: Left (perpendicular to forward)
- **+theta**: Counter-clockwise rotation (when viewed from above)
- **Origin**: Center of robot chassis

**Sensor Mounting (nominal):**
- LD06 LiDAR: Mounted on front-top of robot, facing forward
  - Angle measurement: 0° = forward, +90° = left, -90° = right
  - Distance units: mm
- PMW3901 Optical Flow: Mounted facing downward
  - Measures motion in robot frame
  - X-axis: forward/backward motion
  - Y-axis: left/right motion (perpendicular strafing)
  - Units: accumulated motion counts (converted to mm via scale factor)
- BNO055 IMU: Mounted on chassis
  - Heading (euler_h): degrees relative to initial orientation
  - Gyro Z-axis: rotation rate (rad/s)

### World/Map Frame

- **+X**: Right (across arena width 2.4 m)
- **+Y**: Forward (along arena length 4.9 m)
- **+theta**: Counter-clockwise rotation
- **Origin**: Arena corner (0, 0)

**Coordinate Transformation:** Robot frame → World frame
```
dx_world = cos(theta) * dx_robot - sin(theta) * dy_robot
dy_world = sin(theta) * dx_robot + cos(theta) * dy_robot
```

## Calibration Constants

All calibration constants are defined in `Localisation.cpp` default constructor and can be tuned via setter methods in `robocup_template.ino`:

### Sensor Offsets (from robot center)

```cpp
LIDAR_X_OFFSET_MM = 0.0f      // Forward offset (mm)
LIDAR_Y_OFFSET_MM = 0.0f      // Left offset (mm)
LIDAR_YAW_OFFSET_RAD = 0.0f   // Heading offset (rad)

FLOW_X_OFFSET_MM = 0.0f       // Forward offset (mm)
FLOW_Y_OFFSET_MM = 0.0f       // Left offset (mm)
FLOW_YAW_OFFSET_RAD = 0.0f    // Heading offset (rad)

IMU_YAW_OFFSET_RAD = 0.0f     // Initial heading offset (set at startup)
```

**How to Measure:**
1. Mark robot center (e.g., wheel midpoint between drive wheels)
2. Measure distance from center to each sensor's optical center
3. Measure if sensor is rotated relative to robot body

### Optical Flow Calibration

```cpp
FLOW_MM_PER_COUNT = 0.05f     // mm of motion per sensor count
```

**How to Calibrate:**
1. Move robot exactly 100 mm forward in a straight line
2. Read `opticalFlow.getTotalXmm()` value
3. Compute: `mm_per_count = measured_mm / measured_counts`
4. Update via `localisation.setOpticalFlowMMPerCount()`

### Motor/Encoder Calibration

```cpp
ENCODER_PULSES_PER_METER = 1000.0f   // Pulses per 1 meter traveled
WHEEL_BASE_MM = 200.0f               // Distance between left/right wheels (mm)
```

These are used by RoboSLAM but affect overall odometry quality. Calibrate via motor_control module.

### LiDAR Correction Tuning

```cpp
LIDAR_CORRECTION_MAX_MM = 50.0f      // Max position correction per update (mm)
LIDAR_CORRECTION_MAX_RAD = 0.1f      // Max heading correction (rad ≈ 5.7°)
LIDAR_MIN_VALID_POINTS = 50          // Minimum points to attempt matching
LIDAR_MIN_MATCH_SCORE = 300          // Score threshold 0-1000 (must be ≥300)
```

**Tuning Guide:**
- Increase `MAX_MM` / `MAX_RAD` if corrections are too conservative
- Decrease `MIN_MATCH_SCORE` if LiDAR corrections rarely trigger
- Increase `MIN_VALID_POINTS` if scans are corrupted often

## Usage in Code

### Initialization (in `robocup_template.ino`)

```cpp
Localisation localisation;  // Global instance

void robot_init() {
    // ... other init code ...
    localisation.begin();
    localisation.resetPose(0.0f, 0.0f, 0.0f);  // Start at origin, heading 0
}
```

### Sensor Updates

Sensor data is fused automatically via task callbacks:

```cpp
void imu_task_callback(void) {
    current_imu_data = read_imu();
    localisation.updateIMU(current_imu_data);  // Called automatically
    // ... rest of IMU handling ...
}

void optical_flow_callback(void) {
    int16_t dx = 0, dy = 0;
    if (opticalFlow.read(dx, dy)) {
        opticalFlow.addMotionCounts(dx, dy);
        localisation.updateOpticalFlow(opticalFlow);  // Called automatically
    }
}
```

### Periodic Updates

```cpp
void localisation_update_callback(void) {
    localisation.updateFromSensors();  // Called every 10 ms
    // ... diagnostics printing ...
}
```

### LiDAR Correction

```cpp
void ld06_lidar_callback(void) {
    bool scanReady = ld06.readScan();
    if (scanReady) {
        roboSlam.processScan(ld06, current_imu_data);
        localisation.correctFromLiDAR(ld06, mappingNav);  // Correction triggered
        // ... rest of LiDAR handling ...
    }
}
```

### Getting Pose for Navigation

```cpp
RobotPose pose = localisation.getPose();
float x_mm = pose.x_mm;
float y_mm = pose.y_mm;
float theta_rad = pose.theta_rad;

// For use in MappingNav (in meters):
float x_m = pose.x_mm / 1000.0f;
float y_m = pose.y_mm / 1000.0f;
mappingNav.setPose(x_m, y_m, pose.theta_rad);
```

### Getting Diagnostics

```cpp
LocalisationDiags diags = localisation.getDiags();

// Key diagnostics:
float flow_dx = diags.flow_dx_mm;           // Last optical flow X displacement
float flow_dy = diags.flow_dy_mm;           // Last optical flow Y displacement
float imu_yaw = diags.imu_yaw_rad;          // Current IMU yaw in radians
uint16_t lidar_score = diags.lidar_match_score;  // Match score 0-1000
uint8_t lidar_accepted = diags.lidar_correction_accepted;  // 1 if accepted
uint32_t frame_count = diags.frame_count;   // Total updates processed
uint16_t corrections = diags.lidar_correction_count;  // Total LiDAR corrections
```

## Existing System Preservation

The implementation is designed to **not break existing functionality**:

### RoboSLAM Integration

- RoboSLAM continues to run independently
- It feeds encoder/flow/IMU data into MappingNav for path planning
- Localisation runs in parallel, fusing the same sensors
- **MappingNav still uses RoboSLAM's pose estimate** (not Localisation's yet)
- This allows gradual migration to fused pose if desired

### Telemetry Protocol

- **No changes** to existing telemetry packet format (0x01, 0x02, 0x04, 0x05)
- Localisation diagnostics printed to USB serial (non-blocking)
- Occupancy grid behavior unchanged (UNKNOWN → FREE allowed; OCCUPIED → FREE prevented)
- LiDAR diagnostics fully preserved

### LD06 Scan Diagnostics

All existing diagnostics are maintained:
- Point count tracking
- Zero/invalid point counters
- Distance range statistics
- Angle span analysis
- Jitter detection

## Testing Procedure

### Test A: IMU Heading

1. Run robot with debug output enabled
2. Manually rotate robot on table
3. Verify `imu_yaw_rad` changes smoothly
4. Check theta wrapping at ±π radians

**Expected Result:** Heading should rotate smoothly with hand rotation, wrap correctly around ±180°.

### Test B: Optical Flow

1. Tape robot to table
2. Manually move by exactly 100 mm forward
3. Note optical flow output: `flow_dx_mm` should be ~100
4. Repeat sideways; `flow_dy_mm` should show lateral motion

**Expected Result:** Flow measurements should match physical motion within ±10%.

### Test C: Combined Motion

1. Move robot forward 500 mm
2. Rotate 90° clockwise
3. Move forward 300 mm
4. Verify trajectory makes sense (should form L-shape)

**Expected Result:** Fused pose should show correct L-shaped path, no wild jumps.

### Test D: Stationary LiDAR

1. Robot stationary for 30 seconds
2. Monitor `lidar_match_score` and `lidar_correction_accepted`
3. Verify pose remains stable (drift <10 mm/s)

**Expected Result:** Pose should remain stable; corrections should be minimal.

### Test E: LiDAR Correction

1. Move robot around a known obstacle pattern
2. Monitor `lidar_correction_count` and `lidar_corr_x/y` values
3. Verify corrections reduce accumulated drift

**Expected Result:** Corrections should occur; drift should be bounded.

### Test F: Occupancy Grid

1. Run full navigation cycle
2. Verify:
   - Obstacles become OCCUPIED
   - Observed free space becomes FREE
   - No random cell erasure
   - Grid survives robot rotation
   - Visualizer shows correct grid state

**Expected Result:** Grid should correctly map environment, visualizer functional.

## Known Limitations

1. **Simplified Scan Matching**: Current implementation returns conservative zero correction. A full ICP (Iterative Closest Point) matcher could improve accuracy but adds computational cost.

2. **No Multi-Hypothesis Tracking**: Only maintains single pose estimate. Robust systems track multiple hypotheses, but this adds complexity.

3. **Fixed Uncertainty Model**: Covariance grows linearly with distance. More sophisticated models exist but require tuning.

4. **No Loop Closure**: If robot returns to known location, system doesn't recognize it. Full SLAM would detect and correct.

5. **Optical Flow Drift**: Cumulative errors in optical flow can grow unbounded if LiDAR corrections are infrequent.

6. **No Magnetic Compass**: Relies on gyro integration for heading, which drifts over time.

## Future Enhancements

1. Implement full scan-to-map ICP matcher for LiDAR correction
2. Add particle filter for multi-hypothesis localization
3. Integrate magnetometer compass for absolute heading
4. Add loop-closure detection using LiDAR scan hashing
5. Adaptive uncertainty based on sensor quality metrics
6. Extended Kalman Filter (EKF) for more sophisticated fusion

## References

- **RoboSLAM**: [include/RoboSLAM.h](include/RoboSLAM.h)
- **MappingNav**: [include/MappingNav.h](include/MappingNav.h)
- **Telemetry Protocol**: [include/telemetry.h](include/telemetry.h)
- **LD06 LiDAR Driver**: [include/ld06.h](include/ld06.h)
- **BNO055 IMU Driver**: [include/BNO055_support.h](include/BNO055_support.h)
- **Optical Flow Driver**: [include/optical_flow.h](include/optical_flow.h)

## Coordinate Frame Summary Table

| Frame | +X Direction | +Y Direction | +Theta Direction | Origin |
|-------|--------------|--------------|------------------|--------|
| Robot | Forward | Left | CCW | Robot center |
| World | Right (2.4m) | Forward (4.9m) | CCW | Arena corner (0,0) |
| LD06 | Forward | Left | CCW (0°=forward) | Robot center (with offset) |
| PMW3901 | Forward (robot) | Left (robot) | - | Robot center (with offset) |

