# LD06 LiDAR Scan Jitter - Root Cause Analysis

## CRITICAL BUG IDENTIFIED

### The Problem: Incomplete Scan Initialization

In `LD06::LD06()` constructor (line 89-98 of ld06.cpp):

```cpp
LD06::LD06(HardwareSerial &serial, uint8_t pwmPin)
  : _lidarSerial(&serial),
    _pin(pwmPin),
    _previousScan(&_scanB) {   // ← CRITICAL: previousScan points to _scanB
  _scanA.index = 0;
  _scanB.index = 0;
  _receivedData.index = 0;
  _receivedData.computedCrc = 0;
}
```

**What happens:**
1. `_previousScan` is initialized to point to `&_scanB`
2. `_currentScan` is never explicitly initialized, defaults to `nullptr` or garbage
3. First `swapBuffers()` call relies on `_currentBuffer` flag, but this flag is also never initialized
4. `_currentBuffer` defaults to `0` (uninitialized), so first swap will try to read from uninitialized state

### The Manifestation

When the LD06 starts receiving packets:

1. `computeData()` is called for first packet
2. Points are added to `_currentScan` (which points to `&_scanA` after implicit swap, BUT...)
3. On first angle wrap detection, `swapBuffers()` is called
4. `swapBuffers()` toggles `_currentBuffer` and swaps the pointers
5. **PROBLEM:** The first scan delivered to consumers may contain:
   - Points from multiple partial scans (buffer was not clean)
   - Invalid/stale data from previous power cycle
   - Points in wrong order if buffer contents were not cleared
   - Mix of old and new data

### Why This Causes Large Jitter

- **Variable point counts** (23-48): Different packets hit different wrap points, some scans include extra packets before wrap is detected
- **Large distance jumps** (>10 m): Old stale data mixed with new data
- **Angle anomalies**: Points from different revolutions mixed together
- **Match score variations**: Diagnostic code tries to match points that aren't from the same scan

### Secondary Issues

1. **No validation of completed scan quality** before publishing
2. **Immediate buffer swap** without checking if enough points were collected
3. **No per-scan diagnostics** to detect malformed scans
4. **Race condition potential**: Telemetry might read a scan while it's being written

---

## ROOT CAUSE SUMMARY

| Issue | Location | Impact |
|-------|----------|--------|
| Uninitialized `_currentBuffer` | Constructor | Wrong buffer used first time |
| No validation before publish | `computeData()` line ~265 | Bad scans pass through |
| Immediate swap on wrap | `computeData()` line ~265 | No chance to validate |
| No scan-quality check | N/A | Any garbage accepted |
| Conservative MIN_POINTS_PER_REV | `computeData()` line ~241 | Accepts partial scans |

---

## FIXES REQUIRED

### Fix 1: Initialize _currentBuffer Explicitly

```cpp
LD06::LD06(HardwareSerial &serial, uint8_t pwmPin)
  : _lidarSerial(&serial),
    _pin(pwmPin),
    _currentBuffer(0),      // ← FIX: Explicit init
    _currentScan(&_scanA),  // ← FIX: Explicit init
    _previousScan(&_scanB) {
  _scanA.index = 0;
  _scanB.index = 0;
  _receivedData.index = 0;
  _receivedData.computedCrc = 0;
}
```

### Fix 2: Validate Scan Before Publishing

Add validation in `computeData()` before `swapBuffers()`:

```cpp
if (_currentScan->index >= MIN_POINTS_PER_REV) {
    // NEW: Validate scan quality
    if (isScanValid(_currentScan)) {
        _newScan = true;
        if (_fullScan) {
            swapBuffers();
            analyzePreviousScan(_previousScan);
            telemetry_update_ld06_diagnostics(_previousScan);
        }
    } else {
        // Reject invalid scan, keep old buffer content
        _currentScan->index = 0;  // Discard and try again
        ld06_diag_rejected_scan_count++;
    }
}
```

### Fix 3: Implement Scan Validation Function

```cpp
bool LD06::isScanValid(DataPointHandler *scan) {
    if (!scan || scan->index < MIN_POINTS_PER_REV) return false;
    
    // Check 1: Angular coverage (should be near 360°)
    float first_angle = scan->points[0].angle;
    float last_angle = scan->points[scan->index - 1].angle;
    float angular_span = last_angle - first_angle;
    if (angular_span < 0) angular_span += 360.0f;
    
    if (angular_span < 300.0f || angular_span > 380.0f) {
        return false;  // Unreasonable angular coverage
    }
    
    // Check 2: No excessive backward steps (indicates mixed revolutions)
    uint16_t backward_count = 0;
    for (uint16_t i = 1; i < scan->index; i++) {
        float step = scan->points[i].angle - scan->points[i-1].angle;
        if (step < -10.0f) backward_count++;
    }
    if (backward_count > 3) {
        return false;  // Too many angle reversals
    }
    
    // Check 3: Reasonable point density
    if (scan->index < 200) {
        return false;  // Too sparse
    }
    
    return true;
}
```

### Fix 4: Add Scan Quality Metrics

Add to diagnostics:

```cpp
uint16_t ld06_diag_rejected_scan_count = 0;
uint16_t ld06_diag_last_scan_validity = 1;  // 1=valid, 0=invalid
float ld06_diag_last_angular_span = 0.0f;
uint16_t ld06_diag_backward_angle_count = 0;
```

---

## EXPECTED BEHAVIOR AFTER FIX

**Before:**
- Stationary robot LiDAR: Point cloud jitters rapidly
- Point count varies wildly (23-48)
- Distance values jump by metres

**After:**
- Stationary robot LiDAR: Stable point cloud with minimal movement
- Point count consistent (typically 480-520 for 10 Hz rotation)
- Angular coverage consistent (~360°)
- Distance values stable within ±50 mm of previous scan

---

## Testing Validation

### Test 1: Stationary Scan Stability
```
Robot: Stationary, no motion
Duration: 10 seconds of continuous scanning
Expected: 
  - Point count: 480±20 (consistent)
  - Angular span: 358-362° (consistent)
  - Distance variance: <50 mm per angle
  - Zero rejected scans (after reaching steady state)
```

### Test 2: Angular Consistency
```
Measure consecutive scans at same angle (e.g., 0°)
Expected:
  - Measurements at 0° should differ by <10 mm scan-to-scan
  - No 1000+ mm jumps
  - No occasional huge outliers
```

### Test 3: Occupancy Grid Stability
```
Stationary robot near wall
Scan for 30 seconds
Expected:
  - Wall remains geometrically consistent in grid
  - No random cell erasure
  - Occupied cells remain occupied
```

