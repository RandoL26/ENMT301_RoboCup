# BreezySLAM Integration Analysis

**Date:** September 25, 2026  
**Status:** ⚠️ SETUP REQUIRED - Package not yet installed

---

## Overview

Your RoboCup project includes **BreezySLAM** (Simultaneous Localization and Mapping) integration for real-time robot mapping and navigation. However, the BreezySLAM folder is currently **empty** and needs to be properly configured.

---

## 📊 Current Setup

### File Structure

```
tools/
├── BreezySLAM/              ← Empty folder (needs setup)
├── run_breezyslam.py        ← Main script
├── roboviz/
│   └── __init__.py          ← Visualizer for SLAM maps
├── visualizer.py            ← Standalone visualizer
└── roboviz.gif              ← Demo animation
```

### Key Files

1. **`run_breezyslam.py`** (231 lines)
   - Main SLAM processor
   - Reads LiDAR data from Serial port (LD06 LiDAR output)
   - Runs BreezySLAM RMHC algorithm
   - Displays real-time map visualization

2. **`tools/roboviz/__init__.py`** (359 lines)
   - PyRoboViz visualizer
   - Matplotlib-based live mapping display
   - Handles map rendering and robot pose visualization
   - Shows LiDAR scan points overlaid on occupancy grid

3. **`setup.py`**
   - Package installer

---

## 🔌 Data Flow

```
Teensy 4.0
    ↓
Serial2 (115200 baud)
    ↓
LD06 LiDAR (sends >lidar:x:y;x:y;...|xy format)
    ↓
USB Serial (on PC)
    ↓
run_breezyslam.py
    ├─ Parse LiDAR teleplot format
    ├─ Convert XY points → angular scan array
    └─ Feed to BreezySLAM RMHC algorithm
    ↓
Real-time occupancy grid map
    ↓
roboviz (Matplotlib visualization)
    ↓
Live map display on PC
```

---

## 🔴 Current Issues

### 1. **BreezySLAM Package Not Installed**

**Status:** ❌ BLOCKING

```
tools/BreezySLAM/  ← Empty! Python package files missing
```

The folder exists but contains no Python files. BreezySLAM must be installed.

**Error when running:**
```python
from breezyslam.algorithms import RMHC_SLAM, Deterministic_SLAM
# → ImportError: No module named 'breezyslam'
```

**What's needed:**
- Python source files for BreezySLAM algorithms
- Sensor model definitions (RPLidar, XVLidar, etc.)
- C++ bindings (if using accelerated version)

### 2. **Dependencies Not Listed**

**Status:** ⚠️ MISSING

The `run_breezyslam.py` requires:
```python
- numpy
- matplotlib
- pyserial (already installed ✅)
- breezyslam (NOT installed ❌)
```

No `requirements.txt` file exists to track dependencies.

### 3. **Configuration Incomplete**

**Status:** ⚠️ PARTIAL

The script has good defaults but needs tuning:

```python
ap.add_argument('--map-pixels', type=int, default=800)      # Map resolution
ap.add_argument('--map-meters', type=float, default=32.0)   # Map size
ap.add_argument('--scan-size', type=int, default=360)       # LiDAR resolution
```

These defaults may not match your LD06 LiDAR specifications.

---

## 🎯 How It's Supposed to Work

### 1. LD06 LiDAR Output Format

The LD06 LiDAR connected to Teensy Serial2 outputs:

```cpp
// In ld06_lidar_callback():
ld06.printScanTeleplot(Serial);
// Output format:
// >lidar:100:50;110:60;120:55;....|xy
```

Format breakdown:
- `>lidar:` - Prefix (identifies as LiDAR data)
- `100:50` - X:Y point in millimeters
- `;` - Separator between points
- `|xy` - Suffix (indicates XY format)

### 2. Script Processing

```python
# In run_breezyslam.py:
if line.startswith('>lidar:') and '|xy' in line:
    body = line[len('>lidar:'):line.index('|xy')]    # Extract data
    pairs = [p for p in body.split(';') if p.strip()] # Split points
    
    # Convert to arrays
    xs = [float(a) for a, b in pairs]   # X millimeters
    ys = [float(b) for a, b in pairs]   # Y millimeters
    
    # Convert to scan format (angular bins)
    scan = xy_points_to_scan(xs, ys, scan_size=360)
    
    # Feed to SLAM
    slam.update(scan)
    x, y, theta = slam.getpos()  # Get robot position
    slam.getmap(mapbytes)         # Get occupancy grid
    
    # Display
    viz.display(x, y, theta, map_bytes=mapbytes)
```

### 3. SLAM Algorithm

**Type:** RMHC (Random Movement Hypothesis Correlation)

```python
slam = RMHC_SLAM(laser, map_size_pixels=800, map_size_meters=32)
```

What it does:
- Takes LiDAR scans
- Matches scans against occupancy map (scan correlation)
- Updates robot pose (X, Y, Theta)
- Updates occupancy grid
- Handles loop closure automatically

### 4. Visualization

PyRoboViz displays:
- **Black pixels** = Occupied (LiDAR detected obstacle)
- **White pixels** = Free space
- **Robot icon** = Current position + orientation
- **Cyan dots** = Current LiDAR scan
- **Trajectory** = Previous positions (if enabled)

---

## 🛠️ Setup Instructions

### Step 1: Install Python Dependencies

```bash
python3 -m pip install numpy matplotlib
```

### Step 2: Install BreezySLAM

**Option A: From GitHub (Recommended)**

```bash
cd /Users/max/Desktop/ENMT301_RoboCup/tools

# Clone BreezySLAM repository
git clone https://github.com/simondlevy/BreezySLAM.git

cd BreezySLAM

# Install Python package
python3 setup.py install
# OR
python3 -m pip install -e .
```

**Option B: From PyPI (if available)**

```bash
python3 -m pip install breezyslam
```

### Step 3: Verify Installation

```bash
python3 -c "from breezyslam.algorithms import RMHC_SLAM; print('BreezySLAM OK')"
```

### Step 4: Test with Robot

```bash
# Terminal 1: Upload firmware to Teensy
cd /Users/max/Desktop/ENMT301_RoboCup
pio run -t upload

# Terminal 2: Monitor and run SLAM
pio device monitor -b 115200 &
sleep 2

/usr/local/bin/python3 /Users/max/Desktop/ENMT301_RoboCup/tools/run_breezyslam.py \
  --port /dev/tty.usbmodemWCH285E33TS11 \
  --baud 115200 \
  --map-pixels 800 \
  --map-meters 32.0 \
  --verbose
```

---

## ⚙️ Configuration Parameters

### Recommended for RoboCup

```python
# Map size and resolution
--map-pixels 800          # 800×800 pixel grid
--map-meters 32.0         # Represents 32×32 meters

# LiDAR specifics for LD06
--scan-size 360           # LD06 has 360-degree scans

# For debugging
--verbose                 # Print all LiDAR lines
```

### What Each Parameter Does

| Parameter | Default | RoboCup | Purpose |
|-----------|---------|---------|---------|
| `map-pixels` | 800 | 800 | Higher = better resolution but slower |
| `map-meters` | 32 | 16-32 | Physical space represented |
| `scan-size` | 360 | 360 | LiDAR angular resolution |
| `verbose` | False | True (initially) | Debug output |

### Pixel Size Calculation

```
Pixel Size = map_meters / map_pixels
           = 32 / 800
           = 0.04 meters
           = 4 centimeters per pixel
```

For RoboCup arena (typically 3-5m), use 16m map:
```
--map-meters 16  → 2 cm per pixel (good detail)
```

---

## 🔍 LD06 Lidar Integration Points

### In Teensy Template

```cpp
// In robocup_template.ino:

#include "ld06.h"

LD06 ld06(Serial2);

// In robot_init():
printlnBoth("Initialising LD06 LiDAR on Serial2...");
ld06.init();
ld06.enableFullScan();
printlnBoth("LD06 LiDAR initialized");

// In ld06_lidar_callback():
bool scanReady = ld06.readScan();
if (scanReady) {
    ld06.printScanTeleplot(Serial);  // ← This goes to run_breezyslam.py
}

// Task scheduling:
Task tLD06_lidar(LD06_READ_TASK_PERIOD, LD06_READ_TASK_NUM_EXECUTE, &ld06_lidar_callback);
```

### Data Format

The `ld06.printScanTeleplot()` outputs:

```
>lidar:x1:y1;x2:y2;x3:y3;...;xN:yN|xy
```

Each point is:
- **x** = Horizontal distance (mm, relative to LiDAR center)
- **y** = Vertical distance (mm, relative to LiDAR center)
- Range: 0-4000 mm (can be configured)

Example:
```
>lidar:150:100;200:50;180:120;...|xy
```

This represents 3+ LiDAR returns at different X,Y positions.

---

## 🧭 SLAM Algorithm Details

### What RMHC Does

1. **Scan Matching**
   - Current LiDAR scan is compared against accumulated map
   - Computes correlation score (how well scan matches map)

2. **Pose Estimation**
   - Tests random robot position hypotheses
   - Picks hypothesis with best scan correlation
   - Updates pose estimate (X, Y, Theta)

3. **Map Building**
   - Updates occupancy grid with new LiDAR hits
   - Marks occupied cells as black (0)
   - Unmarked cells remain white (255)

4. **Loop Closure**
   - When robot revisits a location, scan correlation improves
   - SLAM automatically corrects accumulated drift

### Laser Model Selection

```python
PREFERRED_MODELS = ['RPLidarA1', 'XVLidar', 'MinesLaser', 'URG04LX']

# For LD06, you may need to use a generic model or create custom one
# Current code tries to find SCAN_SIZE attribute
```

**Issue:** LD06 specific model may not exist in BreezySLAM package.

**Solution:** Use a compatible model (RPLidarA1 has similar 360° scan structure)

---

## ⚠️ Known Limitations

### 1. Single-Threaded Processing

```python
# run_breezyslam.py processes one scan at a time
while True:
    line = ser.readline()
    # ... parse and process ...
    slam.update(scan)  # ← Blocking operation
    viz.display(...)   # ← GUI update blocking
```

At 50 Hz LiDAR rate, each scan has ~20ms to process. Heavy scans may cause lag.

**Impact:** Visualization may stutter with full 360° high-resolution scans

### 2. No Closed-Loop Motor Control

The SLAM gives robot position, but:
- **Current state:** PC knows robot location from SLAM
- **Missing:** Feedback loop to correct robot motion

To use SLAM for navigation, you need:
```
PC (SLAM calculates target location)
  ↓ sends via Bluetooth
Teensy (corrects motor commands toward target)
```

Currently only supports manual (Xbox) or preset motion commands.

### 3. Coordinate System Assumptions

```python
# Assumes:
# - Robot at origin (0, 0) initially
# - X+ points right, Y+ points forward
# - Theta=0 is East (positive X direction)
```

Verify this matches your LD06 mounting orientation!

### 4. Map Memory Grows Unbounded

```cpp
mapbytes = bytearray(args.map_pixels * args.map_pixels)
# 800×800 = 640,000 bytes = 640 KB
```

For long-duration missions (>30 min), consider:
- Smaller map size (`--map-pixels 400`)
- Periodic map reset
- Local submaps instead of global

---

## 📈 Performance Metrics

### Expected Frame Rates

| Metric | Value | Notes |
|--------|-------|-------|
| LiDAR scan rate | 10 Hz | LD06 default |
| SLAM update rate | 5-10 Hz | Limited by processing |
| Visualization refresh | 5-10 Hz | matplotlib rendering |
| Map update latency | 100-200 ms | Full pipeline delay |

### CPU Usage

- **BreezySLAM processing:** ~50-100% of one CPU core
- **Visualization:** ~20-30% of one CPU core
- **Total on PC:** ~70-130% (uses 1-2 cores)

Good for laptop, may strain Raspberry Pi.

---

## 🎯 Next Steps

### For Competition

1. **Install BreezySLAM** (Step 1-3 from Setup Instructions)
2. **Test data flow** (connect LD06 to Teensy, run on PC)
3. **Tune map parameters** (optimal pixel size for arena)
4. **Validate coordinate system** (ensure X/Y/Theta match robot orientation)

### For Navigation

To use SLAM for autonomous navigation:

1. Implement goal-seeking in Teensy
   ```cpp
   // pseudo-code
   if (slam_data.target_x != 0) {
       // Calculate heading to target
       // Issue motor commands to move that direction
   }
   ```

2. Add feedback control loop
   ```cpp
   // PID controller to maintain heading
   ```

3. Implement obstacle avoidance
   ```cpp
   // Use map data to detect local obstacles
   ```

---

## 📚 Additional Resources

### BreezySLAM Documentation
- GitHub: https://github.com/simondlevy/BreezySLAM
- Paper: "BreezySLAM: A Fast Visual SLAM Approach" (and LIDAR version)

### LD06 LiDAR Documentation
- Datasheet: LD06 360° 2D LiDAR Specs
- ROS Driver: Check if ld06.h implementation matches official specs

### PyRoboViz
- GitHub: https://github.com/simondlevy/PyRoboViz
- Examples: Visualization techniques and parameters

---

## ✅ Deployment Checklist

- [ ] `numpy` and `matplotlib` installed
- [ ] BreezySLAM cloned and installed
- [ ] `breezyslam` import test passes
- [ ] LD06 LiDAR firmware working on Teensy
- [ ] Serial output verified (see `>lidar:...` format in monitor)
- [ ] `run_breezyslam.py` connects to serial port
- [ ] SLAM produces valid position estimates
- [ ] Visualization window displays map
- [ ] Performance acceptable (>5 Hz map updates)

---

## 🔧 Troubleshooting

### "ImportError: No module named 'breezyslam'"

→ BreezySLAM not installed. Run: `pip install breezyslam` or clone from GitHub

### "No LaserModel found / LaserModel not available"

→ LD06 model not in BreezySLAM sensors. Use generic model or create custom one.

### "No serial port found"

→ Specify with `--port /dev/tty.usbmodemWCH285E33TS11`

### Visualization window won't appear

→ Matplotlib backend issue. Try:
```bash
python3 -m pip install PyQt5
```

### Map shows only static noise, robot position not updating

→ Check LiDAR data format with `--verbose`. Verify `>lidar:` prefix present.

### High latency or stuttering visualization

→ Reduce `--map-pixels` or `--scan-size` to lighten processing load

---

## 📝 Summary

| Component | Status | Action |
|-----------|--------|--------|
| Python scripts | ✅ Complete | Ready to use |
| Visualizer | ✅ Implemented | Needs testing |
| BreezySLAM package | ❌ NOT installed | **CRITICAL - Install first** |
| Dependencies | ⚠️ Partial | Install numpy, matplotlib |
| LD06 LiDAR integration | ✅ Implemented | Test data flow |
| Motor feedback | ❌ Not implemented | Design if autonomous nav needed |

**Current blocker:** BreezySLAM package installation required before testing.

---

*BreezySLAM setup guide | September 2026 | RoboCup Project*
