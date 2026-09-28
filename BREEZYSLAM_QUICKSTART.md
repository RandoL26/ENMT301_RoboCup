
# 🚀 BreezySLAM Quick Start Card

## First Time Setup
```bash
cd /Users/max/Desktop/ENMT301_RoboCup
bash setup_breezyslam.sh
```
Takes ~2 minutes. Do this once.

---

## Run SLAM (Every Time)

### Step 1: Upload Firmware
```bash
pio run -t upload
```

### Step 2: Run SLAM Processor
```bash
python3 tools/run_breezyslam.py --port /dev/cu.wchusbserial --baud 115200
```

**That's it!** You should see:
```
Using LaserModel: RPLidarA1
Listening on /dev/cu.wchusbserial baud 115200
BreezySLAM instantiated: scan_size= 360
```

Then a window pops up showing the live map.

---

## Find Your Serial Port

```bash
python3 tools/find_serial_port.py
```

Or manually:
```bash
ls /dev/tty.* | grep -iE 'usb|wchusbserial|anko'
```

Replace `wchusbserial` with your actual port.

---

## Recommended Settings

**For Competition Arena (3-5m):**
```bash
python3 tools/run_breezyslam.py \
  --port /dev/cu.wchusbserial \
  --map-meters 16 \
  --map-pixels 800
```
→ 2cm pixel size, covers 16×16m

**For Small Test Space:**
```bash
python3 tools/run_breezyslam.py \
  --port /dev/cu.wchusbserial \
  --map-meters 8 \
  --map-pixels 400
```
→ 2cm pixel size, covers 8×8m, faster

**For Maximum Detail:**
```bash
python3 tools/run_breezyslam.py \
  --port /dev/cu.wchusbserial \
  --map-meters 8 \
  --map-pixels 800
```
→ 1cm pixel size, detailed but limited coverage

---

## Debug Output

**See all LiDAR data:**
```bash
python3 tools/run_breezyslam.py --verbose
```

**Check Teensy is sending data:**
```bash
pio device monitor -b 115200 | head -50
```

Should show lines like:
```
>lidar:150:100;200:50;180:120|xy
```

---

## What You Should See

1. **Matplotlib window opens** (black background)
2. **Red circle** in center = robot position
3. **Red arrow** = robot heading (direction facing)
4. **Cyan dots** = current LiDAR scan
5. **Green line** = robot's path/trajectory
6. **Map builds** as robot moves (white=free, black=walls)

---

## Common Issues

### Window won't open?
```bash
python3 -m pip install PyQt5
```

### "No module named breezyslam"?
```bash
bash setup_breezyslam.sh
```

### "No serial port found"?
```bash
python3 tools/find_serial_port.py
# Use the port it finds
```

### Map showing only noise?
- Check LD06 is connected to Teensy Serial2
- Run with `--verbose` to see raw data
- Look for `>lidar:` lines in output

### Visualization laggy?
```bash
# Use smaller map
python3 tools/run_breezyslam.py --map-pixels 400
```

---

## Stop & Restart

**Ctrl+C** to stop the script

Close the visualization window to exit cleanly

Start over with Step 2 above

---

## Full Command Reference

```bash
python3 tools/run_breezyslam.py [OPTIONS]

Options:
  --port PORT              Serial port (default: auto-detect)
  --baud BAUD              Baud rate (default: 115200)
  --map-meters SIZE        Map physical size in meters (default: 32)
  --map-pixels PIXELS      Map resolution in pixels (default: 800)
  --scan-size SIZE         LiDAR angular resolution (default: 360)
  --verbose                Print all received lines
```

---

## Performance Summary

| Item | Value |
|------|-------|
| LiDAR update rate | 50 Hz |
| Map update rate | 10-20 Hz |
| CPU usage | ~60-100% |
| Memory usage | ~100 MB |
| Latency | 50-200 ms |

---

**Setup time:** ~2 min (first time only)  
**Run time:** Each session, ~30 seconds  
**Status:** ✅ Ready to go!
