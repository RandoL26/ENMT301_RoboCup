#!/usr/bin/env python3
"""Run BreezySLAM on incoming >lidar teleplot lines from robot serial.

Usage:
  python tools/run_breezyslam.py --port COM3 --baud 115200

The script tries to import the BreezySLAM Python package and a LaserModel
from `breezyslam.sensors`. If not installed, it prints instructions and
exits. It listens to serial, parses `>lidar:x:y;...|xy` lines and converts
XY millimetre samples into an angular scan array for BreezySLAM.

This is a light integration meant to get you started quickly. It prefers
models in order: RPLidarA1, XVLidar, MinesLaser, URG04LX. If none are
available the script will exit.
"""

import sys
import os
import time
import math
import argparse
from collections import defaultdict

# Ensure project tools/ on path
ROOT = os.path.dirname(os.path.dirname(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    import serial
    from serial.tools import list_ports
except Exception:
    print('pyserial required. Install: python -m pip install pyserial')
    raise

# Try to import BreezySLAM and a laser model
try:
    from breezyslam.algorithms import RMHC_SLAM, Deterministic_SLAM
    from importlib import import_module
    sensors = import_module('breezyslam.sensors')
except Exception as e:
    print('BreezySLAM not available:', e)
    print('Install BreezySLAM (see tools/BreezySLAM/python/setup.py), or skip using this runner.')
    breezy_available = False
else:
    breezy_available = True

# Preferential list of sensor classes to try
PREFERRED_MODELS = ['RPLidarA1', 'XVLidar', 'MinesLaser', 'URG04LX', 'RPLidarA1']

# Try to find a laser model from the BreezySLAM sensors module
LaserModel = None
if breezy_available:
    for nm in PREFERRED_MODELS:
        if hasattr(sensors, nm):
            LaserModel = getattr(sensors, nm)
            print('Using LaserModel:', nm)
            break
    if LaserModel is None:
        # pick any class with attribute SCAN_SIZE
        for name in dir(sensors):
            obj = getattr(sensors, name)
            if hasattr(obj, 'SCAN_SIZE'):
                LaserModel = obj
                print('Using discovered LaserModel:', name)
                break

# Import our visualiser (tools.roboviz)
try:
    from tools.roboviz import Visualizer
except Exception as e:
    print('Failed to import tools.roboviz:', e)
    Visualizer = None


def pick_port():
    ports = list(list_ports.comports())
    if not ports:
        return None
    return ports[0].device


def xy_points_to_scan(xs_mm, ys_mm, scan_size, angle_min=0.0, angle_max=360.0, max_range_mm=8000):
    """Convert XY mm points into a scan array (length scan_size) in mm.

    We choose the minimum range per angular bin (closest hit wins). Empty
    bins are set to 0 (meaning no return) which BreezySLAM examples accept.
    """
    bins = [None] * scan_size
    angle_span = angle_max - angle_min
    angle_step = angle_span / float(scan_size)

    for x, y in zip(xs_mm, ys_mm):
        r = math.hypot(x, y)
        if r <= 0.0 or r > max_range_mm:
            continue
        ang = math.degrees(math.atan2(y, x))
        if ang < 0:
            ang += 360.0
        # map to bin index assuming 0..360 degrees
        idx = int((ang - angle_min) / angle_step)
        if idx < 0 or idx >= scan_size:
            continue
        if bins[idx] is None or r < bins[idx]:
            bins[idx] = int(round(r))

    # replace None with 0
    return [b if b is not None else 0 for b in bins]


def main():
    ap = argparse.ArgumentParser(description='Run BreezySLAM on robot >lidar teleplot lines')
    ap.add_argument('--port', '-p', help='Serial port (e.g. COM3)')
    ap.add_argument('--baud', '-b', type=int, default=115200)
    ap.add_argument('--map-pixels', type=int, default=800)
    ap.add_argument('--map-meters', type=float, default=32.0)
    ap.add_argument('--scan-size', type=int, default=360)
    ap.add_argument('--verbose', '-v', action='store_true')
    args = ap.parse_args()

    port = args.port or pick_port()
    if port is None:
        print('No serial port found. Specify --port')
        return

    try:
        ser = serial.Serial(port, args.baud, timeout=1)
    except Exception as e:
        print('Failed to open serial:', e)
        return

    print('Listening on', port, 'baud', args.baud)

    # Instantiate SLAM if possible
    slam = None
    mapbytes = None
    viz = None
    if breezy_available and LaserModel is not None:
        try:
            laser = LaserModel()
            # if model exposes SCAN_SIZE, use that
            scan_size = getattr(laser, 'SCAN_SIZE', args.scan_size)
            slam = RMHC_SLAM(laser, map_size_pixels=args.map_pixels, map_size_meters=args.map_meters)
            mapbytes = bytearray(args.map_pixels * args.map_pixels)
            if Visualizer is not None:
                viz = Visualizer(args.map_meters, map_size_pixels=args.map_pixels)
            print('BreezySLAM instantiated: scan_size=', scan_size)
        except Exception as e:
            print('Failed to instantiate BreezySLAM:', e)
            slam = None
            scan_size = args.scan_size
    else:
        scan_size = args.scan_size
        print('BreezySLAM not available or no LaserModel found. Running in visualiser-only mode.')

    try:
        while True:
            raw = ser.readline()
            if not raw:
                # allow GUI events
                try:
                    import matplotlib.pyplot as plt
                    plt.pause(0.001)
                except Exception:
                    pass
                continue
            try:
                line = raw.decode('utf-8', errors='replace').strip()
            except Exception:
                line = raw.decode('latin1', errors='replace').strip()

            if line == '' or not line.startswith('>'):
                continue

            if args.verbose:
                print('RX:', line)

            if line.startswith('>lidar:') and '|xy' in line:
                body = line[len('>lidar:'):line.index('|xy')]
                pairs = [p for p in body.split(';') if p.strip()]
                xs = []
                ys = []
                for pair in pairs:
                    try:
                        a, b = pair.split(':')
                        xs.append(float(a))
                        ys.append(float(b))
                    except Exception:
                        continue

                if len(xs) == 0:
                    continue

                # Convert XY->scan
                scan = xy_points_to_scan(xs, ys, scan_size)

                if slam is not None:
                    try:
                        slam.update(scan)
                        x_mm, y_mm, theta_deg = slam.getpos()
                        slam.getmap(mapbytes)
                        if viz is not None:
                            if not viz.display(x_mm/1000.0, y_mm/1000.0, theta_deg, map_bytes=mapbytes, title='BreezySLAM'):
                                print('Visualizer window closed. Exiting.')
                                break
                    except Exception as e:
                        print('SLAM update error:', e)
                else:
                    # Fallback: if we have the simple Visualizer, plot the raw lidar points
                    if Visualizer is not None:
                        if viz is None:
                            viz = Visualizer(args.map_meters, map_size_pixels=args.map_pixels)
                        viz.plot_lidar_mm(xs, ys)
                        try:
                            import matplotlib.pyplot as plt
                            plt.pause(0.001)
                        except Exception:
                            pass

    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
