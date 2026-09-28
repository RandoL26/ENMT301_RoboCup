#!/usr/bin/env python3
"""Simulate robot telemetry to test tools.roboviz visualiser without serial hardware.

Usage:
  python tools/sim_visualiser_input.py

This will create the Visualizer, send a map_size, then periodically send pose and
lidar teleplot lines so you can confirm the Matplotlib window updates live.
"""
import sys, os, time, math, importlib.util
ROOT = os.path.dirname(os.path.dirname(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# Try direct import; fallback to file load if in system Python
try:
    from tools.roboviz import process_serial_line
except ModuleNotFoundError:
    spec = importlib.util.spec_from_file_location("roboviz", os.path.join(ROOT, "tools", "roboviz", "__init__.py"))
    roboviz = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(roboviz)
    process_serial_line = roboviz.process_serial_line

print('Starting simulator. Creating map...')
process_serial_line('>map_size:2.4,4.9,800|map')
# initial pose
x = 1.0
y = 0.5
theta = 0.0

try:
    for t in range(2000):
        # move in a small circle
        theta += 2.0
        x += 0.01 * math.cos(math.radians(theta))
        y += 0.01 * math.sin(math.radians(theta))
        line = f'>pose:{x:.3f},{y:.3f},{theta:.2f}|pose'
        process_serial_line(line)

        # generate a few lidar points around robot in mm
        pts = []
        for a in range(0, 360, 20):
            r = 500 + 100 * math.sin(math.radians(t + a))
            # convert polar to xy in mm (robot-centered)
            ang = math.radians(a + theta)
            px = r * math.cos(ang)
            py = r * math.sin(ang)
            pts.append(f'{px:.1f}:{py:.1f}')
        lidar_line = '>lidar:' + ';'.join(pts) + '|xy'
        process_serial_line(lidar_line)

        time.sleep(0.05)
except KeyboardInterrupt:
    print('\nSimulator stopped')
