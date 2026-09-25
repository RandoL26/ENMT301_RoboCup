'''
roboviz.py - Python classes for displaying maps and robots

Requires: numpy, matplotlib

Copyright (C) 2018 Simon D. Levy

This file is part of PyRoboViz.

PyRoboViz is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

PyRoboViz is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
'''

# Essential imports
import matplotlib.pyplot as plt
import matplotlib.cm as colormap
import matplotlib.lines as mlines
import numpy as np

# This helps with Raspberry Pi
# Try to select a usable interactive backend. Prefer Qt5, then Tk, else leave default.
import matplotlib
def _select_backend():
    for b in ('Qt5Agg', 'TkAgg'):
        try:
            matplotlib.use(b)
            return b
        except Exception:
            continue
    return matplotlib.get_backend()

_selected_backend = _select_backend()


class Visualizer(object):

    # Robot display params
    ROBOT_HEIGHT_M = 0.5
    ROBOT_WIDTH_M = 0.3

    def __init__(self, map_size_meters, map_size_pixels=800):

        # Store constants for display
        self.map_size_meters = map_size_meters
        self.map_size_pixels = map_size_pixels

        # Create a byte array to display the map with a color overlay
        self.bgrbytes = bytearray(map_size_pixels * map_size_pixels * 3)
        # Single-channel occupancy grid (0..255) persistent map; 255==free/white, 0==occupied/black
        self.map_bytes = bytearray([255] * (map_size_pixels * map_size_pixels))

        # Make a nice big (10"x10") figure
        fig = plt.figure(figsize=(10, 10))

        # Store Python ID of figure to detect window close
        self.figid = id(fig)

        # Use an "artist" to speed up map drawing
        self.img_artist = None

        # No vehicle to show yet
        self.vehicle = None

        # Create axes
        self.ax = fig.gca()
        self.ax.grid(False)

        # Store previous position for trajectory
        self.prevpos = None

        self.rotate_angle = 0
        self.zero_angle = 0
        self.start_angle = 0
        # Lidar scatter artist (updated by teleplot parser)
        self.lidar_scatter = None

    def plot_lidar_mm(self, xs_mm, ys_mm):
        """Plot lidar points given arrays of X and Y in millimetres.

        Converts mm -> meters -> pixel coordinates used by the visualiser.
        """
        if len(xs_mm) == 0:
            return

        # convert mm -> meters
        xs_m = [x / 1000.0 for x in xs_mm]
        ys_m = [y / 1000.0 for y in ys_mm]

        # scale factor: meters per pixel
        s = self.map_size_meters / self.map_size_pixels

        px = [x / s for x in xs_m]
        py = [y / s for y in ys_m]

        coords = np.column_stack((px, py))

        # Update persistent occupancy grid: mark lidar returns as occupied
        for i in range(len(px)):
            try:
                xi = int(round(px[i]))
                yi = int(round(py[i]))
            except Exception:
                continue
            if xi < 0 or yi < 0 or xi >= self.map_size_pixels or yi >= self.map_size_pixels:
                continue
            # index into row-major map (y * width + x)
            idx = yi * self.map_size_pixels + xi
            self.map_bytes[idx] = 0
            # optionally mark a small neighborhood so points are visible at low resolution
            if xi+1 < self.map_size_pixels:
                self.map_bytes[yi * self.map_size_pixels + (xi+1)] = 0
            if xi-1 >= 0:
                self.map_bytes[yi * self.map_size_pixels + (xi-1)] = 0
            if yi+1 < self.map_size_pixels:
                self.map_bytes[(yi+1) * self.map_size_pixels + xi] = 0
            if yi-1 >= 0:
                self.map_bytes[(yi-1) * self.map_size_pixels + xi] = 0

        # Keep the scatter for a live overlay, but always update the persistent map image
        if self.lidar_scatter is None:
            self.lidar_scatter = self.ax.scatter(px, py, c='c', s=2)
        else:
            self.lidar_scatter.set_offsets(coords)

        # Update the displayed map image from the accumulated occupancy grid
        try:
            self._showMap(self.map_bytes)
        except Exception:
            pass

    def display(self, x_m, y_m, theta_deg,
                start_angle=0,
                title='',
                flip_axes=False,
                map_bytes=None,
                show_trajectory=False,
                obstacles=[]):

        # print(x_m, y_m)

        self._set_pose(x_m, y_m, theta_deg, start_angle,
                      show_trajectory, flip_axes)

        # Always show the full map area with fixed axis limits (map-centric, not robot-centric)
        # The map goes from 0 to map_size_meters in both directions
        # Convert to pixel coordinates
        self.ax.set_xlim([0, self.map_size_pixels])
        self.ax.set_ylim([0, self.map_size_pixels])
        self.ax.set_aspect('equal')

        if map_bytes is not None:
            self._showMap(map_bytes)

        self._show_obstacles(obstacles, flip_axes)

        plt.title(title)

        self.ax.set_xlabel('Y (mm)' if flip_axes else 'X (mm)')
        self.ax.set_ylabel('X (mm)' if flip_axes else 'Y (mm)')

        return self._refresh()

    def _showMap(self, map_bytes):

        mapimg = np.reshape(np.frombuffer(map_bytes, dtype=np.uint8),
                            (self.map_size_pixels, self.map_size_pixels))

        if self.img_artist is None:

            self.img_artist = self.ax.imshow(mapimg, cmap=colormap.gray)

        else:

            self.img_artist.set_data(mapimg)

    def _show_obstacles(self, obstacles, flip_axes):

        for obst in obstacles:
            xs = [x * 100 for x in obst['x']] # (0, 100, 100, 0, 0)
            ys = [y * 100 for y in obst['y']] # (0, 0, 100, 100, 0)
            plt.fill(xs, ys, color='black')


    def _set_pose(self, x_m, y_m, theta_deg, start_angle, showtraj, flip_axes):

        # If zero-angle was indicated, grab first angle to compute rotation
        if start_angle is None and self.zero_angle != 0:
            start_angle = theta_deg
            self.rotate_angle = self.zero_angle - self.start_angle

        # Flip axes if indicated
        if flip_axes:
            x_m, y_m = y_m, x_m
            theta_deg = 90 - theta_deg

        # Rotate by computed angle, or zero if no zero-angle indicated
        d = self.rotate_angle
        a = np.radians(d)
        c = np.cos(a)
        s = np.sin(a)
        x_m, y_m = x_m*c-y_m*s, y_m*c+x_m*s

        # Erase previous vehicle image after first iteration
        if self.vehicle is not None:
            self.vehicle.remove()

        # Use a very short arrow shaft to orient the head of the arrow
        theta_rad = np.radians(theta_deg+d)
        c = np.cos(theta_rad)
        s = np.sin(theta_rad)
        L = 0.1
        dx = L * c
        dy = L * s

        s = self.map_size_meters / self.map_size_pixels

        self.vehicle = self.ax.arrow(x_m/s, y_m/s, dx, dy,
                                     head_width=Visualizer.ROBOT_WIDTH_M/s,
                                     head_length=Visualizer.ROBOT_HEIGHT_M/s,
                                     fc='r', ec='r')

        # Show trajectory if indicated
        currpos = x_m/s, y_m/s
        if showtraj and self.prevpos is not None:
            self.ax.add_line(mlines.Line2D((self.prevpos[0], currpos[0]),
                             (self.prevpos[1], currpos[1])))
        self.prevpos = currpos

    def _refresh(self):

        # If we have a new figure, something went wrong (closing figure failed)
        if self.figid != id(plt.gcf()):
            return False

        # Redraw current objects without blocking
        plt.draw()

        # Refresh display, setting flag on window close or keyboard interrupt
        try:
            plt.pause(.01)  # Arbitrary pause to force redraw
            return True
        except Exception:
            return False

        return True


# -- Serial line parser for telemetry emitted by the robot
_global_viz = None

def process_serial_line(line):
    """Parse a single telemetry line from the robot and update the visualiser.

    Supported messages:
      >map_size:width_m,height_m,pixels|map
      >pose:x_m,y_m,theta_deg|pose
      >lidar:x1:y1;x2:y2;...|xy   (teleplot-style XY pairs in mm)
    """
    global _global_viz
    if not line:
        return None

    line = line.strip()
    if not line.startswith('>'):
        return None

    try:
        # map_size
        if line.startswith('>map_size:') and '|map' in line:
            body = line[len('>map_size:'):line.index('|map')]
            parts = [p.strip() for p in body.split(',')]
            if len(parts) >= 3:
                w = float(parts[0])
                h = float(parts[1])
                pixels = int(float(parts[2]))
                # Visualizer is square; use the larger arena dimension as map size
                map_m = max(w, h)
                _global_viz = Visualizer(map_m, map_size_pixels=pixels)
                try:
                    import matplotlib.pyplot as plt
                    plt.show(block=False)
                    _global_viz._refresh()
                except Exception:
                    pass
                return _global_viz

        # pose
        if line.startswith('>pose:') and '|pose' in line:
            body = line[len('>pose:'):line.index('|pose')]
            parts = [p.strip() for p in body.split(',')]
            if len(parts) >= 3:
                x = float(parts[0])
                y = float(parts[1])
                theta = float(parts[2])
                # Create visualizer on demand if not yet created
                if _global_viz is None:
                    _global_viz = Visualizer(5.0, map_size_pixels=800)  # Default arena size
                    try:
                        import matplotlib.pyplot as plt
                        plt.show(block=False)
                        _global_viz._refresh()
                    except Exception:
                        pass
                # draw (no map bytes) and show trajectory
                _global_viz.display(x, y, theta, show_trajectory=True)
                try:
                    import matplotlib.pyplot as plt
                    plt.pause(0.001)
                except Exception:
                    pass
                return _global_viz

        # lidar teleplot (xy)
        if line.startswith('>lidar:') and '|xy' in line:
            body = line[len('>lidar:'):line.index('|xy')]
            pairs = [p for p in body.split(';') if p.strip()]
            xs = []
            ys = []
            for pair in pairs:
                try:
                    a, b = pair.split(':')
                    xv = float(a)
                    yv = float(b)
                    xs.append(xv)
                    ys.append(yv)
                except Exception:
                    continue
            if len(xs) > 0:
                # Create visualizer on demand if not yet created
                if _global_viz is None:
                    _global_viz = Visualizer(5.0, map_size_pixels=800)  # Default arena size
                    try:
                        import matplotlib.pyplot as plt
                        plt.show(block=False)
                        _global_viz._refresh()
                    except Exception:
                        pass
                # ld06 teleplot prints x/y in mm; plot as mm
                _global_viz.plot_lidar_mm(xs, ys)
                try:
                    import matplotlib.pyplot as plt
                    plt.pause(0.001)
                except Exception:
                    pass
                return _global_viz

    except Exception:
        # be silent on parse errors
        return None

    return None
