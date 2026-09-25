#!/usr/bin/env python3
"""
PyRoboViz: Simple Matplotlib-based visualizer for BreezySLAM occupancy grids.

Provides the `Visualizer` class for displaying real-time SLAM maps with:
- Occupancy grid (black = occupied, white = free)
- Robot pose (position + orientation)
- Current LiDAR scan overlay
- Optional trajectory history
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge
from matplotlib.collections import LineCollection


class Visualizer:
    """
    Live SLAM map visualizer using matplotlib.
    
    Usage:
        viz = Visualizer(map_size_meters=32, map_size_pixels=800)
        
        # In main loop:
        if viz.display(x_mm, y_mm, theta_deg, map_bytes=mapbytes):
            # Continue running
        else:
            # Window was closed
            break
    """
    
    def __init__(self, map_size_meters=32.0, map_size_pixels=800):
        """
        Initialize the visualizer.
        
        Args:
            map_size_meters: Physical size represented by the map (meters)
            map_size_pixels: Grid resolution (pixels per side)
        """
        self.map_size_meters = map_size_meters
        self.map_size_pixels = map_size_pixels
        self.pixel_to_mm = (map_size_meters * 1000.0) / map_size_pixels
        
        # Create figure and axis
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-map_size_meters/2, map_size_meters/2)
        self.ax.set_ylim(-map_size_meters/2, map_size_meters/2)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('BreezySLAM SLAM Map')
        self.ax.grid(True, alpha=0.3)
        
        # Image for occupancy grid
        self.im = self.ax.imshow(
            np.zeros((map_size_pixels, map_size_pixels), dtype=np.uint8),
            extent=[-map_size_meters/2, map_size_meters/2, 
                   -map_size_meters/2, map_size_meters/2],
            origin='lower',
            cmap='gray',
            interpolation='nearest'
        )
        
        # Robot pose indicators
        self.robot_circle = None
        self.robot_arrow = None
        self.scan_scatter = None
        
        # Trajectory history
        self.trajectory_line = None
        self.trajectory_x = []
        self.trajectory_y = []
        self.max_trajectory_points = 500
        
        plt.draw()
    
    def display(self, x_mm, y_mm, theta_deg, map_bytes=None, lidar_x=None, lidar_y=None, title=None):
        """
        Update the display with current robot state and map.
        
        Args:
            x_mm: Robot X position in millimeters
            y_mm: Robot Y position in millimeters
            theta_deg: Robot orientation in degrees (0=East, 90=North)
            map_bytes: Occupancy grid as bytearray (0-255 per pixel)
            lidar_x: List of current LiDAR X points in millimeters (optional)
            lidar_y: List of current LiDAR Y points in millimeters (optional)
            title: Custom title string (optional)
        
        Returns:
            True if window is open, False if user closed it
        """
        try:
            # Convert mm to meters
            x_m = x_mm / 1000.0
            y_m = y_mm / 1000.0
            
            # Update map image if provided
            if map_bytes is not None:
                # Reshape bytes to grid and update image
                grid = np.frombuffer(map_bytes, dtype=np.uint8).reshape(
                    (self.map_size_pixels, self.map_size_pixels)
                )
                # Flip for correct orientation (BreezySLAM uses different convention)
                grid = np.flipud(grid)
                self.im.set_data(grid)
            
            # Remove old robot indicators
            if self.robot_circle is not None:
                self.robot_circle.remove()
            if self.robot_arrow is not None:
                self.robot_arrow.remove()
            if self.scan_scatter is not None:
                self.scan_scatter.remove()
            
            # Draw robot position as circle
            self.robot_circle = Circle(
                (x_m, y_m), 0.1, color='red', zorder=10, label='Robot'
            )
            self.ax.add_patch(self.robot_circle)
            
            # Draw robot heading as arrow
            arrow_len = 0.3
            dx = arrow_len * np.cos(np.radians(theta_deg))
            dy = arrow_len * np.sin(np.radians(theta_deg))
            self.robot_arrow = self.ax.arrow(
                x_m, y_m, dx, dy,
                head_width=0.15, head_length=0.1,
                fc='red', ec='red', zorder=10
            )
            
            # Plot current LiDAR scan if provided
            if lidar_x is not None and lidar_y is not None:
                lidar_x_m = np.array(lidar_x) / 1000.0
                lidar_y_m = np.array(lidar_y) / 1000.0
                self.scan_scatter = self.ax.scatter(
                    lidar_x_m, lidar_y_m,
                    c='cyan', s=5, alpha=0.6, zorder=5, label='LiDAR scan'
                )
            
            # Add to trajectory
            self.trajectory_x.append(x_m)
            self.trajectory_y.append(y_m)
            if len(self.trajectory_x) > self.max_trajectory_points:
                self.trajectory_x.pop(0)
                self.trajectory_y.pop(0)
            
            # Remove old trajectory line
            if self.trajectory_line is not None:
                self.trajectory_line.remove()
            
            # Draw trajectory as line
            if len(self.trajectory_x) > 1:
                self.trajectory_line, = self.ax.plot(
                    self.trajectory_x, self.trajectory_y,
                    'g-', alpha=0.3, linewidth=1, zorder=1, label='Trajectory'
                )
            
            # Update title
            if title is None:
                title = f'BreezySLAM SLAM | Pos: ({x_m:.2f}, {y_m:.2f}) | Heading: {theta_deg:.1f}°'
            self.ax.set_title(title)
            
            # Update legend
            self.ax.legend(loc='upper right', fontsize=8)
            
            # Redraw
            self.fig.canvas.draw_idle()
            plt.pause(0.001)  # Allow GUI events
            
            # Check if window was closed
            if not plt.fignum_exists(self.fig.number):
                return False
            
            return True
            
        except Exception as e:
            print(f'Visualizer error: {e}')
            return False
    
    def plot_lidar_mm(self, xs_mm, ys_mm):
        """
        Plot raw LiDAR points (without SLAM map).
        
        Args:
            xs_mm: List of X points in millimeters
            ys_mm: List of Y points in millimeters
        """
        try:
            xs_m = np.array(xs_mm) / 1000.0
            ys_m = np.array(ys_mm) / 1000.0
            
            if self.scan_scatter is not None:
                self.scan_scatter.remove()
            
            self.scan_scatter = self.ax.scatter(
                xs_m, ys_m, c='cyan', s=5, alpha=0.6, zorder=5
            )
            
            self.fig.canvas.draw_idle()
            plt.pause(0.001)
            
            if not plt.fignum_exists(self.fig.number):
                return False
            return True
            
        except Exception as e:
            print(f'Error plotting LiDAR points: {e}')
            return False
