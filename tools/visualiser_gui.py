# tools/visualizer_gui.py

from __future__ import annotations

import sys
import math
import time
import threading
from queue import Queue, Empty, Full
from dataclasses import dataclass
from typing import Optional

import numpy as np
import serial
import serial.tools.list_ports

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QCheckBox,
    QPushButton,
    QGroupBox,
    QGridLayout,
    QFrame,
)

import pyqtgraph as pg


# ============================================================
# ROBOCUP TELEMETRY PROTOCOL
# ============================================================

SYNC0 = 0xA5
SYNC1 = 0x5A
VERSION = 1

HEADER_FMT = "<BBHH"
HEADER_SIZE = 6
CRC_SIZE = 2

PACKET_POSE_PATH = 0x01
PACKET_GRID_KEYFRAME = 0x02
PACKET_GRID_DELTA = 0x03
PACKET_HEARTBEAT = 0x04
PACKET_SCAN = 0x05
PACKET_DIAG = 0x06

POSE_FMT = "<fffH"
POSE_SIZE = 14

KEYFRAME_META_FMT = "<HHHH"
KEYFRAME_META_SIZE = 8

DELTA_META_FMT = "<H"
DELTA_META_SIZE = 2

HEARTBEAT_FMT = "<IHH"
HEARTBEAT_SIZE = 8

SCAN_META_FMT = "<H"
SCAN_META_SIZE = 2


DEFAULT_GRID_W = 48
DEFAULT_GRID_H = 98
DEFAULT_CELL_MM = 50


# ============================================================
# CRC
# ============================================================

def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:

    crc = init

    for b in data:

        crc ^= b << 8

        for _ in range(8):

            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


# ============================================================
# TELEMETRY STATE
# ============================================================

@dataclass
class TelemetryState:

    seq: int = 0

    pose_x_m: float = 0.0
    pose_y_m: float = 0.0
    pose_theta_rad: float = 0.0

    grid_w: int = DEFAULT_GRID_W
    grid_h: int = DEFAULT_GRID_H
    cell_mm: int = DEFAULT_CELL_MM

    occupancy: Optional[np.ndarray] = None
    terrain: Optional[np.ndarray] = None

    path_cells: Optional[np.ndarray] = None

    lidar_points: Optional[np.ndarray] = None

    uptime_ms: int = 0
    tx_queue_depth: int = 0
    dropped_frames: int = 0

    scan_packet_count: int = 0
    scan_point_count: int = 0


# ============================================================
# TELEMETRY PACKET
# ============================================================

@dataclass
class TelemetryPacket:

    packet_type: int
    seq: int
    payload: bytes


# ============================================================
# FRAME PARSER
# ============================================================

class FrameParser:

    def __init__(self):

        self.buf = bytearray()

    def feed(self, data: bytes):

        self.buf.extend(data)

        packets = []

        while True:

            sync_index = self._find_sync()

            if sync_index < 0:

                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]

                break

            if sync_index > 0:
                del self.buf[:sync_index]

            if len(self.buf) < 2 + HEADER_SIZE:
                break

            version = self.buf[2]
            packet_type = self.buf[3]

            seq = int.from_bytes(
                self.buf[4:6],
                "little",
            )

            payload_len = int.from_bytes(
                self.buf[6:8],
                "little",
            )

            total_len = (
                2
                + HEADER_SIZE
                + payload_len
                + CRC_SIZE
            )

            if len(self.buf) < total_len:
                break

            crc_position = (
                2
                + HEADER_SIZE
                + payload_len
            )

            crc_expected = int.from_bytes(
                self.buf[
                    crc_position:
                    crc_position + 2
                ],
                "little",
            )

            crc_actual = crc16_ccitt(
                bytes(
                    self.buf[
                        2:
                        crc_position
                    ]
                )
            )

            if crc_actual != crc_expected:

                del self.buf[0]

                continue

            if version != VERSION:

                del self.buf[:total_len]

                continue

            payload_start = 2 + HEADER_SIZE

            payload_end = (
                payload_start
                + payload_len
            )

            payload = bytes(
                self.buf[
                    payload_start:
                    payload_end
                ]
            )

            packets.append(
                TelemetryPacket(
                    packet_type,
                    seq,
                    payload,
                )
            )

            del self.buf[:total_len]

        return packets

    def _find_sync(self):

        for i in range(
            len(self.buf) - 1
        ):

            if (
                self.buf[i] == SYNC0
                and
                self.buf[i + 1] == SYNC1
            ):

                return i

        return -1


# ============================================================
# TELEMETRY RECONSTRUCTOR
# ============================================================

class TelemetryReconstructor:

    def __init__(self):

        self.state = TelemetryState()

    def apply(self, packet):

        self.state.seq = packet.seq

        if packet.packet_type == PACKET_POSE_PATH:
            return self.pose(packet.payload)

        if packet.packet_type == PACKET_GRID_KEYFRAME:
            return self.keyframe(packet.payload)

        if packet.packet_type == PACKET_GRID_DELTA:
            return self.delta(packet.payload)

        if packet.packet_type == PACKET_HEARTBEAT:
            return self.heartbeat(packet.payload)

        if packet.packet_type == PACKET_SCAN:
            return self.scan(packet.payload)

        return False

    # --------------------------------------------------------
    # POSE + PATH
    # --------------------------------------------------------

    def pose(self, payload):

        if len(payload) < POSE_SIZE:
            return False

        x, y, theta, path_count = \
            __import__("struct").unpack_from(
                POSE_FMT,
                payload,
                0,
            )

        if not all(
            math.isfinite(v)
            for v in (x, y, theta)
        ):
            return False

        offset = POSE_SIZE

        required = (
            offset
            + path_count * 2
        )

        if len(payload) < required:
            return False

        if path_count:

            raw = np.frombuffer(
                payload[
                    offset:required
                ],
                dtype=np.uint8,
            )

            path = raw.reshape(
                (-1, 2)
            ).astype(np.uint16)

        else:

            path = np.empty(
                (0, 2),
                dtype=np.uint16,
            )

        self.state.pose_x_m = float(x)
        self.state.pose_y_m = float(y)
        self.state.pose_theta_rad = float(theta)

        self.state.path_cells = path

        return True

    # --------------------------------------------------------
    # GRID KEYFRAME
    # --------------------------------------------------------

    def keyframe(self, payload):

        import struct

        if len(payload) < KEYFRAME_META_SIZE:
            return False

        grid_w, grid_h, cell_mm, packed_len = \
            struct.unpack_from(
                KEYFRAME_META_FMT,
                payload,
                0,
            )

        n_cells = (
            grid_w
            * grid_h
        )

        expected = (
            KEYFRAME_META_SIZE
            + packed_len
        )

        if len(payload) < expected:
            return False

        packed = payload[
            KEYFRAME_META_SIZE:
            expected
        ]

        arr = np.frombuffer(
            packed,
            dtype=np.uint8,
        )

        if arr.size == n_cells:

            cells = arr.copy()

        else:

            cells = np.empty(
                arr.size * 2,
                dtype=np.uint8,
            )

            cells[0::2] = (
                arr & 0x0F
            )

            cells[1::2] = (
                arr >> 4
            ) & 0x0F

            cells = cells[:n_cells]

        occupancy = cells & 0x03
        terrain = (
            cells >> 2
        ) & 0x03

        self.state.grid_w = int(grid_w)
        self.state.grid_h = int(grid_h)
        self.state.cell_mm = int(cell_mm)

        self.state.occupancy = (
            occupancy.reshape(
                grid_h,
                grid_w,
            ).copy()
        )

        self.state.terrain = (
            terrain.reshape(
                grid_h,
                grid_w,
            ).copy()
        )

        return True

    # --------------------------------------------------------
    # GRID DELTA
    # --------------------------------------------------------

    def delta(self, payload):

        import struct

        if (
            self.state.occupancy is None
            or
            self.state.terrain is None
        ):
            return False

        if len(payload) < 2:
            return False

        count = struct.unpack_from(
            "<H",
            payload,
            0,
        )[0]

        offset = 2

        occupancy = (
            self.state
            .occupancy
            .reshape(-1)
        )

        terrain = (
            self.state
            .terrain
            .reshape(-1)
        )

        for _ in range(count):

            if offset + 3 > len(payload):
                break

            idx, packed = struct.unpack_from(
                "<HB",
                payload,
                offset,
            )

            offset += 3

            if idx >= occupancy.size:
                continue

            occupancy[idx] = (
                packed & 0x03
            )

            terrain[idx] = (
                packed >> 2
            ) & 0x03

        return True

    # --------------------------------------------------------
    # HEARTBEAT
    # --------------------------------------------------------

    def heartbeat(self, payload):

        import struct

        if len(payload) < HEARTBEAT_SIZE:
            return False

        uptime, queue_depth, drops = \
            struct.unpack_from(
                HEARTBEAT_FMT,
                payload,
                0,
            )

        self.state.uptime_ms = uptime
        self.state.tx_queue_depth = queue_depth
        self.state.dropped_frames = drops

        if len(payload) >= 12:

            scan_packets, scan_points = \
                struct.unpack_from(
                    "<HH",
                    payload,
                    8,
                )

            self.state.scan_packet_count = (
                scan_packets
            )

            self.state.scan_point_count = (
                scan_points
            )

        return True

    # --------------------------------------------------------
    # LIDAR
    # --------------------------------------------------------

    def scan(self, payload):

        import struct

        if len(payload) < 2:
            return False

        count = struct.unpack_from(
            "<H",
            payload,
            0,
        )[0]

        required = (
            2
            + count * 8
        )

        if len(payload) < required:
            return False

        raw = np.frombuffer(
            payload[2:required],
            dtype=np.float32,
        )

        if raw.size != count * 2:
            return False

        self.state.lidar_points = (
            raw.reshape(
                (-1, 2)
            ).copy()
        )

        self.state.scan_point_count = count

        return True


# ============================================================
# SERIAL WORKER
# ============================================================

class SerialWorker(threading.Thread):

    def __init__(
        self,
        port,
        baud,
        output_queue,
    ):

        super().__init__(
            daemon=True
        )

        self.port = port
        self.baud = baud
        self.output_queue = output_queue

        self.stop_event = (
            threading.Event()
        )

    def stop(self):

        self.stop_event.set()

    def run(self):

        parser = FrameParser()
        reconstructor = (
            TelemetryReconstructor()
        )

        while not self.stop_event.is_set():

            try:

                print(
                    f"[serial] connecting "
                    f"{self.port} @ {self.baud}"
                )

                with serial.Serial(
                    self.port,
                    self.baud,
                    timeout=0.1,
                ) as ser:

                    print(
                        "[serial] connected"
                    )

                    while not self.stop_event.is_set():

                        data = ser.read(
                            4096
                        )

                        if not data:
                            continue

                        packets = (
                            parser.feed(data)
                        )

                        for packet in packets:

                            if reconstructor.apply(
                                packet
                            ):

                                self.push(
                                    reconstructor.state
                                )

            except (
                serial.SerialException,
                OSError,
            ) as e:

                print(
                    f"[serial] error: {e}"
                )

                time.sleep(1)

    def push(self, state):

        copied = TelemetryState(
            seq=state.seq,

            pose_x_m=state.pose_x_m,
            pose_y_m=state.pose_y_m,
            pose_theta_rad=state.pose_theta_rad,

            grid_w=state.grid_w,
            grid_h=state.grid_h,
            cell_mm=state.cell_mm,

            occupancy=(
                None
                if state.occupancy is None
                else state.occupancy.copy()
            ),

            terrain=(
                None
                if state.terrain is None
                else state.terrain.copy()
            ),

            path_cells=(
                None
                if state.path_cells is None
                else state.path_cells.copy()
            ),

            lidar_points=(
                None
                if state.lidar_points is None
                else state.lidar_points.copy()
            ),

            uptime_ms=state.uptime_ms,
            tx_queue_depth=state.tx_queue_depth,
            dropped_frames=state.dropped_frames,

            scan_packet_count=(
                state.scan_packet_count
            ),

            scan_point_count=(
                state.scan_point_count
            ),
        )

        try:

            self.output_queue.put_nowait(
                copied
            )

        except Full:

            try:
                self.output_queue.get_nowait()
            except Empty:
                pass

            try:
                self.output_queue.put_nowait(
                    copied
                )
            except Full:
                pass


# ============================================================
# MAIN GUI
# ============================================================

class RoboCupVisualizer(QMainWindow):

    def __init__(
        self,
        port,
        baud,
    ):

        super().__init__()

        self.setWindowTitle(
            "ENMT301 RoboCup Visualizer"
        )

        self.resize(
            1400,
            850,
        )

        # ----------------------------------------------------
        # STATE
        # ----------------------------------------------------

        self.show_grid = True
        self.show_inflated = False
        self.show_lidar = True
        self.show_path = True
        self.show_trail = True
        self.show_start = True
        self.show_goal = True
        self.show_heading = True

        self.follow_robot = False

        self.start_side = "LEFT"

        # IMPORTANT:
        # Replace these with the actual coordinates
        # from your RoboCup arena.
        self.start_left = (
            0.30,
            0.30,
        )

        self.start_right = (
            2.10,
            0.30,
        )

        self.goal = None

        self.trail = []

        self.state = None

        # ----------------------------------------------------
        # QUEUE
        # ----------------------------------------------------

        self.queue = Queue(
            maxsize=2
        )

        # ----------------------------------------------------
        # CENTRAL WIDGET
        # ----------------------------------------------------

        central = QWidget()

        self.setCentralWidget(
            central
        )

        layout = QHBoxLayout(
            central
        )

        # ----------------------------------------------------
        # MAP
        # ----------------------------------------------------

        self.plot = pg.PlotWidget()

        self.plot.setBackground(
            "#080d18"
        )

        self.plot.showGrid(
            x=True,
            y=True,
            alpha=0.2,
        )

        self.plot.setLabel(
            "bottom",
            "X",
            units="m",
        )

        self.plot.setLabel(
            "left",
            "Y",
            units="m",
        )

        self.plot.setAspectLocked(
            True
        )

        layout.addWidget(
            self.plot,
            stretch=1,
        )

        # ----------------------------------------------------
        # SIDE PANEL
        # ----------------------------------------------------

        side = QWidget()

        side.setMaximumWidth(
            320
        )

        side_layout = QVBoxLayout(
            side
        )

        layout.addWidget(
            side
        )

        # ----------------------------------------------------
        # DISPLAY
        # ----------------------------------------------------

        display_group = QGroupBox(
            "DISPLAY"
        )

        display_layout = QVBoxLayout(
            display_group
        )

        side_layout.addWidget(
            display_group
        )

        self.cb_grid = self.add_checkbox(
            display_layout,
            "Occupancy Grid",
            True,
            self.toggle_grid,
        )

        self.cb_inflated = self.add_checkbox(
            display_layout,
            "Inflated Obstacles",
            False,
            self.toggle_inflated,
        )

        self.cb_lidar = self.add_checkbox(
            display_layout,
            "LiDAR Scan",
            True,
            self.toggle_lidar,
        )

        self.cb_path = self.add_checkbox(
            display_layout,
            "Planned Path",
            True,
            self.toggle_path,
        )

        self.cb_trail = self.add_checkbox(
            display_layout,
            "Position Trail",
            True,
            self.toggle_trail,
        )

        self.cb_start = self.add_checkbox(
            display_layout,
            "Start Position",
            True,
            self.toggle_start,
        )

        self.cb_goal = self.add_checkbox(
            display_layout,
            "Goal",
            True,
            self.toggle_goal,
        )

        self.cb_heading = self.add_checkbox(
            display_layout,
            "Robot Heading",
            True,
            self.toggle_heading,
        )

        # ----------------------------------------------------
        # ROBOT INFO
        # ----------------------------------------------------

        robot_group = QGroupBox(
            "ROBOT"
        )

        robot_layout = QVBoxLayout(
            robot_group
        )

        self.robot_label = QLabel(
            "START: LEFT\n"
            "X:      --\n"
            "Y:      --\n"
            "HEADING:--\n"
            "SPEED:  --"
        )

        self.robot_label.setStyleSheet(
            "font-family: monospace;"
        )

        robot_layout.addWidget(
            self.robot_label
        )

        side_layout.addWidget(
            robot_group
        )

        # ----------------------------------------------------
        # PLANNER
        # ----------------------------------------------------

        planner_group = QGroupBox(
            "PLANNER"
        )

        planner_layout = QVBoxLayout(
            planner_group
        )

        self.planner_label = QLabel(
            "STATE: --\n"
            "GOAL:  --\n"
            "PATH:  --"
        )

        self.planner_label.setStyleSheet(
            "font-family: monospace;"
        )

        planner_layout.addWidget(
            self.planner_label
        )

        side_layout.addWidget(
            planner_group
        )

        # ----------------------------------------------------
        # SENSORS
        # ----------------------------------------------------

        sensor_group = QGroupBox(
            "SENSORS"
        )

        sensor_layout = QVBoxLayout(
            sensor_group
        )

        self.sensor_label = QLabel(
            "LiDAR       ✓\n"
            "BNO055      ✓\n"
            "ToF         ✓\n"
            "Optical     ✓\n"
            "Ultrasonic  ✓"
        )

        self.sensor_label.setStyleSheet(
            "font-family: monospace;"
        )

        sensor_layout.addWidget(
            self.sensor_label
        )

        side_layout.addWidget(
            sensor_group
        )

        # ----------------------------------------------------
        # MOTORS
        # ----------------------------------------------------

        motor_group = QGroupBox(
            "MOTORS"
        )

        motor_layout = QVBoxLayout(
            motor_group
        )

        self.motor_label = QLabel(
            "LEFT:  --\n"
            "RIGHT: --"
        )

        self.motor_label.setStyleSheet(
            "font-family: monospace;"
        )

        motor_layout.addWidget(
            self.motor_label
        )

        side_layout.addWidget(
            motor_group
        )

        # ----------------------------------------------------
        # TELEMETRY
        # ----------------------------------------------------

        telemetry_group = QGroupBox(
            "TELEMETRY"
        )

        telemetry_layout = QVBoxLayout(
            telemetry_group
        )

        self.telemetry_label = QLabel(
            "PACKETS:    --\n"
            "CRC ERRORS: --\n"
            "SCAN PTS:   --\n"
            "QUEUE:      --\n"
            "DROPPED:    --\n"
            "UPTIME:     --"
        )

        self.telemetry_label.setStyleSheet(
            "font-family: monospace;"
        )

        telemetry_layout.addWidget(
            self.telemetry_label
        )

        side_layout.addWidget(
            telemetry_group
        )

        # ----------------------------------------------------
        # VIEW CONTROLS
        # ----------------------------------------------------

        view_group = QGroupBox(
            "VIEW"
        )

        view_layout = QVBoxLayout(
            view_group
        )

        self.follow_button = QPushButton(
            "Follow Robot"
        )

        self.follow_button.clicked.connect(
            self.toggle_follow
        )

        view_layout.addWidget(
            self.follow_button
        )

        reset_button = QPushButton(
            "Reset View"
        )

        reset_button.clicked.connect(
            self.reset_view
        )

        view_layout.addWidget(
            reset_button
        )

        side_layout.addWidget(
            view_group
        )

        side_layout.addStretch()

        # ----------------------------------------------------
        # MAP ITEMS
        # ----------------------------------------------------

        self.grid_image = pg.ImageItem()

        self.plot.addItem(
            self.grid_image
        )

        self.grid_image.setZValue(
            0
        )

        self.path_item = pg.PlotDataItem(
            pen=pg.mkPen(
                "#f4c95d",
                width=3,
            )
        )

        self.plot.addItem(
            self.path_item
        )

        self.trail_item = pg.PlotDataItem(
            pen=pg.mkPen(
                "#a78bfa",
                width=2,
            )
        )

        self.plot.addItem(
            self.trail_item
        )

        self.lidar_item = pg.ScatterPlotItem(
            size=5,
            brush=pg.mkBrush(
                "#7dd3fc"
            ),
        )

        self.plot.addItem(
            self.lidar_item
        )

        self.robot_item = pg.ScatterPlotItem(
            size=14,
            brush=pg.mkBrush(
                "#ffffff"
            ),
            pen=pg.mkPen(
                "#000000",
                width=2,
            ),
        )

        self.plot.addItem(
            self.robot_item
        )

        self.heading_item = pg.PlotDataItem(
            pen=pg.mkPen(
                "#6ea8fe",
                width=4,
            )
        )

        self.plot.addItem(
            self.heading_item
        )

        self.start_item = pg.ScatterPlotItem(
            size=15,
            brush=pg.mkBrush(
                "#55c878"
            ),
        )

        self.plot.addItem(
            self.start_item
        )

        self.goal_item = pg.ScatterPlotItem(
            size=20,
            brush=pg.mkBrush(
                "#f4c95d"
            ),
        )

        self.plot.addItem(
            self.goal_item
        )

        # ----------------------------------------------------
        # SERIAL
        # ----------------------------------------------------

        self.serial_worker = SerialWorker(
            port,
            baud,
            self.queue,
        )

        self.serial_worker.start()

        # ----------------------------------------------------
        # GUI TIMER
        # ----------------------------------------------------

        self.timer = QTimer()

        self.timer.timeout.connect(
            self.update_visualizer
        )

        self.timer.start(
            50
        )

    # ========================================================
    # CHECKBOX HELPER
    # ========================================================

    def add_checkbox(
        self,
        layout,
        text,
        checked,
        callback,
    ):

        checkbox = QCheckBox(
            text
        )

        checkbox.setChecked(
            checked
        )

        checkbox.stateChanged.connect(
            callback
        )

        layout.addWidget(
            checkbox
        )

        return checkbox

    # ========================================================
    # DISPLAY TOGGLES
    # ========================================================

    def toggle_grid(self):

        self.show_grid = (
            self.cb_grid.isChecked()
        )

        self.grid_image.setVisible(
            self.show_grid
        )

    def toggle_inflated(self):

        self.show_inflated = (
            self.cb_inflated.isChecked()
        )

        # Real inflated occupancy should eventually
        # come from MappingNav telemetry.
        #
        # For now this toggle is reserved for that layer.

    def toggle_lidar(self):

        self.show_lidar = (
            self.cb_lidar.isChecked()
        )

        self.lidar_item.setVisible(
            self.show_lidar
        )

    def toggle_path(self):

        self.show_path = (
            self.cb_path.isChecked()
        )

        self.path_item.setVisible(
            self.show_path
        )

    def toggle_trail(self):

        self.show_trail = (
            self.cb_trail.isChecked()
        )

        self.trail_item.setVisible(
            self.show_trail
        )

    def toggle_start(self):

        self.show_start = (
            self.cb_start.isChecked()
        )

        self.start_item.setVisible(
            self.show_start
        )

    def toggle_goal(self):

        self.show_goal = (
            self.cb_goal.isChecked()
        )

        self.goal_item.setVisible(
            self.show_goal
        )

    def toggle_heading(self):

        self.show_heading = (
            self.cb_heading.isChecked()
        )

        self.heading_item.setVisible(
            self.show_heading
        )

    # ========================================================
    # FOLLOW ROBOT
    # ========================================================

    def toggle_follow(self):

        self.follow_robot = (
            not self.follow_robot
        )

        if self.follow_robot:

            self.follow_button.setText(
                "Following Robot"
            )

        else:

            self.follow_button.setText(
                "Follow Robot"
            )

    # ========================================================
    # RESET VIEW
    # ========================================================

    def reset_view(self):

        self.follow_robot = False

        self.follow_button.setText(
            "Follow Robot"
        )

        if self.state is not None:

            width = (
                self.state.grid_w
                * self.state.cell_mm
                / 1000.0
            )

            height = (
                self.state.grid_h
                * self.state.cell_mm
                / 1000.0
            )

        else:

            width = 2.4
            height = 4.9

        self.plot.setXRange(
            0,
            width,
            padding=0.03,
        )

        self.plot.setYRange(
            0,
            height,
            padding=0.03,
        )

    # ========================================================
    # UPDATE VISUALIZER
    # ========================================================

    def update_visualizer(self):

        latest = None

        while True:

            try:
                latest = (
                    self.queue.get_nowait()
                )

            except Empty:
                break

        if latest is None:
            return

        self.state = latest

        state = latest

        # ----------------------------------------------------
        # GRID
        # ----------------------------------------------------

        if (
            state.occupancy is not None
            and
            self.show_grid
        ):

            occupancy = (
                state.occupancy
            )

            # Display:
            # 0 = unknown
            # 1 = free
            # 2 = occupied

            image = np.zeros(
                (
                    state.grid_h,
                    state.grid_w,
                ),
                dtype=np.uint8,
            )

            image[
                occupancy == 0
            ] = 70

            image[
                occupancy == 1
            ] = 220

            image[
                occupancy == 2
            ] = 20

            self.grid_image.setImage(
                image.T,
                autoLevels=False,
            )

            width = (
                state.grid_w
                * state.cell_mm
                / 1000.0
            )

            height = (
                state.grid_h
                * state.cell_mm
                / 1000.0
            )

            self.grid_image.setRect(
                0,
                0,
                width,
                height,
            )

        # ----------------------------------------------------
        # ROBOT
        # ----------------------------------------------------

        self.robot_item.setData(
            [state.pose_x_m],
            [state.pose_y_m],
        )

        # ----------------------------------------------------
        # HEADING
        # ----------------------------------------------------

        heading_length = 0.18

        hx = (
            state.pose_x_m
            +
            heading_length
            * math.cos(
                state.pose_theta_rad
            )
        )

        hy = (
            state.pose_y_m
            +
            heading_length
            * math.sin(
                state.pose_theta_rad
            )
        )

        self.heading_item.setData(
            [
                state.pose_x_m,
                hx,
            ],
            [
                state.pose_y_m,
                hy,
            ],
        )

        # ----------------------------------------------------
        # PATH
        # ----------------------------------------------------

        if (
            state.path_cells is not None
            and
            len(state.path_cells) > 0
        ):

            cell_m = (
                state.cell_mm
                / 1000.0
            )

            px = (
                state.path_cells[:, 0]
                + 0.5
            ) * cell_m

            py = (
                state.path_cells[:, 1]
                + 0.5
            ) * cell_m

            self.path_item.setData(
                px,
                py,
            )

            # Automatically use last path cell
            # as displayed goal.
            self.goal = (
                float(px[-1]),
                float(py[-1]),
            )

        # ----------------------------------------------------
        # GOAL
        # ----------------------------------------------------

        if self.goal is not None:

            self.goal_item.setData(
                [self.goal[0]],
                [self.goal[1]],
            )

        # ----------------------------------------------------
        # TRAIL
        # ----------------------------------------------------

        self.trail.append(
            (
                state.pose_x_m,
                state.pose_y_m,
            )
        )

        if len(self.trail) > 500:

            self.trail.pop(0)

        if self.trail:

            tx = [
                p[0]
                for p in self.trail
            ]

            ty = [
                p[1]
                for p in self.trail
            ]

            self.trail_item.setData(
                tx,
                ty,
            )

        # ----------------------------------------------------
        # LIDAR
        # ----------------------------------------------------

        if (
            state.lidar_points is not None
            and
            len(state.lidar_points) > 0
        ):

            angles = (
                state.lidar_points[:, 0]
            )

            distances = (
                state.lidar_points[:, 1]
            )

            xs = (
                state.pose_x_m
                +
                distances
                * np.cos(
                    angles
                    +
                    state.pose_theta_rad
                )
            )

            ys = (
                state.pose_y_m
                +
                distances
                * np.sin(
                    angles
                    +
                    state.pose_theta_rad
                )
            )

            self.lidar_item.setData(
                x=xs,
                y=ys,
            )

        # ----------------------------------------------------
        # START
        # ----------------------------------------------------

        if self.start_side == "LEFT":

            sx, sy = self.start_left

        else:

            sx, sy = self.start_right

        self.start_item.setData(
            [sx],
            [sy],
        )

        # ----------------------------------------------------
        # FOLLOW ROBOT
        # ----------------------------------------------------

        if self.follow_robot:

            view_width = 2.0
            view_height = 2.0

            self.plot.setXRange(
                state.pose_x_m
                - view_width / 2,
                state.pose_x_m
                + view_width / 2,
                padding=0,
            )

            self.plot.setYRange(
                state.pose_y_m
                - view_height / 2,
                state.pose_y_m
                + view_height / 2,
                padding=0,
            )

        # ----------------------------------------------------
        # ROBOT INFO
        # ----------------------------------------------------

        theta_deg = math.degrees(
            state.pose_theta_rad
        )

        self.robot_label.setText(
            f"START:  {self.start_side}\n"
            f"X:      {state.pose_x_m:7.3f} m\n"
            f"Y:      {state.pose_y_m:7.3f} m\n"
            f"HEADING:{theta_deg:7.1f}°\n"
            f"SPEED:  --"
        )

        # ----------------------------------------------------
        # PLANNER INFO
        # ----------------------------------------------------

        path_length = 0

        if state.path_cells is not None:

            path_length = len(
                state.path_cells
            )

        goal_text = "--"

        if self.goal is not None:

            goal_text = (
                f"{self.goal[0]:.2f}, "
                f"{self.goal[1]:.2f}"
            )

        self.planner_label.setText(
            f"STATE: FOLLOWING\n"
            f"GOAL:  {goal_text}\n"
            f"PATH:  {path_length} cells\n"
            f"GRID:  {state.grid_w} x "
            f"{state.grid_h}"
        )

        # ----------------------------------------------------
        # TELEMETRY
        # ----------------------------------------------------

        self.telemetry_label.setText(
            f"SEQ:       {state.seq}\n"
            f"SCAN PTS:  {state.scan_point_count}\n"
            f"QUEUE:     {state.tx_queue_depth}\n"
            f"DROPPED:   {state.dropped_frames}\n"
            f"UPTIME:    "
            f"{state.uptime_ms / 1000:.1f} s"
        )

    # ========================================================
    # CLOSE
    # ========================================================

    def closeEvent(self, event):

        self.serial_worker.stop()

        self.serial_worker.join(
            timeout=1
        )

        event.accept()


# ============================================================
# PORT DETECTION
# ============================================================

def find_port():

    ports = list(
        serial.tools.list_ports.comports()
    )

    if not ports:
        return None

    for port in ports:

        description = (
            port.description
            or ""
        ).lower()

        if (
            "teensy" in description
            or
            "usb" in description
            or
            "serial" in description
        ):

            return port.device

    return ports[0].device


# ============================================================
# MAIN
# ============================================================

def main():

    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--port",
        default=None,
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
    )

    args = parser.parse_args()

    port = (
        args.port
        or
        find_port()
    )

    if port is None:

        raise SystemExit(
            "No serial port found."
        )

    app = QApplication(
        sys.argv
    )

    app.setStyle(
        "Fusion"
    )

    window = RoboCupVisualizer(
        port,
        args.baud,
    )

    window.show()

    sys.exit(
        app.exec()
    )


if __name__ == "__main__":

    main()