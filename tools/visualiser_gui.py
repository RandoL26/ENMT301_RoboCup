# tools/visualizer_gui.py

from __future__ import annotations

import struct
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
PACKET_STATUS = 0x07
PACKET_INFLATED_GRID = 0x08
PACKET_LOCALISATION_DEBUG = 0x09

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

# Match the mounting extrinsics in RoboSLAM::processScan().
LIDAR_OFFSET_X_M = 0.10
LIDAR_OFFSET_Y_M = -0.04
LIDAR_YAW_OFFSET_RAD = 0.0


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
    inflated_grid: Optional[np.ndarray] = None

    path_cells: Optional[np.ndarray] = None

    lidar_points: Optional[np.ndarray] = None

    uptime_ms: int = 0
    tx_queue_depth: int = 0
    dropped_frames: int = 0

    scan_packet_count: int = 0
    scan_point_count: int = 0

    start_side: Optional[int] = None
    sensor_flags: int = 0
    motor_left: int = 0
    motor_right: int = 0
    left_rpm: float = 0.0
    right_rpm: float = 0.0
    planner_goal_set: bool = False
    goal_cell: int = 0
    status_path_length: int = 0
    lidar_diag: Optional[dict] = None

        # Localisation diagnostics
    encoder_left_delta_m: float = 0.0
    encoder_right_delta_m: float = 0.0
    encoder_dtheta_rad: float = 0.0
    imu_dtheta_rad: float = 0.0
    lidar_match_score: float = 0.0
    lidar_correction_accepted: bool = False
    lidar_dx_m: float = 0.0
    lidar_dy_m: float = 0.0
    lidar_dtheta_rad: float = 0.0
    encoder_left_count: int = 0
    encoder_right_count: int = 0

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

        if packet.packet_type == PACKET_DIAG:
            return self.diagnostics(packet.payload)

        if packet.packet_type == PACKET_STATUS:
            return self.status(packet.payload)

        # Inflated obstacle grid.
        if packet.packet_type == PACKET_INFLATED_GRID:
            return self.inflated_grid(packet.payload)

        if packet.packet_type == PACKET_LOCALISATION_DEBUG:
            return self.localisation_debug(
                packet.payload
            )        

        return False

    # --------------------------------------------------------
    # INFLATED GRID
    # --------------------------------------------------------

    def inflated_grid(self, payload):

        if len(payload) < 8:
            return False

        grid_w = int.from_bytes(
            payload[0:2],
            "little",
        )

        grid_h = int.from_bytes(
            payload[2:4],
            "little",
        )

        cell_mm = int.from_bytes(
            payload[4:6],
            "little",
        )

        grid_size = int.from_bytes(
            payload[6:8],
            "little",
        )

        expected_size = (
            grid_w
            * grid_h
        )

        if grid_size != expected_size:
            return False

        if len(payload) < 8 + grid_size:
            return False

        grid = np.frombuffer(
            payload[
                8:
                8 + grid_size
            ],
            dtype=np.uint8,
        ).copy()

        try:

            grid = grid.reshape(
                (
                    grid_h,
                    grid_w,
                )
            )

        except ValueError:

            return False

        self.state.inflated_grid = grid

        self.state.grid_w = int(grid_w)
        self.state.grid_h = int(grid_h)
        self.state.cell_mm = int(cell_mm)

        return True

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
            for v in (
                x,
                y,
                theta,
            )
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
                    offset:
                    required
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
            payload[
                2:
                required
            ],
            dtype="<f4",
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

    def diagnostics(self, payload):
        # C++ emits 3 little-endian u32 values followed by 25 u16 values.
        # This is the verified 62-byte 0x06 layout in telemetry_send_diag_scan.
        if len(payload) != 62:
            return False
        values = struct.unpack("<III25H", payload)
        u16 = values[3:]
        self.state.scan_packet_count = int(values[2])
        self.state.scan_point_count = int(u16[0])
        self.state.lidar_diag = {
            "ready": values[0], "calls": values[1], "sent": values[2],
            "points": u16[0], "zero": u16[1], "invalid": u16[2],
            "min_mm": u16[3], "max_mm": u16[4], "distance_jumps": u16[5],
            "matched": u16[6], "max_delta_mm": u16[7],
            "delta_gt_100": u16[8], "delta_gt_500": u16[9],
            "delta_gt_1000": u16[10], "max_delta_angle_deg": u16[11] / 100.0,
            "mean_angle_diff_deg": u16[12] / 100.0,
            "std_angle_diff_deg": u16[13] / 100.0,
            "first_angle_deg": u16[15] / 100.0,
            "last_angle_deg": u16[16] / 100.0,
            "min_angle_deg": u16[17] / 100.0,
            "max_angle_deg": u16[18] / 100.0,
            "direction_anomalies": u16[19], "large_angle_jumps": u16[20],
            "largest_step_deg": max(u16[21], u16[22]) / 100.0,
            "angular_travel_deg": u16[23] / 100.0,
            "one_revolution": bool(u16[24]),
        }
        return True

    def status(self, payload):
        if len(payload) != 19:
            return False
        (self.state.start_side, self.state.sensor_flags,
         self.state.motor_left, self.state.motor_right,
         self.state.left_rpm, self.state.right_rpm,
         goal_set, self.state.goal_cell,
         self.state.status_path_length) = struct.unpack("<BBhhffBHH", payload)
        self.state.planner_goal_set = bool(goal_set)
        return True


    # --------------------------------------------------------
    # LOCALISATION DEBUG
    # --------------------------------------------------------

    def localisation_debug(self, payload):

        if len(payload) < 36:
            return False

        values = struct.unpack(
            "<9f",
            payload[:36],
        )

        (
            self.state.encoder_left_delta_m,
            self.state.encoder_right_delta_m,
            self.state.encoder_dtheta_rad,
            self.state.imu_dtheta_rad,
            self.state.lidar_match_score,
            self.state.lidar_correction_accepted,
            _pose_x,
            _pose_y,
            _pose_theta,
        ) = values
        self.state.lidar_correction_accepted = bool(
            self.state.lidar_correction_accepted >= 0.5
        )
        if len(payload) >= 56:
            (self.state.lidar_dx_m, self.state.lidar_dy_m,
             self.state.lidar_dtheta_rad, self.state.encoder_left_count,
             self.state.encoder_right_count) = struct.unpack("<3fii", payload[36:56])

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

        self.reset_event = threading.Event()
        self.command_queue = Queue()

    def stop(self):

        self.stop_event.set()


    def reset_data(self):

        self.reset_event.set()

    def send_command(self, command):

        self.command_queue.put(command)

    def run(self):

        parser = FrameParser()
        reconstructor = TelemetryReconstructor()

        while not self.stop_event.is_set():

            if self.reset_event.is_set():
                parser = FrameParser()
                reconstructor = TelemetryReconstructor()

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

                    if self.reset_event.is_set():
                        ser.write(b"MAP RESET\n")
                        ser.reset_input_buffer()
                        parser = FrameParser()
                        reconstructor = TelemetryReconstructor()
                        self.reset_event.clear()
                        while True:
                            try:
                                self.output_queue.get_nowait()
                            except Empty:
                                break

                    while not self.command_queue.empty():

                        try:
                            command = self.command_queue.get_nowait()

                            ser.write(
                                (command + "\n").encode("ascii")
                            )

                            print(
                                f"[serial] command: {command}"
                            )

                        except Empty:
                            break

                    while not self.stop_event.is_set():

                        # Send queued commands to the Teensy.
                        while not self.command_queue.empty():

                            try:
                                command = self.command_queue.get_nowait()

                                ser.write(
                                    (command + "\n").encode("ascii")
                                )

                                print(
                                    f"[serial] command: {command}"
                                )

                            except Empty:
                                break

                        data = ser.read(
                            4096
                        )

                        if self.reset_event.is_set():
                            ser.write(b"MAP RESET\n")
                            ser.reset_input_buffer()
                            parser = FrameParser()
                            reconstructor = TelemetryReconstructor()
                            self.reset_event.clear()
                            while True:
                                try:
                                    self.output_queue.get_nowait()
                                except Empty:
                                    break
                            continue

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

            inflated_grid=(
                None
                if state.inflated_grid is None
                else state.inflated_grid.copy()
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

            start_side=state.start_side,
            sensor_flags=state.sensor_flags,
            motor_left=state.motor_left,
            motor_right=state.motor_right,
            left_rpm=state.left_rpm,
            right_rpm=state.right_rpm,
            planner_goal_set=state.planner_goal_set,
            goal_cell=state.goal_cell,
            status_path_length=state.status_path_length,
            lidar_diag=(None if state.lidar_diag is None else dict(state.lidar_diag)),

                        encoder_left_delta_m=(
                state.encoder_left_delta_m
            ),

            encoder_right_delta_m=(
                state.encoder_right_delta_m
            ),

            encoder_dtheta_rad=(
                state.encoder_dtheta_rad
            ),

            imu_dtheta_rad=(
                state.imu_dtheta_rad
            ),

            lidar_match_score=(
                state.lidar_match_score
            ),

            lidar_correction_accepted=(
                state.lidar_correction_accepted
            ),

            lidar_dx_m=state.lidar_dx_m,
            lidar_dy_m=state.lidar_dy_m,
            lidar_dtheta_rad=state.lidar_dtheta_rad,
            encoder_left_count=state.encoder_left_count,
            encoder_right_count=state.encoder_right_count,

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

        self.localisation_label = QLabel(
            "ENCODER:\n"
            "  counts:  -- / --\n"
            "  left:    --\n"
            "  right:   --\n"
            "  dTheta:  --\n"
            "IMU:\n"
            "  dTheta:  --\n"
            "LiDAR match: --"
        )

        self.localisation_label.setStyleSheet(
            "font-family: monospace;"
        )

        side_layout.addWidget(
            self.localisation_label
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

        self.lidar_diag_label = QLabel("LiDAR: no diagnostics")
        self.lidar_diag_label.setStyleSheet("font-family: monospace;")
        telemetry_layout.addWidget(self.lidar_diag_label)

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

        # ----------------------------------------------------
        # RESET DATA
        # ----------------------------------------------------

        reset_data_button = QPushButton(
            "RESET DATA"
        )

        reset_data_button.clicked.connect(
            self.reset_data
        )

        view_layout.addWidget(
            reset_data_button
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

        self.inflated_image = pg.ImageItem()

        self.plot.addItem(
            self.inflated_image
        )

        self.inflated_image.setZValue(
            1
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

        self.inflated_image.setVisible(
            self.show_inflated
        )

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

    def reset_data(self):

        self.serial_worker.reset_data()

        while True:
            try:
                self.queue.get_nowait()
            except Empty:
                break

        # Clear telemetry state
        self.state = TelemetryState()

        # Clear trail
        self.trail.clear()

        # Clear goal
        self.goal = None

        # Clear map
        self.grid_image.clear()

        # Clear inflated map
        self.inflated_image.clear()

        # Clear path
        self.path_item.clear()

        # Clear trail display
        self.trail_item.clear()

        # Clear LiDAR
        self.lidar_item.clear()

        # Clear robot
        self.robot_item.clear()

        # Clear heading
        self.heading_item.clear()

        # Clear start/goal markers
        self.start_item.clear()
        self.goal_item.clear()

        # Reset status displays
        self.robot_label.setText(
            "START:  --\n"
            "X:      --\n"
            "Y:      --\n"
            "HEADING:--\n"
            "SPEED:  --"
        )

        self.localisation_label.setText(
            "ENCODER:\n"
            "  counts:  -- / --\n"
            "  left:    --\n"
            "  right:   --\n"
            "  dTheta:  --\n"
            "IMU:\n"
            "  dTheta:  --\n"
            "LiDAR match: --"
        )

        self.planner_label.setText(
            "STATE:  --\n"
            "GOAL:   --\n"
            "PATH:   --\n"
            "GRID:   --"
        )

        self.motor_label.setText(
            "LEFT:  --\n"
            "RIGHT: --"
        )

        self.telemetry_label.setText(
            "SEQ:       --\n"
            "SCAN PTS:  --\n"
            "QUEUE:     --\n"
            "DROPPED:   --\n"
            "UPTIME:    --"
        )
        self.lidar_diag_label.setText("LiDAR: no diagnostics")

        # Reset camera as well
        self.reset_view()

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

        if (
            state.inflated_grid is not None
            and self.show_inflated
        ):

            inflated = (
                state.inflated_grid
            )

            image = np.zeros(
                (
                    state.grid_h,
                    state.grid_w,
                ),
                dtype=np.uint8,
            )

            image[
                inflated != 0
            ] = 180

            self.inflated_image.setImage(
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

            self.inflated_image.setRect(
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
                + LIDAR_OFFSET_X_M * math.cos(state.pose_theta_rad)
                - LIDAR_OFFSET_Y_M * math.sin(state.pose_theta_rad)
                +
                distances
                * np.cos(
                    angles
                    +
                    state.pose_theta_rad
                    + LIDAR_YAW_OFFSET_RAD
                )
            )

            ys = (
                state.pose_y_m
                + LIDAR_OFFSET_X_M * math.sin(state.pose_theta_rad)
                + LIDAR_OFFSET_Y_M * math.cos(state.pose_theta_rad)
                +
                distances
                * np.sin(
                    angles
                    +
                    state.pose_theta_rad
                    + LIDAR_YAW_OFFSET_RAD
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

        self.localisation_label.setText(
            f"ENCODER:\n"
            f"  counts:  {state.encoder_left_count:8d} / {state.encoder_right_count:8d}\n"
            f"  left:    {state.encoder_left_delta_m * 1000:7.1f} mm\n"
            f"  right:   {state.encoder_right_delta_m * 1000:7.1f} mm\n"
            f"  dTheta:  {math.degrees(state.encoder_dtheta_rad):7.2f}°\n"
            f"IMU:\n"
            f"  dTheta:  {math.degrees(state.imu_dtheta_rad):7.2f}°\n"
            f"LiDAR match: {state.lidar_match_score:5.0f}/1000 "
            f"accepted={int(state.lidar_correction_accepted)}\n"
            f"  corr:    {state.lidar_dx_m*1000:6.1f}, {state.lidar_dy_m*1000:6.1f} mm "
            f"{math.degrees(state.lidar_dtheta_rad):5.2f}°"
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

        if state.lidar_diag is None:
            self.lidar_diag_label.setText("LiDAR: no diagnostics")
        else:
            d = state.lidar_diag
            self.lidar_diag_label.setText(
                f"LiDAR: {d['points']} pts, {d['angular_travel_deg']:.1f}° travel, "
                f"{d['zero']} zero, {d['invalid']} invalid\n"
                f"Range: {d['min_mm']}–{d['max_mm']} mm; "
                f"direction anomalies: {d['direction_anomalies']}, "
                f"large jumps: {d['large_angle_jumps']}"
            )

        if state.start_side is not None:
            self.start_side = "LEFT" if state.start_side == 0 else "RIGHT"
            flags = state.sensor_flags
            self.sensor_label.setText(
                f"LiDAR       {'OK' if flags & 1 else '--'}\n"
                f"BNO055      {'OK' if flags & 2 else '--'}\n"
                f"ToF         {'OK' if flags & 4 else '--'}\n"
                f"Optical     {'OK' if flags & 8 else '--'}\n"
                f"Ultrasonic  {'OK' if flags & 16 else '--'}"
            )
            self.motor_label.setText(
                f"LEFT:  {state.motor_left:5d} ({state.left_rpm:6.1f} RPM)\n"
                f"RIGHT: {state.motor_right:5d} ({state.right_rpm:6.1f} RPM)"
            )
            goal = str(state.goal_cell) if state.planner_goal_set else "--"
            self.planner_label.setText(
                f"STATE: {'FOLLOWING' if state.planner_goal_set else 'IDLE'}\n"
                f"GOAL CELL: {goal}\n"
                f"PATH: {state.status_path_length} cells\n"
                f"GRID: {state.grid_w} x {state.grid_h}"
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
