#!/usr/bin/env python3
"""Visualiser for Teensy mapping telemetry (binary framed protocol 0xA5 0x5A)."""

from __future__ import annotations

import argparse
import struct
import threading
import time
from dataclasses import dataclass
from queue import Queue, Empty, Full
from typing import Optional

import numpy as np
import serial
import serial.tools.list_ports

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches


SYNC0 = 0xA5
SYNC1 = 0x5A
HEADER_FMT = "<BHH"  # version, seq, payload_len
HEADER_SIZE = struct.calcsize(HEADER_FMT)
POSE_META_FMT = "<fffHHHH"  # x,y,theta,grid_w,grid_h,cell_mm,path_count
POSE_META_SIZE = struct.calcsize(POSE_META_FMT)
CRC_SIZE = 2

# Expected arena/grid defaults (used for display extent and fallback checks)
DEFAULT_GRID_W = 48
DEFAULT_GRID_H = 98
DEFAULT_CELL_MM = 50

#!/usr/bin/env python3
"""
Live Teensy telemetry visualizer for multiplexed BT packets.

Protocol (binary, resilient framing):
- Sync: 0xA5 0x5A
- Header:
    u8  version
    u8  packet_type
    u16 sequence
    u16 payload_len
- Payloads:
    0x01 PosePath
        f32 x_m
        f32 y_m
        f32 theta_rad
        u16 path_count
        path[path_count] as u8 cx, u8 cy
    0x02 GridKeyframe
        u16 grid_w
        u16 grid_h
        u16 cell_mm
        u16 packed_len
        packed_grid[packed_len]
        where packed nibble = [3:2] terrain, [1:0] occupancy
    0x03 GridDelta
        u16 change_count
        changes[change_count] as u16 idx, u8 packed_cell
    0x04 Heartbeat/Stats
        u32 uptime_ms
        u16 tx_queue_depth
        u16 dropped_frames
- Footer:
    u16 CRC16-CCITT over header + payload, excluding sync and CRC itself.

This script keeps the full grid reconstructed on the PC side from a keyframe plus any deltas.
It also ignores malformed frames and retries serial disconnects automatically.
"""

SYNC0 = 0xA5
SYNC1 = 0x5A
VERSION = 1

HEADER_FMT = "<BBHH"  # version, packet_type, sequence, payload_len
HEADER_SIZE = struct.calcsize(HEADER_FMT)
CRC_SIZE = 2

PACKET_POSE_PATH = 0x01
PACKET_GRID_KEYFRAME = 0x02
PACKET_GRID_DELTA = 0x03
PACKET_HEARTBEAT = 0x04
PACKET_SCAN = 0x05

POSE_FMT = "<fffH"
POSE_SIZE = struct.calcsize(POSE_FMT)
KEYFRAME_META_FMT = "<HHHH"
KEYFRAME_META_SIZE = struct.calcsize(KEYFRAME_META_FMT)
DELTA_META_FMT = "<H"
DELTA_META_SIZE = struct.calcsize(DELTA_META_FMT)
HEARTBEAT_FMT = "<IHH"
HEARTBEAT_SIZE = struct.calcsize(HEARTBEAT_FMT)
SCAN_META_FMT = "<H"  # point_count
SCAN_META_SIZE = struct.calcsize(SCAN_META_FMT)

DEFAULT_GRID_W = 48
DEFAULT_GRID_H = 98
DEFAULT_CELL_MM = 50
DEFAULT_ARENA_W_M = 2.4
DEFAULT_ARENA_H_M = 4.9


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
    uptime_ms: int = 0
    tx_queue_depth: int = 0
    dropped_frames: int = 0
    # Optional latest lidar scan points as Nx2 array (angle_rad, distance_m)
    lidar_points: Optional[np.ndarray] = None
    # Scan counters
    scan_packet_count: int = 0
    scan_point_count: int = 0


@dataclass
class TelemetryPacket:
    packet_type: int
    seq: int
    payload: bytes


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


def decode_packed_grid(packed: bytes, n_cells: int) -> tuple[np.ndarray, np.ndarray]:
    arr = np.frombuffer(packed, dtype=np.uint8)
    cells = np.empty(arr.size * 2, dtype=np.uint8)
    cells[0::2] = arr & 0x0F
    cells[1::2] = (arr >> 4) & 0x0F
    cells = cells[:n_cells]
    occupancy = cells & 0x03
    terrain = (cells >> 2) & 0x03
    return occupancy, terrain


class FrameParser:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> list[TelemetryPacket]:
        self.buf.extend(data)
        packets: list[TelemetryPacket] = []

        while True:
            sync_idx = self._find_sync()
            if sync_idx < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                break

            if sync_idx > 0:
                del self.buf[:sync_idx]

            if len(self.buf) < 2 + HEADER_SIZE:
                break

            version, packet_type, seq, payload_len = struct.unpack_from(HEADER_FMT, self.buf, 2)
            total_len = 2 + HEADER_SIZE + payload_len + CRC_SIZE
            if len(self.buf) < total_len:
                break

            crc_expected = struct.unpack_from("<H", self.buf, 2 + HEADER_SIZE + payload_len)[0]
            crc_actual = crc16_ccitt(bytes(self.buf[2 : 2 + HEADER_SIZE + payload_len]))
            if crc_actual != crc_expected:
                del self.buf[0]
                continue

            if version != VERSION:
                del self.buf[:total_len]
                continue

            payload = bytes(self.buf[2 + HEADER_SIZE : 2 + HEADER_SIZE + payload_len])
            packets.append(TelemetryPacket(packet_type=packet_type, seq=seq, payload=payload))
            del self.buf[:total_len]

        return packets

    def _find_sync(self) -> int:
        for i in range(len(self.buf) - 1):
            if self.buf[i] == SYNC0 and self.buf[i + 1] == SYNC1:
                return i
        return -1


class TelemetryReconstructor:
    def __init__(self) -> None:
        self.state = TelemetryState()

    def apply(self, packet: TelemetryPacket) -> bool:
        self.state.seq = packet.seq

        if packet.packet_type == PACKET_POSE_PATH:
            return self._apply_pose_path(packet.payload)
        if packet.packet_type == PACKET_GRID_KEYFRAME:
            return self._apply_keyframe(packet.payload)
        if packet.packet_type == PACKET_GRID_DELTA:
            return self._apply_delta(packet.payload)
        if packet.packet_type == PACKET_HEARTBEAT:
            return self._apply_heartbeat(packet.payload)
        if packet.packet_type == PACKET_SCAN:
            return self._apply_scan(packet.payload)

        return False

    def _apply_pose_path(self, payload: bytes) -> bool:
        if len(payload) < POSE_SIZE:
            return False
        x_m, y_m, theta_rad, path_count = struct.unpack_from(POSE_FMT, payload, 0)
        off = POSE_SIZE
        expected = off + path_count * 2
        if len(payload) < expected:
            return False

        path_raw = np.frombuffer(payload[off:expected], dtype=np.uint8)
        path_cells = path_raw.reshape((-1, 2)).astype(np.uint16) if path_count > 0 else np.zeros((0, 2), dtype=np.uint16)

        self.state.pose_x_m = float(x_m)
        self.state.pose_y_m = float(y_m)
        self.state.pose_theta_rad = float(theta_rad)
        self.state.path_cells = path_cells
        return True

    def _apply_keyframe(self, payload: bytes) -> bool:
        if len(payload) < KEYFRAME_META_SIZE:
            return False

        grid_w, grid_h, cell_mm, packed_len = struct.unpack_from(KEYFRAME_META_FMT, payload, 0)
        n_cells = int(grid_w) * int(grid_h)
        expected = KEYFRAME_META_SIZE + packed_len
        if len(payload) < expected:
            return False

        packed = payload[KEYFRAME_META_SIZE:expected]
        occupancy, terrain = decode_packed_grid(packed, n_cells)
        try:
            occupancy = occupancy.reshape((grid_h, grid_w))
            terrain = terrain.reshape((grid_h, grid_w))
        except ValueError:
            return False

        self.state.grid_w = int(grid_w)
        self.state.grid_h = int(grid_h)
        self.state.cell_mm = int(cell_mm)
        self.state.occupancy = occupancy.copy()
        self.state.terrain = terrain.copy()
        return True

    def _apply_delta(self, payload: bytes) -> bool:
        if self.state.occupancy is None or self.state.terrain is None:
            return False
        if len(payload) < DELTA_META_SIZE:
            return False

        change_count = struct.unpack_from(DELTA_META_FMT, payload, 0)[0]
        off = DELTA_META_SIZE
        expected = off + change_count * 3
        if len(payload) < expected:
            return False

        flat_occ = self.state.occupancy.reshape(-1)
        flat_ter = self.state.terrain.reshape(-1)
        for i in range(change_count):
            idx, packed_cell = struct.unpack_from("<HB", payload, off + i * 3)
            if idx >= flat_occ.size:
                continue
            flat_occ[idx] = packed_cell & 0x03
            flat_ter[idx] = (packed_cell >> 2) & 0x03
        return True

    def _apply_heartbeat(self, payload: bytes) -> bool:
            if len(payload) < HEARTBEAT_SIZE:
                return False
            uptime_ms, tx_queue_depth, dropped_frames = struct.unpack_from(HEARTBEAT_FMT, payload, 0)
            self.state.uptime_ms = int(uptime_ms)
            self.state.tx_queue_depth = int(tx_queue_depth)
            self.state.dropped_frames = int(dropped_frames)
            # Optional extended fields: scan_packet_count (u16), scan_point_count (u16)
            if len(payload) >= HEARTBEAT_SIZE + 4:
                scan_pkt, scan_pts = struct.unpack_from('<HH', payload, HEARTBEAT_SIZE)
                self.state.scan_packet_count = int(scan_pkt)
                self.state.scan_point_count = int(scan_pts)
            return True

    def _apply_scan(self, payload: bytes) -> bool:
        if len(payload) < SCAN_META_SIZE:
            return False
        point_count = struct.unpack_from(SCAN_META_FMT, payload, 0)[0]
        off = SCAN_META_SIZE
        expected = off + point_count * 8
        if len(payload) < expected:
            return False

        arr = np.frombuffer(payload[off:expected], dtype=np.float32)
        if arr.size != point_count * 2:
            return False
        pts = arr.reshape((-1, 2)).copy()
        # update lidar points and counters
        self.state.lidar_points = pts
        self.state.scan_packet_count = getattr(self.state, 'scan_packet_count', 0) + 1
        self.state.scan_point_count = pts.shape[0]
        return True


class SerialWorker(threading.Thread):
    def __init__(self, port: str, baud: int, out_q: Queue, timeout_s: float = 0.1) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.out_q = out_q
        self.stop_evt = threading.Event()

    def stop(self) -> None:
        self.stop_evt.set()

    def run(self) -> None:
        parser = FrameParser()
        recon = TelemetryReconstructor()

        while not self.stop_evt.is_set():
            ser: Optional[serial.Serial] = None
            try:
                print(f"[serial] connecting {self.port} @ {self.baud}...")
                ser = serial.Serial(self.port, self.baud, timeout=self.timeout_s)
                print("[serial] connected")

                while not self.stop_evt.is_set():
                    chunk = ser.read(4096)
                    if not chunk:
                        continue
                    for packet in parser.feed(chunk):
                        if recon.apply(packet):
                            self._push_latest(recon.state)

            except (serial.SerialException, OSError) as e:
                print(f"[serial] disconnected/error: {e}")
                time.sleep(1.0)
            except Exception as e:
                print(f"[serial] unexpected error: {e}")
                time.sleep(1.0)
            finally:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

    def _push_latest(self, state: TelemetryState) -> None:
        copy_state = TelemetryState(
            seq=state.seq,
            pose_x_m=state.pose_x_m,
            pose_y_m=state.pose_y_m,
            pose_theta_rad=state.pose_theta_rad,
            grid_w=state.grid_w,
            grid_h=state.grid_h,
            cell_mm=state.cell_mm,
            occupancy=None if state.occupancy is None else state.occupancy.copy(),
            terrain=None if state.terrain is None else state.terrain.copy(),
            path_cells=None if state.path_cells is None else state.path_cells.copy(),
            uptime_ms=state.uptime_ms,
            tx_queue_depth=state.tx_queue_depth,
            dropped_frames=state.dropped_frames,
            lidar_points=None if state.lidar_points is None else state.lidar_points.copy(),
            scan_packet_count=getattr(state, 'scan_packet_count', 0),
            scan_point_count=getattr(state, 'scan_point_count', 0),
        )

        try:
            self.out_q.put_nowait(copy_state)
        except Full:
            try:
                _ = self.out_q.get_nowait()
            except Empty:
                pass
            try:
                self.out_q.put_nowait(copy_state)
            except Full:
                pass


class LiveVisualizer:
    def __init__(self, fps: float = 20.0) -> None:
        self.fps = max(1.0, fps)
        self.state: Optional[TelemetryState] = None

        self.fig, self.ax = plt.subplots(figsize=(7, 12))
        self.ax.set_title("Teensy Mapping + D* Lite Visualizer")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        blank = np.zeros((DEFAULT_GRID_H, DEFAULT_GRID_W, 3), dtype=np.uint8)
        self.img = self.ax.imshow(
            blank,
            origin="lower",
            extent=[0.0, DEFAULT_ARENA_W_M, 0.0, DEFAULT_ARENA_H_M],
            interpolation="nearest",
        )

        self.robot_dot, = self.ax.plot([], [], marker="o", color="cyan", markersize=6)
        self.heading_line, = self.ax.plot([], [], color="cyan", linewidth=2)
        self.path_line, = self.ax.plot([], [], color="magenta", linewidth=2)
        self.trail_line, = self.ax.plot([], [], color="yellow", linewidth=1)
        self.scan_scatter = self.ax.scatter([], [], s=6, c="yellow", alpha=0.8)
        # Confidence indicator (circle patch)
        self.conf_patch = patches.Circle((0.95, 0.05), 0.03, transform=self.ax.transAxes, facecolor="green", edgecolor="black", zorder=10)
        self.ax.add_patch(self.conf_patch)
        # Arena boundary (Rectangle patch) — updated when grid meta arrives
        self.arena_rect = patches.Rectangle((0, 0), DEFAULT_ARENA_W_M, DEFAULT_ARENA_H_M, fill=False, edgecolor="white", linewidth=2)
        self.ax.add_patch(self.arena_rect)
        self.info_text = self.ax.text(
            0.02,
            0.98,
            "",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            color="white",
            fontsize=10,
            bbox=dict(facecolor="black", alpha=0.55),
        )

        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, alpha=0.2)

        # Maintain a short trail of past poses
        self._trail = []
        self._trail_max = 200

    @staticmethod
    def grid_to_rgb(occupancy: np.ndarray, terrain: np.ndarray) -> np.ndarray:
        # occupancy and terrain are (grid_h, grid_w) with (0,0)=bottom-left.
        # imshow uses origin='lower' so no rotation is required here.
        
        h, w = occupancy.shape
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        # 0=unknown (gray), 1=free space (white), 2=occupied (dark red/black)
        rgb[occupancy == 0] = (90, 90, 90)      # Unknown: gray
        rgb[occupancy == 1] = (220, 220, 220)   # Free: white
        rgb[occupancy == 2] = (20, 20, 20)      # Occupied: black

        ramp_mask = terrain == 1
        bump_mask = terrain == 2
        rgb[ramp_mask] = (rgb[ramp_mask] * 0.5 + np.array([40, 180, 60]) * 0.5).astype(np.uint8)
        rgb[bump_mask] = (rgb[bump_mask] * 0.5 + np.array([230, 180, 30]) * 0.5).astype(np.uint8)
        return rgb

    def update_state(self, state: TelemetryState) -> None:
        self.state = state

        if state.occupancy is not None and state.terrain is not None:
            cell_m = state.cell_mm / 1000.0
            width_m = state.grid_w * cell_m
            height_m = state.grid_h * cell_m
            rgb = self.grid_to_rgb(state.occupancy, state.terrain)
            self.img.set_data(rgb)
            self.img.set_extent([0.0, width_m, 0.0, height_m])
            self.ax.set_xlim(0.0, width_m)
            self.ax.set_ylim(0.0, height_m)

        self.robot_dot.set_data([state.pose_x_m], [state.pose_y_m])

        heading_len = 0.15
        hx = state.pose_x_m + heading_len * np.cos(state.pose_theta_rad)
        hy = state.pose_y_m + heading_len * np.sin(state.pose_theta_rad)
        self.heading_line.set_data([state.pose_x_m, hx], [state.pose_y_m, hy])

        if state.path_cells is not None and state.path_cells.size > 0:
            cell_m = state.cell_mm / 1000.0
            px = (state.path_cells[:, 0].astype(np.float32) + 0.5) * cell_m
            py = (state.path_cells[:, 1].astype(np.float32) + 0.5) * cell_m
            self.path_line.set_data(px, py)
        else:
            self.path_line.set_data([], [])

        theta_deg = np.degrees(state.pose_theta_rad)
        self.info_text.set_text(
            f"seq: {state.seq}\n"
            f"x: {state.pose_x_m:.3f} m\n"
            f"y: {state.pose_y_m:.3f} m\n"
            f"theta: {state.pose_theta_rad:.3f} rad ({theta_deg:.1f} deg)\n"
            f"uptime: {state.uptime_ms} ms\n"
            f"tx_queue: {state.tx_queue_depth}\n"
            f"drops: {state.dropped_frames}\n"
            f"SCAN PKTS: {state.scan_packet_count}\n"
            f"SCAN PTS: {state.scan_point_count}"
        )

        # Update trail
        self._trail.append((state.pose_x_m, state.pose_y_m))
        if len(self._trail) > self._trail_max:
            self._trail.pop(0)
        tx = [p[0] for p in self._trail]
        ty = [p[1] for p in self._trail]
        self.trail_line.set_data(tx, ty)

        # Update scan scatter if present (angles,distances -> xy around robot)
        if state.lidar_points is not None and state.lidar_points.size > 0:
            angles = state.lidar_points[:, 0]
            dists = state.lidar_points[:, 1]
            xs = state.pose_x_m + dists * np.cos(angles + state.pose_theta_rad)
            ys = state.pose_y_m + dists * np.sin(angles + state.pose_theta_rad)
            self.scan_scatter.set_offsets(np.vstack((xs, ys)).T)
        else:
            self.scan_scatter.set_offsets(np.empty((0, 2)))

        # Confidence indicator: simple heuristic (lower is better)
        conf = 1.0 - min(1.0, (state.dropped_frames / 200.0) + (state.tx_queue_depth / 200.0))
        # map to color
        if conf > 0.66:
            col = "green"
        elif conf > 0.33:
            col = "orange"
        else:
            col = "red"
        self.conf_patch.set_facecolor(col)

        # Update arena rectangle to match grid extents
        cell_m = state.cell_mm / 1000.0
        width_m = state.grid_w * cell_m
        height_m = state.grid_h * cell_m
        self.arena_rect.set_width(width_m)
        self.arena_rect.set_height(height_m)

    def animate(self, q: Queue) -> FuncAnimation:
        interval_ms = int(1000.0 / self.fps)

        def tick(_):
            latest = None
            while True:
                try:
                    latest = q.get_nowait()
                except Empty:
                    break
            if latest is not None:
                self.update_state(latest)
            return self.img, self.robot_dot, self.heading_line, self.path_line, self.info_text, self.trail_line, self.scan_scatter

        return FuncAnimation(self.fig, tick, interval=interval_ms, blit=False)


def pick_default_port() -> Optional[str]:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    for p in ports:
        desc = (p.description or "").lower()
        if "teensy" in desc or "usb" in desc or "serial" in desc:
            return p.device
    return ports[0].device


def main() -> None:
    ap = argparse.ArgumentParser(description="Live Teensy mapping/path visualizer")
    ap.add_argument("--port", type=str, default=None, help="Serial port, e.g. COM3")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")
    ap.add_argument("--fps", type=float, default=20.0, help="Display refresh rate")
    args = ap.parse_args()

    port = args.port or pick_default_port()
    if port is None:
        raise SystemExit("No serial ports found. Pass --port explicitly.")

    q: Queue = Queue(maxsize=2)
    worker = SerialWorker(port=port, baud=args.baud, out_q=q)
    worker.start()

    viz = LiveVisualizer(fps=args.fps)
    _ani = viz.animate(q)

    try:
        plt.show()
    finally:
        worker.stop()
        worker.join(timeout=1.0)


if __name__ == "__main__":
    main()
