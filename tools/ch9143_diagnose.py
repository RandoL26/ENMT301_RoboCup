#!/usr/bin/env python3
"""
CH9143 USB-Serial diagnostic tool.

System architecture:
    PC (USB-C) --> CH9143 [USB chip]  ~~~BLE~~~  CH9143 [UART chip] --> Teensy Serial7

Plug the USB-C cable into the PC-side CH9143. It enumerates as a USB Serial Device
(no Bluetooth pairing needed). This tool helps you find its COM port and verify data.

Usage:
    # List all available COM ports:
    python tools/ch9143_diagnose.py --list

    # Auto-scan all ports for streaming data:
    python tools/ch9143_diagnose.py --scan

    # Test a specific port:
    python tools/ch9143_diagnose.py --port COM7 --baud 115200

    # Stream incoming data from a known port (Ctrl+C to stop):
    python tools/ch9143_diagnose.py --port COM7 --baud 115200 --stream

Requirements:
    pip install pyserial
"""

import argparse
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial not installed.  Run:  pip install pyserial")
    sys.exit(1)


SCAN_BAUDS = [115200, 9600, 57600, 38400]
PROBE_TIMEOUT = 1.0   # seconds to wait for response per port/baud combo


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial/COM ports detected.")
        return
    print(f"{'PORT':<12} {'DESC':<40} {'HWID'}")
    print("-" * 80)
    for p in sorted(ports, key=lambda x: x.device):
        print(f"{p.device:<12} {p.description:<40} {p.hwid}")


def probe_port(port: str, baud: int, timeout: float = PROBE_TIMEOUT) -> dict:
    """Try to open a port and read any data / send a ping."""
    result = {"port": port, "baud": baud, "opened": False, "data": "", "error": ""}
    try:
        with serial.Serial(port, baud, timeout=0.2) as ser:
            result["opened"] = True
            # Drain
            ser.reset_input_buffer()
            # Give the Teensy a moment to push stream data
            time.sleep(timeout)
            raw = ser.read(512)
            if raw:
                result["data"] = raw.decode("utf-8", errors="replace").strip()
    except serial.SerialException as exc:
        result["error"] = str(exc)
    except Exception as exc:
        result["error"] = f"Unexpected: {exc}"
    return result


def scan_all():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial/COM ports found.")
        return

    print(f"Scanning {len(ports)} port(s) at bauds {SCAN_BAUDS} ...\n")
    found = []

    for p in sorted(ports, key=lambda x: x.device):
        print(f"  {p.device} — {p.description}")
        for baud in SCAN_BAUDS:
            r = probe_port(p.device, baud, timeout=0.5)
            if not r["opened"]:
                print(f"    [{baud:>7}]  FAILED to open: {r['error']}")
            elif r["data"]:
                preview = r["data"][:80].replace("\n", "\\n")
                print(f"    [{baud:>7}]  *** DATA RECEIVED: {preview}")
                found.append((p.device, baud))
            else:
                print(f"    [{baud:>7}]  opened OK, no data received")
        print()

    if found:
        print("=== Likely CH9143 port(s) ===")
        for dev, baud in found:
            print(f"  {dev} @ {baud}")
        print()
        print("Use:  python tools/xbox_bt_motor_control.py --port <PORT> --baud <BAUD>")
    else:
        print("No data received on any port.")
        print("Checklist:")
        print("  1. Is the USB-C cable plugged into the PC-side CH9143 chip?")
        print("  2. Is the robot powered on (both CH9143 chips need power to establish BLE link)?")
        print("  3. Check Device Manager -> Ports (COM & LPT) for 'USB Serial Device' or 'CH9143'.")
        print("  4. The two CH9143 chips pair over BLE automatically — check both have power/antenna.")
        print("  5. CH9143 UART baud is 115200 — must match Teensy Serial7.begin(115200).")


def stream_port(port: str, baud: int):
    print(f"Streaming from {port} @ {baud}  (Ctrl+C to stop)\n")
    try:
        with serial.Serial(port, baud, timeout=0.1) as ser:
            ser.reset_input_buffer()
            while True:
                line = ser.readline()
                if line:
                    print(line.decode("utf-8", errors="replace"), end="")
    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")


def main():
    parser = argparse.ArgumentParser(description="CH9143 USB-Serial diagnostic tool")
    parser.add_argument("--list",   action="store_true", help="List available COM ports and exit")
    parser.add_argument("--scan",   action="store_true", help="Scan all ports at common bauds")
    parser.add_argument("--port",   help="COM port to use (e.g. COM7)")
    parser.add_argument("--baud",   type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--stream", action="store_true", help="Stream incoming data from --port")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if args.scan:
        scan_all()
        return

    if args.port:
        if args.stream:
            stream_port(args.port, args.baud)
        else:
            print(f"Probing {args.port} @ {args.baud} ...")
            r = probe_port(args.port, args.baud, timeout=2.0)
            if not r["opened"]:
                print(f"FAILED to open: {r['error']}")
                print("\nChecklist:")
                print("  - Is another application (Serial Monitor, xbox_bt_motor_control.py) holding the port?")
                print("  - Is the port number correct?  Run --list to check.")
                print("  - Is the baud rate correct?  CH9143 UART default is 115200.")
            elif r["data"]:
                print(f"SUCCESS — received data:\n{r['data']}")
            else:
                print("Port opened OK but no data in 2 s.")
                print("  - Is the Teensy running and bluetooth.begin() called?")
                print("  - Is the stream test task enabled (tBT_stream_test)?")
        return

    parser.print_help()


if __name__ == "__main__":
    main()
