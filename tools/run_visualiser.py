#!/usr/bin/env python3
"""Run the roboviz visualiser by reading robot Serial telemetry lines.

Usage:
  python tools/run_visualiser.py --port COM3 --baud 115200

The script imports `tools.roboviz.process_serial_line` and feeds each incoming
line to it. It optionally sends `VISUALISER ON` at start and `VISUALISER OFF`
at exit.
"""
import sys
import os
import time
import argparse

# Ensure project root is on sys.path so we can import tools.roboviz
ROOT = os.path.dirname(os.path.dirname(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    from tools.roboviz import process_serial_line
except Exception as e:
    print('Failed to import tools.roboviz.process_serial_line:', e)
    print('Attempting fallback: load tools/roboviz by file path')
    print('Project ROOT =', ROOT)
    print('sys.path[0:5] =', sys.path[0:5])
    try:
        import importlib.util
        roboviz_path = os.path.join(ROOT, 'tools', 'roboviz', '__init__.py')
        if os.path.exists(roboviz_path):
            spec = importlib.util.spec_from_file_location('tools.roboviz', roboviz_path)
            roboviz = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(roboviz)
            process_serial_line = getattr(roboviz, 'process_serial_line')
            print('Loaded tools.roboviz from', roboviz_path)
        else:
            print('Fallback failed: file not found:', roboviz_path)
            raise
    except Exception as e2:
        print('Fallback import failed:', e2)
        print('Make sure you run this from the project root and have requirements installed.')
        raise

try:
    import serial
    from serial.tools import list_ports
except Exception as e:
    print('pyserial is required. Install with: pip install pyserial')
    raise


def pick_port():
    ports = list(list_ports.comports())
    if not ports:
        return None
    # prefer USB serial devices that look like COM on Windows or /dev on Unix
    return ports[0].device


def main():
    ap = argparse.ArgumentParser(description='Run roboviz visualiser from robot Serial')
    ap.add_argument('--port', '-p', help='Serial port (e.g. COM3 or /dev/ttyUSB0)')
    ap.add_argument('--baud', '-b', type=int, default=115200, help='Baud rate')
    ap.add_argument('--no-enable', action='store_true', help="Don't send 'VISUALISER ON' at start")
    ap.add_argument('--no-disable', action='store_true', help="Don't send 'VISUALISER OFF' at exit")
    ap.add_argument('--verbose', '-v', action='store_true', help='Print incoming lines and visualiser events')
    args = ap.parse_args()

    port = args.port or pick_port()
    if port is None:
        print('No serial port found. Specify --port.')
        return

    print(f'Opening serial port {port} @ {args.baud}')

    try:
        ser = serial.Serial(port, args.baud, timeout=1)
    except Exception as e:
        print('Failed to open serial port:', e)
        return

    # small settle
    time.sleep(0.5)

    # Enable interactive plotting so windows update without blocking
    try:
        import matplotlib.pyplot as plt
        plt.ion()
    except Exception:
        pass

    try:
        if not args.no_enable:
            try:
                ser.write(b'VISUALISER ON\n')
            except Exception:
                pass

        print('Listening for telemetry. Press Ctrl-C to quit.')

        last_viz = None
        while True:
            raw = ser.readline()
            if not raw:
                time.sleep(0.001)
                # allow GUI events to process
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

            if line == '':
                continue

            if args.verbose:
                print('RX:', line)

            # feed line into parser; it creates/updates the Visualizer
            try:
                viz = process_serial_line(line)
                # if parser returned a Visualizer, ensure its window is shown
                if viz is not None:
                    try:
                        import matplotlib.pyplot as plt
                        plt.show(block=False)
                        plt.pause(0.001)
                    except Exception:
                        pass

                # verbose visualiser events
                if args.verbose:
                    if viz is not None and last_viz is None:
                        print('Visualiser created')
                    elif viz is not None and last_viz is not None:
                        print('Visualiser updated')
                last_viz = viz
            except Exception as ex:
                if args.verbose:
                    print('Parse error:', ex)
                # keep running on parse errors
                pass
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        if not args.no_disable:
            try:
                ser.write(b'VISUALISER OFF\n')
            except Exception:
                pass
        try:
            ser.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
