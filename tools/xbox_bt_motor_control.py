#!/usr/bin/env python3
"""
Xbox -> CH9143 motor control bridge (PC side).

System architecture:
    PC (USB-C) --> CH9143 [USB chip]  ~~~BLE~~~  CH9143 [UART chip] --> Teensy Serial7

Plug a USB-C cable from the PC into the first CH9143. It enumerates as a standard
USB Serial Device (no Bluetooth pairing required). The two CH9143 chips pair with
each other over BLE automatically. Commands flow:
    PC -> USB Serial -> CH9143 USB chip -> BLE -> CH9143 UART chip -> Serial7 -> Teensy

Reads Xbox controller left stick and sends differential track commands:

    MOTOR <left_speed> <right_speed>\n
Where each speed is in [-100, 100].

Left stick mapping:
- Up/Down   : forward/reverse
- Left/Right: turning

Mixing formula:
    left  = throttle + turn
    right = throttle - turn

Requirements:
    pip install pygame pyserial

Finding the COM port:
    - Open Device Manager -> Ports (COM & LPT)
    - Look for "USB Serial Device" or "CH9143" after plugging in USB-C
    - Or run:  python tools/ch9143_diagnose.py --scan

Example:
    python tools/xbox_bt_motor_control.py --port COM7
"""

import argparse
import signal
import sys
import time

import pygame
import serial


def apply_deadzone(value: float, deadzone: float) -> float:
    """Apply deadzone and re-scale to preserve full range outside the deadzone."""
    if abs(value) <= deadzone:
        return 0.0

    # Re-scale so output is continuous from 0..1 after deadzone
    if value > 0:
        return (value - deadzone) / (1.0 - deadzone)
    return (value + deadzone) / (1.0 - deadzone)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def mix_left_stick_to_tracks(x: float, y: float, max_speed: int) -> tuple[int, int]:
    """
    Convert left-stick x/y in [-1, 1] to differential track speeds in [-max_speed, max_speed].

    x: turn (left negative, right positive)
    y: throttle (forward positive)
    """
    left = y + x
    right = y - x

    peak = max(abs(left), abs(right), 1.0)
    left /= peak
    right /= peak

    left_cmd = int(round(clamp(left, -1.0, 1.0) * max_speed))
    right_cmd = int(round(clamp(right, -1.0, 1.0) * max_speed))
    return left_cmd, right_cmd


def send_motor_command(ser: serial.Serial, left: int, right: int) -> None:
    line = f"MOTOR {left} {right}\n"
    ser.write(line.encode("utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Xbox left-stick motor control via CH9143 USB-C bridge")
    parser.add_argument("--port", required=True, help="COM port for CH9143 USB chip (e.g. COM7)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--deadzone", type=float, default=0.15, help="Left stick deadzone 0..0.9 (default: 0.15)")
    parser.add_argument("--max-speed", type=int, default=100, help="Motor command magnitude limit (default: 100)")
    parser.add_argument("--rate", type=float, default=25.0, help="Command update rate in Hz (default: 25)")
    parser.add_argument("--print-rate", type=float, default=4.0, help="Console status print rate in Hz (default: 4)")
    args = parser.parse_args()

    deadzone = clamp(args.deadzone, 0.0, 0.9)
    max_speed = int(clamp(args.max_speed, 1, 100))
    period_s = 1.0 / max(args.rate, 1.0)
    print_period_s = 1.0 / max(args.print_rate, 0.2)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.02)
    except Exception as exc:
        print(f"Failed to open serial {args.port} @ {args.baud}: {exc}")
        return 1

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() < 1:
        print("No controller found. Plug in Xbox controller and try again.")
        ser.close()
        return 1

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Using controller: {joystick.get_name()}")
    print(f"Sending to {args.port} @ {args.baud}")
    print("Command format: MOTOR <left> <right>")
    print("Press Ctrl+C to stop.")

    running = True

    def handle_exit(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    last_left = None
    last_right = None
    last_print = 0.0

    try:
        while running:
            pygame.event.pump()

            raw_x = joystick.get_axis(0)   # left stick horizontal
            raw_y = -joystick.get_axis(1)  # invert so up is positive

            x = apply_deadzone(raw_x, deadzone)
            y = apply_deadzone(raw_y, deadzone)

            left_cmd, right_cmd = mix_left_stick_to_tracks(x, y, max_speed)

            # Send when command changed, or periodically to keep watchdog/safety happy.
            if (left_cmd != last_left) or (right_cmd != last_right):
                send_motor_command(ser, left_cmd, right_cmd)
                last_left, last_right = left_cmd, right_cmd

            now = time.time()
            if now - last_print >= print_period_s:
                print(f"stick(x={x:+.2f}, y={y:+.2f}) -> L={left_cmd:+4d}, R={right_cmd:+4d}")
                last_print = now

            time.sleep(period_s)

    finally:
        # Safety stop on exit
        try:
            send_motor_command(ser, 0, 0)
        except Exception:
            pass

        ser.close()
        joystick.quit()
        pygame.joystick.quit()
        pygame.quit()
        print("Stopped. Sent MOTOR 0 0.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
