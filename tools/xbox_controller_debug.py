#!/usr/bin/env python3
"""
Xbox Controller Debug Tool

Displays raw controller input in real-time to diagnose button/stick issues.

Usage:
    python3 tools/xbox_controller_debug.py

This will show:
- All button states
- All stick positions
- All trigger values
- All D-Pad positions
"""

import pygame
import sys
import time


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() < 1:
        print("❌ No controller found!")
        print("   1. Plug in Xbox controller")
        print("   2. Try pressing a button to wake it up")
        print("   3. Run this script again")
        return 1

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("=" * 70)
    print(f"Controller: {joystick.get_name()}")
    print(f"Axes: {joystick.get_numaxes()}")
    print(f"Buttons: {joystick.get_numbuttons()}")
    print(f"Hats: {joystick.get_numhats()}")
    print("=" * 70)
    print("\n📋 Button Mapping (Xbox Controller):")
    print("   0=A (Green)    1=B (Red)      2=X (Blue)     3=Y (Yellow)")
    print("   4=LB           5=RB           6=Back         7=Start")
    print("   8=Left Stick   9=Right Stick  10=Guide       11=Share")
    print("\n🎮 Axis Mapping:")
    print("   0=Left Stick X   1=Left Stick Y   2=Right Stick X   3=Right Stick Y")
    print("   4=LT Trigger     5=RT Trigger")
    print("\n🎯 Hat (D-Pad):")
    print("   (-1,-1)=NW  (0,-1)=N  (1,-1)=NE")
    print("   (-1,0)=W    (0,0)=Center  (1,0)=E")
    print("   (-1,1)=SW   (0,1)=S   (1,1)=SE")
    print("\n" + "=" * 70)
    print("Press Ctrl+C to stop\n")

    try:
        last_print = 0
        while True:
            pygame.event.pump()

            # Get all raw data
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            hats = [joystick.get_hat(i) for i in range(joystick.get_numhats())]

            # Print periodically
            now = time.time()
            if now - last_print >= 0.1:  # Update every 100ms
                # Clear screen (works on most terminals)
                print("\033[H\033[J", end="")  # Clear screen and move cursor to top
                
                print("=" * 70)
                print(f"Live Controller State (Updated: {now:.2f})")
                print("=" * 70)

                # Button states
                print("\n🔘 BUTTONS:")
                button_names = ["A", "B", "X", "Y", "LB", "RB", "Back", "Start", 
                              "L-Stick", "R-Stick", "Guide", "Share"]
                for i, (name, state) in enumerate(zip(button_names, buttons[:12])):
                    if i < len(buttons):
                        indicator = "🟢 ON " if state else "⚪ OFF"
                        print(f"   {i:2d}: {name:<12} {indicator}")

                # Stick positions
                print("\n🎮 LEFT STICK:")
                print(f"    X: {axes[0]:+.3f}  (axis 0)")
                print(f"    Y: {axes[1]:+.3f}  (axis 1)")
                
                print("\n🎮 RIGHT STICK:")
                print(f"    X: {axes[2]:+.3f}  (axis 2)")
                print(f"    Y: {axes[3]:+.3f}  (axis 3)")

                # Triggers
                print("\n🔫 TRIGGERS:")
                print(f"    LT: {axes[4]:+.3f}  (axis 4)")
                print(f"    RT: {axes[5]:+.3f}  (axis 5)")

                # D-Pad
                print("\n🎯 D-PAD (HAT):")
                if hats:
                    hat_x, hat_y = hats[0]
                    print(f"    X: {hat_x:+2d}  Y: {hat_y:+2d}")
                    directions = {
                        (0, 0): "Center",
                        (0, -1): "North (Up)",
                        (0, 1): "South (Down)",
                        (-1, 0): "West (Left)",
                        (1, 0): "East (Right)",
                        (1, 1): "SE",
                        (1, -1): "NE",
                        (-1, 1): "SW",
                        (-1, -1): "NW",
                    }
                    direction = directions.get((hat_x, hat_y), "Unknown")
                    print(f"    Direction: {direction}")

                print("\n" + "=" * 70)
                print("💡 TIP: Move left stick and press A button to see values change")
                print("=" * 70)
                last_print = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nStopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
