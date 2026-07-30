#!/usr/bin/env python3
"""
Debug Xbox controller input - shows all buttons and stick values.

Usage:
    /usr/local/bin/python3 tools/controller_debug.py

This will display all controller inputs in real-time.
"""

import pygame
import sys
import time

def main():
    pygame.init()
    pygame.joystick.init()
    
    print("=" * 70)
    print("Xbox Controller Debug - Real-time Input Monitor")
    print("=" * 70)
    
    joystick_count = pygame.joystick.get_count()
    
    if joystick_count < 1:
        print("\n❌ No controller detected!")
        print("\nMake sure:")
        print("  1. Xbox controller is plugged in (USB or wireless dongle)")
        print("  2. Controller is powered on")
        print("  3. Try wiggling the stick to wake it up")
        return 1
    
    print(f"\n✅ Found {joystick_count} controller(s)\n")
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"Controller Name: {joystick.get_name()}")
    print(f"Number of Axes: {joystick.get_numaxes()}")
    print(f"Number of Buttons: {joystick.get_numbuttons()}")
    print(f"Number of Hats: {joystick.get_numhats()}")
    
    print("\n" + "=" * 70)
    print("Reading controller input... (Press Ctrl+C to exit)")
    print("=" * 70 + "\n")
    
    last_print = 0
    print_interval = 0.1  # Print every 0.1 seconds
    
    try:
        while True:
            # Process all pygame events to update joystick state
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return 0
            
            now = time.time()
            if now - last_print >= print_interval:
                # Read all axes (sticks and triggers)
                print("\n[AXES - Analog Inputs]")
                has_axes = False
                for i in range(joystick.get_numaxes()):
                    value = joystick.get_axis(i)
                    # Show all axes, not just moved ones
                    bar = "█" * int(abs(value) * 20)
                    print(f"  Axis {i}: {value:+.3f} {bar}")
                    if abs(value) > 0.05:
                        has_axes = True
                
                # Read all buttons
                print("\n[BUTTONS - Digital Inputs]")
                pressed_buttons = []
                for i in range(joystick.get_numbuttons()):
                    if joystick.get_button(i):
                        pressed_buttons.append(i)
                
                if pressed_buttons:
                    button_names = {
                        0: "A (Green)",
                        1: "B (Red)",
                        2: "X (Blue)",
                        3: "Y (Yellow)",
                        4: "LB",
                        5: "RB",
                        6: "Back",
                        7: "Start",
                        8: "Left Stick Click",
                        9: "Right Stick Click"
                    }
                    for btn in pressed_buttons:
                        name = button_names.get(btn, f"Button {btn}")
                        print(f"  🔘 {name} is PRESSED")
                else:
                    print("  (no buttons pressed)")
                
                # Read hat (D-pad)
                if joystick.get_numhats() > 0:
                    hat = joystick.get_hat(0)
                    if hat != (0, 0):
                        print(f"\n[D-PAD]")
                        print(f"  {hat}")
                
                last_print = now
            
            time.sleep(0.01)  # 100 Hz polling
    
    except KeyboardInterrupt:
        print("\n\n" + "=" * 70)
        print("Stopped. Controller debug complete.")
        print("=" * 70)
        return 0

if __name__ == "__main__":
    sys.exit(main())
