#!/usr/bin/env python3
"""
Xbox Controller Diagnostic for macOS - troubleshoot pygame/SDL controller issues.

Usage:
    /usr/local/bin/python3 tools/controller_diagnostic.py
"""

import os
import subprocess
import sys
import pygame

def check_macos_controller_pairing():
    """Check if Xbox controller is properly paired in macOS System Settings."""
    print("\n" + "=" * 70)
    print("🔍 STEP 1: Checking macOS System Configuration")
    print("=" * 70)
    
    # Check connected Bluetooth devices
    try:
        result = subprocess.run(
            ['system_profiler', 'SPBluetoothDataType'],
            capture_output=True, text=True, timeout=10
        )
        
        if 'Xbox' in result.stdout or 'Series X' in result.stdout:
            print("✅ Xbox controller found in Bluetooth devices")
            # Print relevant lines
            for line in result.stdout.split('\n'):
                if 'Xbox' in line or 'Series' in line or 'Connected' in line or 'Device' in line:
                    print(f"   {line.strip()}")
        else:
            print("⚠️  Xbox controller NOT found in Bluetooth devices")
            print("   Did you pair it in System Settings > Bluetooth?")
            print("\n   To pair:")
            print("   1. Hold Xbox button until light flashes (pairing mode)")
            print("   2. Go to System Settings > Bluetooth")
            print("   3. Click 'Connect' next to Xbox controller")
    except Exception as e:
        print(f"❌ Error checking Bluetooth: {e}")

def check_usb_connection():
    """Check if Xbox controller is connected via USB."""
    print("\n" + "=" * 70)
    print("🔍 STEP 2: Checking USB Devices")
    print("=" * 70)
    
    try:
        result = subprocess.run(
            ['system_profiler', 'SPUSBDataType'],
            capture_output=True, text=True, timeout=10
        )
        
        if 'Xbox' in result.stdout or 'Vendor' in result.stdout:
            for line in result.stdout.split('\n'):
                if 'Xbox' in line or 'VID' in line or 'PID' in line:
                    print(f"   {line.strip()}")
        
        if 'Xbox' not in result.stdout:
            print("⚠️  No Xbox controller found on USB")
            print("   Try plugging in via USB cable (wired mode)")
    except Exception as e:
        print(f"❌ Error checking USB: {e}")

def check_pygame_detection():
    """Check if pygame detects the controller."""
    print("\n" + "=" * 70)
    print("🔍 STEP 3: Testing pygame Detection")
    print("=" * 70)
    
    pygame.init()
    pygame.joystick.init()
    
    count = pygame.joystick.get_count()
    print(f"Controllers detected by pygame: {count}")
    
    if count > 0:
        for i in range(count):
            js = pygame.joystick.Joystick(i)
            js.init()
            print(f"\n  Controller {i}: {js.get_name()}")
            print(f"    - Axes: {js.get_numaxes()}")
            print(f"    - Buttons: {js.get_numbuttons()}")
            print(f"    - Hats: {js.get_numhats()}")
    else:
        print("❌ pygame does NOT see any controllers")
        print("   This is the core issue!")

def check_input_devices():
    """Check /dev entries for input devices."""
    print("\n" + "=" * 70)
    print("🔍 STEP 4: Checking /dev/input Devices")
    print("=" * 70)
    
    # On macOS, input devices are typically in /dev/input* or /var/run/udev/
    try:
        result = subprocess.run(
            ['ls', '-la', '/dev/input*'],
            capture_output=True, text=True, timeout=5, shell=True
        )
        if result.returncode == 0 and result.stdout:
            print(result.stdout)
        else:
            print("ℹ️  No /dev/input devices found (common on macOS)")
    except Exception:
        print("ℹ️  /dev/input not available (expected on macOS)")

def test_controller_read():
    """Try to read controller values."""
    print("\n" + "=" * 70)
    print("🔍 STEP 5: Testing Controller Input")
    print("=" * 70)
    print("\n⏰ Reading controller for 3 seconds...")
    print("🎮 Try moving the left stick now!\n")
    
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() < 1:
        print("❌ No controller found for input test")
        return
    
    js = pygame.joystick.Joystick(0)
    js.init()
    
    import time
    start = time.time()
    max_values = {}
    
    while time.time() - start < 3:
        for event in pygame.event.get():
            pass  # Process events
        
        # Read all axes
        for axis in range(js.get_numaxes()):
            val = js.get_axis(axis)
            if val != 0.0:
                if axis not in max_values:
                    max_values[axis] = val
                else:
                    if abs(val) > abs(max_values[axis]):
                        max_values[axis] = val
        
        # Read all buttons
        for btn in range(js.get_numbuttons()):
            if js.get_button(btn):
                print(f"🔘 Button {btn} pressed!")
        
        time.sleep(0.01)
    
    if max_values:
        print("\n✅ Controller input DETECTED!")
        for axis, val in sorted(max_values.items()):
            print(f"   Axis {axis}: max value {val:.3f}")
    else:
        print("\n❌ NO controller input detected!")
        print("   The controller may not be properly paired or calibrated.")
        print("   Try unplugging/replugging or re-pairing.")

def main():
    print("\n" + "=" * 70)
    print("Xbox Controller Diagnostic Tool for macOS")
    print("=" * 70)
    
    check_macos_controller_pairing()
    check_usb_connection()
    check_pygame_detection()
    check_input_devices()
    test_controller_read()
    
    print("\n" + "=" * 70)
    print("📋 TROUBLESHOOTING STEPS")
    print("=" * 70)
    print("""
1. If pygame detects controller but doesn't read input:
   - Unplug and replug the controller (if USB)
   - Or re-pair via Bluetooth (if wireless)
   
2. If pygame doesn't detect controller at all:
   - Check System Settings > Bluetooth (for wireless)
   - Try plugging in via USB cable (wired mode)
   - Restart pygame/python script
   
3. For Xbox Series X/S on Bluetooth:
   - Hold Xbox button until light flashes
   - Go to System Settings > Bluetooth > Click Xbox controller > Connect
   
4. If still not working:
   - Update Xbox controller firmware via Windows/Xbox app
   - Try a different USB cable (if using wired)
   - Restart your Mac
   
5. As a workaround:
   - Try using a different game controller (PS4, etc.)
   - Use the --help on motor control script to verify config
""")
    print("=" * 70 + "\n")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nStopped by user.")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
