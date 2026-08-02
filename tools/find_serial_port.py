#!/usr/bin/env python3
"""
Find available serial ports on macOS and Windows.

This script helps identify the correct COM port for the CH9143 USB adapter.

Usage:
    python3 tools/find_serial_port.py

Example output:
    Available serial ports:
    /dev/tty.usbserial-14320    (likely: CH9143 USB adapter)
    /dev/tty.Bluetooth-Incoming-Port
"""

import os
import subprocess
import sys


def find_ports_macos():
    """Find serial ports on macOS using /dev/tty.*"""
    ports = []
    try:
        # List all tty devices
        for device in os.listdir('/dev'):
            if device.startswith('tty.'):
                full_path = f'/dev/{device}'
                # Try to get more info
                ports.append(full_path)
    except Exception as e:
        print(f"Error scanning /dev: {e}")
    
    return sorted(ports)


def find_ports_windows():
    """Find serial ports on Windows using COM ports"""
    import serial.tools.list_ports
    ports = [f"{port.device} ({port.description})" for port, desc in serial.tools.list_ports.comports()]
    return ports


def get_port_info_macos(port):
    """Get additional info about a port on macOS"""
    try:
        # Try to get device info
        result = subprocess.run(['system_profiler', 'SPUSBDataType'], 
                              capture_output=True, text=True, timeout=5)
        if 'CH9143' in result.stdout or 'USB Serial' in result.stdout:
            return "likely: CH9143 USB adapter"
    except Exception:
        pass
    
    # Check if it's a known device
    if 'usbserial' in port:
        return "likely: USB Serial adapter"
    elif 'bluetooth' in port.lower():
        return "Bluetooth device"
    elif 'modem' in port.lower():
        return "Modem"
    
    return "Unknown device"


def main():
    print("=" * 60)
    print("Serial Port Finder")
    print("=" * 60)
    
    if sys.platform == "darwin":
        print("\n📱 macOS detected")
        ports = find_ports_macos()
        
        if not ports:
            print("❌ No serial ports found")
            print("\n⚠️  Make sure to:")
            print("   1. Plug in the CH9143 USB adapter via USB-C")
            print("   2. Wait 2-3 seconds for the device to enumerate")
            print("   3. Run this script again")
        else:
            print(f"\n✅ Found {len(ports)} port(s):\n")
            for port in ports:
                info = get_port_info_macos(port)
                print(f"   {port:<35} ({info})")
            
            # Find likely CH9143 ports
            usb_ports = [p for p in ports if 'usbserial' in p]
            if usb_ports:
                print(f"\n🎮 Recommended port for CH9143:")
                print(f"   {usb_ports[0]}")
                print(f"\n💡 Try running:")
                print(f"   /usr/local/bin/python3 tools/xbox_bt_motor_control.py --port {usb_ports[0]}")
    
    elif sys.platform == "win32":
        print("\n🪟 Windows detected")
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            
            if not ports:
                print("❌ No serial ports found")
                print("\n⚠️  Make sure to:")
                print("   1. Plug in the CH9143 USB adapter")
                print("   2. Check Device Manager -> Ports (COM & LPT)")
            else:
                print(f"\n✅ Found {len(ports)} port(s):\n")
                for port, desc in ports:
                    print(f"   {port:<10} ({desc})")
                
                # Find likely CH9143 ports
                ch9143_ports = [p for p, d in ports if 'CH9143' in d or 'USB Serial' in d]
                if ch9143_ports:
                    print(f"\n🎮 Recommended port for CH9143:")
                    print(f"   {ch9143_ports[0][0]}")
                    print(f"\n💡 Try running:")
                    print(f"   python tools/xbox_bt_motor_control.py --port {ch9143_ports[0][0]}")
        except ImportError:
            print("pyserial not installed. Run: pip install pyserial")
    
    else:
        print(f"\n❓ Unknown platform: {sys.platform}")
    
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
