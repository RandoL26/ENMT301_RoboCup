# Xbox Controller Motor Control - Quick Start Guide

## 🎮 Controller Layout & Usage

### Button Mapping
- **A Button (Green)** - Pause/Resume data stream ⏸️ ▶️
- **Left Stick (Analog)**
  - **Up/Down** - Forward/Reverse throttle
  - **Left/Right** - Turn left/right

### Stick Mixing Formula
```
Left Motor  = throttle + turn
Right Motor = throttle - turn
```

This creates smooth differential drive control:
- **Push stick up** → Both motors forward (drive straight)
- **Push stick left** → Left motor slows, right spins (turn left)
- **Push stick right** → Right motor slows, left spins (turn right)
- **Pull stick down** → Both motors reverse (drive backwards)

---

## 🚀 How to Use

### 1. Find Your Serial Port
```bash
/usr/local/bin/python3 /Users/max/Desktop/ENMT301_RoboCup/tools/find_serial_port.py
```

Look for output like:
```
✅ Found 1 port(s):

   /dev/tty.usbmodemWCH285E33TS11     (likely: USB Serial adapter)

🎮 Recommended port for CH9143:
   /dev/tty.usbmodemWCH285E33TS11

💡 Try running:
   /usr/local/bin/python3 tools/xbox_bt_motor_control.py --port /dev/tty.usbmodemWCH285E33TS11
```

### 2. Start the Controller Script
```bash
/usr/local/bin/python3 /Users/max/Desktop/ENMT301_RoboCup/tools/xbox_bt_motor_control.py --port /dev/tty.usbmodemWCH285E33TS11
```

### 3. Use the Controller

**Initial State:** Script starts in **PAUSED** mode
```
>>> Data stream PAUSED ⏸️
```

**Press A Button** to start:
```
>>> Data stream RUNNING ▶️
```

**Move left stick** to control motors:
```
stick(x=+0.50, y=+0.75) -> L=+100, R=+ 25
stick(x=-0.30, y=+0.60) -> L=+ 30, R=+90
```

**Press A Button again** to pause:
```
>>> Data stream PAUSED ⏸️
```

Motor command `0 0` is sent automatically (safe stop).

---

## 📊 Command Line Options

```bash
--port PORT              COM port for CH9143 (required)
--baud BAUD              Serial baud rate (default: 115200)
--deadzone DEADZONE      Stick deadzone 0.0-0.9 (default: 0.15)
--max-speed MAX_SPEED    Motor speed limit (default: 100)
--rate RATE              Update frequency Hz (default: 25)
--print-rate PRINT_RATE  Console print frequency Hz (default: 4)
--echo                   Echo MOTOR commands to console
```

### Example with Custom Settings
```bash
/usr/local/bin/python3 tools/xbox_bt_motor_control.py \
  --port /dev/tty.usbmodemWCH285E33TS11 \
  --max-speed 80 \
  --deadzone 0.2 \
  --echo
```

---

## 🔧 Implementation Details

### State Management
- **`paused`** - Boolean flag controlling data stream (True = paused, False = running)
- **`last_a_button`** - Tracks previous A button state to detect edge (button press)

### Pause/Resume Logic
```python
a_button = joystick.get_button(0)  # Read current A button state
if a_button and not last_a_button:  # Detect rising edge (press event)
    paused = not paused              # Toggle pause state
    if paused:
        send_motor_command(ser, 0, 0)  # Safe stop motors
last_a_button = a_button             # Remember state for next iteration
```

### When Paused
- Stick input is **ignored**
- No motor commands are sent
- No status output is printed
- Safe stop (0, 0) is sent when pausing
- Loop continues checking for A button to resume

### When Running
- Stick input is **read and processed**
- Motor commands sent on stick change or timeout
- Status printed every print_period_s seconds
- Loop continues checking for A button to pause

---

## 🛑 Safety Features

1. **Pause Mode** - Automatically stops motors when paused
2. **Safe Exit** - Ctrl+C sends `MOTOR 0 0` before closing
3. **Deadzone** - Prevents drift from stick noise (default 0.15 = 15%)
4. **Max Speed Limit** - Clamps output to [-100, 100]
5. **Edge Detection** - Button press only triggers on rising edge (prevents repeated toggles)

---

## 📋 Troubleshooting

### "No controller found"
- Make sure Xbox controller is plugged in and powered
- Try wiggling the stick to wake it up
- Run `find_serial_port.py` to verify other connections work

### "Failed to open serial port"
- Check the port name (use `find_serial_port.py`)
- Verify CH9143 USB adapter is plugged in
- Try a different USB port
- Check baud rate matches robot (default 115200)

### "Port disappears / reconnects"
- CH9143 may need to be power-cycled
- Try unplugging and replugging the USB-C cable
- Wait 2-3 seconds for device to enumerate

### Motors not responding
- Check if script shows "Data stream PAUSED" (press A to start)
- Verify serial connection is working
- Check Teensy is receiving commands with serial monitor

### Stick feels unresponsive
- Try adjusting `--deadzone` (lower = more sensitive, but more drift)
- Increase `--rate` for faster response (default 25 Hz = 40ms)

---

## 🔌 Hardware Setup

```
┌────────────────┐
│   Xbox         │
│  Controller    │
└────────┬───────┘
         │ (USB dongle or BLE)
         │
    ┌────▼────────────────┐
    │ PC / Mac            │
    │ pygame detects      │
    │ controller input    │
    └────┬────────────────┘
         │ (USB-C Serial)
         │
    ┌────▼──────────────────────┐
    │ CH9143 USB Chip           │
    │ (enumerates as COM port)  │
    └────┬──────────────────────┘
         │ (BLE auto-pairing)
         │
    ┌────▼──────────────────────┐
    │ CH9143 UART Chip          │
    │ (on robot)                │
    └────┬──────────────────────┘
         │ (UART)
         │
    ┌────▼──────────────────────┐
    │ Teensy 4.0                │
    │ Serial7 receives commands │
    │ MOTOR <left> <right>      │
    └──────────────────────────┘
```

---

## 📝 Sample Output

```
Using controller: Xbox 360 Controller
Sending to /dev/tty.usbmodemWCH285E33TS11 @ 115200
Command format: MOTOR <left> <right>
Press Ctrl+C to stop.

>>> Data stream PAUSED ⏸️
-> MOTOR 0 0

>>> Data stream RUNNING ▶️
stick(x=+0.00, y=+0.75) -> L=+75, R=+75
stick(x=+0.15, y=+0.70) -> L=+55, R=+85
stick(x=+0.35, y=+0.65) -> L=+30, R=+100
stick(x=+0.50, y=+0.50) -> L=+0, R=+100

>>> Data stream PAUSED ⏸️
-> MOTOR 0 0

^C
Stopped. Sent MOTOR 0 0.
```

---

## 🎯 Tips & Tricks

### Alias for Quick Launch
Add to your `~/.zshrc` or `~/.bash_profile`:
```bash
alias xbox-motor='/usr/local/bin/python3 /Users/max/Desktop/ENMT301_RoboCup/tools/xbox_bt_motor_control.py'
```

Then use:
```bash
xbox-motor --port /dev/tty.usbmodemWCH285E33TS11 --echo
```

### Monitor Robot Feedback
In another terminal, while controller is running:
```bash
pio device monitor -b 115200
```

### Test Without Robot
Pass a non-existent port to test the controller:
```bash
/usr/local/bin/python3 tools/xbox_bt_motor_control.py --port /dev/tty.fake 2>&1 | head -10
```

Will fail to open port but shows you what commands would be sent.

---

*Last Updated: July 30, 2026*
*Implementation: Pause/Resume with A button on Xbox controller*
