# RoboCup Robot: Technical Module Documentation

---

## 1. MOTOR CONTROL & LOCOMOTION

### 1.1 DC Motor Driver (JGY370 12V High-Torque Geared)

**Purpose:** Differential-drive propulsion with encoder feedback for odometry.

**Hardware Implementation:**
- **Motor Driver:** Servo library PWM control via hardware pins
- **Speed Control:** -100 to +100 range mapped to pulse widths:
  - 1500 µs = neutral (stop)
  - 1050 µs = full reverse (-100)
  - 1950 µs = full forward (+100)
- **Encoder:** Quadrature encoder with Teensy 4.0 hardware QuadEncoder support
- **Pin Assignments:**
  - Motor 1 (Left): PWM pin 0, Encoder A/B pins 2/3
  - Motor 2 (Right): PWM pin 1, Encoder A/B pins 4/5

**Software Architecture (`dc_motor.cpp`, `dc_motor.h`):**

The `DCMotor` class provides individual motor control with hardware-based encoder tracking:

```cpp
DCMotor leftMotor(DC_M1_PIN, DC_M1_ENC_A, DC_M1_ENC_B);
DCMotor rightMotor(DC_M2_PIN, DC_M2_ENC_A, DC_M2_ENC_B);

leftMotor.begin();           // Initialize with Teensy QuadEncoder
leftMotor.setSpeed(75);      // Set speed 75 (forward)
int32_t pulses = leftMotor.getEncoderPulses();  // Read encoder count
```

**Key Features:**
- **Speed Mapping:** Linear interpolation from speed value to PWM pulse width
- **Encoder Pulses:** Hardware-based quadrature counting (4x resolution on Teensy)
- **Status Printing:** `printStatus()` outputs current speed and encoder state
- **Speed Validation:** Range checking prevents invalid commands

**Typical Performance:**
- Encoder resolution: High-precision quadrature counting
- Control update rate: 40 ms task period
- Dead-band handling via speed slewing (gradual acceleration)

---

### 1.2 Motor Control Class (PID Synchronization)

**Purpose:** Dual-motor control with automatic straight-line synchronization using encoder feedback.

**Hardware Integration (`motor_control.cpp`, `motor_control.h`):**

The `MotorControl` class wraps two `DCMotor` instances and provides coordinated control:

```cpp
MotorControl driveMotor(DC_M1_PIN, DC_M1_ENC_A, DC_M1_ENC_B,
                        DC_M2_PIN, DC_M2_ENC_A, DC_M2_ENC_B);

driveMotor.begin();
driveMotor.setSpeeds(50, 50);  // Equal command speed (50% forward)
```

**Straight-Line Synchronization (PID-Based):**

The core innovation: **Adaptive encoder-based PID correction** compensates for motor mismatch during straight-line motion.

- **Mechanism:** Measures encoder pulse-rate difference per sampling interval
- **PID Tuning Parameters (Runtime Adjustable):**
  - `STRAIGHT_SYNC_KP = 0.15` – Proportional gain
  - `STRAIGHT_SYNC_KI = 0.05` – Integral gain
  - `STRAIGHT_SYNC_KD = 0.10` – Derivative gain
  - `STRAIGHT_SYNC_MAX_CORRECTION = 20` – Max speed correction magnitude
  - `STRAIGHT_SYNC_MIN_SPEED = 20` – Minimum threshold to apply correction
  - `STRAIGHT_SYNC_INTEGRAL_MAX = 50.0` – Integral error limit

**Operation:**
1. Encoder pulses are sampled every 40 ms
2. Left vs. right pulse-rate difference is calculated
3. PID controller generates a correction term (added to left motor, subtracted from right)
4. Correction is clamped to prevent overshoot
5. Result: smooth, drift-free straight-line motion

**Adaptive Offset Learning:**
The motor control class maintains an **adaptive correction table** that learns persistent motor mismatches:
- 101-element array indexed by command magnitude (0–100)
- Stores per-magnitude left-motor offset to equalize encoder rates
- Learns automatically when encoder mismatch exceeds `adaptiveLearnThreshold` (default tunable)
- Maximum offset is clamped by `adaptiveMaxOffset`

**Runtime Tuning via Serial Commands:**
```
PID KP 0.2          – Set proportional gain
PID KI 0.08         – Set integral gain
PID KD 0.12         – Set derivative gain
SYNC SHOW           – Display all sync parameters
SYNC ON/OFF         – Enable/disable synchronization
ADAPT SHOW          – View adaptive offset table
```

**Testing Data Requirements (for report):**
- Straight-line deviation over 1 m, 2 m, 5 m distances
- Typical deviation: < 5–10 cm for well-tuned PID
- Encoder pulse counts at various speed commands
- Response time to speed changes (slew rate)

---

## 2. NAVIGATION SENSORS

### 2.1 BNO055 9-DOF IMU Sensor

**Purpose:** Inertial measurement unit for heading, roll, pitch, and acceleration data. Primary sensor for gyro-based heading estimates and tilt detection.

**Hardware Specification:**
- **Interface:** I2C bus (standard Arduino Wire library)
- **Sensor Fusion:** Built-in Kalman filter (quaternion-based)
- **Data Rates:**
  - Accelerometer: ±2 g to ±16 g (default ±4 g)
  - Gyroscope: ±125 °/s to ±2000 °/s (default ±500 °/s)
  - Magnetometer: 3-axis compass with auto-calibration
  - Temperature sensor: Internal, ±1 °C accuracy

**Software Implementation (`imu_sensor.cpp`, `imu_sensor.h`):**

The `read_imu()` function returns all sensor data in a single `IMU_Data` structure:

```cpp
struct IMU_Data {
  float accel_x, accel_y, accel_z;    // m/s²
  float gyro_x, gyro_y, gyro_z;       // Accumulated angle in degrees
  float mag_x, mag_y, mag_z;          // μT (microtesla)
  float euler_h, euler_r, euler_p;    // heading, roll, pitch (degrees)
  float temp;                         // °C
};

IMU_Data data = read_imu();
```

**Data Processing Pipeline:**

1. **Accelerometer:** Raw value / 100.0 → m/s²
2. **Gyroscope:** 
   - Raw value converted to °/s
   - Integrated over time (delta-t calculation)
   - Accumulated angle tracks absolute rotation
3. **Magnetometer:** Raw value / 16.0 → μT
4. **Euler Angles:** Raw value / 16.0 → degrees
   - Heading (yaw): magnetic north reference
   - Roll: rotation about X-axis
   - Pitch: rotation about Y-axis

**Calibration:**
- Factory calibration typically acceptable for basic navigation
- Magnetometer may need calibration in magnetically noisy environments
- Gyroscope drift accumulates; use periodic re-zeroing during idle periods

**Typical Accuracy:**
- Heading: ±2–5° (IMU Kalman filter estimate)
- Acceleration: ±5% of measured value
- Gyro drift: ~1–2° per minute without heading correction

**Task Integration (40 ms period):**
```cpp
// From robocup_template.ino
Task tIMURead(IMU_READ_TASK_PERIOD, -1, &imu_task_callback);

void imu_task_callback(void) {
    current_imu_data = read_imu();
    printfBoth("Gyro X:%d Y:%d Z:%d | Accel: %d, %d, %d\n", ...);
}
```

---

### 2.2 Color Sensor (Adafruit TCS34725)

**Purpose:** Detect weight color for classification and identification. Used to distinguish between different weight types or to find weight presence.

**Hardware Specification:**
- **Interface:** I2C (Wire library, address 0x29)
- **Light Detection:** RGBC (Red, Green, Blue, Clear) channels
- **Integration Time:** 2.4–700 ms (configurable)
- **Gain Settings:** 1×, 4×, 16×, 60× (adjustable)
- **Built-in LED:** Can be toggled for illumination control
- **Range:** Works best in 10–300 cm with adequate lighting

**Software Implementation (`color_sensor.cpp`, `color_sensor.h`):**

The `ColorSensor` class encapsulates color detection:

```cpp
ColorSensor colorSensor(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
colorSensor.begin();

ColorSensor::ColorData data = colorSensor.readColor();
// data.red, data.green, data.blue, data.clear (raw 16-bit values)
// data.hexColor (0xRRGGBB format)
```

**Color Processing:**

1. **Raw Measurement:** LED is turned on (50 ms wait for stabilization)
2. **16-bit ADC Readings:** Each channel returns 0–65535
3. **Normalization:** Divide R, G, B by Clear channel (intensity compensation)
4. **Scaling to 8-bit:** Multiply by 256 and constrain to 0–255
5. **Hex Encoding:** Pack into single 32-bit value: `0xRRGGBB`

**Example Color Classification:**
```cpp
colorSensor.decodeColor(data);  // Prints color name (if implemented)

// Typical logic:
if ((data.hexColor & 0xFF0000) > 100) {  // High red component
    // Red weight detected
}
```

**Typical Accuracy:**
- Color detection range: ~10–80 cm (depends on lighting)
- Discrimination: Can distinguish basic colors (red, green, blue, yellow)
- Integration time trade-off: Longer → better SNR, slower response

**Task Integration (40 ms period):**
```cpp
void color_sensor_callback(void) {
    ColorSensor::ColorData colorData = colorSensor.readColor();
    colorSensor.decodeColor(colorData);
    // Send to Bluetooth:
    uint8_t r = (colorData.hexColor >> 16) & 0xFF;
    bluetooth.printf("Color: #%02X%02X%02X\n", r, g, b);
}
```

---

### 2.3 ToF Sensor Array (VL53L1X - Time-of-Flight)

**Purpose:** Precise distance measurement (0–4 m) with sub-5° FOV. Ideal for obstacle detection and precise approach to weights.

**Hardware Specification:**
- **Interface:** I2C multiplexing via SX1509 IO Expander
- **Range:** 50 mm to 4000 mm (configurable)
- **Accuracy:** ±50 mm (typical)
- **Field of View:** ~27° full cone
- **Measurement Mode:** Continuous at 50 ms intervals
- **Maximum Sensors:** 8 (via IO Expander pins)
- **Addressing:** Base address 0x30, incremented per sensor (0x30, 0x31, ..., 0x37)

**Hardware Setup (`tof_sensor_array.cpp`):**

The `TOFSensorArray` class manages multiple sensors with individual I2C addressing:

```cpp
const uint8_t VL53L1X_SENSOR_COUNT = 1;
const uint8_t VL53L1X_XSHUT_PINS[VL53L1X_SENSOR_COUNT] = {18};

TOFSensorArray tofSensorArray(1, 0x3F);  // 1 sensor, IO expander at 0x3F
tofSensorArray.setXSHUTPins(VL53L1X_XSHUT_PINS, VL53L1X_SENSOR_COUNT);
tofSensorArray.begin();

TOFSensorArray::TOFData data = tofSensorArray.readDistances();
// data.distances[0] in mm
```

**Initialization Sequence:**
1. IO Expander initialized at 0x3F
2. I2C clock set to 400 kHz
3. All sensor XSHUT pins driven LOW (disable all)
4. For each sensor in sequence:
   - XSHUT pin driven HIGH (enable)
   - Sensor initializes and gets unique I2C address (0x30 + index)
   - Continuous ranging started at 50 ms period

**Read Operation:**
- Each sensor's `read()` method retrieves last distance from continuous mode buffer
- Timeout detection: if read fails, distance set to 0xFFFF
- Typically returns within 1–2 ms

**Measurement Characteristics:**
- **Accuracy:** ±50 mm (better than ultrasonic)
- **Response Time:** 50 ms (continuous mode) or 100 ms (single shot)
- **Immunity:** Infrared-based; immune to acoustic noise (unlike ultrasonic)

**Task Integration (100 ms period):**
```cpp
void vl53l1x_sensor_callback(void) {
    TOFSensorArray::TOFData tofData = tofSensorArray.readDistances();
    tofSensorArray.printDistances(tofData);
}
```

---

### 2.4 IR XY Position Sensor (Wii-style Camera)

**Purpose:** Detect IR-marked weights' 2D position (up to 4 blobs). Enables precise localization of targets.

**Hardware Specification:**
- **Interface:** I2C (address 0x21 default, or 0xB0 >> 1)
- **Blob Tracking:** Supports up to 4 simultaneous IR blobs
- **Resolution:** 1024×768 pixel field of view
- **Refresh Rate:** ~100 Hz (10 ms updates available)
- **Requires:** IR LEDs or reflectors on targets at ~850 nm wavelength

**Software Implementation (`ir_xy_position.cpp`, `ir_xy_position.h`):**

```cpp
IRXYPosition irSensor(0x21);  // I2C address
irSensor.begin();

IRXYPosition::IRData data = irSensor.readPositions();
// data.x[0..3], data.y[0..3] – blob coordinates (0–1023 range)
```

**Data Format (16 bytes returned):**
```
Byte:  0    1    2    3    4    5    6    7 ... (repeats for blobs 2, 3, 4)
       X1_hi X1_lo Y1_hi Y1_lo X2_hi X2_lo Y2_hi Y2_lo

Parsing:
Blob 1 X = ((byte[0] & 0x03) << 8) | byte[1]
Blob 1 Y = ((byte[2] & 0x03) << 8) | byte[3]
```

**Communication Protocol:**
```cpp
// Mode request (initialization)
write2bytes(0x30, 0x01);  // Enable IR data mode

// Data request (periodic)
Wire.beginTransmission(0x21);
Wire.write(0x36);  // Request data
Wire.endTransmission();

// Read response (16 bytes)
Wire.requestFrom(0x21, 16);
```

**Typical Use Cases:**
- **Relative Positioning:** Calculate angle/distance to weight
  - Angle: `atan2(y_center - 384, x_center - 512)` → bearing from image center
  - Distance: Inverse relationship with blob size (not directly available; requires calibration)
- **Multiple Target Tracking:** Up to 4 weights simultaneously

**Accuracy & Limitations:**
- **Angular Resolution:** ~0.1–0.5° per pixel
- **Range:** Effective detection 0.5–2 m (depends on IR brightness)
- **Requires Strong IR:** Daylight interference possible; works best indoors
- **Latency:** ~10 ms for new blob detection

**Task Integration (40 ms period):**
```cpp
void ir_xy_position_callback(void) {
    IRXYPosition::IRData irData = irXYSensor.readPositions();
    irXYSensor.printPositions(irData);
}
```

---

### 2.5 Optical Flow Sensor (PMW3901 - Bitcraze)

**Purpose:** Dead reckoning via surface motion tracking. Accumulates horizontal displacement in mm over time, independent of wheel slipping.

**Hardware Specification:**
- **Interface:** SPI (chip select pin 10)
- **Motion Tracking:** Optical flow with integrated lens
- **Resolution:** ~1 mm displacement detection
- **Refresh Rate:** 100 Hz (10 ms updates)
- **Field of View:** ~42° diagonal
- **Requires:** Adequate floor texture (not reflective surfaces)

**Software Implementation (`optical_flow.cpp`, `optical_flow.h`):**

```cpp
OpticalFlow opticalFlow(10);  // CS pin 10
opticalFlow.begin();

int16_t dx, dy;
opticalFlow.read(dx, dy);  // Read motion delta counts

opticalFlow.addMotionCounts(dx, dy);  // Accumulate
float total_x_mm = opticalFlow.getTotalXmm();
float total_y_mm = opticalFlow.getTotalYmm();
```

**Data Processing:**

1. **Raw Counts:** Direct SPI read of dx, dy motion deltas
2. **Accumulation:** `totalCountsX += dx`, `totalCountsY += dy`
3. **Scaling:** Default 0.05 mm/count (adjustable via `setScaleMMPerCount()`)
4. **Output:** Total displacement in mm since last reset

**Typical Scale Calibration:**
- Motor speed test: Command known speed, measure optical flow distance over time
- Calibration constant: `mm_per_count = measured_distance_mm / accumulated_counts`
- Floor dependency: Glossy/reflective surfaces reduce accuracy

**Advantages Over Encoder Odometry:**
- Immune to wheel slipping
- Detects skidding/sideways drift
- Complements encoder measurements for robust odometry fusion

**Task Integration (40 ms period):**
```cpp
void optical_flow_callback(void) {
    int16_t dx = 0, dy = 0;
    if (opticalFlow.read(dx, dy)) {
        opticalFlow.addMotionCounts(dx, dy);
        float tx = opticalFlow.getTotalXmm();
        float ty = opticalFlow.getTotalYmm();
        printfBoth("OpticalFlow totalX: %.2f mm  totalY: %.2f mm\n", tx, ty);
    }
}
```

---

### 2.6 Ultrasonic Sensor Array (HC-SR04 / Analogs)

**Purpose:** Long-range distance detection and obstacle avoidance. Less precise than ToF but with wider detection cone.

**Hardware Specification:**
- **Interface:** GPIO trigger (digital output) + echo (digital input)
- **Range:** 2 cm to 400 cm typical
- **Accuracy:** ±3 cm (degrades with distance)
- **Update Rate:** 40–100 ms (speed-of-sound limited)
- **Beam Angle:** ~15° cone
- **Maximum Array:** Configurable count (2–4 typical to avoid cross-talk)

**Software Implementation (`ultrasonic_sensor_array.cpp`):**

```cpp
UltrasonicSensorArray ultrasonicArray(2);  // 2 sensors
ultrasonicArray.addSensor(0, TRIGGER_PIN_1, ECHO_PIN_1);
ultrasonicArray.addSensor(1, TRIGGER_PIN_2, ECHO_PIN_2);
ultrasonicArray.begin();

UltrasonicSensorArray::UltrasonicData data = ultrasonicArray.readDistances();
// data.distances[i] in cm, data.count = 2
```

**Measurement Sequence (per sensor):**
1. Trigger pin goes HIGH for 10 µs
2. Sensor emits 40 kHz ultrasonic pulse (8 cycles)
3. Echo pin goes HIGH when pulse is sent, LOW when echo returns
4. Pulse width encodes distance: `distance = (pulse_time_us / 2) / 29.1 cm/µs`

**Data Characteristics:**
- **Speed Dependency:** Temperature affects speed of sound (~0.3 m/s per °C)
- **Reflectivity:** Soft materials (foam, fabric) don't reflect well
- **Cross-talk:** Multiple sensors in close proximity may interfere; recommend 100+ ms spacing

**Task Integration (100 ms period):**
```cpp
void ultrasonic_sensor_callback(void) {
    UltrasonicSensorArray::UltrasonicData data = ultrasonicArray.readDistances();
    ultrasonicArray.printDistances(data);
}
```

---

### 2.7 Proximity Sensor (Inductive, Digital)

**Purpose:** Detect metallic weights at very close range (<5 cm). High-speed switching capability for triggering collection mechanism.

**Hardware Specification:**
- **Interface:** GPIO digital input (pull-up configuration)
- **Detection Range:** 0–5 cm typical
- **Sensitivity:** Typically adjustable via potentiometer
- **Response Time:** <1 ms
- **Logic Output:** LOW = object detected (sink-to-ground), HIGH = no object
- **Debouncing:** Software debounce 50 ms

**Software Implementation (`proximity_sensor.cpp`, `proximity_sensor.h`):**

```cpp
ProximitySensor proximitySensor(PROXIMITY_SENSOR_PIN);  // Pin 20
proximitySensor.begin();  // Initialize with INPUT_PULLUP

proximitySensor.update();  // Call periodically (debounce logic)
if (proximitySensor.isObjectDetected()) {
    // Metallic weight detected!
}
```

**Debounce Logic:**
```
Raw sensor bounces: ↓↑↓↑↓ (jitter)
                        ↓ (debounce delay = 50 ms)
Debounced signal:   ↓────── (clean state)
```

**Typical Use Case:**
- Final confirmation before collection (when gripper is within 1 cm of weight)
- Triggering electromagnet activation
- Feedback that weight is secure in gripper

**Task Integration:**
Integrated into proximity sensor read task (40 ms period).

---

### 2.8 IR Distance Sensor (2Y0A02)

**Purpose:** Mid-range analog distance measurement (~10–80 cm). Provides continuous distance feedback for approach control.

**Hardware Specification:**
- **Interface:** Analog input (0–5V → 0–1023 ADC on 10-bit Teensy)
- **Range:** 10–80 cm (reliable), extends to 150 cm (less accurate)
- **Response Time:** 40 ms
- **Output Type:** Analog voltage inversely proportional to distance
- **Characteristic:** Distance = 27 V / voltage, or ADC-based: distance = 8361 / ADC

**Software Implementation (`ir_distance_sensor.cpp`):**

```cpp
IRDistanceSensor irSensor(A9);  // Analog pin A9
irSensor.begin();

float distance_cm = irSensor.readDistance();  // Returns distance in cm
```

**Conversion Pipeline:**
```
1. Raw ADC: analogRead(A9) → 0–1023
2. Distance calculation: distance_cm = 15000 / ADC_value
   (calibration constant 15000 is tunable via setCalibrationConstant())
3. Output: distance in cm
```

**Calibration Procedure:**
1. Place known objects at fixed distances (10 cm, 30 cm, 50 cm, 80 cm)
2. Record raw ADC values for each
3. Fit linear regression: ADC_value = f(distance)
4. Calculate calibration constant: constant = measured_distance × ADC_value
5. Refine `setCalibrationConstant()` to match expected readings

**Typical Accuracy:**
- ±5 cm in optimal conditions
- Degrades near range limits (< 10 cm, > 100 cm)
- Bright ambient light can increase noise

**Task Integration (50 ms period):**
```cpp
void ir_distance_sensor_callback(void) {
    irDistanceSensor.readDistance();
    irDistanceSensor.printDistance();
}
```

---

## 3. EXTERNAL DEVICES

### 3.1 Herkulex Smart Servo (DRS-0101/DRS-0201)

**Purpose:** Precise angular control for collection mechanism (gripper arm/end-effector). Provides position feedback and torque control.

**Hardware Specification:**
- **Interface:** RS-485 serial communication (115.2 kbaud typical)
- **Voltage:** 7–12 V DC
- **Torque:** 4.0–6.0 kg·cm depending on model
- **Speed:** ~0.068 s/60° at 12 V (position mode)
- **Resolution:** 14-bit (16384 steps/360°)
- **Continuous Rotation Mode:** Supported (wheel-mode)
- **Feedback:** Position, temperature, voltage, load
- **ID:** Broadcast (0xFE) or individual (0x01–0xFE)

**Hardware Integration:**
```cpp
// Pin definitions
#define HERKULEX_ID 1
#define HERKULEX_RX_PIN 0
#define HERKULEX_TX_PIN 1

// Initialization
Herkulex.begin(115200, HERKULEX_RX_PIN, HERKULEX_TX_PIN);
Herkulex.reboot(HERKULEX_ID);
Herkulex.clearError(HERKULEX_ID);
Herkulex.ACK(1);  // ACK on error only
Herkulex.torqueON(HERKULEX_ID);
Herkulex.initialize();
```

**Software Implementation (`Herkulex.cpp`):**

The library provides commands for common operations:

```cpp
// Move to angle
Herkulex.moveOneAngle(HERKULEX_ID, 100, 1000, LED_GREEN);  
// Move to angle 100°, time 1000ms, LED indicator

// Move with speed (continuous mode)
Herkulex.moveSpeedOne(HERKULEX_ID, 100, 1000, LED_BLUE);

// Get position
int position = Herkulex.getPosition(HERKULEX_ID);  // 0–16383 (0–360°)

// Get status
byte status = Herkulex.stat(HERKULEX_ID);  // Check servo health

// Get model info
int model = Herkulex.model();  // Identify servo type
```

**Protocol Overview:**
- Packet format: [0xFF, 0xFF, SIZE, ID, CMD, DATA..., CK1, CK2]
- Checksum: CK1 = (SIZE ^ ID ^ CMD) & 0xFE
- Response: Servo echoes status packet with sensor data

**Typical Collection Sequence:**
```
1. moveOneAngle(ID, 0°, 500ms)    → Open gripper
2. [Wait for robot to position over weight]
3. moveOneAngle(ID, 90°, 500ms)   → Close gripper
4. Read position feedback to confirm
```

**Task Integration (1200 ms test period):**
```cpp
void herkulex_test_callback() {
    static bool toggle = false;
    toggle = !toggle;
    
    int angle = toggle ? 100 : -100;
    Herkulex.torqueON(HERKULEX_ID);
    Herkulex.moveOneAngle(HERKULEX_ID, angle, 1000, LED_BLUE);
    
    // Verify move
    int position = Herkulex.getPosition(HERKULEX_ID);
    printfBoth("Herkulex position: %d\n", position);
}
```

---

### 3.2 Electromagnet (Solenoid Pickup)

**Purpose:** Magnetic grip for ferrous weights. Controlled via GPIO digital output with pulsing capability.

**Hardware Specification:**
- **Control Pin:** GPIO 26 (digital output)
- **Activation:** HIGH = electromagnet energized, LOW = de-energized
- **Voltage:** Typically 12 V (through FET/relay driver)
- **Power Consumption:** ~5–10 W when active
- **Response Time:** <10 ms activation, ~50 ms decay-off
- **Holding Force:** Depends on coil design, typically 50–200 N

**Software Implementation:**

```cpp
#define MAGNET_PIN 26

// Initialization
pinMode(MAGNET_PIN, OUTPUT);
digitalWrite(MAGNET_PIN, LOW);  // Default off

// Activation (in weight_collection routine)
digitalWrite(MAGNET_PIN, HIGH);   // Energize magnet
delay(100);                       // Hold for 100 ms
digitalWrite(MAGNET_PIN, LOW);    // De-energize

// Pulsing pattern for robust pickup
for (int i = 0; i < 3; i++) {
    digitalWrite(MAGNET_PIN, HIGH);
    delay(50);
    digitalWrite(MAGNET_PIN, LOW);
    delay(25);
}
```

**Typical Collection Sequence:**
```
1. Robot approaches weight (IR/color sensor triggers)
2. Herkulex opens gripper (if mechanical gripper present)
3. Robot moves to proximity-sensor range (<5 cm)
4. Electromagnet pulses HIGH for 100 ms
5. Weight is lifted via magnetic attraction
6. Herkulex closes gripper or servo arm raises
7. Return to base with weight secured
```

**Initialization Test (at startup):**
```cpp
// Pulse electromagnet HIGH for 100 ms for initial test
printlnBoth("Pulsing electromagnet HIGH for 100 ms\n");
digitalWrite(MAGNET_PIN, HIGH);
delay(100);
digitalWrite(MAGNET_PIN, LOW);
```

---

## 4. COMMUNICATION MODULES

### 4.1 Bluetooth Module (CH9143)

**Purpose:** Wireless telemetry and remote control. Enables real-time monitoring of sensor data and manual robot control.

**Hardware Specification:**
- **Interface:** UART (hardware serial port)
- **Baud Rate:** 115.2 kbaud (factory default, adjustable)
- **Voltage:** 3.3–5 V DC
- **Range:** ~10 m line-of-sight (indoors, typical)
- **Pairing:** Transparent UART bridge (no special pairing protocol in code)
- **Pins (Teensy 4.0):** Serial7 (RX pin 28, TX pin 29)

**Software Implementation (`ch9143_bluetooth.cpp`, `ch9143_bluetooth.h`):**

```cpp
// Initialize
CH9143Bluetooth bluetooth(&Serial7, 28, 29, 115200);
bluetooth.begin();

// Send data
bluetooth.print("Sensor Data: ");
bluetooth.println(value);

bluetooth.printf("IR Distance: %.2f cm\n", distance);

// Read data (if commands sent from phone/PC)
if (bluetooth.available()) {
    char cmd = bluetooth.read();
    // Process command
}
```

**Integration with Task Callbacks:**

Throughout the main sketch, sensor data is sent via `bluetooth.printf()`:

```cpp
// From imu_task_callback()
printfBoth("Raw Gyro X:%d Y:%d Z:%d | Accel: %d, %d, %d\n", ...);
// printfBoth() sends to both Serial (USB) and Serial7 (Bluetooth)

// From color_sensor_callback()
bluetooth.printf("BT Color: #%02X%02X%02X\n", r, g, b);
```

**Typical Data Stream:**
```
Raw Gyro X:5 Y:-2 Z:1 | Accel: 100, -50, 980
ENC L:1250 R:1248
IR Positions: Blob1(512,400)
OpticalFlow totalX: 123.45 mm  totalY: 2.10 mm
Color: #FF0000
IR Distance: 35.20 cm
```

**Advantages:**
- Real-time monitoring without tethering
- Debugging during operation
- Data logging to external devices
- Remote testing/tuning of PID parameters

---

## 5. NAVIGATION & MAPPING

### 5.1 MappingNav Class

**Purpose:** Probabilistic occupancy grid mapping with D* Lite path planning. Integrates odometry and sensor data for autonomous navigation.

**Arena Specification:**
- **Grid Dimensions:** 48 × 98 cells (2.4 m × 4.9 m arena)
- **Cell Size:** 0.05 m (5 cm resolution)
- **Total Cells:** 4704 (4.7 KB per occupancy layer)

**Key Data Structures (`MappingNav.h`):**

```cpp
struct Pose2D {
    float x_m;        // X position (meters)
    float y_m;        // Y position (meters)
    float theta_rad;  // Heading (radians)
};

struct PoseUpdateInput {
    float encoder_dx_m, encoder_dy_m, encoder_dtheta_rad;
    float flow_dx_m, flow_dy_m;
    float imu_gyro_dtheta_rad;
};

enum Occupancy : uint8_t {
    OCC_UNKNOWN = 0,
    OCC_FREE = 1,
    OCC_OCCUPIED = 2
};

struct SensorRay {
    bool valid;
    bool has_hit;
    SensorKind kind;  // TOF, ULTRASONIC
    
    float angle_offset_rad;    // Relative to robot heading
    float distance_m;          // Distance to hit
    float max_range_m;         // Maximum range
};
```

**Core Operations (`MappingNav.cpp`):**

1. **Pose Initialization:**
   ```cpp
   MappingNav nav;
   nav.reset();  // Start at arena center (1.2 m, 2.45 m)
   nav.setPose(1.2, 2.45, 0.0);  // x, y, heading
   ```

2. **Pose Fusion (Encoder + Optical Flow + IMU):**
   ```cpp
   MappingNav::PoseUpdateInput input;
   input.encoder_dx_m = 0.05;      // 5 cm forward (from encoder)
   input.encoder_dy_m = 0.0;
   input.encoder_dtheta_rad = 0.0;
   
   input.flow_dx_m = 0.048;        // Optical flow check
   input.imu_gyro_dtheta_rad = -0.017;  // -1° heading change
   
   // Fuse with weights: 50% encoder, 80% IMU for heading
   nav.updatePose(input, 0.5f, 0.8f);
   ```

3. **Grid Occupancy Update (Ray-casting):**
   ```cpp
   MappingNav::SensorRay rays[8];
   rays[0] = {true, true, SENSOR_TOF, -45°, 0.8, 4.0};  // Obstacle at 45° left, 0.8 m
   rays[1] = {true, false, SENSOR_TOF, 45°, 4.0, 4.0};  // No hit at 45° right
   
   nav.updateGridFromSensors(rays, 8);  // Mark cells as free/occupied
   ```

4. **Path Planning:**
   ```cpp
   nav.setGoalWorld(2.3, 0.5);  // Goal at (2.3 m, 0.5 m)
   nav.replanPath(0.2);          // Plan with 0.2 m robot radius inflation
   
   float next_x, next_y;
   if (nav.getNextWaypoint(next_x, next_y)) {
       // Steer toward (next_x, next_y)
   }
   ```

5. **Boundary Correction (Kalman-like update):**
   ```cpp
   // If robot is near wall and sensors confirm expected distance:
   nav.applyBoundaryCorrection(rays, 8, 0.3, 0.12);
   // correction_gain = 0.3 (30% weight on boundary match)
   // tolerance = 0.12 m (±12 cm must match)
   ```

**Grid Cell Format (1 byte per cell, bit-packed):**
```
Bit 1-0: Occupancy (OCC_UNKNOWN=0, OCC_FREE=1, OCC_OCCUPIED=2)
Bit 3-2: Terrain (FLAT=0, RAMP=1, SPEED_BUMP=2)
Bit 7-4: (Reserved)
```

**Typical Navigation Loop:**
```cpp
// Every 40 ms:
void navigation_callback() {
    // 1. Collect sensor readings
    MappingNav::SensorRay rays[8] = {...};
    
    // 2. Update odometry (encoder + IMU)
    MappingNav::PoseUpdateInput input = {...};
    nav.updatePose(input, 0.5f, 0.8f);
    
    // 3. Update map
    nav.updateGridFromSensors(rays, 8);
    
    // 4. Correction (if near boundaries)
    nav.applyBoundaryCorrection(rays, 8, 0.3, 0.12);
    
    // 5. Replan if needed
    if (replanning_triggered) {
        nav.replanPath(0.2);
    }
    
    // 6. Get next waypoint and steer
    float next_x, next_y;
    if (nav.getNextWaypoint(next_x, next_y)) {
        steer_to_waypoint(next_x, next_y);
    }
}
```

**Key Design Principles:**
- **Incremental Planning:** D* Lite updates only affected cells
- **Sensor Fusion:** Complementary weighting of encoder vs. optical flow
- **Boundary Awareness:** Arena walls provide natural localization landmarks
- **Resolution:** 5 cm cells balance memory (~5 KB) vs. precision

---

## 6. BEHAVIORAL SYSTEMS

### 6.1 Weight Collection Module

**Purpose:** State machine for searching, detecting, and collecting weights. Integrates color detection, IR position sensing, and electromagnet control.

**Implementation Status:** Skeleton functions defined (`weight_collection.cpp`), core logic pending.

**Proposed State Machine:**

```
SEARCHING
    ↓
    [Color sensor: weight detected?]
    ├─ NO  → continue search motion
    └─ YES → WEIGHT_FOUND
             ↓
             [IR XY sensor: center in frame?]
             ├─ NO  → APPROACH (steer to center)
             └─ YES → PROXIMITY (within 5 cm?)
                      ├─ NO  → APPROACH (move forward)
                      └─ YES → COLLECT (trigger electromagnet)
                               ↓
                               [Weight secure?]
                               ├─ NO  → retry
                               └─ YES → RETURN_TO_BASE
```

**Skeleton Code (Current):**

```cpp
#define NO_WEIGHT               0   
#define WEIGHT_FOUND            1

void weight_scan(/* Parameters */) {
    Serial.println("Looking for weights \n");
    // TODO: Implement color sensor polling
    // TODO: Calculate approach vector from IR position
}

void collect_weight() {
    Serial.println("Collecting weight \n");
    // TODO: Trigger electromagnet
    // TODO: Verify weight secured
    // TODO: Return success/failure status
}
```

**Required Implementation Additions:**

1. **Color-Based Detection:**
   ```cpp
   ColorSensor::ColorData data = colorSensor.readColor();
   if (isWeightColor(data.hexColor)) {
       state = WEIGHT_FOUND;
   }
   ```

2. **IR Positioning:**
   ```cpp
   IRXYPosition::IRData irData = irXYSensor.readPositions();
   float error_x = irData.x[0] - 512;  // Offset from image center
   float approach_angle = atan2(error_x, /* depth estimate */);
   ```

3. **Electromagnet Collection:**
   ```cpp
   digitalWrite(MAGNET_PIN, HIGH);
   delay(100);
   digitalWrite(MAGNET_PIN, LOW);
   ```

4. **Servo Integration (if gripper present):**
   ```cpp
   Herkulex.moveOneAngle(HERKULEX_ID, 90, 500, LED_GREEN);  // Close gripper
   delay(500);
   Herkulex.moveOneAngle(HERKULEX_ID, 0, 500, LED_BLUE);    // Open gripper
   ```

---

### 6.2 Return to Base Module

**Purpose:** Navigate back to starting position and unload collected weights. Uses odometry and home-base detection.

**Implementation Status:** Skeleton functions defined (`return_to_base.cpp`), core logic pending.

**Proposed Sequence:**

```
CARRYING_WEIGHT
    ↓
    return_to_base()
    ├─ Use nav.setGoalWorld(home_x, home_y)
    ├─ Navigate via MappingNav waypoints
    └─ Arrive at base
       ↓
       detect_base()
       ├─ Proximity sensor triggers? (metal detection plate)
       ├─ Confirm position via sensors
       └─ Base detected → proceed
          ↓
          unload_weights()
          ├─ Trigger electromagnet OFF (release weight)
          ├─ Verify weight dropped (proximity clear)
          └─ Return to searching
```

**Skeleton Code (Current):**

```cpp
void return_to_base(/* Parameters */) {
    Serial.println("Returning to base \n");
    // TODO: Calculate bearing to home
    // TODO: Use nav.setGoalWorld() to plan path
    // TODO: Execute waypoint following
    // TODO: Detect arrival
}

void detect_base(/* Parameters */) {
    Serial.println("Base detected \n");
    // TODO: Use proximity sensor or IR markers
    // TODO: Verify home-base position (known location)
}

void unload_weights(/* Parameters */) {
    Serial.println("Unloading weights \n");
    // TODO: Electromagnet OFF
    // TODO: Confirm weight released
    // TODO: Update state to SEARCHING
}
```

**Required Implementation Additions:**

1. **Odometry-Based Return:**
   ```cpp
   MappingNav::Pose2D current = nav.getPose();
   float home_x = 1.2, home_y = 2.45;  // Start position
   
   nav.setGoalWorld(home_x, home_y);
   nav.replanPath(0.2);
   
   // Follow waypoints:
   float next_x, next_y;
   while (nav.getNextWaypoint(next_x, next_y)) {
       steer_to_waypoint(next_x, next_y);
   }
   ```

2. **Base Detection (Proximity + Position Check):**
   ```cpp
   ProximitySensor metal_plate(PROXIMITY_SENSOR_PIN);
   if (metal_plate.isObjectDetected()) {
       // Likely over base (has conductive plate)
       MappingNav::Pose2D pose = nav.getPose();
       if (abs(pose.x_m - home_x) < 0.1 && abs(pose.y_m - home_y) < 0.1) {
           // Position confirmed, base detected
           return true;
       }
   }
   ```

3. **Weight Unloading:**
   ```cpp
   digitalWrite(MAGNET_PIN, LOW);  // Release electromagnet
   delay(100);
   
   if (!proximityXensor.isObjectDetected()) {
       // Weight has fallen, base release successful
       state = SEARCHING;
   }
   ```

---

## SUMMARY TABLE

| Module | Type | Interface | Status | Key Function |
|--------|------|-----------|--------|---------------|
| DC Motor | Actuator | PWM + Encoder | ✓ Implemented | Differential drive control with quadrature feedback |
| MotorControl | Firmware | PID + Sync | ✓ Implemented | Straight-line synchronization using encoder PID |
| BNO055 IMU | Sensor | I2C | ✓ Implemented | 9-DOF inertial measurement (heading, accel, gyro) |
| TCS34725 Color | Sensor | I2C | ✓ Implemented | RGBC detection for weight classification |
| VL53L1X ToF Array | Sensor | I2C + IO Expander | ✓ Implemented | 0–4 m precise distance measurement (up to 8 sensors) |
| IR XY Position | Sensor | I2C | ✓ Implemented | 2D blob tracking (up to 4 IR blobs) |
| PMW3901 Optical Flow | Sensor | SPI | ✓ Implemented | Dead reckoning via surface motion (mm precision) |
| HC-SR04 Ultrasonic Array | Sensor | GPIO | ✓ Implemented | 2–400 cm range detection (2–4 sensors) |
| Proximity Sensor | Sensor | GPIO Digital | ✓ Implemented | 0–5 cm metallic object detection with debounce |
| 2Y0A02 IR Distance | Sensor | ADC Analog | ✓ Implemented | 10–150 cm distance measurement with calibration |
| Herkulex Servo | Actuator | UART RS-485 | ✓ Initialized | Gripper arm control, 14-bit position feedback |
| Electromagnet | Actuator | GPIO Digital | ✓ Initialized | Ferrous weight pickup via pulsed solenoid |
| CH9143 Bluetooth | Comm | UART | ✓ Implemented | Wireless telemetry & remote control (115.2 kbaud) |
| MappingNav | Navigation | Software | ✓ Implemented | Occupancy grid + D* Lite path planning |
| Weight Collection | Behavior | FSM | ⚠ Skeleton | Search → Detect → Approach → Collect state machine |
| Return to Base | Behavior | FSM | ⚠ Skeleton | Odometry-based navigation home + weight unloading |

---

## INTEGRATION NOTES

### Task Scheduling:
All modules run concurrently via TaskScheduler at defined periods:
- **10 ms:** IMU read (fastest, for gyro integration)
- **40 ms:** Motor control, color sensor, IR XY, DC motor sync, weight collection
- **50 ms:** IR distance sensor
- **100 ms:** ToF array, ultrasonic array
- **1200 ms:** Herkulex servo test (can be disabled during operation)

### Typical Data Flow (per 40 ms cycle):
```
1. Sensor read callbacks collect raw data
2. Motor control PID synchronizes left/right encoders
3. Navigation callbacks fuse odometry (encoder + optical flow + IMU)
4. MappingNav updates occupancy grid from ToF/ultrasonic rays
5. Weight collection state machine processes color/IR/proximity data
6. Bluetooth telemetry sends summary to wireless client
7. Motor output commands applied to left/right drive motors
8. Cycle repeats at 40 ms interval
```

### Critical Tuning Parameters:
- **Motor PID:** KP, KI, KD for straight-line sync (currently tunable via serial)
- **Color Thresholds:** RGB ranges for weight detection
- **IR Sensor Calibration:** `calibrationConstant` for distance accuracy
- **Optical Flow Scale:** `mmPerCount` for dead-reckoning
- **Proximity Debounce:** `debounceDelay` (50 ms default)
- **Navigation Weighting:** `translation_encoder_weight`, `heading_imu_weight`

---

**Document Version:** 1.0  
**Last Updated:** 2026-08-10  
**Author:** RoboCup Design Team

