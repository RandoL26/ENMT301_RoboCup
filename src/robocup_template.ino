
/********************************************************************************
 *                               ROBOCUP TEMPLATE                              
 *        
 *  
 *  This is a template program design with modules for 
 *  different components of the robot, and a task scheduler
 *  for controlling how frequently tasks sholud run
 *  
 *  
 *  written by: Logan Chatfield, Ben Fortune, Lachlan McKenzie, Jake Campbell
 *  
 ******************************************************************************/

#include <Servo.h>                  //control the DC motors
#include "Herkulex.h"             //smart servo (uses hardware Serial: pins 0 (RX), 1 (TX))
#include <Adafruit_TCS34725.h>      //colour sensor
#include <Wire.h>                   //for I2C and SPI
#include <TaskScheduler.h>          //scheduler
#include <VL53L1X.h>                //VL53L1X distance sensor
#include <VL53L0X.h>                //VL53L0X distance sensor for top target validation
#include <stdarg.h>                 //for va_list in printf functions
#include <stdio.h>                  //for vsnprintf
#include <string.h>                 //for strcmp/sscanf parsing
#include "BNO055_support.h"         //IMU sensor 

// Custom headers
#include "motors.h"
#include "imu_sensor.h"
#include "weight_collection.h"
#include "return_to_base.h"
#include "proximity_sensor.h"       //inductive proximity sensor
#include "ultrasonic_sensor.h"      //ultrasonic distance sensor
#include "ultrasonic_sensor_array.h" //ultrasonic sensor array (multiple sensors)
#include "color_sensor.h"           //colour sensor module
#include "ir_xy_position.h"         //IR XY position sensor
#include "ir_distance_sensor.h"     //2Y0A02 IR distance sensor
#include "tof_sensor_array.h"       //TOF (VL53L1X) sensor array
#include "target_detector.h"        //Tier-based target / obstacle detection
#include "TOF_X8.h"                 // DFRobot Matrix Lidar 8x8 (TOF_X8)
#include "dc_motor.h"               //DC motor control 
#include "motor_control.h"          //PID/SYNC command handling
#include "ch9143_bluetooth.h"       //CH9143 Bluetooth module
#include "optical_flow.h"           // PMW3901 optical flow (Bitcraze)

//**********************************************************************************
// Local Definitions
//**********************************************************************************

// Task period Definitions
// ALL OF THESE VALUES WILL NEED TO BE SET TO SOMETHING USEFUL !!!!!!!!!!!!!!!!!!!!
#define US_READ_TASK_PERIOD                 40
#define IR_READ_TASK_PERIOD                 40
#define COLOUR_READ_TASK_PERIOD             40
#define IMU_READ_TASK_PERIOD                10
#define SENSOR_AVERAGE_PERIOD               40
#define SET_MOTOR_TASK_PERIOD               40
#define WEIGHT_SCAN_TASK_PERIOD             40
#define COLLECT_WEIGHT_TASK_PERIOD          40
#define RETURN_TO_BASE_TASK_PERIOD          40
#define DETECT_BASE_TASK_PERIOD             40
#define UNLOAD_WEIGHTS_TASK_PERIOD          40
#define CHECK_WATCHDOG_TASK_PERIOD          40
#define VICTORY_DANCE_TASK_PERIOD           40
#define PROXIMITY_SENSOR_READ_PERIOD        40
#define ULTRASONIC_SENSOR_READ_PERIOD       100
#define VL53L1X_SENSOR_READ_PERIOD          100
#define IR_DISTANCE_SENSOR_READ_PERIOD      50
#define DC_MOTOR_CONTROL_PERIOD             40

#define OF_READ_TASK_PERIOD                 40
#define OF_READ_TASK_NUM_EXECUTE            -1

#define HERKULEX_TEST_PERIOD               1200




// Task execution amount definitions
// -1 means indefinitely
#define US_READ_TASK_NUM_EXECUTE           -1
#define IR_READ_TASK_NUM_EXECUTE           -1
#define COLOUR_READ_TASK_NUM_EXECUTE       -1
#define IMU_READ_TASK_NUM_EXECUTE          -1
#define SENSOR_AVERAGE_NUM_EXECUTE         -1
#define SET_MOTOR_TASK_NUM_EXECUTE         -1
#define WEIGHT_SCAN_TASK_NUM_EXECUTE       -1
#define COLLECT_WEIGHT_TASK_NUM_EXECUTE    -1
#define RETURN_TO_BASE_TASK_NUM_EXECUTE    -1
#define DETECT_BASE_TASK_NUM_EXECUTE       -1
#define UNLOAD_WEIGHTS_TASK_NUM_EXECUTE    -1
#define CHECK_WATCHDOG_TASK_NUM_EXECUTE    -1
#define VICTORY_DANCE_TASK_NUM_EXECUTE     -1
#define PROXIMITY_SENSOR_NUM_EXECUTE        -1
#define ULTRASONIC_SENSOR_NUM_EXECUTE       -1
#define VL53L1X_SENSOR_NUM_EXECUTE          -1
#define IR_DISTANCE_SENSOR_NUM_EXECUTE      -1
#define DC_MOTOR_CONTROL_NUM_EXECUTE        -1

// Pin deffinitions
#define IO_POWER  49
#define PROXIMITY_SENSOR_PIN  20  // A6Z
// #define ULTRASONIC_TRIGGER_PIN_1  3   // D2Z - Sensor 1 Trigger
// #define ULTRASONIC_ECHO_PIN_1     2   // D3Z - Sensor 1 Echo
// #define ULTRASONIC_TRIGGER_PIN_2  5   // Sensor 2 Trigger (adjust as needed)
// #define ULTRASONIC_ECHO_PIN_2     4   // Sensor 2 Echo (adjust as needed)
#define IR_DISTANCE_SENSOR_PIN    A9  // Analog pin for 2Y0A02 IR distance sensor

// VL53L1X sensor configuration
const uint8_t VL53L1X_SENSOR_COUNT = 4;  // 4 tier sensors
const uint8_t VL53L1X_XSHUT_PINS[VL53L1X_SENSOR_COUNT] = { 0, 1, 2, 3 };  // Update this with the XSHUT pins for each sensor
#define VL53L0X_TOP_XSHUT_PIN 6   // Digital pin used to control the top VL53L0X sensor XSHUT

// DC Motor PIN definitions
#define DC_M1_PIN 0              //PWM pin for DC motor control (can be extended to 2 motors)
#define DC_M2_PIN 1              //PWM pin for DC motor control (can be extended to 2 motors)
#define DC_M1_ENC_A 2            //Encoder A pin for motor 1 (D2)
#define DC_M1_ENC_B 3            //Encoder B pin for motor 1 (D3)
#define DC_M2_ENC_A 4            //Encoder A pin for motor 2 (D4)
#define DC_M2_ENC_B 5            //Encoder B pin for motor 2 (D5)
#define MOTOR_SPEED 80           // Example motor speed (set to desired value)

//electromagnet PIN definition
#define MAGNET_PIN 26

// Herkulex smart servo ID (broadcast ID 0xfe)
#define HERKULEX_ID 1

// Herkulex serial pins when using SoftwareSerial (rx, tx)
#define HERKULEX_RX_PIN 0
#define HERKULEX_TX_PIN 1

// Serial deffinitions
#define BAUD_RATE 115200
#define BLUETOOTH_BAUD 115200  // CH9143 factory default UART baud rate
#define BLUETOOTH_RX_PIN 28  // Serial7 RX
#define BLUETOOTH_TX_PIN 29  // Serial7 TX

Servo right_motor;
Servo left_motor;

// Centralized motor control instance (includes straight-line sync PID)
MotorControl driveMotor(DC_M1_PIN, DC_M1_ENC_A, DC_M1_ENC_B,
                        DC_M2_PIN, DC_M2_ENC_A, DC_M2_ENC_B);

// BNO055 IMU structure
struct bno055_t bno055;

// Global IMU data storage (for inter-task communication)
IMU_Data current_imu_data;

// Proximity Sensor instance
ProximitySensor proximitySensor(PROXIMITY_SENSOR_PIN);

// Optical flow sensor (PMW3901) using SPI: CS on D10, MOSI D11, MISO D12, SCK D13
OpticalFlow opticalFlow(10);

// Ultrasonic Sensor instance (kept for backwards compatibility)
// UltrasonicSensor ultrasonicSensor(ULTRASONIC_TRIGGER_PIN_1, ULTRASONIC_ECHO_PIN_1);

// Ultrasonic Sensor Array for 2 sensors
UltrasonicSensorArray ultrasonicArray(2);

// Color Sensor instance
ColorSensor colorSensor;

// IR XY Position Sensor instance
IRXYPosition irXYSensor;

// IR Distance Sensor instance (2Y0A02)
IRDistanceSensor irDistanceSensor(IR_DISTANCE_SENSOR_PIN);

// TOF Sensor Array instance (explicit SX1509 I2C address 0x3F)
TOFSensorArray tofSensorArray(VL53L1X_SENSOR_COUNT, 0x3F);
VL53L0X topTofSensor;
bool topTofSensorInitialized = false;
TieredTargetDetector targetDetector(tofSensorArray, 0, 1, 2, 3);

// CH9143 Bluetooth instance
CH9143Bluetooth bluetooth(&Serial7, BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN, BLUETOOTH_BAUD);

// Motor command state (received from Serial or Bluetooth)
int16_t cmd_left_motor = 0;
int16_t cmd_right_motor = 0;
unsigned long cmd_last_rx_ms = 0;

// Output shaping for smooth motion (applied to all command sources)
// tDC_motor runs every 40 ms, so step 8 ~= 200 speed-units/second.
static int16_t applied_left_motor = 0;
static int16_t applied_right_motor = 0;
static const int16_t MOTOR_SLEW_STEP = 8;

// Forward declarations for command handling
void process_bluetooth_motor_commands(void);
void process_usb_motor_commands(void);
void handle_motor_line(const char* line, Print* ackPort);
int16_t slew_toward(int16_t current, int16_t target, int16_t step);

// Task wrapper for proximity sensor reading
void proximity_sensor_callback(void) {
    proximitySensor.update();
    proximitySensor.printStatus();
}

// Task wrapper for ultrasonic sensor reading (array with 2 sensors)
void ultrasonic_sensor_callback(void) {
    if (ultrasonicArray.isInitialized()) {
        UltrasonicSensorArray::UltrasonicData data = ultrasonicArray.readDistances();
        ultrasonicArray.printDistances(data);
        // Optional: free the allocated memory after use
        delete[] data.distances;
    }
}

// Task wrapper for color sensor reading
void color_sensor_callback(void) {
    if (colorSensor.isInitialized()) {
        ColorSensor::ColorData colorData = colorSensor.readColor();
        colorSensor.decodeColor(colorData);  // Print to USB serial
        
        // Also send to Bluetooth
        if (bluetooth.isInitialized()) {
            uint8_t r = (colorData.hexColor >> 16) & 0xFF;
            uint8_t g = (colorData.hexColor >> 8) & 0xFF;
            uint8_t b = colorData.hexColor & 0xFF;
            bluetooth.printf("BT Color: #%02X%02X%02X\n", r, g, b);
        }
    }
}

// Task wrapper for IR XY position sensor reading
void ir_xy_position_callback(void) {
    if (irXYSensor.isInitialized()) {
        IRXYPosition::IRData irData = irXYSensor.readPositions();
        irXYSensor.printPositions(irData);
    }
}

// Task wrapper for optical flow sensor reading
void optical_flow_callback(void) {
    int16_t dx = 0, dy = 0;
    if (opticalFlow.read(dx, dy)) {
        // accumulate counts and print total distance in mm
        opticalFlow.addMotionCounts(dx, dy);
        float tx = opticalFlow.getTotalXmm();
        float ty = opticalFlow.getTotalYmm();
        printfBoth("OpticalFlow totalX: %.2f mm  totalY: %.2f mm\n", tx, ty);
    }
}

// Task wrapper for VL53L1X sensor reading
void vl53l1x_sensor_callback(void) {
    if (tofSensorArray.isInitialized()) {
        TOFSensorArray::TOFData tofData = tofSensorArray.readDistances();

        // Print VL53L1X distances inline
        Serial.print("\t");
        for (uint8_t i = 0; i < tofData.sensorCount; i++) {
            Serial.print("S");
            Serial.print(i);
            Serial.print(":");
            if (tofData.distances[i] == 0xFFFF) {
                Serial.print("TIMEOUT");
            } else {
                Serial.print(tofData.distances[i]);
            }
            if (i < tofData.sensorCount - 1) Serial.print("\t");
        }

        uint16_t topDistance = 0xFFFF;
        if (topTofSensorInitialized) {
            topDistance = topTofSensor.readRangeContinuousMillimeters();
            Serial.print("\tTop:");
            Serial.print(topDistance);
        }

        if (targetDetector.update(topDistance)) {
            TierDetectionResult result = targetDetector.getDetectionResult();
            if (result != TierDetectionResult::None) {
                targetDetector.debugPrint();
                if (result == TierDetectionResult::Target) {
                    Serial.println("Tier detector: TARGET confirmed");
                } else if (result == TierDetectionResult::Obstacle) {
                    Serial.println("Tier detector: OBSTACLE confirmed");
                } else {
                    Serial.println("Tier detector: INDETERMINATE");
                }
            }
        }

        // If the DFRobot Matrix Lidar (8x8) is present, print it as 8 rows
        if (TOF_X8_isInitialized()) {
            uint16_t x8buf[64];
            if (TOF_X8_readAll(x8buf, 64)) {
                // Print each row on its own line, prefixed with a tab for alignment
                Serial.print('\n');
                Serial.print('\n');
                for (uint8_t y = 0; y < 8; y++) {
                    Serial.print('\t');
                    Serial.print("Y");
                    Serial.print(y);
                    Serial.print(": ");
                    for (uint8_t x = 0; x < 8; x++) {
                        Serial.print(x8buf[y * 8 + x]);
                        if (x < 7) Serial.print(",");
                    }
                    Serial.println();
                }
                Serial.print('\n');
            } else {
                Serial.println('\t',"X8:ERR");
            }
        } else {
            // No X8 initialization; print explicit marker so absence is visible
            Serial.print('\t');
            Serial.println("X8:NOTINIT");
        }
    }
}

// Task wrapper for IR distance sensor reading
void ir_distance_sensor_callback(void) {
    if (irDistanceSensor.isInitialized()) {
        irDistanceSensor.readDistance();
        irDistanceSensor.printDistance();
    }
}

// Task wrapper for IMU reading
void imu_task_callback(void) {
    current_imu_data = read_imu();
    // Print IMU data to both Serial and Bluetooth
    printfBoth("Raw Gyro X:%d Y:%d Z:%d | Accel: %d, %d, %d\n",
               current_imu_data.gyro_x,
               current_imu_data.gyro_y,
               current_imu_data.gyro_z,
               current_imu_data.accel_x,
               current_imu_data.accel_y,
               current_imu_data.accel_z);
}

// Task wrapper for DC motor control
void dc_motor_callback(void) {
    int16_t left_target = cmd_left_motor;
    int16_t right_target = cmd_right_motor;

    // Smoothly approach targets to avoid abrupt jumps and harsh reversals.
    // This ensures transitions like full straight -> full left are gradual.
    applied_left_motor = slew_toward(applied_left_motor, left_target, MOTOR_SLEW_STEP);
    applied_right_motor = slew_toward(applied_right_motor, right_target, MOTOR_SLEW_STEP);

    // Latch behavior: hold last commanded speeds until changed
    driveMotor.setSpeeds(applied_left_motor, applied_right_motor);

    // Print encoder pulse counts to USB serial at a limited rate
    static unsigned long lastEncoderPrintMs = 0;
    const unsigned long encoderPrintPeriodMs = 200;
    unsigned long now = millis();

    if (now - lastEncoderPrintMs >= encoderPrintPeriodMs) {
        Serial.print("ENC L:");
        Serial.print(driveMotor.getLeftEncoderPulses());
        Serial.print(" R:");
        Serial.println(driveMotor.getRightEncoderPulses());
        lastEncoderPrintMs = now;
    }
}

int16_t slew_toward(int16_t current, int16_t target, int16_t step) {
    if (step <= 0) {
        return target;
    }

    int delta = (int)target - (int)current;
    if (delta > step) {
        delta = step;
    } else if (delta < -step) {
        delta = -step;
    }

    int next = (int)current + delta;
    return (int16_t)constrain(next, -100, 100);
}

// Bluetooth stream test — called every 200 ms by the task scheduler.
// Sends a counter, timestamp, and motor command state so you can verify
// the BT link is live and measure throughput on the remote end.
void bt_stream_test_callback(void) {
    static uint32_t pkt = 0;
    if (!bluetooth.isInitialized()) return;
    bluetooth.printf("PKT:%lu T:%lu L:%d R:%d\n",
        pkt++, millis(), (int)cmd_left_motor, (int)cmd_right_motor);
}

// Read and parse newline-terminated commands from Bluetooth link
void process_bluetooth_motor_commands(void) {
    static char lineBuffer[64];
    static uint8_t idx = 0;

    while (bluetooth.available()) {
        char c = bluetooth.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            lineBuffer[idx] = '\0';
            if (idx > 0) {
                handle_motor_line(lineBuffer, bluetooth.isInitialized() ? (Print*)&Serial7 : nullptr);
            }
            idx = 0;
            continue;
        }

        if (idx < (sizeof(lineBuffer) - 1)) {
            lineBuffer[idx++] = c;
        } else {
            // Overflow protection: discard current line
            idx = 0;
        }
    }
}

// Read and parse newline-terminated commands from USB serial monitor
void process_usb_motor_commands(void) {
    static char lineBuffer[64];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            lineBuffer[idx] = '\0';
            if (idx > 0) {
                handle_motor_line(lineBuffer, &Serial);
            }
            idx = 0;
            continue;
        }

        if (idx < (sizeof(lineBuffer) - 1)) {
            lineBuffer[idx++] = c;
        } else {
            // Overflow protection: discard current line
            idx = 0;
        }
    }
}

// Supported commands:
//   MOTOR <left> <right>           where left/right are in [-100, 100]
//   STOP
//   PID ... / SYNC ...             handled by motor_control.cpp
//   FLOW RESET                     reset accumulated optical-flow totals
//   FLOW STATUS                    print optical-flow totals (mm)
//   FLOW LED <ON|OFF>              enable/disable optical-flow illumination LED
//   FLOW SCALE <mm_per_count>      set millimetres-per-motion-count scale
void handle_motor_line(const char* line, Print* ackPort) {
    // Convert input line to uppercase for case-insensitive command matching
    char upperLine[64];
    strncpy(upperLine, line, sizeof(upperLine) - 1);
    upperLine[sizeof(upperLine) - 1] = '\0';
    for (int i = 0; upperLine[i] != '\0'; i++) {
        upperLine[i] = toupper((unsigned char)upperLine[i]);
    }

    int left = 0;
    int right = 0;

    if (sscanf(upperLine, "MOTOR %d %d", &left, &right) == 2) {
        left = constrain(left, -100, 100);
        right = constrain(right, -100, 100);

        cmd_left_motor = (int16_t)left;
        cmd_right_motor = (int16_t)right;
        cmd_last_rx_ms = millis();

        if (ackPort != nullptr) {
            ackPort->print("ACK MOTOR ");
            ackPort->print(cmd_left_motor);
            ackPort->print(" ");
            ackPort->println(cmd_right_motor);
        }
        return;
    }

    if (strcmp(upperLine, "STOP") == 0) {
        cmd_left_motor = 0;
        cmd_right_motor = 0;
        cmd_last_rx_ms = millis();
        if (ackPort != nullptr) {
            ackPort->println("ACK STOP");
        }
        return;
    }

    // Optical flow control commands (FLOW ...)
    if (strncmp(upperLine, "FLOW", 4) == 0) {
        if (strcmp(upperLine, "FLOW RESET") == 0) {
            opticalFlow.resetTotals();
            cmd_last_rx_ms = millis();
            if (ackPort != nullptr) ackPort->println("ACK FLOW RESET");
            else Serial.println("ACK FLOW RESET");
            return;
        }

        if (strcmp(upperLine, "FLOW STATUS") == 0) {
            float tx = opticalFlow.getTotalXmm();
            float ty = opticalFlow.getTotalYmm();
            if (ackPort != nullptr) {
                ackPort->printf("FLOW STATUS X: %.2f mm Y: %.2f mm\n", tx, ty);
            } else {
                Serial.printf("FLOW STATUS X: %.2f mm Y: %.2f mm\n", tx, ty);
            }
            return;
        }

        if (strncmp(upperLine, "FLOW LED ", 9) == 0) {
            if (strcmp(upperLine + 9, "ON") == 0) {
                opticalFlow.setLed(true);
                if (ackPort != nullptr) ackPort->println("ACK FLOW LED ON");
                else Serial.println("ACK FLOW LED ON");
                return;
            }
            if (strcmp(upperLine + 9, "OFF") == 0) {
                opticalFlow.setLed(false);
                if (ackPort != nullptr) ackPort->println("ACK FLOW LED OFF");
                else Serial.println("ACK FLOW LED OFF");
                return;
            }
        }

        float scale = 0.0f;
        if (sscanf(upperLine, "FLOW SCALE %f", &scale) == 1) {
            opticalFlow.setScaleMMPerCount(scale);
            if (ackPort != nullptr) {
                ackPort->print("ACK FLOW SCALE ");
                ackPort->println(scale);
            } else {
                Serial.print("ACK FLOW SCALE ");
                Serial.println(scale);
            }
            return;
        }

        if (ackPort != nullptr) ackPort->println("ERR Unknown FLOW command (RESET, STATUS, LED, SCALE)");
        else Serial.println("ERR Unknown FLOW command (RESET, STATUS, LED, SCALE)");
        return;
    }

    if (motorControlHandleCommandLine(upperLine, ackPort)) {
        return;
    }

    if (ackPort != nullptr) {
        ackPort->println("ERR Unknown command (MOTOR <L> <R>, STOP, PID..., SYNC...)");
    }
}

// Herkulex continuous test callback: toggles between -100 and 100 degrees
void herkulex_test_callback() {
    static bool toggle = false;
    int angle = toggle ? 100 : -100;
    int led = toggle ? LED_GREEN : LED_BLUE;
    Herkulex.torqueON(HERKULEX_ID);
    Herkulex.moveOneAngle(HERKULEX_ID, angle, 1000, led);
    Herkulex.moveOneAngle(HERKULEX_ID, -100, 1000, LED_BLUE);
    //printfBoth("Herkulex test move to %d\n", Herkulex.getPosition(HERKULEX_ID));
    toggle = !toggle;
}

//**********************************************************************************
// Task Scheduler and Tasks
//**********************************************************************************

/* The first value is the period, second is how many times it executes
   (-1 means indefinitely), third one is the callback function */

// Tasks for reading sensors 
Task tRead_ultrasonic(US_READ_TASK_PERIOD,       US_READ_TASK_NUM_EXECUTE,        &read_ultrasonic);
Task tRead_infrared(IR_READ_TASK_PERIOD,         IR_READ_TASK_NUM_EXECUTE,        &read_infrared);
Task tRead_colour(COLOUR_READ_TASK_PERIOD,       COLOUR_READ_TASK_NUM_EXECUTE,    &read_colour);
Task tRead_imu(IMU_READ_TASK_PERIOD,             IMU_READ_TASK_NUM_EXECUTE,       &imu_task_callback);
Task tProximity_sensor(PROXIMITY_SENSOR_READ_PERIOD, PROXIMITY_SENSOR_NUM_EXECUTE,  &proximity_sensor_callback);
Task tUltrasonic_sensor(ULTRASONIC_SENSOR_READ_PERIOD, ULTRASONIC_SENSOR_NUM_EXECUTE, &ultrasonic_sensor_callback);
Task tColor_sensor(COLOUR_READ_TASK_PERIOD, COLOUR_READ_TASK_NUM_EXECUTE, &color_sensor_callback);
Task tIR_XY_Position(IR_READ_TASK_PERIOD, IR_READ_TASK_NUM_EXECUTE, &ir_xy_position_callback);
Task tVL53L1X_sensor(VL53L1X_SENSOR_READ_PERIOD, VL53L1X_SENSOR_NUM_EXECUTE, &vl53l1x_sensor_callback);
Task tTOF_X8(VL53L1X_SENSOR_READ_PERIOD, VL53L1X_SENSOR_NUM_EXECUTE, &TOF_X8_task_callback);
Task tIR_Distance_sensor(IR_DISTANCE_SENSOR_READ_PERIOD, IR_DISTANCE_SENSOR_NUM_EXECUTE, &ir_distance_sensor_callback);
Task tOpticalFlow(OF_READ_TASK_PERIOD, OF_READ_TASK_NUM_EXECUTE, &optical_flow_callback);
Task tSensor_average(SENSOR_AVERAGE_PERIOD,      SENSOR_AVERAGE_NUM_EXECUTE,      &sensor_average);
Task tHerkulexTest(HERKULEX_TEST_PERIOD, -1, &herkulex_test_callback);
Task tBT_stream_test(200, -1, &bt_stream_test_callback);  // Stream test: every 200ms

// Task for DC motor control
Task tDC_motor(DC_MOTOR_CONTROL_PERIOD,          DC_MOTOR_CONTROL_NUM_EXECUTE,    &dc_motor_callback);

// Task to set the motor speeds and direction
Task tSet_motor(SET_MOTOR_TASK_PERIOD,           SET_MOTOR_TASK_NUM_EXECUTE,      &set_motor);

// Tasks to scan for weights and collection upon detection
Task tWeight_scan(WEIGHT_SCAN_TASK_PERIOD,       WEIGHT_SCAN_TASK_NUM_EXECUTE,    &weight_scan);
Task tCollect_weight(COLLECT_WEIGHT_TASK_PERIOD, COLLECT_WEIGHT_TASK_NUM_EXECUTE, &collect_weight);

// Tasks to search for bases and unload weights
Task tReturn_to_base(RETURN_TO_BASE_TASK_PERIOD, RETURN_TO_BASE_TASK_NUM_EXECUTE, &return_to_base);
Task tDetect_base(DETECT_BASE_TASK_PERIOD,       DETECT_BASE_TASK_NUM_EXECUTE,    &detect_base);
Task tUnload_weights(UNLOAD_WEIGHTS_TASK_PERIOD, UNLOAD_WEIGHTS_TASK_NUM_EXECUTE, &unload_weights);

// Tasks to check the 'watchdog' timer (These will need to be added in)
//Task tCheck_watchdog(CHECK_WATCHDOG_TASK_PERIOD, CHECK_WATCHDOG_TASK_NUM_EXECUTE, &check_watchdog);
//Task tVictory_dance(VICTORY_DANCE_TASK_PERIOD,   VICTORY_DANCE_TASK_NUM_EXECUTE,  &victory_dance);

Scheduler taskManager;

//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
void robot_init();
void task_init();


// Helper functions to stream output to both Serial and Bluetooth
void printBoth(const char* data) {
    Serial.print(data);
    if (bluetooth.isInitialized()) {
        bluetooth.print(data);
    }
}

void printlnBoth(const char* data) {
    Serial.println(data);
    if (bluetooth.isInitialized()) {
        bluetooth.println(data);
    }
}

void printfBoth(const char* format, ...) {
    char buffer[256];
    va_list args;
    
    // Print to Serial
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.print(buffer);
    
    // Print to Bluetooth
    if (bluetooth.isInitialized()) {
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        bluetooth.print(buffer);
    }
}

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  delay(2000);  // Give USB serial time to stabilize
  Serial.println("\n\n=== RoboCup Robot Starting ===");
  
  // CH9143 two-chip bridge architecture:
  //   PC (USB-C) --> CH9143 [USB chip]  ~~~BLE~~~  CH9143 [UART chip] --> Serial7 --> Teensy
  // The UART chip communicates at 115200 (CH9143 factory default).
  // No Bluetooth pairing needed on the PC \u2014 it connects via USB-C to the USB chip.
    // Serial7 is initialized by bluetooth.begin() during robot_init().
  
  Wire.begin();        // MUST be called FIRST - before any I2C operations
  pin_init();
  robot_init();        // robot_init() calls BNO_Init() which needs I2C
  task_init();
  
  // Now that Bluetooth is initialized, send startup message
  if (bluetooth.isInitialized()) {
    bluetooth.println("Setup Complete");
  }
}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work) 
// Set as high or low
//**********************************************************************************
void pin_init(){
    
    printlnBoth("Pins have been initialised \n"); 

    pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
    digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
    // Initialise electromagnet pin (analog connector used as digital output)
    pinMode(MAGNET_PIN, OUTPUT);
    digitalWrite(MAGNET_PIN, LOW); // ensure off by default
    printlnBoth("Electromagnet pin initialised\n");

    // Pulse electromagnet HIGH for 100 ms for initial test
    printlnBoth("Pulsing electromagnet HIGH for 100 ms\n");
    digitalWrite(MAGNET_PIN, HIGH);
}

//**********************************************************************************
// Set default robot state
//**********************************************************************************
void robot_init() {
    printlnBoth("Initialising Proximity Sensor...");
    proximitySensor.begin();
    
    // printlnBoth("Initialising Ultrasonic Sensor Array...");
    // ultrasonicArray.addSensor(0, ULTRASONIC_TRIGGER_PIN_1, ULTRASONIC_ECHO_PIN_1);
    // ultrasonicArray.addSensor(1, ULTRASONIC_TRIGGER_PIN_2, ULTRASONIC_ECHO_PIN_2);
    // ultrasonicArray.begin();
    
    printlnBoth("Initialising Color Sensor...");
    colorSensor.begin();
    
    printlnBoth("Initialising CH9143 Bluetooth...");
    if (bluetooth.begin()) {
        Serial.println("✓ Bluetooth module initialized");
        bluetooth.println("=== Bluetooth Ready ===");
    } else {
        Serial.println("✗ Bluetooth initialization failed!");
    }

    // Initialise Herkulex smart servo using a hardware UART (avoids SoftwareSerial)
    // On Teensy/Mega use Serial1 (pins 0/1 on many boards)
    printlnBoth("Initialising Herkulex Smart Servo on Serial1...");
    Herkulex.begin(115200, HERKULEX_RX_PIN, HERKULEX_TX_PIN);
    delay(100);
    Herkulex.reboot(HERKULEX_ID);
    delay(500);
    Herkulex.clearError(HERKULEX_ID);
    Herkulex.ACK(1);  // Set ACK mode to 1 (ACK on error only)
    Herkulex.set_ID(HERKULEX_ID, HERKULEX_ID);  // Ensure servo ID is set correctly
    Herkulex.torqueON(HERKULEX_ID);
    Herkulex.initialize();

    delay(200);
    printlnBoth("Herkulex Smart Servo initialized\n");
    // Diagnostics: read status, model and position
    int hk_stat = Herkulex.stat(HERKULEX_ID);
    int hk_model = Herkulex.model();
    int hk_pos = Herkulex.getPosition(HERKULEX_ID);
    printfBoth("Herkulex stat: %d | model: %d | position: %d\n", hk_stat, hk_model, hk_pos);

    // If status OK (0) then perform startup test move
    
    printlnBoth("Herkulex startup test move: moving to -100 then 100\n");
    Herkulex.moveOneAngle(HERKULEX_ID, -100, 1000, LED_BLUE);
    delay(1200);
    Herkulex.moveOneAngle(HERKULEX_ID, 100, 1000, LED_GREEN);
    delay(1200);
    printlnBoth("Herkulex test move complete\n");
    
    
    printlnBoth("Initialising IR XY Position Sensor...");
    irXYSensor.begin();
    
    printlnBoth("Initialising Optical Flow (PMW3901) on SPI CS D10...");
    if (!opticalFlow.begin()) {
        printlnBoth("WARNING: Optical Flow init failed");
    } else {
        printlnBoth("Optical Flow initialized");
    }

    printlnBoth("Initialising IR Distance Sensor (2Y0A02)...");
    irDistanceSensor.begin();
    
    printlnBoth("Initialising TOF (VL53L1X) Sensor Array...");
    Wire.setClock(400000); // use 400 kHz I2C
    
    // Set XSHUT pins for each sensor
    tofSensorArray.setXSHUTPins(VL53L1X_XSHUT_PINS, VL53L1X_SENSOR_COUNT);
    
    // Initialize the TOF sensor array
    if (!tofSensorArray.begin()) {
        printlnBoth("WARNING: Failed to initialize TOF sensor array");
    } else {
        printlnBoth("TOF sensor array initialized successfully");
        tofSensorArray.setDistanceOffset(0, 0);   // Sensor 0 offset
        tofSensorArray.setDistanceOffset(1, -40); // Sensor 1 offset
        tofSensorArray.setDistanceOffset(2, 0);  // Sensor 2 offset
        tofSensorArray.setDistanceOffset(3, 0);  // Sensor 3 offset

        pinMode(VL53L0X_TOP_XSHUT_PIN, OUTPUT);
        digitalWrite(VL53L0X_TOP_XSHUT_PIN, LOW);
        delay(10);
        digitalWrite(VL53L0X_TOP_XSHUT_PIN, HIGH);
        delay(10);

        topTofSensor.setTimeout(500);
        if (!topTofSensor.init()) {
            printlnBoth("WARNING: Failed to initialize top VL53L0X sensor");
        } else {
            topTofSensor.startContinuous(50);
            topTofSensorInitialized = true;
            printlnBoth("Top VL53L0X sensor initialized successfully");
        }

        TieredTargetDetectorConfig targetConfig;
        targetConfig.nearIntersectMm = 300;
        targetConfig.farIntersectMm = 500;
        targetConfig.intersectionToleranceMm = 35;
        targetConfig.topSensorClearanceMm = 80;
        targetConfig.topSensorRejectMarginMm = 40;
        targetDetector.setConfig(targetConfig);
    }

    // Initialize optional DFRobot Matrix Lidar (8x8 matrix) if connected
    printlnBoth("Initialising DFRobot Matrix Lidar (TOF_X8)...");
    if (!TOF_X8_begin()) {
        printlnBoth("WARNING: TOF_X8 initialization failed or not present");
    } else {
        printlnBoth("TOF_X8 initialized successfully");
    }
    
    printlnBoth("Initialising IMU (BNO055)...");
    BNO055_RETURN_FUNCTION_TYPE init_result = BNO_Init(&bno055);
    
    if (init_result == SUCCESS) {
        printlnBoth("IMU initialised successfully");
        delay(500);
        
        // Set operation mode to NDOF (Nine Degrees of Freedom)
        bno055_set_operation_mode(OPERATION_MODE_NDOF);
        delay(50);  // Wait for mode transition
        
        // Calibrate gyroscope to zero on startup
        calibrate_gyroscope();
        
        printlnBoth("Initialising DC Motor...");
        driveMotor.begin();
        
        printlnBoth("Robot is ready \n");
    } else {
        printfBoth("ERROR: Failed to initialise IMU! Error code: %d\n", init_result);
    }
}

//**********************************************************************************
// Initialise the tasks for the scheduler
//**********************************************************************************
void task_init() {  
  
  // This is a class/library function. Initialise the task scheduler
  taskManager.init();     
 
  // Add tasks to the scheduler
  // taskManager.addTask(tRead_ultrasonic);   //reading ultrasonic 
  // taskManager.addTask(tRead_infrared);
  // taskManager.addTask(tRead_colour);
//   taskManager.addTask(tRead_imu);          //reading IMU
//   taskManager.addTask(tProximity_sensor);  //reading proximity sensor
    // taskManager.addTask(tUltrasonic_sensor);  //reading ultrasonic sensor
    // taskManager.addTask(tColor_sensor);       //reading color sensor
    // taskManager.addTask(tIR_XY_Position);     //reading IR XY position sensor
    taskManager.addTask(tVL53L1X_sensor);     //reading VL53L1X sensors
    taskManager.addTask(tTOF_X8);              //reading DFRobot Matrix Lidar 8x8 (if present)
    //taskManager.addTask(tIR_Distance_sensor); //reading IR distance sensor (2Y0A02)  
    // taskManager.addTask(tSensor_average);
    // taskManager.addTask(tDC_motor);          //DC motor control
  // taskManager.addTask(tSet_motor); 
  // taskManager.addTask(tWeight_scan);
  // taskManager.addTask(tCollect_weight);
  // taskManager.addTask(tReturn_to_base);
  // taskManager.addTask(tDetect_base);
  // taskManager.addTask(tUnload_weights);

  //taskManager.addTask(tCheck_watchdog);
  //taskManager.addTask(tVictory_dance);      

    // taskManager.addTask(tHerkulexTest);
    // taskManager.addTask(tBT_stream_test);  // Disabled for control reliability
    // taskManager.addTask(tOpticalFlow);        //reading optical flow sensor

    //enable the tasks
  tRead_ultrasonic.enable();
  tRead_infrared.enable();
  tRead_colour.enable();
  tRead_imu.enable();
  tProximity_sensor.enable();
  tUltrasonic_sensor.enable();
  tColor_sensor.enable();
  tIR_XY_Position.enable();
  tVL53L1X_sensor.enable();
  tIR_Distance_sensor.enable();
    tTOF_X8.enable();
  tSensor_average.enable();
  tDC_motor.enable();
  tSet_motor.enable();
  tWeight_scan.enable();
  tCollect_weight.enable();
  tReturn_to_base.enable();
  tDetect_base.enable();
  tUnload_weights.enable();
 //tCheck_watchdog.enable();
 //tVictory_dance.enable();
   tHerkulexTest.enable();
    // tBT_stream_test.enable();  // Disabled for control reliability
    tOpticalFlow.enable();

 printlnBoth("Tasks have been initialised \n");
}



//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
    // Consume inbound Bluetooth control commands continuously
    process_bluetooth_motor_commands();
    // Also accept commands from USB serial monitor (direct wired testing)
    process_usb_motor_commands();
  
    taskManager.execute();    //execute the scheduler
    //Serial.println("Another scheduler execution cycle has oocured \n");
}

