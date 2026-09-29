
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
// Herkulex support removed
#include <Adafruit_TCS34725.h>      //colour sensor
#include <Wire.h>                   //for I2C and SPI
#include <TaskScheduler.h>          //scheduler
#include <VL53L1X.h>                //VL53L1X distance sensor
#include <stdarg.h>                 //for va_list in printf functions
#include <stdio.h>                  //for vsnprintf
#include <string.h>                 //for strcmp/sscanf parsing
#include <stdbool.h>
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
#include "TOF_X8.h"                 // DFRobot 8x8 matrix ToF lidar
#include "dc_motor.h"               //DC motor control 
#include "motor_control.h"          //PID/SYNC command handling
#include "ch9143_bluetooth.h"       //CH9143 Bluetooth module
#include "optical_flow.h"           // PMW3901 optical flow (Bitcraze)
#include "ld06.h"                   // LD06 LiDAR
#include "BigServo.h"               // Big servo
#include "RoboSLAM.h"
#include "POIDetector.h"             // ToF-based POI/weight detection
#include "ToFCoverageMap.h"          // ToF coverage tracking
#include "ToFSearchPlanner.h"        // Search target generation
#include "ToFGeometry.h"             // Shared ToF sensor geometry
#include "telemetry.h"
#include "target_detector.h"

// Debug printouts

#ifndef TELEMETRY_ALLOW_ASCII_TELEPLOT
#define TELEMETRY_ALLOW_ASCII_TELEPLOT 0
#endif

#define OPTICAL_FLOW_DEBUG 0

#define TOF_DEBUG 1

#define PROXIMITY_DEBUG 0

#define DCMOTOR_DEBUG 0

#define LOCALISATION_DEBUG 0

#define OPTICAL_FLOW_SERIAL_TEST 0

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
#define BIG_SERVO_PERIOD                    100


#define OF_READ_TASK_PERIOD                 40
#define OF_READ_TASK_NUM_EXECUTE            -1

#define LD06_READ_TASK_PERIOD               1
#define LD06_READ_TASK_NUM_EXECUTE          -1

#define POSE_PREDICTION_PERIOD               10
#define POSE_PREDICTION_NUM_EXECUTE          -1

#define POI_DETECTOR_UPDATE_PERIOD          100  // 10 Hz: process ToF readings
#define POI_DETECTOR_UPDATE_NUM_EXECUTE     -1

#define TOF_SEARCH_PLANNER_PERIOD           500  // 2 Hz: generate search targets less frequently
#define TOF_SEARCH_PLANNER_NUM_EXECUTE      -1

// Herkulex test period removed




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
#define BIG_SERVO_NUM_EXECUTE               -1

// Pin deffinitions
#define IO_POWER  49
#define PROXIMITY_SENSOR_PIN  20  // A6Z
// #define ULTRASONIC_TRIGGER_PIN_1  3   // D2Z - Sensor 1 Trigger
// #define ULTRASONIC_ECHO_PIN_1     2   // D3Z - Sensor 1 Echo
// #define ULTRASONIC_TRIGGER_PIN_2  5   // Sensor 2 Trigger (adjust as needed)
// #define ULTRASONIC_ECHO_PIN_2     4   // Sensor 2 Echo (adjust as needed)
#define IR_DISTANCE_SENSOR_PIN    A9  // Analog pin for 2Y0A02 IR distance sensor

// VL53L1X sensor configuration
const uint8_t VL53L1X_SENSOR_COUNT = 4;  // Update this if you add more sensors
const uint8_t VL53L1X_XSHUT_PINS[VL53L1X_SENSOR_COUNT] = {0, 3, 2, 1};  // Update this with the XSHUT pins for each sensor

#define START_BUTTON_PIN 21

// ============================================================
// ROBOT START POSITION
// ============================================================

enum RobotStartSide {
    START_LEFT = 0,
    START_RIGHT = 1
};

// Change this to START_LEFT or START_RIGHT
RobotStartSide robotStartSide = START_LEFT;

// IMPORTANT:
// Replace these with the actual coordinates from your
// RoboCup arena/brief once confirmed.

const float START_LEFT_X_M = 0.30f;
const float START_LEFT_Y_M = 0.30f;
const float START_LEFT_THETA_RAD = 0.0f;

const float START_RIGHT_X_M = 2.10f;
const float START_RIGHT_Y_M = 0.30f;
const float START_RIGHT_THETA_RAD = PI;

// ============================================================
// SENSOR STATUS
// ============================================================

bool telemetry_lidar_ok = false;
bool telemetry_imu_ok = false;
bool telemetry_tof_ok = false;
bool telemetry_optical_flow_ok = false;
bool telemetry_ultrasonic_ok = false;
bool binary_telemetry_active = OPTICAL_FLOW_SERIAL_TEST;


bool robotStarted = false;
bool lastStartButtonState = HIGH;
unsigned long startButtonDebounceTime = 0;
const unsigned long START_BUTTON_DEBOUNCE_MS = 50;

bool pickingUp = false;
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

// Herkulex support removed

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
// Initial heading offset (set at IMU init to make headings relative to startup)
float imu_heading_offset = 0.0f;

// Proximity Sensor instance
ProximitySensor proximitySensor(PROXIMITY_SENSOR_PIN);

// Optical flow sensor (PMW3901) using SPI: CS on D10, MOSI D11, MISO D12, SCK D13
OpticalFlow opticalFlow(10);

// LD06 LiDAR on Serial2 (UART @ 230400 configured in ld06.init())
LD06 ld06(Serial2);

// Mapping and on-board SLAM
MappingNav mappingNav;
RoboSLAM roboSlam(mappingNav, driveMotor);

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

// TOF Sensor Array instance
TOFSensorArray tofSensorArray(VL53L1X_SENSOR_COUNT);

// ToF sensor indices
constexpr uint8_t TOF_NEAR_A = 0;
constexpr uint8_t TOF_NEAR_B = 1;
constexpr uint8_t TOF_FAR_A  = 2;
constexpr uint8_t TOF_FAR_B  = 3;

TieredTargetDetector targetDetector(
    tofSensorArray,
    ld06,
    TOF_NEAR_A,
    TOF_NEAR_B,
    TOF_FAR_A,
    TOF_FAR_B
);

// POI detection system
POIDetector poiDetector;
ToFCoverageMap tofCoverageMap;
ToFSearchPlanner searchPlanner;

// ToF sensor calibration (MUST BE CALIBRATED FOR YOUR ROBOT)
ToFExtrinsics tof_extrinsics = {0.0f, 0.0f, 0.0f};  // x_offset_m, y_offset_m, yaw_offset_rad

// Additional diagnostic counters for testing
uint32_t optical_flow_update_count = 0;  // Tracks optical flow updates
uint32_t tof_reading_count = 0;          // Tracks ToF readings processed
uint32_t poi_candidate_count = 0;        // Tracks POI candidates created
uint32_t poi_confirmed_count = 0;        // Tracks confirmed POIs

// Note: scan_ready_count, scan_telemetry_call_count, last_scan_point_count are defined in telemetry.cpp

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
    #if PROXIMITY_DEBUG
    proximitySensor.printStatus();
    #endif
}

// Task wrapper for ultrasonic sensor reading (array with 2 sensors)
void ultrasonic_sensor_callback(void) {
    // The ultrasonic array is not configured in robot_init() yet.
    if (!ultrasonicArray.isInitialized()) {
        static bool warned = false;
        if (!warned) {
            Serial.println("WARNING: Ultrasonic sensor array is not initialized");
            warned = true;
        }
        return;
    }

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
}

// Task wrapper for optical flow sensor reading
void optical_flow_callback(void) {
    int16_t dx = 0, dy = 0;
    if (opticalFlow.read(dx, dy)) {
        // accumulate counts and print total distance in mm
        opticalFlow.addMotionCounts(dx, dy);
        float tx = opticalFlow.getTotalXmm();
        float ty = opticalFlow.getTotalYmm();
        optical_flow_update_count++;  // DIAGNOSTIC
#if OPTICAL_FLOW_SERIAL_TEST
        static uint32_t lastPrintMs = 0;
        const uint32_t now = millis();
        if (now - lastPrintMs >= 200) {
            Serial.printf("X=%.2f mm, Y=%.2f mm\n", tx, ty);
            lastPrintMs = now;
        }
#else
        printfBoth("OpticalFlow totalX: %.2f mm  totalY: %.2f mm\n", tx, ty);
#endif
        #if OPTICAL_FLOW_DEBUG
        Serial.printf("OpticalFlow totalX: %.2f mm  totalY: %.2f mm\n",
                    totalX, totalY);
        #endif
        // Update localisation with optical flow
        // localisation.updateOpticalFlow(opticalFlow);
    }
}

// Task wrapper for LD06 LiDAR reading
void ld06_lidar_callback(void) {
    // Read lidar and, when a full scan is ready, emit Teleplot-format points
    bool scanReady = ld06.readScan();
    if (scanReady) {
        // increment diagnostic: readScan() returned true
        scan_ready_count++;
    }
    if (scanReady) {
        // Feed on-board RoboSLAM then also emit Teleplot for external tools if needed
        roboSlam.processScan(ld06);
        
        // Diagnostic print: number of points available for telemetry

        // Emit a downsampled scan for the visualizer (non-blocking, small)
        // increment diagnostic: about to call telemetry send
        scan_telemetry_call_count++;
        last_scan_point_count = ld06.getNbPointsInScan();
        telemetry_send_downsampled_scan(ld06, mappingNav, 48);

#if TELEMETRY_ALLOW_ASCII_TELEPLOT
        Serial.print("TELEM SCAN POINTS=");
        Serial.println(ld06.getNbPointsInScan());
        // Keep Teleplot for backwards compatibility
        ld06.printScanTeleplot(Serial);
#endif
    }
}

// Temporary LiDAR coordinate test
void lidar_front_test_callback(void) {
    Serial.println("[TARGET TEST CALLBACK]");
    targetDetector.debugPrint();
}

// Task wrapper for localisation sensor fusion update
// void localisation_update_callback(void) {
//     // Perform sensor fusion (prediction from IMU + flow)
//     localisation.updateFromSensors();
    
//     // Optional: print diagnostics periodically (every 1000 ms)
//     static unsigned long last_diag_print = 0;
//     unsigned long now = millis();
//     if (now - last_diag_print >= 1000) {
//         last_diag_print = now;
        
//         // Print fused localisation state
//         RobotPose pose = localisation.getPose();
//         LocalisationDiags diags = localisation.getDiags();
//         #if LOCALISATION_DEBUG
//         printfBoth("LOCALISATION: x=%.1f mm, y=%.1f mm, theta=%.3f rad | "
//                    "flow: dx=%.1f, dy=%.1f | "
//                    "lidar_match=%u, accepted=%u, frames=%lu\n",
//                    pose.x_mm, pose.y_mm, pose.theta_rad,
//                    diags.flow_dx_mm, diags.flow_dy_mm,
//                    diags.lidar_match_score, diags.lidar_correction_accepted,
//                    diags.frame_count);
//         #endif
//     }
// }
// 100 Hz authoritative pose prediction task
void pose_prediction_callback(void) {
    roboSlam.updatePrediction(current_imu_data, millis());
}

// Task wrapper for VL53L1X sensor reading
void vl53l1x_sensor_callback(void) {
    if (!tofSensorArray.isInitialized()) {
        static bool warned = false;

        if (!warned) {
            Serial.println(
                "WARNING: VL53L1X sensor array not initialized, "
                "no S0-S3 data available"
            );
            warned = true;
        }

        return;
    }

    TOFSensorArray::TOFData tofData =
        tofSensorArray.readDistances();

    Serial.print("\n");

    #if TOF_DEBUG
    #if TELEMETRY_ALLOW_ASCII_TELEPLOT
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

        if (i < tofData.sensorCount - 1) {
            Serial.print("\n");
        }
    }
    #endif
    #endif

    Serial.println();

    targetDetector.update();

    Serial.print("TARGET DETECTOR RUNNING | confidence=");
    Serial.println(targetDetector.getTargetConfidence());
}

// Task wrapper for POI detector (processes ToF readings)
void poi_detector_callback(void) {
    if (tofSensorArray.isInitialized()) {
        TOFSensorArray::TOFData tofData = tofSensorArray.readDistances();
        tof_reading_count++;  // DIAGNOSTIC
        
        // Get latest ToF reading (assume single sensor for now)
        float tof_distance_m = (tofData.sensorCount > 0) ? (tofData.distances[0] / 1000.0f) : 0.0f;
        
        // Update POI detector with fused pose and occupancy map
        poiDetector.update(mappingNav, tof_distance_m);
        
        // Update ToF coverage map if reading is valid
        if (tof_distance_m > 0.0f) {
            const MappingNav::Pose2D navPose = mappingNav.getPose();
            float sensor_x_m, sensor_y_m, sensor_yaw_rad;
            ToFGeometry::getSensorWorldFrame(navPose, tof_extrinsics,
                                             sensor_x_m, sensor_y_m, sensor_yaw_rad);
            tofCoverageMap.markBeamCovered(sensor_x_m, sensor_y_m, sensor_yaw_rad, tof_distance_m);
        }
    }
}

// Task wrapper for ToF search planner (generates next search target)
void tof_search_planner_callback(void) {
    // Check if search is complete
    if (searchPlanner.isSearchComplete(tofCoverageMap, mappingNav)) {
        // Search complete; no new targets needed
        return;
    }
    
    // Generate candidate poses from MappingNav, the authoritative pose.
    const uint16_t MAX_CANDIDATES = 30;
    SearchCandidate candidates[MAX_CANDIDATES];
    
    MappingNav::Pose2D current_pose = mappingNav.getPose();
    
    uint16_t candidate_count = searchPlanner.generateCandidates(
        current_pose, tofCoverageMap, mappingNav, candidates, MAX_CANDIDATES);
    
    // Select best candidate
    int16_t best_idx = searchPlanner.selectBestCandidate(candidates, candidate_count);
    if (best_idx < 0) {
        // No useful candidates available
        return;
    }
    
    // Extract best candidate pose
    SearchCandidate &best = candidates[best_idx];
    
    // Issue navigation goal to D* Lite pathfinder
    bool goal_set = mappingNav.setGoalWorld(best.x_m, best.y_m);
    if (!goal_set) {
        // Goal is unreachable or outside arena
        return;
    }
    
    // For now, just print diagnostics
    #if TELEMETRY_ALLOW_ASCII_TELEPLOT
    Serial.printf("POI_SEARCH: target=(%.2f, %.2f, %.2f) coverage=%.1f%% score=%.0f\n",
                  best.x_m, best.y_m, best.theta_rad,
                  best.new_coverage_percent, best.utility_score);
    #endif
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
    // Print tidy IMU headings (relative to startup) and accelerations
    float heading_rel = current_imu_data.euler_h - imu_heading_offset;
    // Normalize heading to [-180,180]
    while (heading_rel > 180.0f) heading_rel -= 360.0f;
    while (heading_rel <= -180.0f) heading_rel += 360.0f;
    // Compute accel-derived roll/pitch for sanity check
    float ax = current_imu_data.accel_x;
    float ay = current_imu_data.accel_y;
    float az = current_imu_data.accel_z;
    // Prevent division by zero
    float denom = sqrtf(ay*ay + az*az);
    float pitch_acc = 0.0f;
    if (denom > 1e-6f) pitch_acc = atan2f(-ax, denom) * 180.0f / PI;
    float roll_acc = 0.0f;
    if (fabsf(az) > 1e-6f) roll_acc = atan2f(ay, az) * 180.0f / PI;
        // Choose roll/pitch source: use accel-derived values when stationary (|g|-9.81 small)
        float g = sqrtf(ax*ax + ay*ay + az*az);
        const float G = 9.80665f;
        const float G_THRESH = 0.5f; // m/s^2
        float roll_out, pitch_out;
        const char *src = "BNO";
        if (fabsf(g - G) <= G_THRESH) {
            // stationary: accelerometer gives reliable roll/pitch
            roll_out = roll_acc;
            pitch_out = pitch_acc;
            src = "ACC";
        } else {
            roll_out = current_imu_data.euler_r;
            pitch_out = current_imu_data.euler_p;
        }
        // Single-line output: Heading | roll/pitch(source) | accel XYZ
        // printfBoth("Heading: %.2f\xC2\xB0 | R: %.2f\xC2\xB0 P: %.2f\xC2\xB0 (%s) | Accel(m/s^2): %.2f, %.2f, %.2f\n",
        //                      heading_rel,
        //                      roll_out,
        //                      pitch_out,
        //                      src,
        //                      ax, ay, az);
}

// Task wrapper for DC motor control
void dc_motor_callback(void) {
    if (!robotStarted) {
        cmd_left_motor = 0;
        cmd_right_motor = 0;
        applied_left_motor = 0;
        applied_right_motor = 0;
        driveMotor.setSpeeds(0, 0);
        return;
    }

    int16_t left_target = cmd_left_motor;
    int16_t right_target = cmd_right_motor;

    // Smoothly approach targets to avoid abrupt jumps and harsh reversals.
    // This ensures transitions like full straight -> full left are gradual.
    applied_left_motor = slew_toward(applied_left_motor, left_target, MOTOR_SLEW_STEP);
    applied_right_motor = slew_toward(applied_right_motor, right_target, MOTOR_SLEW_STEP);

    // Latch behavior: hold last commanded speeds until changed
    driveMotor.setSpeeds(applied_left_motor, applied_right_motor);

    // Print encoder counts only when ASCII diagnostics are enabled. The USB
    // serial port otherwise carries framed binary telemetry.
#if TELEMETRY_ALLOW_ASCII_TELEPLOT
    static unsigned long lastEncoderPrintMs = 0;
    const unsigned long encoderPrintPeriodMs = 200;
    unsigned long now = millis();

    #if DCMOTOR_DEBUG
    if (now - lastEncoderPrintMs >= encoderPrintPeriodMs) {
        Serial.print("ENC L:");
        Serial.print(driveMotor.getLeftEncoderPulses());
        Serial.print(" R:");
        Serial.println(driveMotor.getRightEncoderPulses());
        lastEncoderPrintMs = now;
    }
    #endif 
#endif
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

    if (strcmp(upperLine, "MAP RESET") == 0) {
        mappingNav.reset();
        roboSlam.begin();
        setRobotStartPosition();

        cmd_left_motor = 0;
        cmd_right_motor = 0;
        applied_left_motor = 0;
        applied_right_motor = 0;

        if (ackPort != nullptr) {
            ackPort->println("ACK MAP RESET");
        } else {
            Serial.println("ACK MAP RESET");
        }

        return;
    }

    int left = 0;
    int right = 0;
    printBoth("processing command");
    if (sscanf(upperLine, "MOTOR %d %d", &left, &right) == 2) {
        left = constrain(left, -100, 100);
        right = constrain(right, -100, 100);

        cmd_left_motor = (int16_t)left;
        cmd_right_motor = (int16_t)right;
        cmd_last_rx_ms = millis();
        printBoth("sent command");
        if (ackPort != nullptr) {
            ackPort->print("ACK MOTOR ");
            ackPort->print(cmd_left_motor);
            ackPort->print(" ");
            ackPort->println(cmd_right_motor);
            printBoth("received command");
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


int collect_time = 0;
void big_servo_callback() {
    if (!(proximitySensor.isObjectDetected())){
        if (!pickingUp) {
            pickingUp = true;
        }
        
    }
    if (pickingUp) {
        collect_time++;

        collect_weight(collect_time);
        if (collect_time >= 70) {
            pickingUp = false;
            collect_time = 0;
        }
    }
    printBoth("collect_time: ");
    
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
// Task tColor_sensor(COLOUR_READ_TASK_PERIOD, COLOUR_READ_TASK_NUM_EXECUTE, &color_sensor_callback);
// Task tIR_XY_Position(IR_READ_TASK_PERIOD, IR_READ_TASK_NUM_EXECUTE, &ir_xy_position_callback);
Task tVL53L1X_sensor(VL53L1X_SENSOR_READ_PERIOD, VL53L1X_SENSOR_NUM_EXECUTE, &vl53l1x_sensor_callback);
Task tIR_Distance_sensor(IR_DISTANCE_SENSOR_READ_PERIOD, IR_DISTANCE_SENSOR_NUM_EXECUTE, &ir_distance_sensor_callback);
Task tOpticalFlow(OF_READ_TASK_PERIOD, OF_READ_TASK_NUM_EXECUTE, &optical_flow_callback);
Task tLD06_lidar(LD06_READ_TASK_PERIOD, LD06_READ_TASK_NUM_EXECUTE, &ld06_lidar_callback);
Task tLidarFrontTest(500, -1, &lidar_front_test_callback); // Temporary pls remove
Task tPosePrediction(POSE_PREDICTION_PERIOD, POSE_PREDICTION_NUM_EXECUTE, &pose_prediction_callback);
Task tPOI_Detector(POI_DETECTOR_UPDATE_PERIOD, POI_DETECTOR_UPDATE_NUM_EXECUTE, &poi_detector_callback);
Task tToF_SearchPlanner(TOF_SEARCH_PLANNER_PERIOD, TOF_SEARCH_PLANNER_NUM_EXECUTE, &tof_search_planner_callback);
Task tSensor_average(SENSOR_AVERAGE_PERIOD,      SENSOR_AVERAGE_NUM_EXECUTE,      &sensor_average);
// Herkulex test task removed
Task tBT_stream_test(200, -1, &bt_stream_test_callback);  // Stream test: every 200ms

// Task for DC motor control
Task tDC_motor(DC_MOTOR_CONTROL_PERIOD,          DC_MOTOR_CONTROL_NUM_EXECUTE,    &dc_motor_callback);

// Task to set the motor speeds and direction
Task tSet_motor(SET_MOTOR_TASK_PERIOD,           SET_MOTOR_TASK_NUM_EXECUTE,      &set_motor);

// Tasks to scan for weights and collection upon detection
Task tWeight_scan(WEIGHT_SCAN_TASK_PERIOD,       WEIGHT_SCAN_TASK_NUM_EXECUTE,    &weight_scan);
//Task tCollect_weight(COLLECT_WEIGHT_TASK_PERIOD, COLLECT_WEIGHT_TASK_NUM_EXECUTE, &collect_weight);

// Tasks to search for bases and unload weights
Task tReturn_to_base(RETURN_TO_BASE_TASK_PERIOD, RETURN_TO_BASE_TASK_NUM_EXECUTE, &return_to_base);
Task tDetect_base(DETECT_BASE_TASK_PERIOD,       DETECT_BASE_TASK_NUM_EXECUTE,    &detect_base);
Task tUnload_weights(UNLOAD_WEIGHTS_TASK_PERIOD, UNLOAD_WEIGHTS_TASK_NUM_EXECUTE, &unload_weights);

// Tasks to check the 'watchdog' timer (These will need to be added in)
//Task tCheck_watchdog(CHECK_WATCHDOG_TASK_PERIOD, CHECK_WATCHDOG_TASK_NUM_EXECUTE, &check_watchdog);
//Task tVictory_dance(VICTORY_DANCE_TASK_PERIOD,   VICTORY_DANCE_TASK_NUM_EXECUTE,  &victory_dance);
Task tBig_Servo(BIG_SERVO_PERIOD, BIG_SERVO_NUM_EXECUTE, &big_servo_callback);

Scheduler taskManager;

//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
void robot_init();
void task_init();


// Helper functions to stream output to both Serial and Bluetooth
void printBoth(const char* data) {
    if (!binary_telemetry_active || TELEMETRY_ALLOW_ASCII_TELEPLOT) {
        Serial.print(data);
    }
    if (bluetooth.isInitialized()) {
        bluetooth.print(data);
    }
}

void printlnBoth(const char* data) {
    if (!binary_telemetry_active || TELEMETRY_ALLOW_ASCII_TELEPLOT) {
        Serial.println(data);
    }
    if (bluetooth.isInitialized()) {
        bluetooth.println(data);
    }
}

void printfBoth(const char* format, ...) {
    char buffer[256];
    va_list args;
    
    // Once framed telemetry is active, keep diagnostic text off USB Serial.
    if (!binary_telemetry_active || TELEMETRY_ALLOW_ASCII_TELEPLOT) {
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        Serial.print(buffer);
    }
    
    // Print to Bluetooth
    if (bluetooth.isInitialized()) {
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        bluetooth.print(buffer);
    }
}

void setRobotStartPosition()
{
    if (robotStartSide == START_LEFT)
    {
        mappingNav.setPose(
            START_LEFT_X_M,
            START_LEFT_Y_M,
            START_LEFT_THETA_RAD
        );

    }
    else
    {
        mappingNav.setPose(
            START_RIGHT_X_M,
            START_RIGHT_Y_M,
            START_RIGHT_THETA_RAD
        );

    }
}

void checkStartButton() {
    bool buttonState = digitalRead(START_BUTTON_PIN);

    // Button pressed: HIGH -> LOW
    if (lastStartButtonState == HIGH && buttonState == LOW) {
        unsigned long now = millis();

        if (now - startButtonDebounceTime >= START_BUTTON_DEBOUNCE_MS) {
            startButtonDebounceTime = now;

            if (!robotStarted) {
                // Reset mapping
                mappingNav.reset();
                setRobotStartPosition();

                // Reset RoboSLAM odometry/state
                roboSlam.begin();

                // Start robot
                robotStarted = true;

                // Make absolutely sure motors start stopped
                cmd_left_motor = 0;
                cmd_right_motor = 0;
                applied_left_motor = 0;
                applied_right_motor = 0;

                printlnBoth("START BUTTON PRESSED - MAP/POSE RESET - ROBOT STARTED");
            }
        }
    }

    lastStartButtonState = buttonState;
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
  bigServo_setup();
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
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    // Optional startup test pulse; leave the magnet safely off afterward.
    digitalWrite(MAGNET_PIN, HIGH);
    delay(100);
    digitalWrite(MAGNET_PIN, LOW);
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

    // Herkulex support removed: initialization omitted
    
    // Initialise Herkulex smart servo using a hardware UART (avoids SoftwareSerial)
    // On Teensy/Mega use Serial1 (pins 0/1 on many boards)
    /*
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
    */
    
    printlnBoth("Initialising IR XY Position Sensor...");
    irXYSensor.begin();
    
    printlnBoth("Initialising Optical Flow (PMW3901) on SPI CS D10...");
    if (!opticalFlow.begin()) {

    telemetry_optical_flow_ok = false;

    printlnBoth(
        "WARNING: Optical Flow init failed"
    );

    } else {

    telemetry_optical_flow_ok = true;
    opticalFlow.setLed(true);

    printlnBoth(
        "Optical Flow initialized"
    );
    }

    printlnBoth("Initialising LD06 LiDAR on Serial2...");
    ld06.init();

    ld06.enableCRC();
    ld06.enableFullScan();

    telemetry_lidar_ok = true;

    printlnBoth(
        "LD06 LiDAR initialized "
        "(CRC enabled, full-scan enabled)"
    );

    printlnBoth("Initialising IR Distance Sensor (2Y0A02)...");
    irDistanceSensor.begin();
    
    printlnBoth("Initialising TOF (VL53L1X) Sensor Array...");
    Wire.setClock(400000); // use 400 kHz I2C

    // Initialize VL53L1X sensor array.
    printlnBoth("Initialising VL53L1X Sensor Array...");
    tofSensorArray.setXSHUTPins(VL53L1X_XSHUT_PINS, VL53L1X_SENSOR_COUNT);
    
    // Initialize the TOF sensor array
    if (!tofSensorArray.begin()) {

        telemetry_tof_ok = false;

    printlnBoth("WARNING: Failed to initialize TOF sensor array");

    } else {

        telemetry_tof_ok = true;

    printlnBoth("TOF sensor array initialized successfully");
        tofSensorArray.setDistanceOffset(0, 0);   // Sensor 0 offset
        tofSensorArray.setDistanceOffset(1, -35); // Sensor 1 offset
        tofSensorArray.setDistanceOffset(2, 0);  // Sensor 2 offset
        tofSensorArray.setDistanceOffset(3, -5);  // Sensor 3 offset
    }

        TieredTargetDetectorConfig targetConfig;
        targetConfig.nearIntersectMm = 200;
        targetConfig.farIntersectMm = 400;
        targetConfig.intersectionToleranceMm = 50;
        // Map the prior "top sensor" semantics into the new lidar-based
        // config: treat the old clearance/reject-margin as an expected
        // target height and tolerance, and tighten lidar matching accordingly.
        targetConfig.targetHeightMm = 80;
        targetConfig.targetHeightToleranceMm = 40;
        targetConfig.toleratedWidthMm = 50; // tolerated target lateral width (mm)
        // Lidar matching tolerances (how closely a lidar return must match a TOF
        // hit to be considered the same surface / an obstacle)
        targetConfig.maxDetectionRangeMm = 800;
        targetConfig.lidarMatchToleranceMm = 40;
        targetConfig.lidarBearingToleranceDeg = 1.0f;
        targetConfig.useAngleGate = false;

        // Separate LiDAR obstacle required before ToF target tracking can start.
        targetConfig.lidarObstacleMinMm = 80;
        targetConfig.lidarObstacleMaxMm = 700;
        targetConfig.minimumSeparateLidarPoints = 3;
        targetConfig.separateObstacleDistanceMm = 100;
        targetConfig.lidarClusterRadiusMm = 80;
        targetDetector.setConfig(targetConfig);
    
    printlnBoth("Initialising IMU (BNO055)...");
    BNO055_RETURN_FUNCTION_TYPE init_result = BNO_Init(&bno055);
    
    if (init_result == SUCCESS) {
        telemetry_imu_ok = true;
        printlnBoth("IMU initialised successfully");
        delay(500);
        
        // Set operation mode to NDOF (Nine Degrees of Freedom)
        bno055_set_operation_mode(OPERATION_MODE_NDOF);
        delay(50);  // Wait for mode transition
        
        // Calibrate gyroscope to zero on startup
        calibrate_gyroscope();
        // Read IMU once to capture initial fused heading as offset
        delay(100);
        IMU_Data init_imu = read_imu();
        imu_heading_offset = init_imu.euler_h;
        // Print calibration status (0..3 for each: sys, gyro, accel, mag)
        unsigned char sys_cal=0, gyr_cal=0, acc_cal=0, mag_cal=0;
        bno055_get_syscalib_status(&sys_cal);
        bno055_get_gyrocalib_status(&gyr_cal);
        bno055_get_accelcalib_status(&acc_cal);
        bno055_get_magcalib_status(&mag_cal);
        printfBoth("BNO055 calib status - SYS:%u GYR:%u ACC:%u MAG:%u\n", sys_cal, gyr_cal, acc_cal, mag_cal);
        
        printlnBoth("Initialising DC Motor...");
        driveMotor.begin();
        // Initialize RoboSLAM baseline (read initial encoder/flow/imu state)
        roboSlam.begin();
        setRobotStartPosition();
        
        // Initialize POI detection system
        printlnBoth("Initialising POI Detector...");
        // CALIBRATION REQUIRED: Measure your robot's ToF mounting offset
        tof_extrinsics.x_offset_m = 0.0f;   // TODO: measure forward offset from robot center
        tof_extrinsics.y_offset_m = 0.0f;   // TODO: measure lateral offset
        tof_extrinsics.yaw_offset_rad = 0.0f;  // TODO: measure sensor yaw angle
        poiDetector.begin(tof_extrinsics);
        tofCoverageMap.begin();
        searchPlanner.begin();
        
        // Initialize telemetry over USB Serial unless running a text-only sensor test.
#if !OPTICAL_FLOW_SERIAL_TEST
        telemetry_init(Serial);
        binary_telemetry_active = true;
#endif
        
        printlnBoth("Robot is ready \n");
    } else {

        telemetry_imu_ok = false;

        printfBoth(
            "ERROR: Failed to initialise IMU! Error code: %d\n",
            init_result
        );
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
  taskManager.addTask(tRead_imu);          //reading IMU
    // taskManager.addTask(tProximity_sensor);  //reading proximity sensor
    // taskManager.addTask(tUltrasonic_sensor);  //reading ultrasonic sensor
    // taskManager.addTask(tColor_sensor);       //reading color sensor
    // taskManager.addTask(tIR_XY_Position);     //reading IR XY position sensor
    taskManager.addTask(tVL53L1X_sensor);     //reading VL53L1X sensors
    //taskManager.addTask(tIR_Distance_sensor); //reading IR distance sensor (2Y0A02)  
    // taskManager.addTask(tSensor_average);
    taskManager.addTask(tDC_motor);          //DC motor control
    //taskManager.addTask(tSet_motor); 
  // taskManager.addTask(tWeight_scan);
  // taskManager.addTask(tCollect_weight);
  // taskManager.addTask(tReturn_to_base);
  // taskManager.addTask(tDetect_base);
  // taskManager.addTask(tUnload_weights);

  //taskManager.addTask(tCheck_watchdog);
  //taskManager.addTask(tVictory_dance);      

    // taskManager.addTask(tHerkulexTest);
    //taskManager.addTask(tBT_stream_test);  // Disabled for control reliability
    taskManager.addTask(tOpticalFlow);        //reading optical flow sensor
    taskManager.addTask(tLD06_lidar);          //reading LD06 lidar
    taskManager.addTask(tLidarFrontTest);
    // taskManager.addTask(tLocalisation);        //sensor fusion localisation update
    taskManager.addTask(tPosePrediction);
    // taskManager.addTask(tPOI_Detector);        //POI detection from ToF
    // taskManager.addTask(tToF_SearchPlanner);   //search target generation
    //taskManager.addTask(tBig_Servo);
    //enable the tasks
  tRead_ultrasonic.enable();
  tRead_infrared.enable();
  tRead_colour.enable();
  //tRead_imu.enable();
  //tProximity_sensor.enable();
  //tUltrasonic_sensor.enable();
  //tColor_sensor.enable();
  //tIR_XY_Position.enable();
  tVL53L1X_sensor.enable();
  tIR_Distance_sensor.enable();
  tSensor_average.enable();
  tDC_motor.enable();
  //tSet_motor.enable();
  tWeight_scan.enable();
//   tCollect_weight.enable();
  tReturn_to_base.enable();
  tDetect_base.enable();
  tUnload_weights.enable();
  //tBig_Servo.enable();
//   tCheck_watchdog.enable();
//   tVictory_dance.enable();
    // Herkulex test task removed
   //tHerkulexTest.enable();
    //tBT_stream_test.enable();  // Disabled for control reliability
    tOpticalFlow.enable();
    tLD06_lidar.enable();
    tLidarFrontTest.enable();
    tPosePrediction.enable();
    tPOI_Detector.enable();
    tToF_SearchPlanner.enable();

 printlnBoth("Tasks have been initialised \n");
}


void send_robot_status_telemetry()
{
    uint8_t start_side;

    if (robotStartSide == START_LEFT)
        start_side = TELEMETRY_START_LEFT;
    else
        start_side = TELEMETRY_START_RIGHT;

    bool goal_set =
        mappingNav.isGoalSet();

    uint16_t goal_cell = 0;

    if (goal_set)
    {
        goal_cell =
            mappingNav.getGoalCellIndex();
    }

    telemetry_send_status(

        start_side,

        telemetry_lidar_ok,
        telemetry_imu_ok,
        telemetry_tof_ok,
        telemetry_optical_flow_ok,
        telemetry_ultrasonic_ok,

        cmd_left_motor,
        cmd_right_motor,

        driveMotor.getMeasuredLeftRPM(),
        driveMotor.getMeasuredRightRPM(),

        goal_set,

        goal_cell,

        mappingNav.getPathLength()
    );
}
//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
    
    checkStartButton();
    process_bluetooth_motor_commands();
    process_usb_motor_commands();

    taskManager.execute();

    static unsigned long last_status_ms = 0;
    static unsigned long last_hb_ms = 0;
    static unsigned long last_pose_ms = 0;
    static unsigned long last_grid_ms = 0;
    static unsigned long last_inflated_grid_ms = 0;
    static unsigned long last_replan_ms = 0;
    unsigned long now = millis();

    if (now - last_replan_ms >= 500) {
        const float robot_radius_m =
            sqrtf(0.25f * MappingNav::ROBOT_LENGTH_M * MappingNav::ROBOT_LENGTH_M +
                0.25f * MappingNav::ROBOT_WIDTH_M * MappingNav::ROBOT_WIDTH_M) +
            MappingNav::ROBOT_SAFETY_MARGIN_M;

        mappingNav.replanPath(robot_radius_m);
        last_replan_ms = now;
    }

    if (now - last_status_ms >= 200) {
        send_robot_status_telemetry();
        last_status_ms = now;
    }

    if (now - last_hb_ms >= 1000) {
        telemetry_send_heartbeat(now, 0, 0);
        telemetry_send_diag_scan();

        last_hb_ms = now;
    }

    if (now - last_pose_ms >= 200) {
        telemetry_send_pose_and_path(mappingNav);
        last_pose_ms = now;
    }

    if (now - last_grid_ms >= 1500) {
        telemetry_send_grid_keyframe(mappingNav);
        last_grid_ms = now;
    }
    if (now - last_inflated_grid_ms >= 1500) {
        telemetry_send_inflated_grid(mappingNav);
        last_inflated_grid_ms = now;
    }

}

