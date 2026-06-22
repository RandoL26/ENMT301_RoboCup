
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
#include <stdarg.h>                 //for va_list in printf functions
#include <stdio.h>                  //for vsnprintf
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
#include "dc_motor.h"               //DC motor control 
#include "ch9143_bluetooth.h"       //CH9143 Bluetooth module

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
#define ULTRASONIC_TRIGGER_PIN_1  3   // D2Z - Sensor 1 Trigger
#define ULTRASONIC_ECHO_PIN_1     2   // D3Z - Sensor 1 Echo
#define ULTRASONIC_TRIGGER_PIN_2  5   // Sensor 2 Trigger (adjust as needed)
#define ULTRASONIC_ECHO_PIN_2     4   // Sensor 2 Echo (adjust as needed)
#define IR_DISTANCE_SENSOR_PIN    A9  // Analog pin for 2Y0A02 IR distance sensor


// VL53L1X sensor configuration
const uint8_t VL53L1X_SENSOR_COUNT = 1;  // Update this if you add more sensors
const uint8_t VL53L1X_XSHUT_PINS[VL53L1X_SENSOR_COUNT] = { 18 };  // Update this with the XSHUT pins for each sensor

// DC Motor PIN definitions
#define DC_M1_PIN 0              //PWM pin for DC motor control (can be extended to 2 motors)
#define DC_M2_PIN 1              //PWM pin for DC motor control (can be extended to 2 motors)
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
#define BLUETOOTH_BAUD 9600
#define BLUETOOTH_RX_PIN 0  // Serial3 RX (try this)
#define BLUETOOTH_TX_PIN 1  // Serial3 TX (try this)

Servo right_motor;
Servo left_motor;

// DC Motor instance (Channel 1)
DCMotor dcMotor(DC_M1_PIN);
DCMotor dcMotor2(DC_M2_PIN);  // Uncomment if using a second motor

// BNO055 IMU structure
struct bno055_t bno055;

// Global IMU data storage (for inter-task communication)
IMU_Data current_imu_data;

// Proximity Sensor instance
ProximitySensor proximitySensor(PROXIMITY_SENSOR_PIN);

// Ultrasonic Sensor instance (kept for backwards compatibility)
UltrasonicSensor ultrasonicSensor(ULTRASONIC_TRIGGER_PIN_1, ULTRASONIC_ECHO_PIN_1);

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

// CH9143 Bluetooth instance
CH9143Bluetooth bluetooth(&Serial3, BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN, BLUETOOTH_BAUD);

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

// Task wrapper for VL53L1X sensor reading
void vl53l1x_sensor_callback(void) {
    if (tofSensorArray.isInitialized()) {
        TOFSensorArray::TOFData tofData = tofSensorArray.readDistances();
        tofSensorArray.printDistances(tofData);
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
    // Example: Set motor to 50% forward speed
    dcMotor.setSpeed(-(MOTOR_SPEED));  // Uncomment to test
    dcMotor.printStatus();
    dcMotor2.setSpeed(MOTOR_SPEED); // Uncomment if using a second motor
    dcMotor2.printStatus(); // Uncomment if using a second motor
}

// Herkulex continuous test callback: toggles between -100 and 100 degrees
void herkulex_test_callback() {
    static bool toggle = false;
    int angle = toggle ? 100 : -100;
    int led = toggle ? LED_GREEN : LED_BLUE;
    Herkulex.torqueON(HERKULEX_ID);
    Herkulex.moveOneAngle(HERKULEX_ID, angle, 1000, led);
    Herkulex.moveOneAngle(HERKULEX_ID, -100, 1000, LED_BLUE);
    printfBoth("Herkulex test move to %d\n", Herkulex.getPosition(HERKULEX_ID));
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
Task tIR_Distance_sensor(IR_DISTANCE_SENSOR_READ_PERIOD, IR_DISTANCE_SENSOR_NUM_EXECUTE, &ir_distance_sensor_callback);
Task tSensor_average(SENSOR_AVERAGE_PERIOD,      SENSOR_AVERAGE_NUM_EXECUTE,      &sensor_average);
Task tHerkulexTest(HERKULEX_TEST_PERIOD, -1, &herkulex_test_callback);

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
  
  // Test Serial3 directly before initializing Bluetooth class
  Serial3.begin(BLUETOOTH_BAUD);
  delay(100);
  Serial.println("Testing Serial3...");
  Serial3.println("DIRECT_TEST: Serial3 Working");
  Serial3.flush();
  
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
    
    printlnBoth("Initialising Ultrasonic Sensor Array...");
    ultrasonicArray.addSensor(0, ULTRASONIC_TRIGGER_PIN_1, ULTRASONIC_ECHO_PIN_1);
    ultrasonicArray.addSensor(1, ULTRASONIC_TRIGGER_PIN_2, ULTRASONIC_ECHO_PIN_2);
    ultrasonicArray.begin();
    
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
        dcMotor.begin();
        dcMotor2.begin();  // Uncomment if using a second motor
        
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
    // taskManager.addTask(tVL53L1X_sensor);     //reading VL53L1X sensors
    //taskManager.addTask(tIR_Distance_sensor); //reading IR distance sensor (2Y0A02)  
    // taskManager.addTask(tSensor_average);
    //taskManager.addTask(tDC_motor);          + //DC motor control
  // taskManager.addTask(tSet_motor); 
  // taskManager.addTask(tWeight_scan);
  // taskManager.addTask(tCollect_weight);
  // taskManager.addTask(tReturn_to_base);
  // taskManager.addTask(tDetect_base);
  // taskManager.addTask(tUnload_weights);

  //taskManager.addTask(tCheck_watchdog);
  //taskManager.addTask(tVictory_dance);      

    taskManager.addTask(tHerkulexTest);

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

 printlnBoth("Tasks have been initialised \n");
}



//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
  
  taskManager.execute();    //execute the scheduler
  //Serial.println("Another scheduler execution cycle has oocured \n");
}

