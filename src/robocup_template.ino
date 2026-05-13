
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
//#include <Herkulex.h>             //smart servo
#include <Adafruit_TCS34725.h>      //colour sensor
#include <Wire.h>                   //for I2C and SPI
#include <TaskScheduler.h>          //scheduler
#include <VL53L1X.h>                //VL53L1X distance sensor
#include "BNO055_support.h"         //IMU sensor 

// Custom headers
#include "motors.h"
#include "imu_sensor.h"
#include "weight_collection.h"
#include "return_to_base.h"
#include "proximity_sensor.h"       //inductive proximity sensor
#include "ultrasonic_sensor.h"      //ultrasonic distance sensor
#include "color_sensor.h"           //colour sensor module
#include "ir_xy_position.h"         //IR XY position sensor
#include "tof_sensor_array.h"       //TOF (VL53L1X) sensor array 

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

// Pin deffinitions
#define IO_POWER  49
#define PROXIMITY_SENSOR_PIN  20  // A6Z
#define ULTRASONIC_TRIGGER_PIN  3   // D2Z
#define ULTRASONIC_ECHO_PIN  2      // D3Z


// VL53L1X sensor configuration
const uint8_t VL53L1X_SENSOR_COUNT = 1;  // Update this if you add more sensors
const uint8_t VL53L1X_XSHUT_PINS[VL53L1X_SENSOR_COUNT] = { 18 };  // Update this with the XSHUT pins for each sensor

// Serial deffinitions
#define BAUD_RATE 115200

Servo right_motor;
Servo left_motor;

// BNO055 IMU structure
struct bno055_t bno055;

// Global IMU data storage (for inter-task communication)
IMU_Data current_imu_data;

// Proximity Sensor instance
ProximitySensor proximitySensor(PROXIMITY_SENSOR_PIN);

// Ultrasonic Sensor instance
UltrasonicSensor ultrasonicSensor(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);

// Color Sensor instance
ColorSensor colorSensor;

// IR XY Position Sensor instance
IRXYPosition irXYSensor;

// TOF Sensor Array instance
TOFSensorArray tofSensorArray(VL53L1X_SENSOR_COUNT);

// Task wrapper for proximity sensor reading
void proximity_sensor_callback(void) {
    proximitySensor.update();
    proximitySensor.printStatus();
}

// Task wrapper for ultrasonic sensor reading
void ultrasonic_sensor_callback(void) {
    ultrasonicSensor.update();
    ultrasonicSensor.printStatus();
}

// Task wrapper for color sensor reading
void color_sensor_callback(void) {
    if (colorSensor.isInitialized()) {
        ColorSensor::ColorData colorData = colorSensor.readColor();
        colorSensor.printColorData(colorData);
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

// Task wrapper for IMU reading
void imu_task_callback(void) {
    current_imu_data = read_imu();
    // Print IMU data to serial
    Serial.print("Raw Gyro X:");
    Serial.print(current_imu_data.gyro_x);
    Serial.print(" Y:");
    Serial.print(current_imu_data.gyro_y);
    Serial.print(" Z:");
    Serial.print(current_imu_data.gyro_z);
    Serial.print(" | Accel: ");
    Serial.print(current_imu_data.accel_x);
    Serial.print(", ");
    Serial.print(current_imu_data.accel_y);
    Serial.print(", ");
    Serial.println(current_imu_data.accel_z);
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
Task tSensor_average(SENSOR_AVERAGE_PERIOD,      SENSOR_AVERAGE_NUM_EXECUTE,      &sensor_average);

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

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  delay(2000);  // Give USB serial time to stabilize
  Serial.println("\n\n=== RoboCup Robot Starting ===");
  Wire.begin();        // MUST be called FIRST - before any I2C operations
  pin_init();
  robot_init();        // robot_init() calls BNO_Init() which needs I2C
  task_init();
}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work) 
// Set as high or low
//**********************************************************************************
void pin_init(){
    
    Serial.println("Pins have been initialised \n"); 

    pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
    digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
}

//**********************************************************************************
// Set default robot state
//**********************************************************************************
void robot_init() {
    Serial.println("Initialising Proximity Sensor...");
    proximitySensor.begin();
    
    Serial.println("Initialising Ultrasonic Sensor...");
    ultrasonicSensor.begin();
    
    Serial.println("Initialising Color Sensor...");
    colorSensor.begin();
    
    Serial.println("Initialising IR XY Position Sensor...");
    irXYSensor.begin();
    
    Serial.println("Initialising TOF (VL53L1X) Sensor Array...");
    Wire.setClock(400000); // use 400 kHz I2C
    
    // Set XSHUT pins for each sensor
    tofSensorArray.setXSHUTPins(VL53L1X_XSHUT_PINS, VL53L1X_SENSOR_COUNT);
    
    // Initialize the TOF sensor array
    if (!tofSensorArray.begin()) {
        Serial.println("WARNING: Failed to initialize TOF sensor array");
    } else {
        Serial.println("TOF sensor array initialized successfully");
    }
    
    Serial.println("Initialising IMU (BNO055)...");
    BNO055_RETURN_FUNCTION_TYPE init_result = BNO_Init(&bno055);
    
    if (init_result == SUCCESS) {
        Serial.println("IMU initialised successfully");
        delay(500);
        
        // Set operation mode to NDOF (Nine Degrees of Freedom)
        bno055_set_operation_mode(OPERATION_MODE_NDOF);
        delay(50);  // Wait for mode transition
        
        // Calibrate gyroscope to zero on startup
        calibrate_gyroscope();
        
        Serial.println("Robot is ready \n");
    } else {
        Serial.print("ERROR: Failed to initialise IMU! Error code: ");
        Serial.println(init_result);
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
    taskManager.addTask(tColor_sensor);       //reading color sensor
    taskManager.addTask(tIR_XY_Position);     //reading IR XY position sensor
    taskManager.addTask(tVL53L1X_sensor);     //reading VL53L1X sensors  
    taskManager.addTask(tSensor_average);
  // taskManager.addTask(tSet_motor); 
  // taskManager.addTask(tWeight_scan);
  // taskManager.addTask(tCollect_weight);
  // taskManager.addTask(tReturn_to_base);
  // taskManager.addTask(tDetect_base);
  // taskManager.addTask(tUnload_weights);

  //taskManager.addTask(tCheck_watchdog);
  //taskManager.addTask(tVictory_dance);      

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
  tSensor_average.enable();
  tSet_motor.enable();
  tWeight_scan.enable();
  tCollect_weight.enable();
  tReturn_to_base.enable();
  tDetect_base.enable();
  tUnload_weights.enable();
 //tCheck_watchdog.enable();
 //tVictory_dance.enable();

 Serial.println("Tasks have been initialised \n");
}



//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
  
  taskManager.execute();    //execute the scheduler
  //Serial.println("Another scheduler execution cycle has oocured \n");
}
