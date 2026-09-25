//************************************
//         sensors.cpp       
//************************************

 // This file contains functions used to read and average
 // the sensors.


#include "imu_sensor.h"
#include "Arduino.h"
#include "BNO055_support.h"
#include <cstring>

// Local definitions
//#define 

// External BNO055 structure (must be defined in main .ino file)
extern struct bno055_t bno055;

// Gyroscope accumulated angle (integrated from angular velocity)
struct {
  float x_angle;  // Accumulated rotation angle in degrees
  float y_angle;
  float z_angle;
} gyro_angle = {0, 0, 0};

// Last timestamp for gyro integration
unsigned long last_gyro_time_ms = 0;
// Accelerometer EMA filter state
static float accel_x_f = 0.0f;
static float accel_y_f = 0.0f;
static float accel_z_f = 0.0f;
// EMA smoothing factor (alpha): lower = smoother/slower
static const float ACCEL_EMA_ALPHA = 0.08f;

// Read ultrasonic value
void read_ultrasonic(/* Parameters */){
  Serial.println("Ultrasonic value \n");
}

// Read infrared value
void read_infrared(/* Parameters */){
  Serial.println("Infrared value \n");  
}

// Read colour sensor value
void read_colour(/* Parameters */){
  Serial.println("colour value \n");  
}

// Pass in data and average the lot
void sensor_average(/* Parameters */){
  Serial.println("Averaging the sensors \n");
}

/*****************************************************************************
 * Description: Read IMU (BNO055) sensor values
 *
 * This function reads acceleration, gyroscope, magnetometer, and Euler angle
 * data from the BNO055 IMU sensor. It returns all values in a single structure.
 *
 * \return IMU_Data structure containing:
 *   - Accelerometer values (m/s²)
 *   - Gyroscope values (rad/s)
 *   - Magnetometer values (μT)
 *   - Euler angles (degrees)
 *   - Temperature (°C)
 *
 ****************************************************************************/
IMU_Data read_imu(void) {
  IMU_Data imu_data;
  memset(&imu_data, 0, sizeof(IMU_Data));
  
  BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
  
  // Read Accelerometer
  struct bno055_accel accel_data;
  memset(&accel_data, 0, sizeof(struct bno055_accel));
  comres = bno055_read_accel_xyz(&accel_data);
  if (comres == SUCCESS) {
    // Raw accel from BNO055 (scaled by driver): convert and filter
    float raw_ax = (float)accel_data.x / 100.0f;
    float raw_ay = (float)accel_data.y / 100.0f;
    float raw_az = (float)accel_data.z / 100.0f;
    // Initialize filters on first read (if zero)
    if (accel_x_f == 0.0f && accel_y_f == 0.0f && accel_z_f == 0.0f) {
      accel_x_f = raw_ax;
      accel_y_f = raw_ay;
      accel_z_f = raw_az;
    } else {
      accel_x_f = ACCEL_EMA_ALPHA * raw_ax + (1.0f - ACCEL_EMA_ALPHA) * accel_x_f;
      accel_y_f = ACCEL_EMA_ALPHA * raw_ay + (1.0f - ACCEL_EMA_ALPHA) * accel_y_f;
      accel_z_f = ACCEL_EMA_ALPHA * raw_az + (1.0f - ACCEL_EMA_ALPHA) * accel_z_f;
    }
    // Store filtered values into returned structure
    imu_data.accel_x = accel_x_f;
    imu_data.accel_y = accel_y_f;
    imu_data.accel_z = accel_z_f;
  } else {
    Serial.print("Accel read failed: ");
    Serial.println(comres);
  }
  
  // Read Gyroscope - integrate angular velocity to get absolute angle
  struct bno055_gyro gyro_data;
  memset(&gyro_data, 0, sizeof(struct bno055_gyro));
  comres = bno055_read_gyro_xyz(&gyro_data);
  if (comres == SUCCESS) {
    // Get current time for delta calculation
    unsigned long current_time_ms = millis();
    
    // Calculate delta time in seconds (only integrate if initialized)
    if (last_gyro_time_ms > 0) {
      float delta_time_s = (float)(current_time_ms - last_gyro_time_ms) / 1000.0f;
      
      // Convert angular velocity to degrees/s and integrate
      float gyro_x_deg_s = ((float)gyro_data.x / 900.0f) * RAD_TO_DEG;
      float gyro_y_deg_s = ((float)gyro_data.y / 900.0f) * RAD_TO_DEG;
      float gyro_z_deg_s = ((float)gyro_data.z / 900.0f) * RAD_TO_DEG;
      
      // Accumulate angle: angle += velocity * time
      gyro_angle.x_angle += gyro_x_deg_s * delta_time_s;
      gyro_angle.y_angle += gyro_y_deg_s * delta_time_s;
      gyro_angle.z_angle += gyro_z_deg_s * delta_time_s;
    }
    
    // Update timestamp for next iteration
    last_gyro_time_ms = current_time_ms;
    
    // Return accumulated angles
    imu_data.gyro_x = gyro_angle.x_angle;
    imu_data.gyro_y = gyro_angle.y_angle;
    imu_data.gyro_z = gyro_angle.z_angle;
  } else {
    Serial.print("Gyro read failed: ");
    Serial.println(comres);
  }
  
  // Read Magnetometer
  struct bno055_mag mag_data;
  memset(&mag_data, 0, sizeof(struct bno055_mag));
  comres = bno055_read_mag_xyz(&mag_data);
  if (comres == SUCCESS) {
    imu_data.mag_x = (float)mag_data.x / 16.0;
    imu_data.mag_y = (float)mag_data.y / 16.0;
    imu_data.mag_z = (float)mag_data.z / 16.0;
  } else {
    Serial.print("Mag read failed: ");
    Serial.println(comres);
  }
  
  // Read Euler angles
  struct bno055_euler euler_data;
  memset(&euler_data, 0, sizeof(struct bno055_euler));
  comres = bno055_read_euler_hrp(&euler_data);
  if (comres == SUCCESS) {
    imu_data.euler_h = (float)euler_data.h / 16.0;
    imu_data.euler_r = (float)euler_data.r / 16.0;
    imu_data.euler_p = (float)euler_data.p / 16.0;
  } else {
    Serial.print("Euler read failed: ");
    Serial.println(comres);
  }
  
  // Read Temperature
  BNO055_S16 temp_data = 0;
  comres = bno055_read_temperature_data(&temp_data);
  if (comres == SUCCESS) {
    imu_data.temp = (float)temp_data;
  } else {
    Serial.print("Temp read failed: ");
    Serial.println(comres);
  }
  
  return imu_data;
}

/*****************************************************************************
 * Description: Reset gyroscope angle tracking to zero
 *
 * This function resets the accumulated gyroscope angles to zero, setting
 * the current orientation as the reference point (0, 0, 0). All future
 * rotations will be measured relative to this point.
 *
 ****************************************************************************/
void calibrate_gyroscope(void) {
  // Reset accumulated angles to zero - current orientation is now reference point
  gyro_angle.x_angle = 0;
  gyro_angle.y_angle = 0;
  gyro_angle.z_angle = 0;
  
  // Initialize timestamp for next reading
  last_gyro_time_ms = millis();
  
  Serial.println("Gyroscope reference point set to zero");
}


