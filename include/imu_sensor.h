//************************************
//         sensors.h     
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

// IMU data structure
struct IMU_Data {
  float accel_x, accel_y, accel_z;    // Acceleration (m/s²)
  float gyro_x, gyro_y, gyro_z;       // Angular velocity (rad/s)
  float mag_x, mag_y, mag_z;          // Magnetic field (μT)
  float euler_h, euler_r, euler_p;    // Euler angles: heading, roll, pitch (degrees)
  float temp;                         // Temperature (°C)
  bool gyro_valid;                    // True only when the most recent gyro read succeeded
};

// Read ultrasonic value
void read_ultrasonic(/* Parameters */);

// Read infrared value
void read_infrared(/* Parameters */);

void read_colour(/* Parameters */);

// Pass in data and average the lot
void sensor_average(/* Parameters */);

// Read IMU (BNO055) values
// Returns: IMU_Data structure containing all sensor values
IMU_Data read_imu(void);

// Calibrate gyroscope to zero on startup
void calibrate_gyroscope(void);

#endif /* SENSORS_H_ */
