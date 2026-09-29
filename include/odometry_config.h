#ifndef ODOMETRY_CONFIG_H_
#define ODOMETRY_CONFIG_H_

#include <stdint.h>

// Hardware calibration shared by the main RoboSLAM and seek_weight paths.
// Zero counts/metre disables encoder translation; zero track width disables
// encoder-derived heading, which remains supplied by the IMU.
namespace OdometryConfig {
static constexpr float ENCODER_COUNTS_PER_M = 11000.0f;
static constexpr float ENCODER_TRACK_M = 0.0f;       // MEASURE ME
static constexpr int8_t LEFT_ENCODER_SIGN = +1;      // verify positive on forward roll
static constexpr int8_t RIGHT_ENCODER_SIGN = +1;     // verify positive on forward roll
static constexpr int8_t IMU_YAW_SIGN = +1;            // verify CCW gyro sign on robot
}

#endif
