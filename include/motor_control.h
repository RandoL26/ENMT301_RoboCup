//************************************
//         motor_control.h
//************************************

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "dc_motor.h"
#include <stdint.h>

class Stream;

// Global PID turning parameters (runtime tunable).
// Values are defined in motor_control.cpp and can be changed via serial commands.
extern float TURN_PID_KP;
extern float TURN_PID_KI;
extern float TURN_PID_KD;
extern float TURN_PID_INTEGRAL_LIMIT;
extern int16_t TURN_PID_MIN_SPEED;
extern uint16_t TURN_PID_LOOP_DELAY_MS;

class MotorControl {
private:
    DCMotor leftMotor;
    DCMotor rightMotor;

    // Normalize any angle to [0, 360).
    static float normalizeAngle360(float angleDeg);
    // Signed shortest error in [-180, 180].
    static float shortestAngleError(float targetDeg, float currentDeg);

public:
    MotorControl(uint8_t leftMotorPin, uint8_t leftEncA, uint8_t leftEncB,
                 uint8_t rightMotorPin, uint8_t rightEncA, uint8_t rightEncB);

    void begin(void);
    void setLeft(int16_t speed);
    void setRight(int16_t speed);
    // Set track speeds independently in range [-100, 100].
    void setSpeeds(int16_t leftSpeed, int16_t rightSpeed);
    // Full left turn for tracks: left = -100, right = 100
    void turnLeft(void);
    // Full right turn for tracks: left = 100, right = -100
    void turnRight(void);

    // Turn to absolute IMU heading (degrees) using shortest path.
    // Returns true when target is reached within tolerance before timeout.
    bool turnToAngle(float targetAngleDeg,
                     float toleranceDeg = 2.0f,
                     uint16_t timeoutMs = 4000,
                     int16_t turnSpeed = 100,
                     Stream* tuningSerial = nullptr);

    void stop(void);
};

// Runtime PID tuning over serial (line-based commands).
// Example commands:
//   PID SHOW
//   PID KP 1.8
//   PID KI 0.02
//   PID KD 0.10
//   PID IMAX 120
//   PID MINSPD 20
//   PID LOOPMS 10
void motorControlProcessSerialCommand(Stream& serial);
void motorControlPrintPidConfig(Stream& serial);

#endif /* MOTOR_CONTROL_H_ */