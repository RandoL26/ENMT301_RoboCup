//************************************
//         motor_control.h
//************************************

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "dc_motor.h"
#include <stdint.h>

class Stream;
class Print;

// Global PID turning parameters (runtime tunable).
// Values are defined in motor_control.cpp and can be changed via serial commands.
extern float TURN_PID_KP;
extern float TURN_PID_KI;
extern float TURN_PID_KD;
extern float TURN_PID_INTEGRAL_LIMIT;
extern int16_t TURN_PID_MIN_SPEED;
extern uint16_t TURN_PID_LOOP_DELAY_MS;

// Straight-line encoder synchronization parameters (PID control).
extern bool STRAIGHT_SYNC_ENABLED;
extern float STRAIGHT_SYNC_KP;
extern float STRAIGHT_SYNC_KI;
extern float STRAIGHT_SYNC_KD;
extern int16_t STRAIGHT_SYNC_MAX_CORRECTION;
extern int16_t STRAIGHT_SYNC_MIN_SPEED;
extern float STRAIGHT_SYNC_INTEGRAL_MAX;

class MotorControl {
private:
    DCMotor leftMotor;
    DCMotor rightMotor;
    
    // Straight-line PID sync state
    int32_t syncLastLeftEncoderPulses;
    int32_t syncLastRightEncoderPulses;
    unsigned long syncLastMs;
    bool syncInitialized;
    float syncIntegralError;    // Integral accumulator
    float syncLastError;        // Previous error for derivative
    int16_t lastCmdLeftSpeed;   // Track command changes
    int16_t lastCmdRightSpeed;  // Track command changes

    // Adaptive per-command offset table to compensate persistent motor mismatch.
    // Index by magnitude [0..100], stores signed offset to apply to left motor
    // (added to left speed) so that equal commands produce equal encoder rates.
    bool adaptiveCorrectionEnabled;
    int8_t adaptiveOffset[101];
    int8_t adaptiveMaxOffset; // clamp for offsets
    uint8_t adaptiveLearnThreshold; // minimum pulses/sec difference to trigger learning

    // Apply PID-based correction from encoder pulse-rate mismatch.
    void applyStraightSpeedSync(int16_t& leftSpeed, int16_t& rightSpeed);
    
    // Older encoder rate state (deprecated, kept for compatibility)
    int32_t lastLeftEncoderPulses;
    int32_t lastRightEncoderPulses;
    unsigned long lastEncoderSampleMs;
    bool encoderRateInitialized;
    // Per-motor PID control (RPM targets)
    bool motorPidEnabled;
    float targetLeftRPM;
    float targetRightRPM;
    // PID gains (output maps to -100..100 speed range for an RPM error)
    float MOTOR_PID_KP;
    float MOTOR_PID_KI;
    float MOTOR_PID_KD;
    // PID state
    float leftPidIntegral;
    float rightPidIntegral;
    float leftPidPrevError;
    float rightPidPrevError;
    float motorPidIntegralLimit;
    // Encoder ticks per revolution (quadrature x4). Set to your encoder CPR*4.
    float ticksPerRev;
    // Optional Teensy IntervalTimer-based fixed-rate sampler
    bool fixedSamplerEnabled;
    float fixedSamplerDtSec;
    volatile float sampledLeftRate;   // pulses/sec or RPM depending use
    volatile float sampledRightRate;

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
    int32_t getLeftEncoderPulses(void) const;
    int32_t getRightEncoderPulses(void) const;
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

    // Fixed-rate sampling (Teensy IntervalTimer). dtSeconds ~= 0.01f for 100Hz.
    void enableFixedRateSampler(bool enable, float dtSeconds = 0.01f);
    // Motor PID control API
    void enableMotorPid(bool enable);
    void setTargetRPMs(float leftRpm, float rightRpm);
    void setMotorPidGains(float kp, float ki, float kd);
    float getMotorPidKp() const;
    float getMotorPidKi() const;
    float getMotorPidKd() const;
    float getMeasuredLeftRPM() const;
    float getMeasuredRightRPM() const;
    bool isMotorPidEnabled() const;
    float getTargetLeftRPM() const;
    float getTargetRightRPM() const;
    void setTicksPerRev(float ticks);
    float getTicksPerRev() const;

    // Adaptive correction controls
    void enableAdaptiveCorrection(bool enable);
    void resetAdaptiveOffsets(void);
    int8_t getAdaptiveOffset(uint8_t magnitude) const;
    void setAdaptiveMaxOffset(uint8_t maxOff);
    void setAdaptiveLearnThreshold(uint8_t thresh);
    void setAdaptiveOffset(uint8_t magnitude, int8_t offset);
    bool isAdaptiveEnabled() const;
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
//   SYNC SHOW
//   SYNC KP 0.15
//   SYNC KI 0.05
//   SYNC KD 0.10
//   SYNC MAX 20
//   SYNC MIN 20
void motorControlProcessSerialCommand(Stream& serial);
void motorControlPrintPidConfig(Stream& serial);
bool motorControlHandleCommandLine(const char* line, Print* out = nullptr);

#endif /* MOTOR_CONTROL_H_ */