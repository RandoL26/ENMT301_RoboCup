#include "motor_control.h"
#include "imu_sensor.h"
#include "Arduino.h"
#include <math.h>
#include <string.h>

// PID tuning globals (can be adjusted at runtime through serial commands).
float TURN_PID_KP = 1.8f;
float TURN_PID_KI = 0.0f;
float TURN_PID_KD = 0.10f;
float TURN_PID_INTEGRAL_LIMIT = 120.0f;
int16_t TURN_PID_MIN_SPEED = 20;
uint16_t TURN_PID_LOOP_DELAY_MS = 10;

// Straight-line encoder synchronization PID tuning (runtime adjustable).
bool STRAIGHT_SYNC_ENABLED = true;
float STRAIGHT_SYNC_KP = 0.15f;
float STRAIGHT_SYNC_KI = 0.05f;
float STRAIGHT_SYNC_KD = 0.10f;
int16_t STRAIGHT_SYNC_MAX_CORRECTION = 20;
int16_t STRAIGHT_SYNC_MIN_SPEED = 20;
float STRAIGHT_SYNC_INTEGRAL_MAX = 50.0f;

// Print the current runtime tuning values.
void motorControlPrintPidConfig(Stream& serial) {
    serial.println("PID CONFIG:");
    serial.print("KP="); serial.println(TURN_PID_KP, 4);
    serial.print("KI="); serial.println(TURN_PID_KI, 4);
    serial.print("KD="); serial.println(TURN_PID_KD, 4);
    serial.print("IMAX="); serial.println(TURN_PID_INTEGRAL_LIMIT, 4);
    serial.print("MINSPD="); serial.println(TURN_PID_MIN_SPEED);
    serial.print("LOOPMS="); serial.println(TURN_PID_LOOP_DELAY_MS);
}

bool motorControlHandleCommandLine(const char* inputLine, Print* out) {
    if (inputLine == nullptr) {
        return false;
    }

    String line = inputLine;
    line.trim();
    if (line.length() == 0) {
        return false;
    }

    line.toUpperCase();

    auto printUnknown = [&]() {
        if (out != nullptr) {
            out->println("Unknown command. Use: PID SHOW | PID KP <v> | PID KI <v> | PID KD <v> | PID IMAX <v> | PID MINSPD <0-100> | PID LOOPMS <1-1000> | SYNC SHOW | SYNC ON | SYNC OFF | SYNC KP <v> | SYNC KI <v> | SYNC KD <v> | SYNC MAX <1-100> | SYNC MIN <0-100>");
        }
    };

    if (line == "PID SHOW") {
        if (out != nullptr) {
            out->println("PID CONFIG:");
            out->print("KP="); out->println(TURN_PID_KP, 4);
            out->print("KI="); out->println(TURN_PID_KI, 4);
            out->print("KD="); out->println(TURN_PID_KD, 4);
            out->print("IMAX="); out->println(TURN_PID_INTEGRAL_LIMIT, 4);
            out->print("MINSPD="); out->println(TURN_PID_MIN_SPEED);
            out->print("LOOPMS="); out->println(TURN_PID_LOOP_DELAY_MS);
        }
        return true;
    }

    float value = 0.0f;
    int intValue = 0;

    if (sscanf(line.c_str(), "PID KP %f", &value) == 1) {
        TURN_PID_KP = value;
        if (out != nullptr) {
            out->print("OK KP="); out->println(TURN_PID_KP, 4);
        }
        return true;
    }

    if (sscanf(line.c_str(), "PID KI %f", &value) == 1) {
        TURN_PID_KI = value;
        if (out != nullptr) {
            out->print("OK KI="); out->println(TURN_PID_KI, 4);
        }
        return true;
    }

    if (sscanf(line.c_str(), "PID KD %f", &value) == 1) {
        TURN_PID_KD = value;
        if (out != nullptr) {
            out->print("OK KD="); out->println(TURN_PID_KD, 4);
        }
        return true;
    }

    if (sscanf(line.c_str(), "PID IMAX %f", &value) == 1) {
        TURN_PID_INTEGRAL_LIMIT = value;
        if (out != nullptr) {
            out->print("OK IMAX="); out->println(TURN_PID_INTEGRAL_LIMIT, 4);
        }
        return true;
    }

    if (sscanf(line.c_str(), "PID MINSPD %d", &intValue) == 1) {
        TURN_PID_MIN_SPEED = (int16_t)constrain(intValue, 0, 100);
        if (out != nullptr) {
            out->print("OK MINSPD="); out->println(TURN_PID_MIN_SPEED);
        }
        return true;
    }

    if (sscanf(line.c_str(), "PID LOOPMS %d", &intValue) == 1) {
        TURN_PID_LOOP_DELAY_MS = (uint16_t)constrain(intValue, 1, 1000);
        if (out != nullptr) {
            out->print("OK LOOPMS="); out->println(TURN_PID_LOOP_DELAY_MS);
        }
        return true;
    }

    if (line == "SYNC SHOW") {
        if (out != nullptr) {
            out->println("SYNC CONFIG:");
            out->print("ENABLED="); out->println(STRAIGHT_SYNC_ENABLED ? 1 : 0);
            out->print("KP="); out->println(STRAIGHT_SYNC_KP, 4);
            out->print("KI="); out->println(STRAIGHT_SYNC_KI, 4);
            out->print("KD="); out->println(STRAIGHT_SYNC_KD, 4);
            out->print("MAX="); out->println(STRAIGHT_SYNC_MAX_CORRECTION);
            out->print("MIN="); out->println(STRAIGHT_SYNC_MIN_SPEED);
        }
        return true;
    }

    if (line == "SYNC ON") {
        STRAIGHT_SYNC_ENABLED = true;
        if (out != nullptr) {
            out->println("OK SYNC ON");
        }
        return true;
    }

    if (line == "SYNC OFF") {
        STRAIGHT_SYNC_ENABLED = false;
        if (out != nullptr) {
            out->println("OK SYNC OFF");
        }
        return true;
    }

    if (sscanf(line.c_str(), "SYNC KP %f", &value) == 1) {
        STRAIGHT_SYNC_KP = value;
        if (out != nullptr) {
            out->print("OK SYNC KP="); out->println(STRAIGHT_SYNC_KP, 4);
        }
        return true;
    }

    if (sscanf(line.c_str(), "SYNC KI %f", &value) == 1) {
        STRAIGHT_SYNC_KI = value;
        if (out != nullptr) {
            out->print("OK SYNC KI="); out->println(STRAIGHT_SYNC_KI, 4);
        }
        return true;
    }

    if (sscanf(line.c_str(), "SYNC KD %f", &value) == 1) {
        STRAIGHT_SYNC_KD = value;
        if (out != nullptr) {
            out->print("OK SYNC KD="); out->println(STRAIGHT_SYNC_KD, 4);
        }
        return true;
    }

    if (sscanf(line.c_str(), "SYNC MAX %d", &intValue) == 1) {
        STRAIGHT_SYNC_MAX_CORRECTION = (int16_t)constrain(intValue, 1, 100);
        if (out != nullptr) {
            out->print("OK SYNC MAX="); out->println(STRAIGHT_SYNC_MAX_CORRECTION);
        }
        return true;
    }

    if (sscanf(line.c_str(), "SYNC MIN %d", &intValue) == 1) {
        STRAIGHT_SYNC_MIN_SPEED = (int16_t)constrain(intValue, 0, 100);
        if (out != nullptr) {
            out->print("OK SYNC MIN="); out->println(STRAIGHT_SYNC_MIN_SPEED);
        }
        return true;
    }

    printUnknown();
    return false;
}

void motorControlProcessSerialCommand(Stream& serial) {
    // Non-blocking: return immediately when no complete command is waiting.
    if (serial.available() <= 0) {
        return;
    }

    String line = serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
        return;
    }
    motorControlHandleCommandLine(line.c_str(), &serial);
}

MotorControl::MotorControl(uint8_t leftMotorPin, uint8_t leftEncA, uint8_t leftEncB,
                           uint8_t rightMotorPin, uint8_t rightEncA, uint8_t rightEncB)
    : leftMotor(leftMotorPin, leftEncA, leftEncB),
      rightMotor(rightMotorPin, rightEncA, rightEncB),
      syncLastLeftEncoderPulses(0),
      syncLastRightEncoderPulses(0),
      syncLastMs(0),
      syncInitialized(false),
      syncIntegralError(0.0f),
      syncLastError(0.0f),
      lastCmdLeftSpeed(0),
      lastCmdRightSpeed(0),
      lastLeftEncoderPulses(0),
      lastRightEncoderPulses(0),
      lastEncoderSampleMs(0),
      encoderRateInitialized(false) {
}

float MotorControl::normalizeAngle360(float angleDeg) {
    while (angleDeg < 0.0f) {
        angleDeg += 360.0f;
    }
    while (angleDeg >= 360.0f) {
        angleDeg -= 360.0f;
    }
    return angleDeg;
}

float MotorControl::shortestAngleError(float targetDeg, float currentDeg) {
    // Compute wrapped difference then fold to the shortest signed path.
    float error = normalizeAngle360(targetDeg) - normalizeAngle360(currentDeg);

    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }

    return error;
}

void MotorControl::begin(void) {
    leftMotor.begin();
    rightMotor.begin();
    lastLeftEncoderPulses = leftMotor.getEncoderPulses();
    lastRightEncoderPulses = rightMotor.getEncoderPulses();
    lastEncoderSampleMs = millis();
    encoderRateInitialized = true;
}

void MotorControl::setLeft(int16_t speed) {
    leftMotor.setSpeed(speed);
}

void MotorControl::setRight(int16_t speed) {
    rightMotor.setSpeed(speed);
}

void MotorControl::setSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    // Detect command change and reset sync state to avoid carryover error
    if (leftSpeed != lastCmdLeftSpeed || rightSpeed != lastCmdRightSpeed) {
        syncInitialized = false;
        syncIntegralError = 0.0f;
        syncLastError = 0.0f;
        lastCmdLeftSpeed = leftSpeed;
        lastCmdRightSpeed = rightSpeed;
    }
    
    applyStraightSpeedSync(leftSpeed, rightSpeed);
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
}

int32_t MotorControl::getLeftEncoderPulses(void) const {
    return leftMotor.getEncoderPulses();
}

int32_t MotorControl::getRightEncoderPulses(void) const {
    return rightMotor.getEncoderPulses();
}

void MotorControl::turnLeft(void) {
    leftMotor.setSpeed(-100);
    rightMotor.setSpeed(100);
}

void MotorControl::turnRight(void) {
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(-100);
}

bool MotorControl::turnToAngle(float targetAngleDeg,
                               float toleranceDeg,
                               uint16_t timeoutMs,
                               int16_t turnSpeed,
                               Stream* tuningSerial) {
    const unsigned long startMs = millis();
    const int16_t maxSpeed = (int16_t)constrain((int)abs(turnSpeed), 0, 100);
    const float target = normalizeAngle360(targetAngleDeg);
    float integral = 0.0f;
    float prevError = 0.0f;
    unsigned long prevMs = millis();

    if (maxSpeed <= 0) {
        stop();
        return false;
    }

    while ((millis() - startMs) < timeoutMs) {
        // Optional live tuning during a blocking turn.
        if (tuningSerial != nullptr) {
            motorControlProcessSerialCommand(*tuningSerial);
        }

        unsigned long nowMs = millis();
        float dt = (float)(nowMs - prevMs) / 1000.0f;
        if (dt <= 0.0f) {
            // Guard against divide-by-zero in derivative calculation.
            dt = 0.001f;
        }

        IMU_Data imuData = read_imu();
        const float current = normalizeAngle360(imuData.euler_h);
        float error = shortestAngleError(target, current);

        if (fabs(error) <= toleranceDeg) {
            stop();
            return true;
        }

        // Reduce windup when crossing the target direction.
        if ((prevError > 0.0f && error < 0.0f) || (prevError < 0.0f && error > 0.0f)) {
            integral = 0.0f;
        }

        // Integrator with anti-windup clamp.
        integral += error * dt;
        if (integral > TURN_PID_INTEGRAL_LIMIT) {
            integral = TURN_PID_INTEGRAL_LIMIT;
        } else if (integral < -TURN_PID_INTEGRAL_LIMIT) {
            integral = -TURN_PID_INTEGRAL_LIMIT;
        }

        const float derivative = (error - prevError) / dt;
        const float output = (TURN_PID_KP * error) +
                             (TURN_PID_KI * integral) +
                             (TURN_PID_KD * derivative);

        // Convert PID output magnitude to a symmetric turn command.
        int16_t command = (int16_t)fabs(output);
        command = (int16_t)constrain((int)command, 0, (int)maxSpeed);

        // Ensure enough torque to overcome stiction at low output.
        if (command > 0 && command < TURN_PID_MIN_SPEED) {
            command = TURN_PID_MIN_SPEED;
            command = (int16_t)constrain((int)command, 0, (int)maxSpeed);
        }

        // BNO055 heading increases clockwise.
        // Positive error => shortest turn is clockwise (right turn).
        if (error > 0.0f) {
            setSpeeds(command, -command);
        } else {
            setSpeeds(-command, command);
        }

        prevError = error;
        prevMs = nowMs;

        delay(TURN_PID_LOOP_DELAY_MS);
    }

    stop();
    return false;
}

void MotorControl::stop(void) {
    leftMotor.stop();
    rightMotor.stop();
}

void MotorControl::applyStraightSpeedSync(int16_t& leftSpeed, int16_t& rightSpeed) {
    if (!STRAIGHT_SYNC_ENABLED) {
        return;
    }

    const bool sameDirection = ((leftSpeed > 0 && rightSpeed > 0) || (leftSpeed < 0 && rightSpeed < 0));
    const bool sameMagnitude = (abs(leftSpeed) == abs(rightSpeed));
    const bool aboveMinSpeed = (abs(leftSpeed) >= STRAIGHT_SYNC_MIN_SPEED);

    if (!(sameDirection && sameMagnitude && aboveMinSpeed)) {
        // CRITICAL: Reset integral during idle to prevent windup
        // This prevents accumulated error from fighting motor during next acceleration
        syncInitialized = false;
        syncIntegralError = 0.0f;
        syncLastError = 0.0f;
        syncLastLeftEncoderPulses = leftMotor.getEncoderPulses();
        syncLastRightEncoderPulses = rightMotor.getEncoderPulses();
        syncLastMs = millis();
        return;
    }

    unsigned long now = millis();
    int32_t leftNow = leftMotor.getEncoderPulses();
    int32_t rightNow = rightMotor.getEncoderPulses();

    if (!syncInitialized) {
        syncLastLeftEncoderPulses = leftNow;
        syncLastRightEncoderPulses = rightNow;
        syncLastMs = now;
        syncInitialized = true;
        return;
    }

    unsigned long dtMs = now - syncLastMs;
    if (dtMs == 0) {
        return;
    }

    int32_t dLeft = leftNow - syncLastLeftEncoderPulses;
    int32_t dRight = rightNow - syncLastRightEncoderPulses;

    syncLastLeftEncoderPulses = leftNow;
    syncLastRightEncoderPulses = rightNow;
    syncLastMs = now;

    // Calculate mismatch (error) in pulses/sec
    float leftRate = ((float)abs(dLeft) * 1000.0f) / (float)dtMs;
    float rightRate = ((float)abs(dRight) * 1000.0f) / (float)dtMs;
    float error = leftRate - rightRate;

    // PID calculation
    float dt = (float)dtMs / 1000.0f;

    // Proportional term
    float pTerm = STRAIGHT_SYNC_KP * error;

    // Integral term with anti-windup
    syncIntegralError += error * dt;
    if (syncIntegralError > STRAIGHT_SYNC_INTEGRAL_MAX) {
        syncIntegralError = STRAIGHT_SYNC_INTEGRAL_MAX;
    } else if (syncIntegralError < -STRAIGHT_SYNC_INTEGRAL_MAX) {
        syncIntegralError = -STRAIGHT_SYNC_INTEGRAL_MAX;
    }
    float iTerm = STRAIGHT_SYNC_KI * syncIntegralError;

    // Derivative term
    float dTerm = STRAIGHT_SYNC_KD * (error - syncLastError) / dt;
    syncLastError = error;

    // PID output
    float pidOutput = pTerm + iTerm + dTerm;
    int16_t correction = (int16_t)roundf(pidOutput);
    correction = (int16_t)constrain((int)correction,
                                    -(int)STRAIGHT_SYNC_MAX_CORRECTION,
                                    (int)STRAIGHT_SYNC_MAX_CORRECTION);

    int16_t l = (int16_t)(leftSpeed - correction);
    int16_t r = (int16_t)(rightSpeed + correction);

    leftSpeed = (int16_t)constrain((int)l, -100, 100);
    rightSpeed = (int16_t)constrain((int)r, -100, 100);
}