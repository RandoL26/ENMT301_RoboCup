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

    line.toUpperCase();

    if (line == "PID SHOW") {
        motorControlPrintPidConfig(serial);
        return;
    }

    float value = 0.0f;   // For floating-point commands (KP, KI, KD, IMAX)

    if (sscanf(line.c_str(), "PID KP %f", &value) == 1) {
        TURN_PID_KP = value;
        serial.print("OK KP="); serial.println(TURN_PID_KP, 4);
        return;
    }

    if (sscanf(line.c_str(), "PID KI %f", &value) == 1) {
        TURN_PID_KI = value;
        serial.print("OK KI="); serial.println(TURN_PID_KI, 4);
        return;
    }

    if (sscanf(line.c_str(), "PID KD %f", &value) == 1) {
        TURN_PID_KD = value;
        serial.print("OK KD="); serial.println(TURN_PID_KD, 4);
        return;
    }

    if (sscanf(line.c_str(), "PID IMAX %f", &value) == 1) {
        TURN_PID_INTEGRAL_LIMIT = value;
        serial.print("OK IMAX="); serial.println(TURN_PID_INTEGRAL_LIMIT, 4);
        return;
    }

    int intValue = 0;     // For integer commands (MINSPD, LOOPMS)
    if (sscanf(line.c_str(), "PID MINSPD %d", &intValue) == 1) {
        TURN_PID_MIN_SPEED = (int16_t)constrain(intValue, 0, 100);
        serial.print("OK MINSPD="); serial.println(TURN_PID_MIN_SPEED);
        return;
    }

    if (sscanf(line.c_str(), "PID LOOPMS %d", &intValue) == 1) {
        TURN_PID_LOOP_DELAY_MS = (uint16_t)constrain(intValue, 1, 1000);
        serial.print("OK LOOPMS="); serial.println(TURN_PID_LOOP_DELAY_MS);
        return;
    }

    serial.println("Unknown command. Use: PID SHOW | PID KP <v> | PID KI <v> | PID KD <v> | PID IMAX <v> | PID MINSPD <0-100> | PID LOOPMS <1-1000>");
}

MotorControl::MotorControl(int leftMotorPin, int rightMotorPin)
                : leftMotor(leftMotorPin),
                    rightMotor(rightMotorPin) {
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
}

void MotorControl::setLeft(int16_t speed) {
    leftMotor.setSpeed(speed);
}

void MotorControl::setRight(int16_t speed) {
    rightMotor.setSpeed(speed);
}

void MotorControl::setSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
}

void MotorControl::turnLeft(void) {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(100);
}

void MotorControl::turnRight(void) {
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(0);
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

        // No direction pin available: use differential forward-only steering.
        if (error > 0.0f) {
            setSpeeds(command, 0);
        } else {
            setSpeeds(0, command);
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