#include "motor_control.h"
#include "imu_sensor.h"
#include "Arduino.h"
#include "optical_flow.h"
#include <math.h>
#include <string.h>
#include <IntervalTimer.h>

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

// Pointer to the active MotorControl instance (if created). Used by serial
// command handlers to control adaptive settings for the active controller.
static MotorControl* gMotorControlInstance = nullptr;

// Optical flow sensor instance is defined in the main sketch (robocup_template.ino)
extern OpticalFlow opticalFlow;

static IntervalTimer gControlTimer;

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
            out->println("Unknown command. Use: PID, SYNC, ADAPT, MPID, or FLOW. Example: PID SHOW | SYNC SHOW | ADAPT SHOW | MPID SHOW | FLOW SHOW");
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

    // ADAPT commands: manage the adaptive per-magnitude offset table
    if (line == "ADAPT SHOW") {
        if (out != nullptr) {
            out->println("ADAPT CONFIG:");
            if (gMotorControlInstance != nullptr) {
                out->print("ENABLED="); out->println(gMotorControlInstance->isAdaptiveEnabled() ? 1 : 0);
            } else {
                out->println("No MotorControl instance");
            }
            // Show table summary (non-zero entries)
            if (gMotorControlInstance != nullptr) {
                out->println("OFFSETS (mag:off)");
                for (int m = 0; m <= 100; ++m) {
                    int8_t o = gMotorControlInstance->getAdaptiveOffset((uint8_t)m);
                    if (o != 0) {
                        out->print(m);
                        out->print(':');
                        out->println(o);
                    }
                }
            }
        }
        return true;
    }

    if (line == "ADAPT ON") {
        if (gMotorControlInstance != nullptr) gMotorControlInstance->enableAdaptiveCorrection(true);
        if (out != nullptr) out->println("OK ADAPT ON");
        return true;
    }

    if (line == "ADAPT OFF") {
        if (gMotorControlInstance != nullptr) gMotorControlInstance->enableAdaptiveCorrection(false);
        if (out != nullptr) out->println("OK ADAPT OFF");
        return true;
    }

    if (line == "ADAPT RESET") {
        if (gMotorControlInstance != nullptr) gMotorControlInstance->resetAdaptiveOffsets();
        if (out != nullptr) out->println("OK ADAPT RESET");
        return true;
    }

    if (sscanf(line.c_str(), "ADAPT MAX %d", &intValue) == 1) {
        if (gMotorControlInstance != nullptr) gMotorControlInstance->setAdaptiveMaxOffset((uint8_t)constrain(intValue, 1, 50));
        if (out != nullptr) out->print("OK ADAPT MAX="); if (out!=nullptr) out->println(intValue);
        return true;
    }

    if (sscanf(line.c_str(), "ADAPT THRESH %d", &intValue) == 1) {
        if (gMotorControlInstance != nullptr) gMotorControlInstance->setAdaptiveLearnThreshold((uint8_t)constrain(intValue, 1, 100));
        if (out != nullptr) out->print("OK ADAPT THRESH="); if (out!=nullptr) out->println(intValue);
        return true;
    }

    if (sscanf(line.c_str(), "ADAPT GET %d", &intValue) == 1) {
        if (gMotorControlInstance != nullptr) {
            int mag = constrain(intValue, 0, 100);
            int8_t off = gMotorControlInstance->getAdaptiveOffset((uint8_t)mag);
            if (out != nullptr) {
                out->print("ADAPT "); out->print(mag); out->print("="); out->println(off);
            }
        }
        return true;
    }

    if (sscanf(line.c_str(), "ADAPT SET %d %d", &intValue, &intValue) == 2) {
        // sscanf reuse: read first into intValue then second into intValue would overwrite; parse manually
    }

    // Manual set using token parsing for two args
    if (line.startsWith("ADAPT SET ")) {
        // Extract remainder
        String rest = line.substring(10);
        int mag = -1;
        int off = 0;
        if (sscanf(rest.c_str(), "%d %d", &mag, &off) == 2) {
            if (gMotorControlInstance != nullptr && mag >= 0 && mag <= 100) {
                gMotorControlInstance->setAdaptiveOffset((uint8_t)mag, (int8_t)off);
                if (out != nullptr) {
                    out->print("OK ADAPT SET "); out->print(mag); out->print(' '); out->println(off);
                }
            }
            return true;
        }

        // MPID commands: motor PID control
        if (line == "MPID SHOW") {
            if (out != nullptr) {
                out->println("MPID CONFIG:");
                if (gMotorControlInstance != nullptr) {
                    out->print("ADAPT_ENABLED="); out->println(gMotorControlInstance->isAdaptiveEnabled() ? 1 : 0);
                    out->print("MOTOR_PID_ENABLED="); out->println(gMotorControlInstance->isMotorPidEnabled() ? 1 : 0);
                    out->print("TARGET_LRPM="); out->println(gMotorControlInstance->getTargetLeftRPM());
                    out->print("TARGET_RRPM="); out->println(gMotorControlInstance->getTargetRightRPM());
                    out->print("MEAS_LRPM="); out->println(gMotorControlInstance->getMeasuredLeftRPM());
                    out->print("MEAS_RRPM="); out->println(gMotorControlInstance->getMeasuredRightRPM());
                    out->print("GAINS KP,KI,KD="); out->print(gMotorControlInstance->getMotorPidKp(), 6); out->print(','); out->print(gMotorControlInstance->getMotorPidKi(), 6); out->print(','); out->println(gMotorControlInstance->getMotorPidKd(), 6);
                    out->print("TICKS_PER_REV="); out->println(gMotorControlInstance->getTicksPerRev());
                } else {
                    out->println("No MotorControl instance");
                }
            }
            return true;
        }

        if (line == "MPID ON") {
            if (gMotorControlInstance != nullptr) gMotorControlInstance->enableMotorPid(true);
            if (out != nullptr) out->println("OK MPID ON");
            return true;
        }

        if (line == "MPID OFF") {
            if (gMotorControlInstance != nullptr) gMotorControlInstance->enableMotorPid(false);
            if (out != nullptr) out->println("OK MPID OFF");
            return true;
        }

        if (sscanf(line.c_str(), "MPID TARG %f %f", &value, &value) == 2) {
            // sscanf can't reuse same var; parse manually below
        }

        if (line.startsWith("MPID TARG ")) {
            String rest = line.substring(10);
            float l=0.0f,r=0.0f;
            if (sscanf(rest.c_str(), "%f %f", &l, &r) == 2) {
                if (gMotorControlInstance != nullptr) gMotorControlInstance->setTargetRPMs(l, r);
                if (out != nullptr) { out->print("OK MPID TARG "); out->print(l); out->print(' '); out->println(r); }
                return true;
            }
        }

        if (sscanf(line.c_str(), "MPID GAINS %f %f %f", &value, &value, &value) == 3) {
            // placeholder to allow branch fallthrough to manual parse
        }

        if (line.startsWith("MPID GAINS ")) {
            String rest = line.substring(11);
            float kp=0, ki=0, kd=0;
            if (sscanf(rest.c_str(), "%f %f %f", &kp, &ki, &kd) == 3) {
                if (gMotorControlInstance != nullptr) gMotorControlInstance->setMotorPidGains(kp, ki, kd);
                if (out != nullptr) { out->print("OK MPID GAINS "); out->print(kp); out->print(' '); out->print(ki); out->print(' '); out->println(kd); }
                return true;
            }
        }

        if (sscanf(line.c_str(), "MPID TICKS %f", &value) == 1) {
            if (gMotorControlInstance != nullptr) gMotorControlInstance->setTicksPerRev(value);
            if (out != nullptr) { out->print("OK MPID TICKS="); out->println(value); }
            return true;
        }
        // FLOW commands: control and view optical flow totals and scale
        if (line == "FLOW SHOW") {
            if (out != nullptr) {
                out->println("FLOW CONFIG:");
                out->print("TOTAL_X_MM="); out->println(opticalFlow.getTotalXmm(), 3);
                out->print("TOTAL_Y_MM="); out->println(opticalFlow.getTotalYmm(), 3);
                out->print("MM_PER_COUNT="); out->println(opticalFlow.getScaleMMPerCount(), 6);
            }
            return true;
        }

        if (line == "FLOW RESET") {
            opticalFlow.resetTotals();
            if (out != nullptr) out->println("OK FLOW RESET");
            return true;
        }

        if (sscanf(line.c_str(), "FLOW SCALE %f", &value) == 1) {
            opticalFlow.setScaleMMPerCount(value);
            if (out != nullptr) { out->print("OK FLOW SCALE="); out->println(value); }
            return true;
        }

        if (line == "FLOW LED ON") {
            opticalFlow.setLed(true);
            if (out != nullptr) out->println("OK FLOW LED ON");
            return true;
        }

        if (line == "FLOW LED OFF") {
            opticalFlow.setLed(false);
            if (out != nullptr) out->println("OK FLOW LED OFF");
            return true;
        }
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

    adaptiveCorrectionEnabled = false;
    adaptiveMaxOffset = 6; // allow up to +/-6 PWM steps correction
    adaptiveLearnThreshold = 2; // pulses/sec threshold to trigger learning
    for (int i = 0; i <= 100; ++i) {
        adaptiveOffset[i] = 0;
    }
        // Register this instance for serial command access.
        gMotorControlInstance = this;

    fixedSamplerEnabled = false;
    fixedSamplerDtSec = 0.01f;
    sampledLeftRate = 0.0f;
    sampledRightRate = 0.0f;
    // Motor PID defaults
    motorPidEnabled = false;
    targetLeftRPM = 0.0f;
    targetRightRPM = 0.0f;
    MOTOR_PID_KP = 0.5f;
    MOTOR_PID_KI = 0.0f;
    MOTOR_PID_KD = 0.0f;
    leftPidIntegral = 0.0f;
    rightPidIntegral = 0.0f;
    leftPidPrevError = 0.0f;
    rightPidPrevError = 0.0f;
    motorPidIntegralLimit = 200.0f; // RPM·s cap to avoid windup
    ticksPerRev = 2048.0f; // default, change to your encoder CPR*4
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
    // initialize adaptive table on begin
    for (int i = 0; i <= 100; ++i) {
        adaptiveOffset[i] = 0;
    }

    // If fixed sampler enabled, start hardware timer sampling at fixed interval
    if (fixedSamplerEnabled) {
        unsigned long us = (unsigned long)(fixedSamplerDtSec * 1e6);
        if (us < 1000) us = 1000; // at least 1ms
        gControlTimer.begin([](){
            if (gMotorControlInstance == nullptr) return;
            // sample encoder counts and compute pulses/sec
            static int32_t lastA = 0, lastB = 0;
            int32_t nowA = gMotorControlInstance->getLeftEncoderPulses();
            int32_t nowB = gMotorControlInstance->getRightEncoderPulses();
            int32_t dA = nowA - lastA;
            int32_t dB = nowB - lastB;
            lastA = nowA;
            lastB = nowB;
            float dt = gMotorControlInstance->fixedSamplerDtSec;
            if (dt <= 0.0f) dt = 0.01f;
            gMotorControlInstance->sampledLeftRate = ((float)dA) / dt; // pulses/sec
            gMotorControlInstance->sampledRightRate = ((float)dB) / dt;
            gMotorControlInstance->encoderRateInitialized = true;
            // Motor PID: compute RPM and update motor outputs if enabled
            if (gMotorControlInstance->motorPidEnabled) {
                // Convert pulses/sec to RPM: RPM = (pulses/sec) / ticksPerRev * 60
                float leftRPM = (gMotorControlInstance->sampledLeftRate / gMotorControlInstance->ticksPerRev) * 60.0f;
                float rightRPM = (gMotorControlInstance->sampledRightRate / gMotorControlInstance->ticksPerRev) * 60.0f;

                // LEFT PID
                float lError = gMotorControlInstance->targetLeftRPM - leftRPM;
                gMotorControlInstance->leftPidIntegral += lError * dt;
                if (gMotorControlInstance->leftPidIntegral > gMotorControlInstance->motorPidIntegralLimit) gMotorControlInstance->leftPidIntegral = gMotorControlInstance->motorPidIntegralLimit;
                if (gMotorControlInstance->leftPidIntegral < -gMotorControlInstance->motorPidIntegralLimit) gMotorControlInstance->leftPidIntegral = -gMotorControlInstance->motorPidIntegralLimit;
                float lDeriv = (lError - gMotorControlInstance->leftPidPrevError) / dt;
                float lOut = (gMotorControlInstance->MOTOR_PID_KP * lError) + (gMotorControlInstance->MOTOR_PID_KI * gMotorControlInstance->leftPidIntegral) + (gMotorControlInstance->MOTOR_PID_KD * lDeriv);
                gMotorControlInstance->leftPidPrevError = lError;

                // RIGHT PID
                float rError = gMotorControlInstance->targetRightRPM - rightRPM;
                gMotorControlInstance->rightPidIntegral += rError * dt;
                if (gMotorControlInstance->rightPidIntegral > gMotorControlInstance->motorPidIntegralLimit) gMotorControlInstance->rightPidIntegral = gMotorControlInstance->motorPidIntegralLimit;
                if (gMotorControlInstance->rightPidIntegral < -gMotorControlInstance->motorPidIntegralLimit) gMotorControlInstance->rightPidIntegral = -gMotorControlInstance->motorPidIntegralLimit;
                float rDeriv = (rError - gMotorControlInstance->rightPidPrevError) / dt;
                float rOut = (gMotorControlInstance->MOTOR_PID_KP * rError) + (gMotorControlInstance->MOTOR_PID_KI * gMotorControlInstance->rightPidIntegral) + (gMotorControlInstance->MOTOR_PID_KD * rDeriv);
                gMotorControlInstance->rightPidPrevError = rError;

                // Map PID output (which is in RPM units scaled by gains) to speed command
                // The PID gains should be tuned so outputs fall in [-100,100]. Clip to safe range.
                int16_t cmdL = (int16_t)constrain((int)roundf(lOut), -100, 100);
                int16_t cmdR = (int16_t)constrain((int)roundf(rOut), -100, 100);

                gMotorControlInstance->leftMotor.setSpeed(cmdL);
                gMotorControlInstance->rightMotor.setSpeed(cmdR);
            }
        }, us);
    }
}

void MotorControl::enableAdaptiveCorrection(bool enable) {
    adaptiveCorrectionEnabled = enable;
}

void MotorControl::enableMotorPid(bool enable) {
    motorPidEnabled = enable;
    // Reset integrators when enabling/disabling
    leftPidIntegral = 0.0f;
    rightPidIntegral = 0.0f;
    leftPidPrevError = 0.0f;
    rightPidPrevError = 0.0f;
}

void MotorControl::setTargetRPMs(float leftRpm, float rightRpm) {
    targetLeftRPM = leftRpm;
    targetRightRPM = rightRpm;
}

void MotorControl::setMotorPidGains(float kp, float ki, float kd) {
    MOTOR_PID_KP = kp;
    MOTOR_PID_KI = ki;
    MOTOR_PID_KD = kd;
}

float MotorControl::getMeasuredLeftRPM() const {
    return (sampledLeftRate / ticksPerRev) * 60.0f;
}

float MotorControl::getMeasuredRightRPM() const {
    return (sampledRightRate / ticksPerRev) * 60.0f;
}

bool MotorControl::isMotorPidEnabled() const {
    return motorPidEnabled;
}

float MotorControl::getTargetLeftRPM() const {
    return targetLeftRPM;
}

float MotorControl::getTargetRightRPM() const {
    return targetRightRPM;
}

void MotorControl::setTicksPerRev(float ticks) {
    ticksPerRev = ticks;
}

float MotorControl::getTicksPerRev() const {
    return ticksPerRev;
}

float MotorControl::getMotorPidKp() const { return MOTOR_PID_KP; }
float MotorControl::getMotorPidKi() const { return MOTOR_PID_KI; }
float MotorControl::getMotorPidKd() const { return MOTOR_PID_KD; }

void MotorControl::resetAdaptiveOffsets(void) {
    for (int i = 0; i <= 100; ++i) {
        adaptiveOffset[i] = 0;
    }
}

int8_t MotorControl::getAdaptiveOffset(uint8_t magnitude) const {
    if (magnitude > 100) return 0;
    return adaptiveOffset[magnitude];
}

bool MotorControl::isAdaptiveEnabled() const {
    return adaptiveCorrectionEnabled;
}

void MotorControl::setAdaptiveMaxOffset(uint8_t maxOff) {
    if (maxOff == 0) maxOff = 1;
    if (maxOff > 50) maxOff = 50;
    adaptiveMaxOffset = (int8_t)maxOff;
}

void MotorControl::setAdaptiveLearnThreshold(uint8_t thresh) {
    adaptiveLearnThreshold = (thresh == 0) ? 1 : thresh;
}

void MotorControl::setAdaptiveOffset(uint8_t magnitude, int8_t offset) {
    if (magnitude > 100) return;
    if (offset > adaptiveMaxOffset) offset = adaptiveMaxOffset;
    if (offset < -adaptiveMaxOffset) offset = -adaptiveMaxOffset;
    adaptiveOffset[magnitude] = offset;
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

    // If motor PID is enabled, disable it for the duration of the blocking turn
    bool prevMotorPid = motorPidEnabled;
    if (prevMotorPid) {
        enableMotorPid(false);
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
    // Restore motor PID state
    if (prevMotorPid) {
        enableMotorPid(true);
    }
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

    // Prefer fixed-rate sampled rates when the fixed sampler is enabled and initialized
    float leftRate = 0.0f;
    float rightRate = 0.0f;
    float dt = 0.0f;
#if defined(__IMXRT1062__)
    if (fixedSamplerEnabled && encoderRateInitialized) {
        leftRate = sampledLeftRate;   // pulses/sec
        rightRate = sampledRightRate; // pulses/sec
        dt = fixedSamplerDtSec;
        // update stored last counters to avoid confusing legacy path
        syncLastLeftEncoderPulses = leftNow;
        syncLastRightEncoderPulses = rightNow;
        syncLastMs = now;
    } else
#endif
    {
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
        leftRate = ((float)abs(dLeft) * 1000.0f) / (float)dtMs;
        rightRate = ((float)abs(dRight) * 1000.0f) / (float)dtMs;
        dt = (float)dtMs / 1000.0f;
    }

    float error = leftRate - rightRate;

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

    // Adaptive learning: update per-magnitude offset table when enabled.
    if (adaptiveCorrectionEnabled) {
        int mag = abs(lastCmdLeftSpeed);
        if (mag >= 0 && mag <= 100) {
            // Only learn when the rate mismatch is meaningful
            if (fabs(error) >= (float)adaptiveLearnThreshold) {
                // If left is faster than right, decrement left offset (make left command smaller)
                int8_t delta = (error > 0.0f) ? -1 : 1;
                int newOff = (int)adaptiveOffset[mag] + (int)delta;
                if (newOff > adaptiveMaxOffset) newOff = adaptiveMaxOffset;
                if (newOff < -adaptiveMaxOffset) newOff = -adaptiveMaxOffset;
                adaptiveOffset[mag] = (int8_t)newOff;
            }

            // Apply learned offset respecting drive direction
            int sign = (lastCmdLeftSpeed >= 0) ? 1 : -1;
            int adj = (int)adaptiveOffset[mag] * sign;
            leftSpeed = (int16_t)constrain((int)leftSpeed + adj, -100, 100);
        }
    }
}

void MotorControl::enableFixedRateSampler(bool enable, float dtSeconds) {
    fixedSamplerEnabled = enable;
    if (dtSeconds > 0.0f) fixedSamplerDtSec = dtSeconds;
}