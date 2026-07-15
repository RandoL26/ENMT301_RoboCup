//************************************
//         dc_motor.cpp    
//************************************
// DC Motor control implementation
// Uses Servo library PWM output for motor driver control

#include "dc_motor.h"
#include "Arduino.h"

DCMotor* DCMotor::instances[2] = {nullptr, nullptr};

void DCMotor::encoderISR0() {
    if (instances[0] != nullptr) {
        instances[0]->handleEncoderInterrupt();
    }
}

void DCMotor::encoderISR1() {
    if (instances[1] != nullptr) {
        instances[1]->handleEncoderInterrupt();
    }
}

/**
 * Constructor - Initialize DC motor with pin number and encoder pins
 * @param motor_pin: PWM pin for motor control
 * @param enc_pin_a: Encoder A pin
 * @param enc_pin_b: Encoder B pin
 */
DCMotor::DCMotor(uint8_t motor_pin, uint8_t enc_pin_a, uint8_t enc_pin_b)
    : pin(motor_pin),
      current_speed(0),
      encoderPinA(enc_pin_a),
      encoderPinB(enc_pin_b),
      encoderPulses(0),
      directionForward(true),
      encoderPinALast(LOW),
      encoderConfigured(false),
      motorIndex(255) {

    // Assign motor index using a static counter (first instance = 0, second = 1)
    static uint8_t nextIndex = 0;
    if (nextIndex < 2) {
        motorIndex = nextIndex++;
    }
}

/**
 * Initialize motor - attach servo to pin and set to neutral
 */
void DCMotor::begin(void) {
    Serial.begin(115200);
    servo.attach(pin);
    stop();  // Set to neutral (1500µs)

    if (motorIndex < 2) {
        pinMode(encoderPinA, INPUT);
        pinMode(encoderPinB, INPUT);
        encoderPinALast = digitalRead(encoderPinA);
        directionForward = true;
        encoderPulses = 0;

        instances[motorIndex] = this;
        int interruptNumber = digitalPinToInterrupt(encoderPinA);
        Serial.print("Encoder A pin: ");
        Serial.print(encoderPinA);
        Serial.print(" -> interruptNumber: ");
        Serial.print(interruptNumber);
        Serial.print(" motorIndex: ");
        Serial.println(motorIndex);
        if (interruptNumber != NOT_AN_INTERRUPT) {
            if (motorIndex == 0) {
                attachInterrupt(interruptNumber, DCMotor::encoderISR0, CHANGE);
            } else {
                attachInterrupt(interruptNumber, DCMotor::encoderISR1, CHANGE);
            }
            encoderConfigured = true;
            Serial.println("attachInterrupt called OK");
        } else {
            Serial.println("ERROR: NOT_AN_INTERRUPT returned for encoder pin");
        }
    }

    Serial.print("DC Motor initialized on pin: ");
    Serial.println(pin);
    if (encoderConfigured) {
        Serial.print("Encoder initialized A/B pins: ");
        Serial.print(encoderPinA);
        Serial.print("/");
        Serial.print(encoderPinB);
        Serial.print(" (motorIndex=");
        Serial.print(motorIndex);
        Serial.println(")");
        // DEBUG: Test if pins are readable
        Serial.print("Initial pin states: A=");
        Serial.print(digitalRead(encoderPinA));
        Serial.print(" B=");
        Serial.println(digitalRead(encoderPinB));
    } else {
        Serial.println("WARNING: Encoder not configured for this motor");
    }
}

void DCMotor::handleEncoderInterrupt() {
    int stateA = digitalRead(encoderPinA);
    if ((encoderPinALast == LOW) && (stateA == HIGH)) {
        int stateB = digitalRead(encoderPinB);
        if ((stateB == LOW) && directionForward) {
            directionForward = false; // Reverse
        } else if ((stateB == HIGH) && !directionForward) {
            directionForward = true;  // Forward
        }
    }
    encoderPinALast = (uint8_t)stateA;

    if (!directionForward) {
        encoderPulses++;
    } else {
        encoderPulses--;
    }
}

/**
 * Set motor speed
 * @param speed: -100 (full reverse) to 100 (full forward), 0 = stop
 */
void DCMotor::setSpeed(int16_t speed) {
    // Validate speed range
    if (!isValidSpeed(speed)) {
        Serial.print("ERROR: Invalid speed value: ");
        Serial.print(speed);
        Serial.println(" (valid range: -100 to 100)");
        return;
    }

    // Only update/print when value actually changes
    if (speed == current_speed) {
        return;
    }
    
    current_speed = speed;
    
    // Convert speed (-100 to 100) to pulse width (1000 to 2000 µs)
    // Speed 0 = 1500µs (neutral/stop)
    // Speed -100 = 1000µs (full reverse)
    // Speed 100 = 2000µs (full forward)
    uint16_t pulse_width = NEUTRAL_PULSE + (speed * 5);  // Each speed unit = 5µs
    
    servo.writeMicroseconds(pulse_width);
    
    Serial.print("DC Motor speed set to: ");
    Serial.print(speed);
    Serial.print(" (Pulse: ");
    Serial.print(pulse_width);
    Serial.println(" µs)");
}

/**
 * Get current motor speed
 * @return: Current speed value (-100 to 100)
 */
int16_t DCMotor::getSpeed(void) const {
    return current_speed;
}

/**
 * Stop motor (set speed to 0)
 */
void DCMotor::stop(void) {
    current_speed = 0;
    servo.writeMicroseconds(NEUTRAL_PULSE);
    Serial.println("DC Motor stopped (neutral position)");
}

/**
 * Check if speed value is within valid range
 * @param speed: Speed value to check
 * @return: true if valid, false otherwise
 */
bool DCMotor::isValidSpeed(int16_t speed) const {
    return (speed >= MIN_SPEED && speed <= MAX_SPEED);
}

int32_t DCMotor::getEncoderPulses(void) const {
    noInterrupts();
    int32_t count = encoderPulses;
    interrupts();
    return count;
}

void DCMotor::resetEncoderPulses(void) {
    noInterrupts();
    encoderPulses = 0;
    interrupts();
}

/**
 * Print motor status to serial
 */
void DCMotor::printStatus(void) const {
    Serial.print("DC Motor Status - Speed: ");
    Serial.print(current_speed);
    Serial.print(" [-100 to 100], Encoder pulses: ");
    Serial.print(getEncoderPulses());
    Serial.print(", Direction: ");
    Serial.println(directionForward ? "Forward" : "Reverse");
}
