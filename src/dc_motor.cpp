//************************************
//         dc_motor.cpp    
//************************************
// DC Motor control implementation
// Uses Servo library PWM output for motor driver control

#include "dc_motor.h"
#include "Arduino.h"

#if defined(__IMXRT1062__)
// Teensy 4 QuadEncoder uses read()/setInitConfig()/init()
#include <QuadEncoder.h>
#endif

// Teensy-only build: no legacy attachInterrupt-based encoder handling.

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
            hwEncoder(nullptr) {

    // Constructor keeps simple for Teensy hardware encoder usage
}

/**
 * Initialize motor - attach servo to pin and set to neutral
 */
void DCMotor::begin(void) {
    Serial.begin(115200);
    servo.attach(pin);
    stop();  // Set to neutral (1500µs)
    // Initialize hardware QuadEncoder for Teensy 4
    // Note: Teensy 4.0 requires valid hardware pin pairs for QuadTimer channels.
    if (hwEncoder == nullptr) {
        hwEncoder = new QuadEncoder(0, encoderPinA, encoderPinB, 0);
        hwEncoder->setInitConfig();
        hwEncoder->init();
    }
    Serial.print("DC Motor initialized (Teensy QuadEncoder) on pin: ");
    Serial.println(pin);
}
// No legacy ISR handler on Teensy; QuadEncoder provides hardware counting.

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
    
    // Convert speed (-100 to 100) to pulse width (1050 to 1950 µs)
    // Speed 0 = 1500µs (neutral/stop)
    // Speed -100 = 1050µs (full reverse)
    // Speed 100 = 1950µs (full forward)
    uint16_t pulse_width = (uint16_t)map((long)speed, (long)MIN_SPEED, (long)MAX_SPEED,
                                         (long)MIN_PULSE, (long)MAX_PULSE);
    
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
    if (hwEncoder != nullptr) {
        return hwEncoder->read();
    }
    return 0;
}

void DCMotor::resetEncoderPulses(void) {
    if (hwEncoder != nullptr) {
        hwEncoder->write(0);
    }
}

/**
 * Print motor status to serial
 */
void DCMotor::printStatus(void) const {
    Serial.print("DC Motor Status - Speed: ");
    Serial.print(current_speed);
    Serial.print(" [-100 to 100], Encoder pulses: ");
    Serial.print(getEncoderPulses());
    Serial.println();
}
