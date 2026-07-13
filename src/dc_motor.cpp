//************************************
//         dc_motor.cpp    
//************************************
// DC Motor control implementation
// Uses Servo library PWM output for motor driver control

#include "dc_motor.h"
#include "Arduino.h"

/**
 * Constructor - Initialize DC motor with pin number
 * @param motor_pin: PWM pin for motor control
 */
DCMotor::DCMotor(uint8_t motor_pin) : pin(motor_pin), current_speed(0) {
}

/**
 * Initialize motor - attach servo to pin and set to neutral
 */
void DCMotor::begin(void) {
    servo.attach(pin);
    stop();  // Set to neutral (1500µs)
    Serial.print("DC Motor initialized on pin: ");
    Serial.println(pin);
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

/**
 * Print motor status to serial
 */
void DCMotor::printStatus(void) const {
    Serial.print("DC Motor Status - Speed: ");
    Serial.print(current_speed);
    Serial.println(" [-100 to 100]");
}
