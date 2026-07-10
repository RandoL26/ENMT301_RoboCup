//************************************
//         dc_motor.cpp    
//************************************
// DC Motor control implementation
// Uses Servo library PWM output for motor driver control

#include "dc_motor.h"
#include "Arduino.h"

/**
 * Constructor - Initialize DC motor with servo signal pin
 * @param pwmPin: PWM pin for motor control signal
 * @param encoderPin: Optional digital pin used for encoder pulse input
 */
DCMotor::DCMotor(int pwmPin, int encoderPin)
    : pwm_pin(pwmPin),
            encoder_pin(encoderPin),
            encoder_count(0),
            encoder_enabled(encoderPin >= 0),
            last_encoder_state(LOW),
            current_speed(0) {
}

/**
 * Initialize motor - attach servo to pin and set to neutral
 */
void DCMotor::begin(void) {
    servo.attach(pwm_pin, 1050, 1950);

    // Many servo-signal motor controllers require a neutral/arming window.
    servo.writeMicroseconds(1500);
    delay(1200);

    if (encoder_enabled) {
        pinMode(encoder_pin, INPUT_PULLUP);
        last_encoder_state = digitalRead(encoder_pin);
        encoder_count = 0;
    }

    stop();
    Serial.print("DC Motor initialized on signal pin: ");
    Serial.print(pwm_pin);
    if (encoder_enabled) {
        Serial.print(" | Encoder pin: ");
        Serial.println(encoder_pin);
    } else {
        Serial.println(" | Encoder: disabled");
    }
}

/**
 * Set motor speed
 * @param speed: -100 (full reverse) to 100 (full forward), 0 = stop
 */
void DCMotor::setSpeed(int speed) {
    // Validate speed range
    if (!isValidSpeed(speed)) {
        Serial.print("ERROR: Invalid speed value: ");
        Serial.print(speed);
        Serial.println(" (valid range: -100 to 100)");
        return;
    }
    
    current_speed = speed;

    const int pulse_width = constrain(1500 + ((speed * 9) / 2), 1050, 1950);
    servo.writeMicroseconds(pulse_width);

    Serial.print("DC Motor speed set to: ");
    Serial.print(speed);
    Serial.print(" (Pulse: ");
    Serial.print(pulse_width);
    Serial.println(" us)");
}

/**
 * Poll encoder pin and count transitions.
 */
void DCMotor::updateEncoder(void) {
    if (!encoder_enabled) {
        return;
    }

    const int state = digitalRead(encoder_pin);
    if (state != last_encoder_state) {
        encoder_count++;
        last_encoder_state = state;
    }
}

bool DCMotor::hasEncoder(void) const {
    return encoder_enabled;
}

int DCMotor::getEncoderPin(void) const {
    return encoder_pin;
}

long DCMotor::getEncoderCount(void) const {
    return encoder_count;
}

void DCMotor::resetEncoderCount(void) {
    encoder_count = 0;
    if (encoder_enabled) {
        last_encoder_state = digitalRead(encoder_pin);
    }
}

/**
 * Get current motor speed
 * @return: Current speed value (-100 to 100)
 */
int DCMotor::getSpeed(void) const {
    return current_speed;
}

/**
 * Stop motor (set speed to 0)
 */
void DCMotor::stop(void) {
    current_speed = 0;
    servo.writeMicroseconds(1500);
    Serial.println("DC Motor stopped (neutral 1500 us)");
}

/**
 * Check if speed value is within valid range
 * @param speed: Speed value to check
 * @return: true if valid, false otherwise
 */
bool DCMotor::isValidSpeed(int speed) const {
    return (speed >= MIN_SPEED && speed <= MAX_SPEED);
}

/**
 * Print motor status to serial
 */
void DCMotor::printStatus(void) const {
    Serial.print("DC Motor Status - Speed: ");
    Serial.print(current_speed);
    Serial.print(" [-100 to 100]");
    if (encoder_enabled) {
        Serial.print(" | Encoder count: ");
        Serial.println(encoder_count);
    } else {
        Serial.println();
    }
}
