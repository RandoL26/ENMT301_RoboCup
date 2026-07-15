//************************************
//         dc_motor.h    
//************************************
// DC Motor control for Reversible High torque Turbo worm Geared motor (JGY370 DC 12V)
// Uses Servo library for PWM pulse control (1ms to 2ms)
// Two channels available for future expansion

#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include <Servo.h>

class DCMotor {
private:
    Servo servo;
    uint8_t pin;
    int16_t current_speed;  // Range: -100 to 100
    uint8_t encoderPinA;
    uint8_t encoderPinB;
    volatile int32_t encoderPulses;
    volatile bool directionForward;
    volatile uint8_t encoderPinALast;
    bool encoderConfigured;
    uint8_t motorIndex;

    static DCMotor* instances[2];
    static void encoderISR0();
    static void encoderISR1();
    void handleEncoderInterrupt();

    static const int16_t MIN_SPEED = -100;
    static const int16_t MAX_SPEED = 100;
    static const uint16_t NEUTRAL_PULSE = 1500;  // 1.5ms = stop/neutral
    static const uint16_t MIN_PULSE = 1000;      // 1ms = full reverse
    static const uint16_t MAX_PULSE = 2000;      // 2ms = full forward
    
public:
    DCMotor(uint8_t motor_pin, uint8_t enc_pin_a, uint8_t enc_pin_b);
    
    // Initialize motor on the specified pin
    void begin(void);
    
    // Set motor speed (-100 to 100)
    // Negative values = reverse, 0 = stop, Positive values = forward
    void setSpeed(int16_t speed);
    
    // Get current motor speed
    int16_t getSpeed(void) const;
    
    // Stop motor (speed = 0)
    void stop(void);
    
    // Check if speed value is within valid range
    bool isValidSpeed(int16_t speed) const;

    // Get current encoder pulse count
    int32_t getEncoderPulses(void) const;

    // Reset encoder pulse count to zero
    void resetEncoderPulses(void);
    
    // Print motor status to serial
    void printStatus(void) const;
};

#endif /* DC_MOTOR_H_ */
