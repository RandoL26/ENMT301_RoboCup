//************************************
//         dc_motor.h    
//************************************
// DC Motor control for a servo-pulse input motor controller
// Encoders are read from separate digital input pins

#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include <Arduino.h>
#include <Servo.h>

class DCMotor {
private:
    Servo servo;
    int pwm_pin;
    int encoder_pin;
    long encoder_count;
    bool encoder_enabled;
    int last_encoder_state;
    int current_speed;  // Range: -100 to 100
    static const int MIN_SPEED = -100;
    static const int MAX_SPEED = 100;
    
public:
    DCMotor(int pwmPin, int encoderPin = -1);
    
    // Initialize motor on the specified pin
    void begin(void);
    
    // Set motor speed (-100 to 100)
    // -100 = full reverse, 0 = stop, 100 = full forward
    void setSpeed(int speed);

    // Poll encoder input and accumulate edge count.
    // Call this periodically from a scheduler/task loop.
    void updateEncoder(void);

    // Encoder utilities
    bool hasEncoder(void) const;
    int getEncoderPin(void) const;
    long getEncoderCount(void) const;
    void resetEncoderCount(void);
    
    // Get current motor speed
    int getSpeed(void) const;
    
    // Stop motor (speed = 0)
    void stop(void);
    
    // Check if speed value is within valid range
    bool isValidSpeed(int speed) const;
    
    // Print motor status to serial
    void printStatus(void) const;
};

#endif /* DC_MOTOR_H_ */
