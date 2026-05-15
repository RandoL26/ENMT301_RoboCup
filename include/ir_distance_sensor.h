#ifndef IR_DISTANCE_SENSOR_H
#define IR_DISTANCE_SENSOR_H

#include <Arduino.h>
#include <stdint.h>

class IRDistanceSensor {
private:
    uint8_t analogPin;          // Analog pin for sensor input
    float lastDistance;         // Last measured distance in cm
    bool initialized;
    float calibrationConstant;  // Calibration constant
    
public:
    // Constructor with analog pin
    IRDistanceSensor(uint8_t pin = A0);
    
    // Initialize the IR distance sensor
    bool begin();
    
    // Read distance from sensor (returns distance in cm)
    float readDistance();
    
    // Get the last measured distance
    float getLastDistance() const;
    
    // Print distance to serial
    void printDistance();
    
    // Check if sensor is initialized
    bool isInitialized() const;
    
    // Set calibration constant (use this to fine-tune readings)
    // Distance = calibrationConstant / ADC_value
    void setCalibrationConstant(float constant);
};

#endif // IR_DISTANCE_SENSOR_H
