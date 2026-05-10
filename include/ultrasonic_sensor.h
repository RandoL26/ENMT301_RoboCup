#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
private:
    uint8_t triggerPin;
    uint8_t echoPin;
    double lastDistance;  // in cm
    
public:
    // Constructor
    UltrasonicSensor(uint8_t trigger, uint8_t echo);
    
    // Initialize the sensor
    void begin();
    
    // Read sensor value
    void update();
    
    // Get distance in cm
    double getDistanceCm() const;
    
    // Print sensor status to serial with visual bar
    void printStatus() const;
    
    // Print visual representation (bar graph)
    void printVisualBar() const;
};

#endif // ULTRASONIC_SENSOR_H
