#ifndef PROXIMITY_SENSOR_H
#define PROXIMITY_SENSOR_H

#include <Arduino.h>

class ProximitySensor {
private:
    int sensorPin;
    bool objectDetected;
    unsigned long lastDebounceTime;
    unsigned long debounceDelay;
    int lastSensorState;
    
public:
    // Constructor
    ProximitySensor(int pin);
    
    // Initialize the sensor
    void begin();
    
    // Read sensor value
    void update();
    
    // Get current detection state
    bool isObjectDetected() const;
    
    // Print sensor status to serial
    void printStatus() const;
    
    // Get raw sensor reading (0 or 1)
    int getRawReading() const;
};

#endif // PROXIMITY_SENSOR_H
