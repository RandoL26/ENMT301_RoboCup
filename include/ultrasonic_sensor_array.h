#ifndef ULTRASONIC_SENSOR_ARRAY_H
#define ULTRASONIC_SENSOR_ARRAY_H

#include <Arduino.h>
#include "ultrasonic_sensor.h"

class UltrasonicSensorArray {
private:
    UltrasonicSensor** sensors;
    uint8_t sensorCount;
    bool initialized;
    
public:
    // Structure to hold data from all sensors
    struct UltrasonicData {
        double* distances;  // Array of distances in cm
        uint8_t count;      // Number of sensors
    };
    
    // Constructor
    // count: number of ultrasonic sensors
    UltrasonicSensorArray(uint8_t count = 2);
    
    // Destructor
    ~UltrasonicSensorArray();
    
    // Add a sensor to the array
    // index: position in array (0 to count-1)
    // triggerPin: trigger pin for this sensor
    // echoPin: echo pin for this sensor
    void addSensor(uint8_t index, uint8_t triggerPin, uint8_t echoPin);
    
    // Initialize all sensors
    bool begin();
    
    // Read all sensors
    UltrasonicData readDistances();
    
    // Get distance from specific sensor
    double getDistance(uint8_t index) const;
    
    // Print all sensor data
    void printDistances(const UltrasonicData& data) const;
    
    // Print visual bars for all sensors
    void printVisualBars() const;
    
    // Check if initialized
    bool isInitialized() const;
    
    // Get sensor count
    uint8_t getSensorCount() const;
    
    // Update a specific sensor
    void updateSensor(uint8_t index);
    
    // Update all sensors
    void updateAll();
};

#endif // ULTRASONIC_SENSOR_ARRAY_H
