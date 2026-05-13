#ifndef TOF_SENSOR_ARRAY_H
#define TOF_SENSOR_ARRAY_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

class TOFSensorArray {
private:
    static const uint8_t MAX_SENSORS = 8;
    
    uint8_t sensorCount;
    uint8_t xshutPins[MAX_SENSORS];
    VL53L1X sensors[MAX_SENSORS];
    SX1509 *ioExpander;
    uint8_t ioExpanderAddress;
    bool initialized;
    uint16_t lastDistances[MAX_SENSORS];
    
public:
    // Structure to hold distance data from all sensors
    struct TOFData {
        uint16_t distances[MAX_SENSORS];
        uint8_t sensorCount;
    };
    
    // Constructor
    TOFSensorArray(uint8_t numSensors = 1, 
                   uint8_t ioExpanderAddr = 0x3F);
    
    // Set XSHUT pins for each sensor
    void setXSHUTPins(const uint8_t *pins, uint8_t count);
    
    // Initialize the TOF sensor array
    bool begin();
    
    // Read distances from all sensors
    TOFData readDistances();
    
    // Print distance data to serial
    void printDistances(const TOFData& data);
    
    // Get single sensor distance
    uint16_t getDistance(uint8_t sensorIndex);
    
    // Check if sensor is initialized
    bool isInitialized() const;
    
    // Get number of sensors
    uint8_t getSensorCount() const;
};

#endif // TOF_SENSOR_ARRAY_H
