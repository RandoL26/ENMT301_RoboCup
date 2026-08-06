#include "tof_sensor_array.h"

// Constructor
TOFSensorArray::TOFSensorArray(uint8_t numSensors, uint8_t ioExpanderAddr)
    : sensorCount(numSensors), ioExpanderAddress(ioExpanderAddr), initialized(false) {
    
    // Initialize arrays
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        xshutPins[i] = 0xFF; // invalid marker
        lastDistances[i] = 0;
        distanceOffsets[i] = 0;
    }
    
    // Create IO expander instance
    ioExpander = new SX1509();
}

// Set XSHUT pins for each sensor
void TOFSensorArray::setXSHUTPins(const uint8_t *pins, uint8_t count) {
    if (count < sensorCount) {
        Serial.println("Error: not enough XSHUT pins configured for TOF sensors");
        Serial.print("  Expected: ");
        Serial.print(sensorCount);
        Serial.print(", got: ");
        Serial.println(count);
    }

    uint8_t copyCount = (count < sensorCount) ? count : sensorCount;
    for (uint8_t i = 0; i < copyCount; i++) {
        xshutPins[i] = pins[i];
    }
}

// Initialize the TOF sensor array
bool TOFSensorArray::begin() {
    if (sensorCount == 0 || sensorCount > MAX_SENSORS) {
        Serial.println("Error: Invalid sensor count");
        return false;
    }
    
    Serial.print("Attempting to initialize IO Expander at address 0x");
    Serial.println(ioExpanderAddress, HEX);
    
    // Initialize IO Expander
    if (!ioExpander->begin(ioExpanderAddress)) {
        Serial.println("Error: Failed to initialize IO Expander - check I2C connection and address");
        return false;
    }
    
    Serial.println("IO Expander initialized successfully");
    
    // Set I2C clock speed
    Wire.setClock(400000);  // use 400 kHz I2C
    Serial.println("I2C clock set to 400 kHz");
    
    // Disable/reset all sensors by driving their XSHUT pins low
    Serial.println("Resetting all TOF sensors (XSHUT LOW)...");
    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print("  Setting pin ");
        Serial.print(xshutPins[i]);
        Serial.println(" to OUTPUT");
        ioExpander->pinMode(xshutPins[i], OUTPUT);
        ioExpander->digitalWrite(xshutPins[i], LOW);
    }
    
    delay(10);
    
    // Enable, initialize, and start each sensor, one by one
    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print("Initializing sensor ");
        Serial.print(i);
        Serial.print(" (XSHUT on pin ");
        Serial.print(xshutPins[i]);
        Serial.println(")...");
        
        // Stop driving this sensor's XSHUT low (bring sensor out of reset)
        ioExpander->digitalWrite(xshutPins[i], HIGH);
        delay(50);

        // Initialize sensor
        sensors[i].setTimeout(500);
        if (!sensors[i].init()) {
            Serial.print("Failed to detect and initialize sensor ");
            Serial.print(i);
            Serial.print(" on expander pin ");
            Serial.println(xshutPins[i]);
            return false;
        }
        // Set unique I2C address for each sensor
        // Use 0x36..0x39 to avoid colliding with other devices (e.g., 0x33 Matrix Lidar)
        sensors[i].setAddress(0x36 + i);
        sensors[i].startContinuous(50);
        
        Serial.print("TOF Sensor ");
        Serial.print(i);
        Serial.println(" initialized successfully");
    }
    
    initialized = true;
    return true;
}

// Read distances from all sensors
TOFSensorArray::TOFData TOFSensorArray::readDistances() {
    TOFData data;
    data.sensorCount = sensorCount;
    
    if (!initialized) {
        Serial.println("Error: TOF sensor array not initialized");
        for (uint8_t i = 0; i < MAX_SENSORS; i++) {
            data.distances[i] = 0;
        }
        return data;
    }
    
    // Read distance from each sensor
    for (uint8_t i = 0; i < sensorCount; i++) {
        uint16_t rawDistance = sensors[i].read();
        if (sensors[i].timeoutOccurred()) {
            data.distances[i] = 0xFFFF;  // Mark timeout with max value
            lastDistances[i] = data.distances[i];
            continue;
        }

        int32_t correctedDistance = (int32_t)rawDistance + distanceOffsets[i];
        if (correctedDistance < 0) {
            correctedDistance = 0;
        }

        data.distances[i] = (uint16_t)correctedDistance;
        lastDistances[i] = data.distances[i];
    }
    
    return data;
}

// Print distance data to serial
void TOFSensorArray::printDistances(const TOFData& data) {
    for (uint8_t i = 0; i < data.sensorCount; i++) {
        Serial.print("S");
        Serial.print(i);
        Serial.print(": ");
        
        if (data.distances[i] == 0xFFFF) {
            Serial.print("TIMEOUT");
        } else {
            Serial.print(data.distances[i]);
            Serial.print(" mm");
        }
        
        if (i < data.sensorCount - 1) {
            Serial.print("\t");
        }
    }
    Serial.println();
}

// Set a per-sensor distance offset (mm)
void TOFSensorArray::setDistanceOffset(uint8_t sensorIndex, int16_t offsetMm) {
    if (sensorIndex < sensorCount) {
        distanceOffsets[sensorIndex] = offsetMm;
    }
}

// Get single sensor distance
uint16_t TOFSensorArray::getDistance(uint8_t sensorIndex) {
    if (sensorIndex < sensorCount) {
        return lastDistances[sensorIndex];
    }
    return 0;
}

// Check if sensor is initialized
bool TOFSensorArray::isInitialized() const {
    return initialized;
}

// Get number of sensors
uint8_t TOFSensorArray::getSensorCount() const {
    return sensorCount;
}
