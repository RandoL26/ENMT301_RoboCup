#include "tof_sensor_array.h"

// Constructor
TOFSensorArray::TOFSensorArray(uint8_t numSensors, uint8_t ioExpanderAddr)
    : sensorCount(numSensors), ioExpanderAddress(ioExpanderAddr), initialized(false),
      topSensor(nullptr), topSensorXshutExpanderPin(0xFF), topSensorDistance(0), topSensorInitialized(false) {
    
    // Initialize arrays
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        xshutPins[i] = 0xFF; // invalid marker
        lastDistances[i] = 0;
        distanceOffsets[i] = 0;
    }
    
    // Create IO expander instance
    ioExpander = new SX1509();
    
    // Create VL53L0X instance for top sensor
    topSensor = new VL53L0X();
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
    Serial.print("setXSHUTPins: ");
    for (uint8_t i = 0; i < copyCount; i++) {
        xshutPins[i] = pins[i];
        Serial.print(xshutPins[i]);
        if (i < copyCount - 1) Serial.print(", ");
    }
    Serial.println();
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
    
    // Print configured XSHUT pin mapping
    Serial.print("Configured XSHUT pins: ");
    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print(xshutPins[i]);
        if (i < sensorCount - 1) Serial.print(", ");
    }
    Serial.println();

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
    
    auto isTopSensorPin = [this](uint8_t pin) {
        return topSensorXshutExpanderPin != 0xFF && pin == topSensorXshutExpanderPin;
    };

    // Enable, initialize, and start each sensor, one by one
    uint8_t l1xIndex = 0;
    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print("Initializing sensor ");
        Serial.print(i);
        Serial.print(" (XSHUT on pin ");
        Serial.print(xshutPins[i]);
        Serial.println(")...");
        // Validate XSHUT pin
        if (xshutPins[i] == 0xFF) {
            Serial.print("Error: XSHUT pin for sensor ");
            Serial.print(i);
            Serial.println(" is not configured (0xFF). Aborting initialization.");
            return false;
        }

        // Stop driving this sensor's XSHUT low (bring sensor out of reset)
        ioExpander->digitalWrite(xshutPins[i], HIGH);
        // Give the sensor extra time to boot and appear on I2C
        delay(120);

        if (isTopSensorPin(xshutPins[i])) {
            // Initialize the top VL53L0X sensor
            topSensor->setTimeout(1000);
            if (!topSensor->init()) {
                Serial.print("Failed to detect and initialize top VL53L0X on expander pin ");
                Serial.println(xshutPins[i]);
                return false;
            }
            topSensor->startContinuous(50);
            topSensorInitialized = true;
            Serial.println("Top VL53L0X initialized successfully");
            continue;
        }

        // Initialize VL53L1X sensor
        sensors[i].setTimeout(1000);
        if (!sensors[i].init()) {
            Serial.print("Failed to detect and initialize sensor ");
            Serial.print(i);
            Serial.print(" on expander pin ");
            Serial.println(xshutPins[i]);
            // Diagnostic readback: check expander pin state
            Serial.print("Expander pin ");
            Serial.print(xshutPins[i]);
            Serial.print(" readback = ");
            int pinState = ioExpander->digitalRead(xshutPins[i]);
            Serial.println(pinState);

            // Do a quick I2C scan to show which addresses respond on the bus
            Serial.println("I2C scan: scanning addresses 0x01..0x7E...");
            for (uint8_t addr = 1; addr < 0x7F; addr++) {
                Wire.beginTransmission(addr);
                uint8_t err = Wire.endTransmission();
                if (err == 0) {
                    Serial.print("  Found device at 0x");
                    if (addr < 16) Serial.print("0");
                    Serial.println(addr, HEX);
                }
            }

            return false;
        }
        // Set unique I2C address for each VL53L1X sensor
        // Use 0x36..0x39 to avoid colliding with other devices (e.g., 0x33 Matrix Lidar)
        sensors[i].setAddress(0x36 + l1xIndex);
        sensors[i].startContinuous(50);
        l1xIndex++;
        
        Serial.print("TOF Sensor ");
        Serial.print(i);
        Serial.println(" initialized successfully");
    }
    
    initialized = true;
    return true;
}

// Diagnostic helper: enable single XSHUT and scan I2C
void TOFSensorArray::diagnoseSensorByIndex(uint8_t index) {
    if (index >= sensorCount) {
        Serial.print("diagnoseSensorByIndex: invalid index ");
        Serial.println(index);
        return;
    }

    uint8_t pin = xshutPins[index];
    if (pin == 0xFF) {
        Serial.print("diagnoseSensorByIndex: XSHUT for index ");
        Serial.print(index);
        Serial.println(" not configured");
        return;
    }

    Serial.print("Diagnose sensor "); Serial.println(index);

    // Ensure all XSHUT low first
    for (uint8_t i = 0; i < sensorCount; i++) {
        ioExpander->digitalWrite(xshutPins[i], LOW);
    }
    delay(10);

    // Bring only this sensor out of reset
    ioExpander->digitalWrite(pin, HIGH);
    delay(200);

    Serial.print("Expander pin "); Serial.print(pin); Serial.print(" readback = ");
    int state = ioExpander->digitalRead(pin);
    Serial.println(state);

    Serial.println("I2C scan after enabling single XSHUT:");
    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("  Found device at 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
        }
    }

    // Return pin to LOW
    ioExpander->digitalWrite(pin, LOW);
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
        uint16_t rawDistance = 0;
        bool timeout = false;

        if (topSensorInitialized && xshutPins[i] == topSensorXshutExpanderPin) {
            rawDistance = topSensor->readRangeContinuousMillimeters();
            timeout = topSensor->timeoutOccurred();
        } else {
            rawDistance = sensors[i].read();
            timeout = sensors[i].timeoutOccurred();
        }

        if (timeout) {
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

// Top sensor: set XSHUT expander pin
void TOFSensorArray::setTopSensorXshutPin(uint8_t expanderPin) {
    topSensorXshutExpanderPin = expanderPin;
}

// Top sensor: initialize VL53L0X
bool TOFSensorArray::initializeTopSensor() {
    if (topSensorInitialized) {
        return true; // already initialized by begin()
    }

    if (topSensorXshutExpanderPin == 0xFF || topSensor == nullptr || ioExpander == nullptr) {
        Serial.println("Error: Top sensor XSHUT pin not configured or expander not available");
        return false;
    }

    Serial.print("Initializing top VL53L0X on expander pin ");
    Serial.println(topSensorXshutExpanderPin);

    // Hold in reset
    ioExpander->pinMode(topSensorXshutExpanderPin, OUTPUT);
    ioExpander->digitalWrite(topSensorXshutExpanderPin, LOW);
    delay(10);

    // Release from reset
    ioExpander->digitalWrite(topSensorXshutExpanderPin, HIGH);
    delay(100);

    // Initialize sensor
    topSensor->setTimeout(500);
    if (!topSensor->init()) {
        Serial.println("Failed to initialize top VL53L0X sensor");
        return false;
    }

    topSensor->startContinuous(50);
    topSensorInitialized = true;
    Serial.println("Top VL53L0X sensor initialized successfully");
    return true;
}

// Top sensor: read distance
uint16_t TOFSensorArray::readTopSensorDistance() {
    if (topSensorInitialized && topSensor != nullptr) {
        topSensorDistance = topSensor->readRangeContinuousMillimeters();
        return topSensorDistance;
    }
    return 0;
}

// Top sensor: check if initialized
bool TOFSensorArray::isTopSensorInitialized() const {
    return topSensorInitialized;
}

bool TOFSensorArray::isTopSensorInArray() const {
    if (topSensorXshutExpanderPin == 0xFF) {
        return false;
    }
    for (uint8_t i = 0; i < sensorCount; i++) {
        if (xshutPins[i] == topSensorXshutExpanderPin) {
            return true;
        }
    }
    return false;
}
