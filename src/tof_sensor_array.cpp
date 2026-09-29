#include "tof_sensor_array.h"

// Constructor
TOFSensorArray::TOFSensorArray(uint8_t numSensors, uint8_t ioExpanderAddr)
    : sensorCount(numSensors),
      ioExpanderAddress(ioExpanderAddr),
      initialized(false) {

    // Initialize arrays
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        xshutPins[i] = 0xFF;
        lastDistances[i] = 0;
        distanceOffsets[i] = 0;

        historyCount[i] = 0;
        historyIndex[i] = 0;

        for (uint8_t j = 0; j < AVERAGE_SAMPLES; j++) {
            distanceHistory[i][j] = 0;
        }
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

    Serial.print("setXSHUTPins: ");

    for (uint8_t i = 0; i < copyCount; i++) {
        xshutPins[i] = pins[i];
        Serial.print(xshutPins[i]);

        if (i < copyCount - 1) {
            Serial.print(", ");
        }
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
        Serial.println(
            "Error: Failed to initialize IO Expander - "
            "check I2C connection and address"
        );
        return false;
    }

    Serial.println("IO Expander initialized successfully");

    // Set I2C clock speed
    Wire.setClock(400000);
    Serial.println("I2C clock set to 400 kHz");

    // Print configured XSHUT pin mapping
    Serial.print("Configured XSHUT pins: ");

    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print(xshutPins[i]);

        if (i < sensorCount - 1) {
            Serial.print(", ");
        }
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

    // Enable, initialize, and start each sensor one by one
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
            Serial.println(
                " is not configured (0xFF). "
                "Aborting initialization."
            );
            return false;
        }

        // Bring sensor out of reset
        ioExpander->digitalWrite(xshutPins[i], HIGH);

        // Give sensor time to boot and appear on I2C
        delay(120);

        // Initialize VL53L1X sensor
        sensors[i].setTimeout(1000);

        if (!sensors[i].init()) {
            Serial.print("Failed to detect and initialize sensor ");
            Serial.print(i);
            Serial.print(" on expander pin ");
            Serial.println(xshutPins[i]);

            // Diagnostic readback
            Serial.print("Expander pin ");
            Serial.print(xshutPins[i]);
            Serial.print(" readback = ");

            int pinState = ioExpander->digitalRead(xshutPins[i]);
            Serial.println(pinState);

            // I2C scan
            Serial.println("I2C scan: scanning addresses 0x01..0x7E...");

            for (uint8_t addr = 1; addr < 0x7F; addr++) {
                Wire.beginTransmission(addr);
                uint8_t err = Wire.endTransmission();

                if (err == 0) {
                    Serial.print("  Found device at 0x");

                    if (addr < 16) {
                        Serial.print("0");
                    }

                    Serial.println(addr, HEX);
                }
            }

            return false;
        }

        // Set unique I2C address for each VL53L1X sensor
        // Use 0x36..0x39 to avoid collisions with other devices
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

    Serial.print("Diagnose sensor ");
    Serial.println(index);

    // Ensure all XSHUT pins are low first
    for (uint8_t i = 0; i < sensorCount; i++) {
        ioExpander->digitalWrite(xshutPins[i], LOW);
    }

    delay(10);

    // Bring only this sensor out of reset
    ioExpander->digitalWrite(pin, HIGH);
    delay(200);

    Serial.print("Expander pin ");
    Serial.print(pin);
    Serial.print(" readback = ");

    int state = ioExpander->digitalRead(pin);
    Serial.println(state);

    Serial.println("I2C scan after enabling single XSHUT:");

    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();

        if (err == 0) {
            Serial.print("  Found device at 0x");

            if (addr < 16) {
                Serial.print("0");
            }

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

    // Read distance from each VL53L1X sensor
    for (uint8_t i = 0; i < sensorCount; i++) {
        uint16_t rawDistance = sensors[i].read();
        bool timeout = sensors[i].timeoutOccurred();

        if (timeout) {
            data.distances[i] = 0xFFFF; // Mark timeout
            lastDistances[i] = data.distances[i];
            continue;
        }

        int32_t correctedDistance = (int32_t)rawDistance + distanceOffsets[i];
        if (correctedDistance < 0) {
            correctedDistance = 0;
        }

        uint16_t corrected = (uint16_t)correctedDistance;

        // Store the new reading
        distanceHistory[i][historyIndex[i]] = corrected;

        historyIndex[i] = (historyIndex[i] + 1) % AVERAGE_SAMPLES;

        if (historyCount[i] < AVERAGE_SAMPLES) {
            historyCount[i]++;
        }

        // Calculate the average
        uint32_t sum = 0;

        for (uint8_t j = 0; j < historyCount[i]; j++) {
            sum += distanceHistory[i][j];
        }

        uint16_t averageDistance = sum / historyCount[i];

        data.distances[i] = averageDistance;
        lastDistances[i] = averageDistance;
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
void TOFSensorArray::setDistanceOffset(
    uint8_t sensorIndex,
    int16_t offsetMm
) {
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

// Check if sensor array is initialized
bool TOFSensorArray::isInitialized() const {
    return initialized;
}

// Get number of sensors
uint8_t TOFSensorArray::getSensorCount() const {
    return sensorCount;
}
