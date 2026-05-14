#include "ultrasonic_sensor_array.h"

// Constructor
UltrasonicSensorArray::UltrasonicSensorArray(uint8_t count)
    : sensorCount(count), initialized(false) {
    sensors = new UltrasonicSensor*[sensorCount];
    
    // Initialize all pointers to null
    for (uint8_t i = 0; i < sensorCount; i++) {
        sensors[i] = nullptr;
    }
}

// Destructor
UltrasonicSensorArray::~UltrasonicSensorArray() {
    if (sensors != nullptr) {
        for (uint8_t i = 0; i < sensorCount; i++) {
            if (sensors[i] != nullptr) {
                delete sensors[i];
                sensors[i] = nullptr;
            }
        }
        delete[] sensors;
        sensors = nullptr;
    }
}

// Add a sensor to the array
void UltrasonicSensorArray::addSensor(uint8_t index, uint8_t triggerPin, uint8_t echoPin) {
    if (index >= sensorCount) {
        Serial.println("ERROR: Sensor index out of range");
        return;
    }
    
    if (sensors[index] != nullptr) {
        delete sensors[index];
    }
    
    sensors[index] = new UltrasonicSensor(triggerPin, echoPin);
}

// Initialize all sensors
bool UltrasonicSensorArray::begin() {
    for (uint8_t i = 0; i < sensorCount; i++) {
        if (sensors[i] != nullptr) {
            sensors[i]->begin();
        }
    }
    
    initialized = true;
    return true;
}

// Read all sensors
UltrasonicSensorArray::UltrasonicData UltrasonicSensorArray::readDistances() {
    UltrasonicData data;
    data.distances = new double[sensorCount];
    data.count = sensorCount;
    
    if (!initialized) {
        Serial.println("ERROR: Ultrasonic sensor array not initialized");
        for (uint8_t i = 0; i < sensorCount; i++) {
            data.distances[i] = -1.0;
        }
        return data;
    }
    
    for (uint8_t i = 0; i < sensorCount; i++) {
        if (sensors[i] != nullptr) {
            sensors[i]->update();
            data.distances[i] = sensors[i]->getDistanceCm();
        } else {
            data.distances[i] = -1.0;
        }
    }
    
    return data;
}

// Get distance from specific sensor
double UltrasonicSensorArray::getDistance(uint8_t index) const {
    if (index >= sensorCount || sensors[index] == nullptr) {
        return -1.0;
    }
    return sensors[index]->getDistanceCm();
}

// Print all sensor data on one line
void UltrasonicSensorArray::printDistances(const UltrasonicData& data) const {
    Serial.print("US: ");
    for (uint8_t i = 0; i < data.count; i++) {
        if (i > 0) Serial.print(" | ");
        Serial.print("S");
        Serial.print(i);
        Serial.print(": ");
        if (data.distances[i] < 0) {
            Serial.print("ERR");
        } else {
            Serial.print(data.distances[i], 1);
            Serial.print("cm");
        }
    }
    Serial.println();
}

// Print visual bars for all sensors
void UltrasonicSensorArray::printVisualBars() const {
    for (uint8_t i = 0; i < sensorCount; i++) {
        if (sensors[i] != nullptr) {
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(": ");
            sensors[i]->printVisualBar();
        }
    }
}

// Check if initialized
bool UltrasonicSensorArray::isInitialized() const {
    return initialized;
}

// Get sensor count
uint8_t UltrasonicSensorArray::getSensorCount() const {
    return sensorCount;
}

// Update a specific sensor
void UltrasonicSensorArray::updateSensor(uint8_t index) {
    if (index < sensorCount && sensors[index] != nullptr) {
        sensors[index]->update();
    }
}

// Update all sensors
void UltrasonicSensorArray::updateAll() {
    for (uint8_t i = 0; i < sensorCount; i++) {
        if (sensors[i] != nullptr) {
            sensors[i]->update();
        }
    }
}
