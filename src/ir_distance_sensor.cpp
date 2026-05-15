#include "ir_distance_sensor.h"

// Constructor
IRDistanceSensor::IRDistanceSensor(uint8_t pin)
    : analogPin(pin), lastDistance(0), initialized(false), calibrationConstant(8361.0) {
}

// Initialize the IR distance sensor
bool IRDistanceSensor::begin() {
    // Set analog pin as input
    pinMode(analogPin, INPUT);
    
    initialized = true;
    Serial.println("IR Distance Sensor (2Y0A02) initialized successfully");
    return true;
}

// Read distance from sensor (returns distance in cm)
float IRDistanceSensor::readDistance() {
    if (!initialized) {
        Serial.println("Error: IR distance sensor not initialized");
        return 0;
    }
    
    // Read analog value (0-1023 for 10-bit ADC)
    int rawValue = analogRead(analogPin);
    
    // Convert ADC reading to distance
    // 2Y0A02 characteristic: distance (cm) = 27 / voltage (V)
    // voltage = ADC_value / 1023 * 3.3 (for Teensy at 3.3V)
    // So: distance = 27 / (ADC_value / 1023 * 3.3) = (27 * 1023) / (ADC_value * 3.3)
    // Simplified: distance = 8361 / ADC_value (in cm)
    
    if (rawValue > 0) {
        // Direct conversion using calibrated constant
        lastDistance = calibrationConstant / rawValue;
        
        Serial.print("DEBUG IR ADC: ");
        Serial.print(rawValue);
        Serial.print(" | Calculated Distance: ");
        Serial.print(lastDistance);
        Serial.println(" cm");
        
        // NO CLAMPING - let's see the actual values
        // Clamp distance to reasonable values (sensor range ~10-80cm for 2Y0A02)
        // if (lastDistance < 5) lastDistance = 5;
        // if (lastDistance > 150) lastDistance = 150;
    } else {
        lastDistance = 0;
    }
    
    return lastDistance;
}

// Get the last measured distance
float IRDistanceSensor::getLastDistance() const {
    return lastDistance;
}

// Print distance to serial
void IRDistanceSensor::printDistance() {
    Serial.print("IR Distance: ");
    Serial.print(lastDistance);
    Serial.println(" cm");
}

// Check if sensor is initialized
bool IRDistanceSensor::isInitialized() const {
    return initialized;
}

// Set calibration constant (use this to fine-tune readings)
// Distance = calibrationConstant / ADC_value
void IRDistanceSensor::setCalibrationConstant(float constant) {
    calibrationConstant = constant;
    Serial.print("IR calibration constant set to: ");
    Serial.println(calibrationConstant);
}
