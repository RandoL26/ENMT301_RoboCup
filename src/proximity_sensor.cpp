#include "proximity_sensor.h"

// Constructor
ProximitySensor::ProximitySensor(int pin)
    : sensorPin(pin), 
      objectDetected(false), 
      lastDebounceTime(0),
      debounceDelay(50),  // 50ms debounce delay
      lastSensorState(LOW) {  // Start as LOW to match pull-up HIGH state
}

// Initialize the sensor
void ProximitySensor::begin() {
    pinMode(sensorPin, INPUT_PULLUP);  // Pull-up keeps pin HIGH when idle, sensor pulls LOW when object detected
    Serial.println("Proximity Sensor initialized on pin " + String(sensorPin));
}

// Read and update sensor value with debouncing
void ProximitySensor::update() {
    int currentSensorState = digitalRead(sensorPin);
    
    // Debouncing logic
    if (currentSensorState != lastSensorState) {
        lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
        objectDetected = (currentSensorState == HIGH);  // HIGH = object detected
    }
    
    lastSensorState = currentSensorState;
}

// Get current detection state
bool ProximitySensor::isObjectDetected() const {
    return objectDetected;
}

// Get raw sensor reading
int ProximitySensor::getRawReading() const {
    return digitalRead(sensorPin);
}

// Print sensor status to serial
void ProximitySensor::printStatus() const {
    Serial.print("Proximity Sensor (Raw: ");
    Serial.print(getRawReading());
    Serial.print("): ");
    if (!(objectDetected)) {
        Serial.println("OBJECT DETECTED");
    } else {
        Serial.println("No object");
    }
}
