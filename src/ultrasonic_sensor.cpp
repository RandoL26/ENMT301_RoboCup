#include "ultrasonic_sensor.h"

// Constructor
UltrasonicSensor::UltrasonicSensor(uint8_t trigger, uint8_t echo)
    : triggerPin(trigger), echoPin(echo), lastDistance(0) {
}

// Initialize the sensor
void UltrasonicSensor::begin() {
    // Set pin modes - be explicit
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
    
    pinMode(echoPin, INPUT);
    
    Serial.println("Ultrasonic Sensor (HC-SR04) initialized");
    Serial.print("Trigger Pin: ");
    Serial.print(triggerPin);
    Serial.print(" (D");
    Serial.print(triggerPin);
    Serial.print("Z), Echo Pin: ");
    Serial.print(echoPin);
    Serial.println(" (D3Z)");
    
    Serial.print("Echo pin initial state: ");
    Serial.println(digitalRead(echoPin));
}

// Read sensor value - using simple timing method instead of HCSR04 library
void UltrasonicSensor::update() {

    // Ensure trigger is LOW for 2us
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    
    // Send 10 microsecond pulse
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    
    // Wait for echo to settle (sensor needs time to respond)
    delayMicroseconds(100);
    
    
        // Measure pulse duration on echo pin
        // pulseIn will wait for pin to go HIGH, then measure time until it goes LOW
        long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
        
        // Calculate distance: distance = (duration * 0.0343) / 2
        if (duration > 0) {
            lastDistance = (duration * 0.0343) / 2.0;
        } else {
            lastDistance = 0;
        }
    
    
    // Delay before next measurement (HC-SR04 needs ~60ms between measurements)
    delay(75);
}

// Get distance in cm
double UltrasonicSensor::getDistanceCm() const {
    return lastDistance;
}

// Print sensor status with visual representation
void UltrasonicSensor::printStatus() const {
    Serial.print("Distance: ");
    Serial.print(lastDistance, 1);
    Serial.print(" cm  ");
    Serial.println();
}
