#include "color_sensor.h"

// Constructor
ColorSensor::ColorSensor(uint8_t integrationTime, uint8_t gain)
    : integrationTime(integrationTime), gain(gain), initialized(false) {
    tcs = new Adafruit_TCS34725(integrationTime, (tcs34725Gain_t)gain);
}

// Destructor
ColorSensor::~ColorSensor() {
    if (tcs != nullptr) {
        delete tcs;
        tcs = nullptr;
    }
}

// Initialize the color sensor
bool ColorSensor::begin() {
    if (tcs == nullptr) {
        Serial.println("Error: Color sensor object not initialized");
        return false;
    }
    
    if (tcs->begin()) {
        initialized = true;
        Serial.println("Color sensor initialized successfully");
        return true;
    } else {
        Serial.println("No TCS34725 found... check your connections");
        initialized = false;
        return false;
    }
}

// Read raw color data and calculate hex color
ColorSensor::ColorData ColorSensor::readColor() {
    ColorData data = {0, 0, 0, 0, 0};
    
    if (!initialized || tcs == nullptr) {
        Serial.println("Error: Color sensor not initialized");
        return data;
    }
    
    // Turn LED on for measurement
    tcs->setInterrupt(false);
    delay(60);  // takes 50ms to read
    
    // Get raw color data
    tcs->getRawData(&data.red, &data.green, &data.blue, &data.clear);
    
    // Turn LED off
    tcs->setInterrupt(true);
    
    // Calculate normalized hex color
    if (data.clear > 0) {
        float r = data.red;
        float g = data.green;
        float b = data.blue;
        
        r /= data.clear;
        g /= data.clear;
        b /= data.clear;
        
        r *= 256;
        g *= 256;
        b *= 256;
        
        // Constrain values to 0-255
        r = constrain(r, 0, 255);
        g = constrain(g, 0, 255);
        b = constrain(b, 0, 255);
        
        // Pack into hex color
        data.hexColor = ((uint32_t)(int)r << 16) | ((uint32_t)(int)g << 8) | (uint32_t)(int)b;
    }
    
    return data;
}

// Turn LED on/off
void ColorSensor::setLED(bool on) {
    if (initialized && tcs != nullptr) {
        tcs->setInterrupt(!on);  // Note: interrupt true = LED off
    }
}

// Print color data to serial
void ColorSensor::printColorData(const ColorData& data) {
    Serial.print("C:\t"); Serial.print(data.clear);
    Serial.print("\tR:\t"); Serial.print(data.red);
    Serial.print("\tG:\t"); Serial.print(data.green);
    Serial.print("\tB:\t"); Serial.print(data.blue);
    Serial.print("\t");
    Serial.print((data.hexColor >> 16) & 0xFF, HEX);
    Serial.print((data.hexColor >> 8) & 0xFF, HEX);
    Serial.print(data.hexColor & 0xFF, HEX);
    Serial.println();
}

// Check if sensor is initialized
bool ColorSensor::isInitialized() const {
    return initialized;
}
