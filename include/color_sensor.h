#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <Adafruit_TCS34725.h>

class ColorSensor {
private:
    Adafruit_TCS34725 *tcs;
    bool initialized;
    uint8_t integrationTime;
    uint8_t gain;
    
public:
    // Structure to hold color data
    struct ColorData {
        uint16_t clear;
        uint16_t red;
        uint16_t green;
        uint16_t blue;
        uint32_t hexColor;
    };
    
    // Constructor with default integration time and gain
    // integrationTime: use TCS34725_INTEGRATIONTIME_50MS (0xF6) by default
    // gain: use TCS34725_GAIN_4X (0x01) by default
    ColorSensor(uint8_t integrationTime = 0xF6,  // TCS34725_INTEGRATIONTIME_50MS
                uint8_t gain = 0x01);            // TCS34725_GAIN_4X
    
    // Destructor
    ~ColorSensor();
    
    // Initialize the color sensor
    bool begin();
    
    // Read raw color data
    ColorData readColor();
    
    // Turn LED on/off
    void setLED(bool on);
    
    // Print color data to serial
    void printColorData(const ColorData& data);
    
    // Check if sensor is initialized
    bool isInitialized() const;
};

#endif // COLOR_SENSOR_H
