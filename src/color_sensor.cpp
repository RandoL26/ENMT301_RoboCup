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

// Decode color and print color name with hex value
void ColorSensor::decodeColor(const ColorData& data) {
    uint32_t hex = data.hexColor;
    uint8_t r = (hex >> 16) & 0xFF;
    uint8_t g = (hex >> 8) & 0xFF;
    uint8_t b = hex & 0xFF;
    
    const char* colorName = "Unknown";
    
    // Calculate color saturation and brightness
    uint8_t maxVal = (r > g) ? (r > b ? r : b) : (g > b ? g : b);
    uint8_t minVal = (r < g) ? (r < b ? r : b) : (g < b ? g : b);
    uint8_t saturation = (maxVal > 0) ? ((maxVal - minVal) * 255) / maxVal : 0;
    uint8_t brightness = maxVal;
    
    // Check for white (all channels high and similar)
    if (brightness > 200 && saturation < 30) {
        colorName = "WHITE";
    }
    // Check for black (all channels low)
    else if (brightness < 50) {
        colorName = "BLACK";
    }
    // Check for gray (desaturated, moderate brightness)
    else if (saturation < 50 && brightness > 50 && brightness < 200) {
        colorName = "GRAY";
    }
    // Red dominant (R > G and R > B by significant margin)
    else if (r > g && r > b && (r - g) > 30 && (r - b) > 30 && brightness > 100) {
        colorName = "RED";
    }
    // Green dominant
    else if (g > r && g > b && (g - r) > 30 && (g - b) > 30 && brightness > 100) {
        colorName = "GREEN";
    }
    // Blue dominant
    else if (b > r && b > g && (b - r) > 30 && (b - g) > 30 && brightness > 100) {
        colorName = "BLUE";
    }
    // Yellow (R + G, low B)
    else if (r > 100 && g > 100 && b < 100 && abs(r - g) < 50) {
        colorName = "YELLOW";
    }
    // Cyan (G + B, low R)
    else if (g > 100 && b > 100 && r < 100 && abs(g - b) < 50) {
        colorName = "CYAN";
    }
    // Magenta (R + B, low G)
    else if (r > 100 && b > 100 && g < 100 && abs(r - b) < 50) {
        colorName = "MAGENTA";
    }
    // Orange (R high, G medium, B low)
    else if (r > 150 && g > 80 && g < 150 && b < 80) {
        colorName = "ORANGE";
    }
    // Brown/Tan (R and G similar, B lower)
    else if (r > 70 && g > 70 && abs(r - g) < 40 && b < 100 && brightness > 80 && saturation < 70) {
        colorName = "BROWN";
    }
    // Pink (R high, G medium, B medium-high)
    else if (r > 150 && g > 80 && b > 80 && (r - g) > 30) {
        colorName = "PINK";
    }
    // Purple (R medium, B high, G low)
    else if (b > 100 && r > 80 && g < 100 && (b - g) > 40) {
        colorName = "PURPLE";
    }
    
    // Print color name and hex value
    Serial.print("Color: ");
    Serial.print(colorName);
    Serial.print(" | HEX: #");
    
    // Print hex with leading zeros
    if (r < 16) Serial.print("0");
    Serial.print(r, HEX);
    if (g < 16) Serial.print("0");
    Serial.print(g, HEX);
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    
    Serial.print(" | RGB(");
    Serial.print(r);
    Serial.print(",");
    Serial.print(g);
    Serial.print(",");
    Serial.print(b);
    Serial.println(")");
}

// Check if sensor is initialized
bool ColorSensor::isInitialized() const {
    return initialized;
}
