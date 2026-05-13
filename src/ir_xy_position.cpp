#include "ir_xy_position.h"

// Constructor
IRXYPosition::IRXYPosition(uint8_t sensorAddress)
    : slaveAddress(sensorAddress), initialized(false) {
    // Initialize arrays to zero
    for (int i = 0; i < 4; i++) {
        Ix[i] = 0;
        Iy[i] = 0;
    }
}

// Helper function to write 2 bytes to I2C
void IRXYPosition::write2bytes(byte d1, byte d2) {
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1);
    Wire.write(d2);
    Wire.endTransmission();
}

// Initialize the IR sensor
bool IRXYPosition::begin() {
    // Initialize IR sensor via I2C
    // Request IR data mode
    write2bytes(0x30, 0x01);  // Enable mode request
    delay(100);
    
    initialized = true;
    Serial.println("IR XY Position sensor initialized successfully");
    return true;
}

// Read raw IR position data
IRXYPosition::IRData IRXYPosition::readPositions() {
    IRData data;
    
    if (!initialized) {
        Serial.println("Error: IR sensor not initialized");
        // Return zero data
        for (int i = 0; i < 4; i++) {
            data.x[i] = 0;
            data.y[i] = 0;
        }
        return data;
    }
    
    // Request IR data from sensor
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x36);  // Request data
    Wire.endTransmission();
    
    // Read response
    Wire.requestFrom(slaveAddress, 16);  // Request 16 bytes of data
    
    if (Wire.available() >= 16) {
        for (int i = 0; i < 16; i++) {
            data_buf[i] = Wire.read();
        }
        
        // Parse the IR blob data
        // Byte structure: [x1_hi, x1_lo, y1_hi, y1_lo, x2_hi, x2_lo, y2_hi, y2_lo, ...]
        for (int i = 0; i < 4; i++) {
            int x_index = i * 4;
            int y_index = i * 4 + 2;
            
            // Combine high and low bytes
            Ix[i] = ((data_buf[x_index] & 0x03) << 8) | data_buf[x_index + 1];
            Iy[i] = ((data_buf[y_index] & 0x03) << 8) | data_buf[y_index + 1];
            
            data.x[i] = Ix[i];
            data.y[i] = Iy[i];
        }
    } else {
        // No data available
        for (int i = 0; i < 4; i++) {
            data.x[i] = 0;
            data.y[i] = 0;
        }
    }
    
    return data;
}

// Print IR data to serial
void IRXYPosition::printPositions(const IRData& data) {
    Serial.print("IR Positions: ");
    for (int i = 0; i < 4; i++) {
        if (data.x[i] != 0 || data.y[i] != 0) {
            Serial.print("Blob");
            Serial.print(i + 1);
            Serial.print("(");
            Serial.print(data.x[i]);
            Serial.print(",");
            Serial.print(data.y[i]);
            Serial.print(") ");
        }
    }
    Serial.println();
}

// Check if sensor is initialized
bool IRXYPosition::isInitialized() const {
    return initialized;
}
