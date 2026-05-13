#ifndef IR_XY_POSITION_H
#define IR_XY_POSITION_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

class IRXYPosition {
private:
    uint8_t slaveAddress;  // I2C slave address (0x21 for IR sensor at 0xB0)
    int Ix[4];             // X positions for 4 IR blobs
    int Iy[4];             // Y positions for 4 IR blobs
    byte data_buf[16];     // Raw data buffer
    bool initialized;
    
    // Helper function to write 2 bytes to I2C
    void write2bytes(byte d1, byte d2);
    
public:
    // Structure to hold IR blob position data
    struct IRData {
        int x[4];  // X positions for up to 4 blobs
        int y[4];  // Y positions for up to 4 blobs
    };
    
    // Constructor with default IR sensor address
    IRXYPosition(uint8_t sensorAddress = 0x21);  // Default is 0xB0 >> 1
    
    // Initialize the IR sensor
    bool begin();
    
    // Read raw IR position data
    IRData readPositions();
    
    // Print IR data to serial
    void printPositions(const IRData& data);
    
    // Check if sensor is initialized
    bool isInitialized() const;
};

#endif // IR_XY_POSITION_H
