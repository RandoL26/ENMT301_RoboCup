#include "ch9143_bluetooth.h"
#include <stdarg.h>
#include <stdio.h>

// Constructor
CH9143Bluetooth::CH9143Bluetooth(HardwareSerial* serialPort, uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
    : serial(serialPort), rxPin(rxPin), txPin(txPin), baudRate(baudRate), initialized(false) {
}

// Destructor
CH9143Bluetooth::~CH9143Bluetooth() {
    // No specific cleanup needed for HardwareSerial
}

// Initialize Bluetooth connection
bool CH9143Bluetooth::begin() {
    if (serial == nullptr) {
        Serial.println("ERROR: CH9143 serial port not initialized");
        return false;
    }
    
    // Begin serial communication with specified baud rate
    // Note: Teensy pins are fixed for each serial port (Serial1 uses pins 0/1, etc)
    serial->begin(baudRate);
    delay(100);  // Give serial time to initialize
    
    initialized = true;
    
    // Test message
    Serial.println("CH9143 Bluetooth initialized successfully");
    serial->println("TEST: Bluetooth Online");
    serial->flush();
    
    return true;
}

// Send string to Bluetooth
void CH9143Bluetooth::print(const char* data) {
    if (initialized && serial != nullptr) {
        serial->print(data);
        serial->flush();
    }
}

void CH9143Bluetooth::println(const char* data) {
    if (initialized && serial != nullptr) {
        serial->println(data);
        serial->flush();
    }
}

// Send integer to Bluetooth
void CH9143Bluetooth::print(int value) {
    if (initialized && serial != nullptr) {
        serial->print(value);
        serial->flush();
    }
}

void CH9143Bluetooth::println(int value) {
    if (initialized && serial != nullptr) {
        serial->println(value);
        serial->flush();
    }
}

// Send float to Bluetooth
void CH9143Bluetooth::print(float value, int decimals) {
    if (initialized && serial != nullptr) {
        serial->print(value, decimals);
        serial->flush();
    }
}

void CH9143Bluetooth::println(float value, int decimals) {
    if (initialized && serial != nullptr) {
        serial->println(value, decimals);
        serial->flush();
    }
}

// Send formatted string to Bluetooth (similar to sprintf)
void CH9143Bluetooth::printf(const char* format, ...) {
    if (!initialized || serial == nullptr) {
        return;
    }
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    serial->print(buffer);
    serial->flush();
}

// Check if data is available to read from Bluetooth
bool CH9143Bluetooth::available() {
    if (initialized && serial != nullptr) {
        return serial->available();
    }
    return false;
}

// Read a single character from Bluetooth
char CH9143Bluetooth::read() {
    if (initialized && serial != nullptr && serial->available()) {
        return serial->read();
    }
    return '\0';
}

// Flush the receive buffer
void CH9143Bluetooth::flush() {
    if (initialized && serial != nullptr) {
        while (serial->available()) {
            serial->read();
        }
    }
}

// Check if initialized
bool CH9143Bluetooth::isInitialized() const {
    return initialized;
}

// Send raw byte
void CH9143Bluetooth::write(uint8_t byte) {
    if (initialized && serial != nullptr) {
        serial->write(byte);
    }
}

// Send raw buffer
void CH9143Bluetooth::write(const uint8_t* buffer, size_t length) {
    if (initialized && serial != nullptr) {
        serial->write(buffer, length);
    }
}
