#ifndef CH9143_BLUETOOTH_H
#define CH9143_BLUETOOTH_H

#include <Arduino.h>
#include <HardwareSerial.h>

class CH9143Bluetooth {
private:
    HardwareSerial* serial;
    bool initialized;
    uint32_t baudRate;
    uint8_t rxPin;
    uint8_t txPin;
    
public:
    // Constructor
    // serialPort: pointer to HardwareSerial object (e.g., &Serial1)
    // rxPin: RX pin number
    // txPin: TX pin number
    // baudRate: baud rate (default 9600 for CH9143)
    CH9143Bluetooth(HardwareSerial* serialPort, uint8_t rxPin, uint8_t txPin, uint32_t baudRate = 9600);
    
    // Destructor
    ~CH9143Bluetooth();
    
    // Initialize Bluetooth connection
    bool begin();
    
    // Send data to Bluetooth
    void print(const char* data);
    void println(const char* data);
    void print(int value);
    void println(int value);
    void print(float value, int decimals = 2);
    void println(float value, int decimals = 2);
    
    // Send formatted string (similar to Serial.print)
    void printf(const char* format, ...);
    
    // Read data from Bluetooth (if available)
    bool available();
    char read();
    
    // Clear receive buffer
    void flush();
    
    // Check if initialized
    bool isInitialized() const;
    
    // Send raw byte
    void write(uint8_t byte);
    void write(const uint8_t* buffer, size_t length);
};

#endif // CH9143_BLUETOOTH_H
