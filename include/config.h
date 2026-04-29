#ifndef CONFIG_H
#define CONFIG_H

// Teensy 4.0 Configuration
#define TEENSY_40 1
#define CPU_FREQ_MHZ 600
#define BAUD_RATE 115200

// Pin Definitions
#define LED_PIN LED_BUILTIN
#define SERIAL_TX 1
#define SERIAL_RX 0

// Debug Settings
#define DEBUG 1
#define DEBUG_SERIAL Serial

// Debug macros
#if DEBUG
    #define DEBUG_BEGIN() Serial.begin(BAUD_RATE)
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_BEGIN()
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(fmt, ...)
#endif

#endif
