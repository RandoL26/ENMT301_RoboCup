/*
 * Module wrapper for DFRobot Matrix Lidar 8x8 demo
 * Adapted from the original example to integrate with the RoboCup
 * template/task scheduler. This file provides a minimal module-style
 * API: `TOF_X8_begin()`, `TOF_X8_isInitialized()`, `TOF_X8_readAll()` and
 * `TOF_X8_print()` along with a `TOF_X8_task_callback()` suitable for
 * registering as a Task in the scheduler.
 */

#include "DFRobot_MatrixLidar.h"
#include <Wire.h>

// If the robocup template provides these helpers, declare them so we can
// print to both Serial and Bluetooth. If not present at link time, the
// declarations are harmless (they must be defined elsewhere when used).
extern void printlnBoth(const char* data);
extern void printBoth(const char* data);
extern void printfBoth(const char* format, ...);

// I2C address used by the example (keep as-is unless changed)
static DFRobot_MatrixLidar_I2C tof(0x33);
static uint16_t tof_buf[64];
static bool tof_initialized = false;

// Initialize the sensor. Returns true on success.
bool TOF_X8_begin() {
  // Detailed initialization with retries and I2C probe
  const int maxAttempts = 3;
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    char buf[64];
    snprintf(buf, sizeof(buf), "TOF_X8: init attempt %d/%d", attempt, maxAttempts);
    // prefer shared print if available
    printlnBoth(buf);

    // Quick I2C probe for the configured address
    Wire.beginTransmission(0x33);
    int i2cErr = Wire.endTransmission(); // 0 == success
    if (i2cErr != 0) {
      snprintf(buf, sizeof(buf), "TOF_X8: I2C probe returned %d", i2cErr);
      printlnBoth(buf);
    } else {
      printlnBoth("TOF_X8: I2C probe OK");
    }

    // Attempt library begin()
    uint8_t res = tof.begin();
    if (res == 0) {
      // Configure matrix mode (8x8)
      if (tof.setRangingMode(eMatrix_8X8) != 0) {
        printlnBoth("TOF_X8: setRangingMode failed");
        return false;
      }
      tof_initialized = true;
      printlnBoth("TOF_X8: initialized");
      return true;
    }

    snprintf(buf, sizeof(buf), "TOF_X8: begin() returned %u", (unsigned)res);
    printlnBoth(buf);

    delay(200);
  }

  printlnBoth("TOF_X8: init failed after retries");
  return false;
}

// Check whether sensor was initialized
bool TOF_X8_isInitialized() {
  return tof_initialized;
}

// Read all 8x8 distance values into the provided buffer (must be at least 64 entries)
// Returns true on successful read.
bool TOF_X8_readAll(uint16_t *outBuf, size_t outLen) {
  if (!tof_initialized || outBuf == nullptr || outLen < 64) return false;
  tof.getAllData(outBuf);
  return true;
}

// Print the last read buffer to Serial in row/column format
void TOF_X8_print(const uint16_t *buf) {
  if (buf == nullptr) return;
  for (uint8_t y = 0; y < 8; y++) {
    Serial.print("Y");
    Serial.print(y);
    Serial.print(": ");
    for (uint8_t x = 0; x < 8; x++) {
      Serial.print(buf[y * 8 + x]);
      if (x < 7) Serial.print(",");
    }
    Serial.println();
  }
  Serial.println("------------------------------");
}

// Task-scheduler compatible callback. Reads and prints data if initialized.
void TOF_X8_task_callback() {
  if (!tof_initialized) return;
  if (!TOF_X8_readAll(tof_buf, 64)) return;
  TOF_X8_print(tof_buf);
}
