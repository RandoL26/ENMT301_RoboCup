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

// Driver instance pointer (we'll construct with detected address at runtime)
static DFRobot_MatrixLidar_I2C *tof = nullptr;
static uint16_t tof_buf[64];
static bool tof_initialized = false;
static uint8_t tof_addr = 0x33; // last-used address (for diagnostics)

// Initialize the sensor. Returns true on success.
bool TOF_X8_begin() {
  // Try to find the Matrix Lidar on the I2C bus by scanning addresses.
  // We'll prefer the default 0x33 but accept any responding address and
  // attempt to initialize the driver there. This avoids touching other
  // TOF devices and doesn't require hardware address changes.

  const uint8_t defaultAddr = 0x33;
  const uint8_t startAddr = 0x08;
  const uint8_t endAddr = 0x77;

  // First try the default address directly
  uint8_t tryAddrs[3] = { defaultAddr, 0x32, 0x34 };
  const int tryCount = 3;

  for (int t = 0; t < tryCount; t++) {
    uint8_t a = tryAddrs[t];
    char msg[64];
    snprintf(msg, sizeof(msg), "TOF_X8: probing preferred addr 0x%02X", a);
    printlnBoth(msg);

    Wire.beginTransmission(a);
    int r = Wire.endTransmission();
    if (r != 0) {
      snprintf(msg, sizeof(msg), "TOF_X8: probe 0x%02X returned %d", a, r);
      printlnBoth(msg);
      continue;
    }

    // Construct driver instance for this address and try init
    // Try a couple of init strategies in case the device needs different timing or clock
    if (tof) { delete tof; tof = nullptr; }
    // Ensure I2C is started and at a conservative speed first
    Wire.setClock(100000);
    Wire.begin();
    delay(20);
    tof = new DFRobot_MatrixLidar_I2C(a);
    uint8_t res = tof->begin();
    if (res != 0) {
      // try small delay then try again at 100k
      delay(50);
      res = tof->begin();
    }
    if (res != 0) {
      // try switching to 400k and retry
      Wire.setClock(400000);
      delay(20);
      res = tof->begin();
    }
    if (res == 0) {
      // The device sometimes needs a short settle before accepting mode commands.
      bool modeOk = false;
      for (int mtry = 1; mtry <= 5; mtry++) {
        char mbuf[48];
        snprintf(mbuf, sizeof(mbuf), "TOF_X8: setRangingMode attempt %d/5", mtry);
        printlnBoth(mbuf);
        if (tof->setRangingMode(eMatrix_8X8) == 0) { modeOk = true; break; }
        delay(200);
      }
      if (!modeOk) {
        printlnBoth("TOF_X8: setRangingMode failed");
        delete tof; tof = nullptr;
        continue;
      }
      tof_initialized = true;
      tof_addr = a;
      char ok[48];
      snprintf(ok, sizeof(ok), "TOF_X8: initialized at 0x%02X", a);
      printlnBoth(ok);
      return true;
    }

    snprintf(msg, sizeof(msg), "TOF_X8: begin() @0x%02X returned %u", a, (unsigned)res);
    printlnBoth(msg);
    delete tof; tof = nullptr;
  }

  // If preferred addresses failed, do a full I2C scan and try any responding address
  printlnBoth("TOF_X8: scanning I2C bus for candidate addresses...");
  for (uint8_t a = startAddr; a <= endAddr; a++) {
    Wire.beginTransmission(a);
    int r = Wire.endTransmission();
    if (r == 0) {
      char s[48];
      snprintf(s, sizeof(s), "TOF_X8: device found at 0x%02X", a);
      printlnBoth(s);

      // Try initializing driver at this address
      if (tof) { delete tof; tof = nullptr; }
      tof = new DFRobot_MatrixLidar_I2C(a);
      uint8_t res = tof->begin();
      if (res == 0) {
        if (tof->setRangingMode(eMatrix_8X8) != 0) {
          printlnBoth("TOF_X8: setRangingMode failed");
          delete tof; tof = nullptr;
          Wire.setClock(400000);
          return false;
        }
        tof_initialized = true;
        tof_addr = a;
        char ok[48];
        snprintf(ok, sizeof(ok), "TOF_X8: initialized at 0x%02X", a);
        printlnBoth(ok);
        Wire.setClock(400000);
        return true;
      }
      delete tof; tof = nullptr;
    }
  }

  // Restore I2C speed and print summary
  Wire.setClock(400000);
  printlnBoth("TOF_X8: init failed; no usable address found");
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
  if (!tof) return false;
  tof->getAllData(outBuf);
  return true;
}

// Task-scheduler compatible callback. Reads and prints data if initialized.
void TOF_X8_task_callback() {
  if (!tof_initialized) return;
  if (!TOF_X8_readAll(tof_buf, 64)) return;
}
