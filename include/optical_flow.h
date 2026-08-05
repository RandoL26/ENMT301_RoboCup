#ifndef __OPTICAL_FLOW_H__
#define __OPTICAL_FLOW_H__

#include <Arduino.h>
#include "Bitcraze_PMW3901.h"

class OpticalFlow {
public:
  OpticalFlow(uint8_t csPin = 10);
  bool begin();
  bool read(int16_t &dx, int16_t &dy);
  void setLed(bool on);
  // Accumulate motion counts and convert to distance
  void addMotionCounts(int16_t dx, int16_t dy);
  void resetTotals();
  void setScaleMMPerCount(float mmPerCount);
  float getScaleMMPerCount() const;
  float getTotalXmm() const;
  float getTotalYmm() const;

private:
  Bitcraze_PMW3901 sensor;
  bool initialized;
  int32_t totalCountsX;
  int32_t totalCountsY;
  float mmPerCount; // millimetres per motion-count
};

#endif // __OPTICAL_FLOW_H__
