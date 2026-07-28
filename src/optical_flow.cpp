#include "optical_flow.h"

OpticalFlow::OpticalFlow(uint8_t csPin)
  : sensor(csPin), initialized(false) {
  totalCountsX = 0;
  totalCountsY = 0;
  // Default scale: 0.05 mm per count (adjustable via setScaleMMPerCount)
  mmPerCount = 0.05f;
}

bool OpticalFlow::begin() {
  initialized = sensor.begin();
  return initialized;
}

bool OpticalFlow::read(int16_t &dx, int16_t &dy) {
  if (!initialized) return false;
  sensor.readMotionCount(&dx, &dy);
  return true;
}

void OpticalFlow::setLed(bool on) {
  if (!initialized) return;
  sensor.setLed(on);
}

void OpticalFlow::addMotionCounts(int16_t dx, int16_t dy) {
  totalCountsX += dx;
  totalCountsY += dy;
}

void OpticalFlow::resetTotals() {
  totalCountsX = 0;
  totalCountsY = 0;
}

void OpticalFlow::setScaleMMPerCount(float mmPerCount_) {
  if (mmPerCount_ <= 0.0f) return;
  mmPerCount = mmPerCount_;
}

float OpticalFlow::getTotalXmm() const {
  return ((float)totalCountsX) * mmPerCount;
}

float OpticalFlow::getTotalYmm() const {
  return ((float)totalCountsY) * mmPerCount;
}

float OpticalFlow::getScaleMMPerCount() const {
  return mmPerCount;
}
