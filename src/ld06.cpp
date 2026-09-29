#include "ld06.h"
#include "telemetry.h"

// Keep the normal LiDAR path silent because USB Serial carries binary frames.
#define LD06_DEBUG_ASCII 0

// Diagnostic globals (defined here)
uint16_t ld06_diag_num_points = 0;
uint16_t ld06_diag_first_angle_cdeg = 0;
uint16_t ld06_diag_last_angle_cdeg = 0;
uint16_t ld06_diag_min_angle_cdeg = 0;
uint16_t ld06_diag_max_angle_cdeg = 0;
uint16_t ld06_diag_neg_steps_count = 0;
uint16_t ld06_diag_large_pos_jumps_count = 0;
uint16_t ld06_diag_largest_pos_step_cdeg = 0;
uint16_t ld06_diag_largest_neg_step_cdeg = 0;
uint16_t ld06_diag_total_span_cdeg = 0;
uint8_t  ld06_diag_approx_one_revolution = 0;
uint16_t ld06_diag_first_large_jump_idx = 0xFFFF;
uint16_t ld06_diag_first_neg_step_idx = 0xFFFF;

// New diagnostics for scan validation
uint16_t ld06_diag_rejected_scan_count = 0;  // Scans rejected due to invalid quality
uint16_t ld06_diag_backward_angle_count = 0; // Backward angle steps in last scan
float ld06_diag_last_angular_span = 0.0f;    // Last scan's angular coverage

static_assert(LD06_PTS_PER_PACKETS == 12, "LD06 parser expects 12 measurements per packet");
static_assert(LD06_PACKET_SIZE == 47, "LD06 parser expects 47-byte packets");

static bool ld06_read_packet_strict(HardwareSerial *serial, uint8_t *pkt) {
  // Strict sync on 0x54 0x2C, then read the remaining 45 bytes.
  static bool saw54 = false;

  while (serial->available()) {
    int b = serial->read();
    if (b < 0) {
      break;
    }
    uint8_t ub = (uint8_t)b;

    if (!saw54) {
      if (ub == LD06_HEADER) {
        saw54 = true;
      }
      continue;
    }

    // saw 0x54 previously; now require strict 0x2C
    if (ub == LD06_VER_SIZE) {
      pkt[0] = LD06_HEADER;
      pkt[1] = LD06_VER_SIZE;
      const size_t remain = LD06_PACKET_SIZE - 2;  // 45 bytes
      size_t got = serial->readBytes(pkt + 2, remain);
      saw54 = false;
      if (got != remain) {
        return false;
      }
      return true;
    }

    // Resync: if current byte is another 0x54, keep potential header.
    saw54 = (ub == LD06_HEADER);
  }

  return false;
}

static bool ld06_packet_structurally_valid(const uint8_t *pkt) {
  const LD06Packet &packet = *reinterpret_cast<const LD06Packet *>(pkt);

  if (packet.header != LD06_HEADER) {
    return false;
  }
  if (packet.version_size != LD06_VER_SIZE) {
    return false;
  }
  if (packet.startAngle > 36000U || packet.endAngle > 36000U) {
    return false;
  }

  // Angle-step validity gate: must be physically plausible.
  float fsa = (float)packet.startAngle / 100.0f;
  float lsa = (float)packet.endAngle / 100.0f;
  float range = lsa - fsa;
  if (range < 0.0f) {
    range += 360.0f;
  }
  float angleStep = range / (LD06_PTS_PER_PACKETS - 1);
  if (!(angleStep > 0.0f && angleStep <= LD06_ANGLE_STEP_MAX)) {
    return false;
  }

  // Full packet must contain all 12 measures (enforced by fixed-size strict read).
  return true;
}

static bool ld06_packet_crc_valid(const uint8_t *pkt) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < LD06_PACKET_SIZE - 1; ++i) {
    crc = CrcTable[crc ^ pkt[i]];
  }
  return crc == pkt[LD06_PACKET_SIZE - 1];
}

static void analyzePreviousScan(DataPointHandler *scan) {
  if (!scan) return;
  uint16_t n = scan->index;
  ld06_diag_num_points = n;
  if (n == 0) {
    ld06_diag_first_angle_cdeg = 0;
    ld06_diag_last_angle_cdeg = 0;
    ld06_diag_min_angle_cdeg = 0;
    ld06_diag_max_angle_cdeg = 0;
    ld06_diag_neg_steps_count = 0;
    ld06_diag_large_pos_jumps_count = 0;
    ld06_diag_largest_pos_step_cdeg = 0;
    ld06_diag_largest_neg_step_cdeg = 0;
    ld06_diag_total_span_cdeg = 0;
    ld06_diag_approx_one_revolution = 0;
    ld06_diag_first_large_jump_idx = 0xFFFF;
    ld06_diag_first_neg_step_idx = 0xFFFF;
    return;
  }

  float first_angle = scan->points[0].angle;
  float last_angle = scan->points[n-1].angle;
  float min_angle = first_angle;
  float max_angle = first_angle;
  uint16_t neg_count = 0;
  uint16_t large_pos_count = 0;
  float largest_pos = 0.0f;
  float largest_neg = 0.0f;
  float total_span = 0.0f;
  ld06_diag_first_large_jump_idx = 0xFFFF;
  ld06_diag_first_neg_step_idx = 0xFFFF;

  for (uint16_t i = 0; i < n; ++i) {
    float a = scan->points[i].angle;
    if (a < min_angle) min_angle = a;
    if (a > max_angle) max_angle = a;
    if (i > 0) {
      float prev = scan->points[i-1].angle;
      float step = a - prev;
      if (step <= -360.0f) step += 360.0f;
      if (step > 360.0f) step -= 360.0f;
      if (step < 0.0f) {
        ++neg_count;
        if (ld06_diag_first_neg_step_idx == 0xFFFF) ld06_diag_first_neg_step_idx = i;
        float negmag = -step;
        if (negmag > largest_neg) largest_neg = negmag;
      } else {
        if (step > largest_pos) largest_pos = step;
        if (step > 10.0f) {
          ++large_pos_count;
          if (ld06_diag_first_large_jump_idx == 0xFFFF) ld06_diag_first_large_jump_idx = i;
        }
      }
      float forward_step = step >= 0.0f ? step : (step + 360.0f);
      total_span += forward_step;
    }
  }

  ld06_diag_first_angle_cdeg = (uint16_t)fminf(0xFFFF, roundf(first_angle * 100.0f));
  ld06_diag_last_angle_cdeg = (uint16_t)fminf(0xFFFF, roundf(last_angle * 100.0f));
  ld06_diag_min_angle_cdeg = (uint16_t)fminf(0xFFFF, roundf(min_angle * 100.0f));
  ld06_diag_max_angle_cdeg = (uint16_t)fminf(0xFFFF, roundf(max_angle * 100.0f));
  ld06_diag_neg_steps_count = neg_count;
  ld06_diag_large_pos_jumps_count = large_pos_count;
  ld06_diag_largest_pos_step_cdeg = (uint16_t)fminf(0xFFFF, roundf(largest_pos * 100.0f));
  ld06_diag_largest_neg_step_cdeg = (uint16_t)fminf(0xFFFF, roundf(largest_neg * 100.0f));
  ld06_diag_total_span_cdeg = (uint16_t)fminf(0xFFFF, roundf(total_span * 100.0f));
  ld06_diag_approx_one_revolution = (total_span > 350.0f && total_span < 370.0f) ? 1 : 0;
}

LD06::LD06(HardwareSerial &serial, uint8_t pwmPin)
  : _lidarSerial(&serial),
    _pin(pwmPin),
    _currentBuffer(0),      // FIX: Explicit initialization
    _currentScan(&_scanA),  // FIX: Explicit initialization to _scanA
    _previousScan(&_scanB) {
  _scanA.index = 0;
  _scanB.index = 0;
  _receivedData.index = 0;
  _receivedData.computedCrc = 0;
  _completedScanPoints = 0;
  _scanReadyLatched = false;
}

void LD06::init() {
  // LD06 typically runs at 230400; ensure serial is configured to match hardware
  _lidarSerial->begin(230400);
  if (_pin != 255) {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, HIGH);
  }
}

/* Read lidar packet data with or without checking CRC,
   return : true if a valid package was received (with CRC : only if CRC ok)
*/
bool LD06::readData() {
  return _useCRC ? readDataCRC() : readDataNoCRC();
}

/* Read lidar packet data and check CRC,
   return : true if a valid packet is received
*/
bool LD06::readDataCRC() {
  // Strict sync parser: validate frame structure and CRC before processing.
  uint8_t pkt[LD06_PACKET_SIZE];
  while (ld06_read_packet_strict(_lidarSerial, pkt)) {
    if (!ld06_packet_crc_valid(pkt)) {
      ++_checksumFailCount;
      continue;
    }
    if (!ld06_packet_structurally_valid(pkt)) {
      continue;  // discard invalid packet and keep scanning
    }

    memcpy(_receivedData.packet.bytes, pkt, LD06_PACKET_SIZE);
    computeData();
    #if LD06_DEBUG_ASCII
    Serial.printf("LD06: packet processed start=%.2f\n", (float)_receivedData.packet.startAngle / 100.0f);
    #endif
    return true;
  }
  return false;
}

bool LD06::readDataNoCRC() {
  // Same strict framing and structural validation, but no CRC rejection.
  uint8_t pkt[LD06_PACKET_SIZE];
  while (ld06_read_packet_strict(_lidarSerial, pkt)) {
    if (!ld06_packet_structurally_valid(pkt)) {
      continue;  // discard invalid packet and keep scanning
    }

    memcpy(_receivedData.packet.bytes, pkt, LD06_PACKET_SIZE);
    computeData();
    return true;
  }
  return false;
}

bool LD06::readScan() {
  // Non-blocking wrapper used by higher-level code: return true when a new
  // scan has been assembled into _previousScan. This implementation will
  // consume incoming bytes via readData/readDataCRC and report the new-scan
  // flag if set.
  _newScan = false;
  (void)readData();
  if (_scanReadyLatched) {
    _scanReadyLatched = false;
    return true;
  }
  return false;
}

void LD06::computeData() {
  static bool  isInit         = false;
  static float lastPhysAngle  = -10000.0f;  // Previous point LD06 raw physAngle (unwrapped per-packet)
  static float startPhysAngle = 0.0f;      // LD06 starting angle data CW
  // Detection thresholds
  const float WRAP_DEG_THRESHOLD = 300.0f; // drop amount indicating genuine wrap (deg)

  float angleStep = getAngleStep();
  if (angleStep > LD06_ANGLE_STEP_MAX || angleStep <= 0.0f) {
    // Invalid packets should be rejected by parser before computeData().
    // Keep this guard as a fail-safe without resetting initialization state.
    return;
  }
  

  float fsa = _receivedData.packet.startAngle / 100.0f;

  (void)fsa; // no debug prints in normal operation

  DataPoint data;

  float firstPhysAngle = 0.0f;
  float lastPhysAnglePacket = 0.0f;
  for (uint16_t i = 0; i < LD06_PTS_PER_PACKETS; i++) {

    // Compute a raw physAngle (may be <0 or >=360 before normalization)
    float physAngleRaw = fsa + i * angleStep;

    // Detect wrap using raw physAngle sequence: a genuine wrap shows a large
    // drop (close to 360 deg) between consecutive raw physAngles. Packets
    // may arrive out of order; ignore small backward steps.
    if (lastPhysAngle > -1000.0f) {
      float drop = lastPhysAngle - physAngleRaw;
      if (drop > WRAP_DEG_THRESHOLD && _fullScan) {
        // Publish full revolutions only after scan-quality validation.
        if (isScanValid(_currentScan)) {
          const uint16_t completedPoints = _currentScan->index;
          swapBuffers();
          analyzePreviousScan(_previousScan);
          telemetry_update_ld06_diagnostics(_previousScan);
          _completedScanPoints = completedPoints;
          _scanReadyLatched = true;
          _newScan = true;
        } else {
          ++ld06_diag_rejected_scan_count;
          _currentScan->index = 0;
        }
        // reset startPhysAngle to the raw angle of the new revolution
        startPhysAngle = physAngleRaw;
      }
    } else {
      // first valid point seen
      startPhysAngle = physAngleRaw;
      isInit = true;
    }
    lastPhysAngle = physAngleRaw;

    // Normalize for storage and user-facing angle (mathematical CCW)

    float physAngle = physAngleRaw;
    while (physAngle >= 360.0f) physAngle -= 360.0f;
    while (physAngle <   0.0f) physAngle += 360.0f;

    if (i == 0) firstPhysAngle = physAngle;
    if (i == LD06_PTS_PER_PACKETS - 1) lastPhysAnglePacket = physAngle;


    float angle;
    if (_upsideDown) {
      angle = physAngle;                 // Upside down turn CW to CCW => Nothing to change
    } else {
      angle = 360.0f - physAngle;        // Convert LD06 angle data CW to mathematical CCW convension
      if (angle >= 360.0f) angle -= 360.0f;
    }

    // Normalize [0 360°]
    if (angle < 0.0f)     angle += 360.0f;
    if (angle >= 360.0f)  angle -= 360.0f;

    _angles[i] = angle;

    if (isInit && _currentScan->index < LD06_MAX_PTS_SCAN) {
      data.angle     = angle;                                   // Mathematical angle CCW
      data.distance  = _receivedData.packet.measures[i].distance;
      data.intensity = _receivedData.packet.measures[i].intensity;

      const bool filterOk = (!_useFiltering || filter(data));
      if (filterOk) {
#ifdef LD06_COMPUTE_XY
        float angRad = (data.angle + _angularPosition + _angularOffset) * PI / 180.0f;
        float cosPos = cos(_angularPosition * PI / 180.0f);
        float sinPos = sin(_angularPosition * PI / 180.0f);

        data.x = _xPosition
                 + _xOffset * cosPos - _yOffset * sinPos
                 + data.distance * cos(angRad);

        data.y = _yPosition
                 + _xOffset * sinPos + _yOffset * cosPos
                 - data.distance * sin(angRad);
#endif
        _currentScan->points[_currentScan->index++] = data;
      }
    }
  }

  // En mode "non full scan", chaque paquet déclenche un swap
  if (!_fullScan) {
    uint16_t completedPoints = _currentScan->index;
    swapBuffers();
    analyzePreviousScan(_previousScan);
    // In non-fullScan mode each packet is treated as a completed chunk; update diagnostics.
    telemetry_update_ld06_diagnostics(_previousScan);
    _completedScanPoints = completedPoints;
    _scanReadyLatched = true;
    _newScan = true;
  }
}

// Print full scan using csv format
void LD06::printScanCSV(Stream &serialport) {
  static bool init = false;
  if (!init) {
    serialport.println(F("N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)"));
    init = true;
  }
  if (_previousScan->index) {
    for (uint16_t i = 0; i < _previousScan->index; i++) {
      serialport.print(i);
      serialport.print(",");
      serialport.print(_previousScan->points[i].angle, 2);
      serialport.print(",");
      serialport.print(_previousScan->points[i].distance);
      serialport.print(",");
      serialport.print(_previousScan->points[i].intensity);
#ifdef LD06_COMPUTE_XY
      serialport.print(",");
      serialport.print(_previousScan->points[i].x);
      serialport.print(",");
      serialport.print(_previousScan->points[i].y);
#endif
      serialport.println();
    }
    serialport.println();
  }
}

// Print full scan using teleplot format (check :https://teleplot.fr/)
#ifdef LD06_COMPUTE_XY
void LD06::printScanTeleplot(Stream &serialport) {
  if (_previousScan->index) {
    serialport.print(F(">lidar:"));
    for (uint16_t i = 0; i < _previousScan->index; i++) {
      serialport.print(_previousScan->points[i].x);
      serialport.print(":");
      serialport.print(_previousScan->points[i].y);
      serialport.print(";");
    }
    serialport.println(F("|xy"));
  }
}
#endif

// Settings
void LD06::enableCRC() {
  _useCRC = true;
}

void LD06::disableCRC() {
  _useCRC = false;
}

void LD06::enableFullScan() {   // readScan will return true only when a new 360° scan is available
  _fullScan = true;
}
void LD06::disableFullScan() {  // readScan will return true for each data chunk
  _fullScan = false;
}

void LD06::enableFiltering() {
  _useFiltering = true;
}

void LD06::disableFiltering() {
  _useFiltering = false;
}

void LD06::setIntensityThreshold(uint8_t threshold) {
  _threshold = threshold;
}

void LD06::setMaxDistance(uint16_t maxDist) {
  _maxDist = maxDist;
}

void LD06::setMinDistance(uint16_t minDist) {
  _minDist = minDist;
}

void LD06::setDistanceRange(uint16_t minDist, uint16_t maxDist) {
  _minDist = minDist;
  _maxDist = maxDist;
}

int16_t LD06::rescaleAngle(int16_t angle) {
  if (angle > 360)
    angle %= 360;
  else
    while (angle < 0)
      angle += 360;
  return angle;
}

void LD06::setMaxAngle(int16_t maxAngle) {
  _maxAngle = rescaleAngle(maxAngle);
}

void LD06::setMinAngle(int16_t minAngle) {
  _minAngle = minAngle;
}

void LD06::setAngleRange(int16_t minAngle, int16_t maxAngle) {
  _minAngle = rescaleAngle(minAngle);
  _maxAngle = rescaleAngle(maxAngle);
}

void LD06::setUpsideDown(bool upsideDown) {
  _upsideDown = upsideDown;
}

#ifdef LD06_COMPUTE_XY
void LD06::setOffsetPosition(int16_t xPos = 0, int16_t yPos = 0, float anglePos = 0) {
  _xOffset = xPos;
  _yOffset = yPos;
  _angularOffset = anglePos;
}
#endif

// FIX: Validate scan quality before publishing
// Returns true if scan appears to be a valid complete or mostly complete revolution
bool LD06::isScanValid(DataPointHandler* scan) {
  if (!scan || scan->index < 100) {
    // Insufficient data for a meaningful scan
    return false;
  }

  // Calculate angular span
  if (scan->index < 2) return false;
  
  float firstAngle = scan->points[0].angle;
  float lastAngle = scan->points[scan->index - 1].angle;
  
  // Handle wrap-around: if first angle > last angle, we've wrapped 0°
  float angularSpan;
  if (lastAngle >= firstAngle) {
    angularSpan = lastAngle - firstAngle;
  } else {
    angularSpan = (360.0f - firstAngle) + lastAngle; // Wrap-around case
  }
  
  // A valid revolution should span 300-380 degrees
  // Lower bound catches incomplete scans; upper bound catches duplicates
  if (angularSpan < 300.0f || angularSpan > 380.0f) {
    ld06_diag_last_angular_span = angularSpan;
    return false;
  }

  // Count backward angle steps (indicates packet reordering or data corruption)
  uint16_t backwardStepCount = 0;
  for (uint16_t i = 1; i < scan->index; i++) {
    float prevAngle = scan->points[i-1].angle;
    float currAngle = scan->points[i].angle;
    
    // Allow one wrap-around per scan (0° transition); count others as anomalies
    if (currAngle < prevAngle && (prevAngle - currAngle) < 300.0f) {
      backwardStepCount++;
    }
  }
  
  // More than a few backward steps indicates corruption
  if (backwardStepCount > 3) {
    ld06_diag_backward_angle_count = backwardStepCount;
    return false;
  }

  // Scan appears valid
  ld06_diag_backward_angle_count = backwardStepCount;
  ld06_diag_last_angular_span = angularSpan;
  return true;
}

void LD06::swapBuffers() {
  _currentBuffer = !_currentBuffer;
  if (_currentBuffer) {
    _currentScan = &_scanB;
    _previousScan = &_scanA;
  } else {
    _currentScan = &_scanA;
    _previousScan = &_scanB;
  }
  _currentScan->index = 0;
}

