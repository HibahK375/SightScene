#ifndef FUSION_H
#define FUSION_H

#include <Arduino.h>
#include "AnomalyDetector.h"

enum EventLevel {
  EVENT_NONE,
  EVENT_WEAK,
  EVENT_MODERATE,
  EVENT_STRONG
};

struct FusedEvent {
  EventLevel level;
  bool audioContributed;  // was audio's data fresh enough to be used
  bool imuContributed;
  float audioZ;            // last known audio maxAbsZ (0 if not contributed)
  float imuZ;
  uint32_t timestampMs;
};

void fusionInit();

// Call with the audio result whenever a new audio window completes (pass
// nullptr for imuResult), and with the IMU result whenever a new IMU window
// completes (pass nullptr for audioResult). Internally remembers the most
// recent result from each side and re-evaluates the combined decision
// every call.
FusedEvent fusionUpdate(const AudioAnomalyResult *audioResult, const IMUAnomalyResult *imuResult);

// Human-readable label for logging.
const char *eventLevelName(EventLevel level);

#endif // FUSION_H