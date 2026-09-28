#include "Fusion.h"
#include "config.h"

// Remembers the latest result from each modality so fusion can be
// re-evaluated whenever either side updates, even though audio and IMU
// windows don't complete in the same loop() iteration.
static bool haveAudio = false;
static bool haveIMU = false;
static AudioAnomalyResult lastAudio;
static IMUAnomalyResult lastIMU;

void fusionInit() {
  haveAudio = false;
  haveIMU = false;
}

const char *eventLevelName(EventLevel level) {
  switch (level) {
    case EVENT_STRONG:   return "STRONG";
    case EVENT_MODERATE: return "MODERATE";
    case EVENT_WEAK:     return "WEAK";
    default:             return "none";
  }
}

FusedEvent fusionUpdate(const AudioAnomalyResult *audioResult, const IMUAnomalyResult *imuResult) {
  if (audioResult) {
    lastAudio = *audioResult;
    haveAudio = true;
  }
  if (imuResult) {
    lastIMU = *imuResult;
    haveIMU = true;
  }

  uint32_t now = millis();
  FusedEvent out;
  out.timestampMs = now;

  // A modality "contributes" only if we have data from it and it's recent
  // enough to be trusted as describing the current moment.
  out.audioContributed = haveAudio && (now - lastAudio.timestampMs <= FUSION_STALE_MS);
  out.imuContributed   = haveIMU && (now - lastIMU.timestampMs <= FUSION_STALE_MS);

  bool audioAnomaly = out.audioContributed && lastAudio.isAnomaly;
  bool imuAnomaly    = out.imuContributed && lastIMU.isAnomaly;

  out.audioZ = out.audioContributed ? lastAudio.maxAbsZ : 0.0f;
  out.imuZ   = out.imuContributed ? lastIMU.maxAbsZ : 0.0f;

  float strongZCutoff = Z_SCORE_THRESHOLD * FUSION_STRONG_Z_MULTIPLIER;

  if (audioAnomaly && imuAnomaly) {
    out.level = EVENT_STRONG;               // both sensors agree
  } else if (audioAnomaly && out.audioZ > strongZCutoff) {
    out.level = EVENT_MODERATE;              // audio alone, but very confident
  } else if (imuAnomaly && out.imuZ > strongZCutoff) {
    out.level = EVENT_MODERATE;              // IMU alone, but very confident
  } else if (audioAnomaly || imuAnomaly) {
    out.level = EVENT_WEAK;                  // one sensor, borderline - ambiguous case
  } else {
    out.level = EVENT_NONE;
  }

  return out;
}