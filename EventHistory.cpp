#include "EventHistory.h"
#include "config.h"

static uint32_t history[EVENT_HISTORY_SIZE];

static size_t writeIdx = 0;
static size_t count = 0;

static uint32_t lastEventTimestamp = 0;
static bool haveLastEvent = false;

// A fused anomaly occurring within this interval after the previous
// recorded event is considered part of the same event episode.
static const uint32_t EVENT_EPISODE_GAP_MS = FUSION_STALE_MS;

void eventHistoryInit() {
  memset(history, 0, sizeof(history));

  writeIdx = 0;
  count = 0;

  lastEventTimestamp = 0;
  haveLastEvent = false;
}

void eventHistoryRecord(uint32_t timestampMs) {

  // Do not record repeated fusion outputs belonging to the same
  // continuing event.
  if (haveLastEvent &&
      (uint32_t)(timestampMs - lastEventTimestamp) < EVENT_EPISODE_GAP_MS) {
    return;
  }

  history[writeIdx] = timestampMs;

  writeIdx = (writeIdx + 1) % EVENT_HISTORY_SIZE;

  if (count < EVENT_HISTORY_SIZE) {
    count++;
  }

  lastEventTimestamp = timestampMs;
  haveLastEvent = true;
}

int eventHistoryCountRecent(uint32_t windowMs) {

  uint32_t now = millis();

  int recent = 0;

  for (size_t i = 0; i < count; i++) {

    if ((uint32_t)(now - history[i]) <= windowMs) {
      recent++;
    }
  }

  return recent;
}