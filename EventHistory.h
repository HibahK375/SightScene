#ifndef EVENT_HISTORY_H
#define EVENT_HISTORY_H

#include <Arduino.h>

// Tracks recent distinct fused event episodes in a small ring buffer.
// A continuous anomaly is treated as one event episode rather than
// recording every anomalous processing window.
//
// ContextModifier uses this history to determine whether multiple
// distinct events have occurred within the recurrence window.

void eventHistoryInit();

// Record the beginning of a new event episode.
void eventHistoryRecord(uint32_t timestampMs);

// Returns how many recorded event episodes fall within the last windowMs.
int eventHistoryCountRecent(uint32_t windowMs);

#endif // EVENT_HISTORY_H