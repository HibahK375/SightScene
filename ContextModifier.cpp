#include "ContextModifier.h"
#include "config.h"
#include "EventHistory.h"
#include <time.h>

void contextModifierInit() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  eventHistoryInit();
}

float getEffectiveThreshold(float baseThreshold) {
  float threshold = baseThreshold;

  // ---- Time-of-day context ----
  struct tm timeinfo;
  // Short timeout (100ms) - if time isn't synced yet (e.g. right after
  // boot, before NTP responds), this adjustment is just skipped rather
  // than blocking the detector.
  if (getLocalTime(&timeinfo, 100)) {
    int hour = timeinfo.tm_hour;
    bool isNight = (hour >= NIGHT_START_HOUR) || (hour < NIGHT_END_HOUR);
    if (isNight) {
      threshold *= NIGHT_THRESHOLD_MULTIPLIER;
    }
  }

  // ---- Recurrence context ----
  // "Has something like this been happening repeatedly?" - if several
  // real events have landed within the recurrence window, treat the
  // environment as currently in an elevated state and become more
  // sensitive, rather than judging each event in isolation.
  int recentCount = eventHistoryCountRecent(RECURRENCE_WINDOW_MS);
  if (recentCount >= RECURRENCE_COUNT_THRESHOLD) {
    threshold *= RECURRENCE_THRESHOLD_MULTIPLIER;
  }

  // Temporary diagnostic - remove or comment out once you've confirmed
  // this is behaving correctly, it prints on every single window.
  Serial.printf("[CONTEXT] base=%.2f recentEvents=%d effective=%.2f\n",
                baseThreshold, recentCount, threshold);

  return threshold;
}

/*#include "ContextModifier.h"
#include "config.h"
#include "EventHistory.h"
#include <time.h>

void contextModifierInit() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  eventHistoryInit();
}

float getEffectiveThreshold(float baseThreshold) {
  float threshold = baseThreshold;

  // ---- Time-of-day context ----
  struct tm timeinfo;
  // Short timeout (100ms) - if time isn't synced yet (e.g. right after
  // boot, before NTP responds), this adjustment is just skipped rather
  // than blocking the detector.
  if (getLocalTime(&timeinfo, 100)) {
    int hour = timeinfo.tm_hour;
    bool isNight = (hour >= NIGHT_START_HOUR) || (hour < NIGHT_END_HOUR);
    if (isNight) {
      threshold *= NIGHT_THRESHOLD_MULTIPLIER;
    }
  }

  // ---- Recurrence context ----
  // "Has something like this been happening repeatedly?" - if several
  // real events have landed within the recurrence window, treat the
  // environment as currently in an elevated state and become more
  // sensitive, rather than judging each event in isolation.
  int recentCount = eventHistoryCountRecent(RECURRENCE_WINDOW_MS);
  if (recentCount >= RECURRENCE_COUNT_THRESHOLD) {
    threshold *= RECURRENCE_THRESHOLD_MULTIPLIER;
  }

  return threshold;
} */
