#ifndef CONTEXT_MODIFIER_H
#define CONTEXT_MODIFIER_H

#include <Arduino.h>

// Starts NTP time sync (call once in setup(), after WiFi is connected).
void contextModifierInit();

// Context-aware threshold adjustment. Combines two signals:
//   1. Time-of-day - lowers the threshold at night (quieter baseline,
//      smaller disturbances are more meaningful).
//   2. Recurrence - lowers the threshold further if several real events
//      have happened recently (a repeating pattern is more significant
//      than one isolated event).
// Runs entirely on-device.
float getEffectiveThreshold(float baseThreshold);

#endif // CONTEXT_MODIFIER_H
