#include "Alert.h"

// ---- Settings (any of these can be overridden in config.h) ----
#ifndef BUZZER_PIN
#define BUZZER_PIN 5         // CHECK: must not clash with your I2S mic / IMU pins
#endif
#ifndef BUZZER_ENABLED
#define BUZZER_ENABLED 1     // 0 = never sound
#endif
#ifndef BUZZER_PASSIVE
#define BUZZER_PASSIVE 1     // 0 = active buzzer, 1 = passive (needs Arduino-ESP32 core 3.x)
#endif
#ifndef LED_RGB_PIN
#define LED_RGB_PIN 48       // WS2812 on most ESP32-S3 SuperMini boards
#endif
#ifndef LED_BRIGHTNESS
#define LED_BRIGHTNESS 40    // 0-255, full brightness is glaring
#endif

struct Rgb { uint8_t r, g, b; };
static const Rgb C_OFF = {0, 0, 0};
static const Rgb C_IDLE = {0, 40, 0};
static const Rgb C_WEAK = {0, 80, 255};
static const Rgb C_MOD = {255, 110, 0};
static const Rgb C_STRONG = {255, 0, 0};

static bool same(const Rgb &a, const Rgb &b) { return a.r == b.r && a.g == b.g && a.b == b.b; }

static int rankOf(EventLevel l) {
  switch (l) {
    case EVENT_STRONG:   return 3;
    case EVENT_MODERATE: return 2;
    case EVENT_WEAK:     return 1;
    default:             return 0;
  }
}
static uint32_t holdMs(EventLevel l) {
  switch (l) {
    case EVENT_STRONG:   return 6000;
    case EVENT_MODERATE: return 3000;
    case EVENT_WEAK:     return 1200;
    default:             return 0;
  }
}
static Rgb colorOf(EventLevel l) {
  switch (l) {
    case EVENT_STRONG:   return C_STRONG;
    case EVENT_MODERATE: return C_MOD;
    case EVENT_WEAK:     return C_WEAK;
    default:             return C_IDLE;
  }
}

static void ledShow(const Rgb &c) {
  neopixelWrite(LED_RGB_PIN,
                (uint16_t)c.r * LED_BRIGHTNESS / 255,
                (uint16_t)c.g * LED_BRIGHTNESS / 255,
                (uint16_t)c.b * LED_BRIGHTNESS / 255);
}

static void buzzerSet(bool on) {
#if BUZZER_ENABLED
  #if BUZZER_PASSIVE
    if (on) tone(BUZZER_PIN, 2700); else noTone(BUZZER_PIN);
  #else
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
  #endif
#endif
}

// Beep patterns: alternating on/off durations in ms, starting with "on".
static const uint16_t PAT_STRONG[] = {150, 100, 150, 100, 150};
static const uint16_t PAT_MODERATE[] = {120};
static const uint16_t *pat = nullptr;
static uint8_t patLen = 0, patIdx = 0;
static uint32_t patNextMs = 0;

static void startPattern(const uint16_t *p, uint8_t len) {
  pat = p; patLen = len; patIdx = 0;
  patNextMs = millis() + p[0];
  buzzerSet(true);
}
static void buzzerUpdate(uint32_t now) {
  if (!pat || (int32_t)(now - patNextMs) < 0) return;
  patIdx++;
  if (patIdx >= patLen) { buzzerSet(false); pat = nullptr; return; }
  buzzerSet(patIdx % 2 == 0);
  patNextMs = now + pat[patIdx];
}

static EventLevel heldLevel = EVENT_NONE;
static uint32_t heldUntilMs = 0;
static Rgb lastShown = C_OFF;

void alertInit() {
#if BUZZER_ENABLED && !BUZZER_PASSIVE
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
#endif
  // Quick self-test so you can confirm the wiring at boot.
  ledShow(C_STRONG); buzzerSet(true); delay(60); buzzerSet(false); delay(140);
  ledShow(C_MOD);    delay(200);
  ledShow(C_WEAK);   delay(200);
  ledShow(C_OFF);
  lastShown = C_OFF;
}

void alertOnEvent(EventLevel level) {
  if (rankOf(level) == 0) return;
  uint32_t now = millis();
  bool holding = (int32_t)(heldUntilMs - now) > 0;
  bool newEpisode = !holding || rankOf(level) > rankOf(heldLevel);

  if (!holding || rankOf(level) >= rankOf(heldLevel)) {  // never downgrade a live alert
    heldLevel = level;
    heldUntilMs = now + holdMs(level);
  }
  if (newEpisode) {
    if (level == EVENT_STRONG) startPattern(PAT_STRONG, 5);
    else if (level == EVENT_MODERATE) startPattern(PAT_MODERATE, 1);
  }
}

void alertUpdate() {
  uint32_t now = millis();
  buzzerUpdate(now);

  Rgb c = C_IDLE;
  if ((int32_t)(heldUntilMs - now) > 0) {
    c = colorOf(heldLevel);
    if (heldLevel == EVENT_STRONG && ((now / 125) & 1)) c = C_OFF;  // ~4 Hz flash
  }
  if (!same(c, lastShown)) { ledShow(c); lastShown = c; }
}