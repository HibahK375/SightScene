#include "MQTTClient.h"
#include "config.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>

static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);

static uint32_t lastReconnectAttempt = 0;
static uint32_t lastWiFiRetry = 0;

// ============================================================
// USB FAILSAFE
// ============================================================
// usb_bridge.py (laptop) publishes a tiny heartbeat on this topic every
// 500 ms. Hearing it proves the whole path laptop -> broker -> Wi-Fi -> ESP
// is alive. If it stops (Wi-Fi down, broker unreachable, laptop-side client
// gone) or a publish fails, every message goes out over USB instead.
// While MQTT is healthy, USB stays silent.
static const char *TOPIC_PC_HB = "envmonitor/pc_hb";
static const uint32_t HB_TIMEOUT_MS = 2000;
static uint32_t lastHeartbeat = 0;   // millis() of last heartbeat, 0 = none yet
static bool wasHealthy = false;

// Recent events, replayed over USB the instant MQTT is declared dead, so
// events published during the detection window are not lost. The laptop
// de-duplicates by (topic, t).
static const int EVENT_RING = 12;
static char eventRing[EVENT_RING][256];
static int ringNext = 0;
static int ringCount = 0;

static void onMqttMessage(char *topic, byte *payload, unsigned int length) {
  if (strcmp(topic, TOPIC_PC_HB) == 0) lastHeartbeat = millis();
}

// PubSubClient::connected() can stay true for a while after a link dies,
// so it is not enough on its own - the heartbeat is the real health signal.
static bool mqttHealthy() {
  return WiFi.status() == WL_CONNECTED && mqttClient.connected() &&
         lastHeartbeat != 0 && (millis() - lastHeartbeat) < HB_TIMEOUT_MS;
}

// One line per message:  @T <topic> <json>
// The laptop ignores every line that doesn't start with "@T ".
static void usbSend(const char *topic, const char *json) {
  Serial.print("@T ");
  Serial.print(topic);
  Serial.print(' ');
  Serial.println(json);
}

// Normal path: MQTT. If it's unhealthy or the publish fails: USB.
static void emit(const char *topic, const char *json) {
  if (mqttHealthy() && mqttClient.publish(topic, json)) return;
  usbSend(topic, json);
}

static void replayRecentEvents() {
  int start = (ringNext - ringCount + EVENT_RING) % EVENT_RING;
  for (int i = 0; i < ringCount; i++) {
    usbSend(MQTT_TOPIC_EVENT, eventRing[(start + i) % EVENT_RING]);
  }
}

static void trackHealth() {
  bool healthy = mqttHealthy();
  if (wasHealthy && !healthy) replayRecentEvents();
  wasHealthy = healthy;
}

// ============================================================
// WIFI / MQTT CONNECTION
// ============================================================

void mqttInit() {
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);   // never stall sensing if nobody is reading USB
#endif

  WiFi.begin(WIFI_SSID);
  Serial.print("Connecting to WiFi");

  // Bounded wait (10s), not infinite - so the device still boots and runs
  // everything else (audio/IMU/features/detection) even with no WiFi at all.
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setKeepAlive(10);
  mqttClient.setSocketTimeout(3);
  mqttClient.setBufferSize(512);   // default 256 B can silently reject larger payloads

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nNo WiFi yet - continuing without it. Data goes out over USB");
    Serial.println("until MQTT comes online (WiFi retries in background).");
  }
}

static void mqttReconnect() {
  uint32_t now = millis();
  if (now - lastReconnectAttempt < 5000) return;   // don't hammer the broker
  lastReconnectAttempt = now;

  wifiClient.stop();   // tear down any half-dead socket before retrying

  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    Serial.println("MQTT connected");
    mqttClient.subscribe(TOPIC_PC_HB);
    lastHeartbeat = 0;   // must hear a fresh heartbeat before trusting MQTT again
  }
  // Silent on failure - routine (broker not up yet, WiFi just reconnected).
}

void mqttLoop() {
  uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    // Retry WiFi every 15s in the background - covers both "never had
    // WiFi at boot" and "WiFi dropped mid-run".
    if (now - lastWiFiRetry > 15000) {
      lastWiFiRetry = now;
      WiFi.begin(WIFI_SSID);
    }
  } else {
    if (!mqttClient.connected()) mqttReconnect();
    mqttClient.loop();
  }

  trackHealth();   // runs on every path, including "no WiFi"
}

// ============================================================
// PUBLISHERS - same payloads as before, routed through emit()
// ============================================================

void mqttPublishAudio(const AudioFeatures &af, const AudioAnomalyResult &ar) {
  char payload[320];
  snprintf(payload, sizeof(payload),
           "{\"t\":%lu,\"rms\":%.1f,\"zcr\":%.3f,\"centroid\":%.1f,\"entropy\":%.3f,\"flux\":%.0f,"
           "\"zRMS\":%.2f,\"zZCR\":%.2f,\"zCentroid\":%.2f,\"zEntropy\":%.2f,\"zFlux\":%.2f,\"maxZ\":%.2f,\"anomaly\":%d}",
           af.timestampMs, af.rms, af.zcr, af.spectralCentroid, af.spectralEntropy, af.spectralFlux,
           ar.zRMS, ar.zZCR, ar.zCentroid, ar.zEntropy, ar.zFlux, ar.maxAbsZ, ar.isAnomaly ? 1 : 0);
  emit(MQTT_TOPIC_AUDIO, payload);
}

void mqttPublishIMU(const IMUFeatures &imf, const IMUAnomalyResult &ir) {
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"t\":%lu,\"mean\":%.3f,\"std\":%.3f,\"peak\":%.3f,\"rms\":%.3f,"
           "\"zMean\":%.2f,\"zStd\":%.2f,\"zPeak\":%.2f,\"zRMS\":%.2f,\"maxZ\":%.2f,\"anomaly\":%d}",
           imf.timestampMs, imf.mean, imf.std, imf.peak, imf.rms,
           ir.zMean, ir.zStd, ir.zPeak, ir.zRMS, ir.maxAbsZ, ir.isAnomaly ? 1 : 0);
  emit(MQTT_TOPIC_IMU, payload);
}

void mqttPublishEvent(const FusedEvent &fused) {
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"t\":%lu,\"level\":\"%s\",\"audioZ\":%.2f,\"imuZ\":%.2f,\"audioOk\":%d,\"imuOk\":%d}",
           fused.timestampMs, eventLevelName(fused.level),
           fused.audioZ, fused.imuZ,
           fused.audioContributed ? 1 : 0, fused.imuContributed ? 1 : 0);

  // Remember real events (not "none") so they can be replayed over USB.
  if (strcasecmp(eventLevelName(fused.level), "none") != 0) {
    strncpy(eventRing[ringNext], payload, sizeof(eventRing[0]));
    eventRing[ringNext][sizeof(eventRing[0]) - 1] = '\0';
    ringNext = (ringNext + 1) % EVENT_RING;
    if (ringCount < EVENT_RING) ringCount++;
  }

  emit(MQTT_TOPIC_EVENT, payload);
}