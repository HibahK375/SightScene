#include "MQTTClient.h"
#include "config.h"
#include <WiFi.h>
#include <PubSubClient.h>

static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static uint32_t lastReconnectAttempt = 0;
static uint32_t lastWiFiRetry = 0;

void mqttInit() {
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

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nNo WiFi yet - continuing without it. MQTT/dashboard will");
    Serial.println("come online automatically once WiFi connects (retries in background).");
  }
}

static void mqttReconnect() {
  uint32_t now = millis();
  if (now - lastReconnectAttempt < 5000) return; // don't hammer the broker
  lastReconnectAttempt = now;

  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    Serial.println("MQTT connected");
  }
  // Silent on failure - this is routine (broker not up yet, WiFi just
  // reconnected, etc.), not worth spamming Serial every 5s.
}

void mqttLoop() {
  if (WiFi.status() != WL_CONNECTED) {
    // Retry WiFi every 15s in the background - covers both "never had
    // WiFi at boot" and "WiFi dropped mid-run".
    uint32_t now = millis();
    if (now - lastWiFiRetry > 15000) {
      lastWiFiRetry = now;
      WiFi.begin(WIFI_SSID);
    }
    return;
  }

  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();
}

void mqttPublishAudio(const AudioFeatures &af, const AudioAnomalyResult &ar) {
  if (!mqttClient.connected()) return; // no WiFi/broker yet - just skip, nothing lost that matters for a demo

  char payload[320];
  snprintf(payload, sizeof(payload),
    "{\"t\":%lu,\"rms\":%.1f,\"zcr\":%.3f,\"centroid\":%.1f,\"entropy\":%.3f,\"flux\":%.0f,"
    "\"zRMS\":%.2f,\"zZCR\":%.2f,\"zCentroid\":%.2f,\"zEntropy\":%.2f,\"zFlux\":%.2f,\"maxZ\":%.2f,\"anomaly\":%d}",
    af.timestampMs, af.rms, af.zcr, af.spectralCentroid, af.spectralEntropy, af.spectralFlux,
    ar.zRMS, ar.zZCR, ar.zCentroid, ar.zEntropy, ar.zFlux, ar.maxAbsZ, ar.isAnomaly ? 1 : 0);

  mqttClient.publish(MQTT_TOPIC_AUDIO, payload);
}

void mqttPublishIMU(const IMUFeatures &imf, const IMUAnomalyResult &ir) {
  if (!mqttClient.connected()) return;

  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"t\":%lu,\"mean\":%.3f,\"std\":%.3f,\"peak\":%.3f,\"rms\":%.3f,"
    "\"zMean\":%.2f,\"zStd\":%.2f,\"zPeak\":%.2f,\"zRMS\":%.2f,\"maxZ\":%.2f,\"anomaly\":%d}",
    imf.timestampMs, imf.mean, imf.std, imf.peak, imf.rms,
    ir.zMean, ir.zStd, ir.zPeak, ir.zRMS, ir.maxAbsZ, ir.isAnomaly ? 1 : 0);

  mqttClient.publish(MQTT_TOPIC_IMU, payload);
}

void mqttPublishEvent(const FusedEvent &fused) {
  if (!mqttClient.connected()) return;

  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"t\":%lu,\"level\":\"%s\",\"audioZ\":%.2f,\"imuZ\":%.2f,\"audioOk\":%d,\"imuOk\":%d}",
    fused.timestampMs, eventLevelName(fused.level),
    fused.audioZ, fused.imuZ,
    fused.audioContributed ? 1 : 0, fused.imuContributed ? 1 : 0);

  mqttClient.publish(MQTT_TOPIC_EVENT, payload);
}