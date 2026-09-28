#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <Arduino.h>
#include "feature_extraction.h"
#include "AnomalyDetector.h"
#include "Fusion.h"

// Connects to WiFi and the MQTT broker. Blocks (with retries) until WiFi
// is up; MQTT reconnects are handled inside mqttLoop() so a temporarily
// unreachable broker doesn't halt the device.
void mqttInit();

// Call every loop() iteration - maintains the MQTT connection and lets
// the client process incoming/outgoing traffic.
void mqttLoop();

// Publishes raw audio features + that window's anomaly z-scores.
void mqttPublishAudio(const AudioFeatures &af, const AudioAnomalyResult &ar);

// Publishes raw IMU features + that window's anomaly z-scores.
void mqttPublishIMU(const IMUFeatures &imf, const IMUAnomalyResult &ir);

// Publishes the current fused decision (level + both modalities' z-scores).
// The autoencoder score (once built) slots in here later as an extra field.
void mqttPublishEvent(const FusedEvent &fused);

#endif // MQTT_CLIENT_H