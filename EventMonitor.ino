#include "config.h"
#include "AudioCapture.h"
#include "IMUCapture.h"
#include "feature_extraction.h"
#include "AnomalyDetector.h"
#include "Fusion.h"
#include "MQTTClient.h"
#include "ContextModifier.h"
#include "EventHistory.h"


static int16_t audioBuf[AUDIO_CHUNK_SAMPLES];
static uint32_t lastIMUReadMs = 0;

// Only a new STRONG (dual-sensor-confirmed) event counts toward recurrence.
// A sustained STRONG event is recorded only once.
static EventLevel lastRecordedLevel = EVENT_NONE;

static void maybeRecordEvent(const FusedEvent &fe) {
  if (fe.level == EVENT_STRONG && lastRecordedLevel != EVENT_STRONG) {
    eventHistoryRecord(fe.timestampMs);
  }

  lastRecordedLevel = fe.level;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  audioCaptureInit();
  imuCaptureInit();
  featureExtractionInit();
  anomalyDetectorInit();
  fusionInit();
  mqttInit();
  contextModifierInit(); // needs WiFi, which mqttInit() just connected


  Serial.println("EnvironmentalEventMonitor running - publishing to MQTT");
  
}

void loop() {
  mqttLoop();

  // ---- Audio ----
  uint32_t audioTimestamp;
  size_t samplesRead =
      audioCaptureRead(audioBuf, AUDIO_CHUNK_SAMPLES, &audioTimestamp);

  featureExtractionPushAudio(audioBuf, samplesRead);

  AudioFeatures af;

  if (featureExtractionGetAudioFeatures(af)) {
    AudioAnomalyResult ar = anomalyDetectorUpdateAudio(af);

    FusedEvent fe = fusionUpdate(&ar, nullptr);

    // Record only a NEW STRONG event
    maybeRecordEvent(fe);

    mqttPublishAudio(af, ar);
    mqttPublishEvent(fe);

    Serial.printf(
        "AUDIO  t=%lu  z=%.2f  %s  ->  FUSED: %s\n",
        ar.timestampMs,
        ar.maxAbsZ,
        ar.isAnomaly ? "anomaly" : "normal",
        eventLevelName(fe.level));
  }

  // ---- IMU ----
  uint32_t now = millis();

  if (now - lastIMUReadMs >= IMU_PERIOD_MS) {
    lastIMUReadMs = now;

    IMUSample imuSample;

    if (imuCaptureRead(imuSample)) {
      float accelMag =
          sqrt(imuSample.accelX * imuSample.accelX +
               imuSample.accelY * imuSample.accelY +
               imuSample.accelZ * imuSample.accelZ);

      featureExtractionPushIMU(
          accelMag,
          imuSample.timestampMs);

      IMUFeatures imf;

      if (featureExtractionGetIMUFeatures(imf)) {
        IMUAnomalyResult ir =
            anomalyDetectorUpdateIMU(imf);

        FusedEvent fe = fusionUpdate(nullptr, &ir);

        // Record only a NEW STRONG event
        maybeRecordEvent(fe);

        mqttPublishIMU(imf, ir);
        mqttPublishEvent(fe);

        Serial.printf(
            "IMU    t=%lu  z=%.2f  %s  ->  FUSED: %s\n",
            ir.timestampMs,
            ir.maxAbsZ,
            ir.isAnomaly ? "anomaly" : "normal",
            eventLevelName(fe.level));
      }
    }
  }
}