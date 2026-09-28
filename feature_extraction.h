#ifndef FEATURE_EXTRACTION_H
#define FEATURE_EXTRACTION_H

#include <Arduino.h>

struct AudioFeatures {
  float rms;
  float zcr;                // zero-crossing rate, full window
  float spectralCentroid;   // from AUDIO_FFT_FRAME_SIZE frame
  float spectralEntropy;    // from AUDIO_FFT_FRAME_SIZE frame
  float spectralFlux;       // vs previous frame's spectrum
  uint32_t timestampMs;
};

struct IMUFeatures {
  float mean;
  float std;
  float peak;
  float rms;
  uint32_t timestampMs;
};

void featureExtractionInit();

// Feed newly captured audio samples in (call after audioCaptureRead).
void featureExtractionPushAudio(const int16_t *samples, size_t count);

// Feed one IMU accel-magnitude sample in (call after imuCaptureRead).
void featureExtractionPushIMU(float accelMag, uint32_t timestampMs);

// Returns true and fills out when a new audio feature window is ready
// (roughly every HOP_MS, once the buffer has filled once).
bool featureExtractionGetAudioFeatures(AudioFeatures &out);

// Returns true and fills out when a new IMU feature window is ready.
bool featureExtractionGetIMUFeatures(IMUFeatures &out);

#endif // FEATURE_EXTRACTION_H