#ifndef ANOMALY_DETECTOR_H
#define ANOMALY_DETECTOR_H

#include <Arduino.h>
#include "feature_extraction.h"

struct AudioAnomalyResult {
  float zRMS, zZCR, zCentroid, zEntropy, zFlux;
  float maxAbsZ;     // largest |z| across the 5 features = the anomaly score
  bool isAnomaly;    // maxAbsZ > Z_SCORE_THRESHOLD, and past warm-up
  uint32_t timestampMs;
};

struct IMUAnomalyResult {
  float zMean, zStd, zPeak, zRMS;
  float maxAbsZ;
  bool isAnomaly;
  uint32_t timestampMs;
};

void anomalyDetectorInit();

AudioAnomalyResult anomalyDetectorUpdateAudio(const AudioFeatures &af);
IMUAnomalyResult anomalyDetectorUpdateIMU(const IMUFeatures &imf);

#endif // ANOMALY_DETECTOR_H