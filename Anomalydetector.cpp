#include "AnomalyDetector.h"
#include "config.h"
#include "ContextModifier.h"

// One EMA-weighted running mean/variance tracker per feature.
struct EMAStat {
  float mean;
  float variance;
  uint32_t count;
};

static void emaStatInit(EMAStat &s) {
  s.mean = 0;
  s.variance = 0;
  s.count = 0;
}

// Updates the running mean/variance with a new value, then returns the
// z-score of that value against the (pre-update) baseline.
static float emaStatUpdateAndZScore(EMAStat &s, float x) {
  if (s.count == 0) {
    // First sample: no baseline yet, seed mean and report z=0.
    s.mean = x;
    s.variance = 0;
    s.count = 1;
    return 0.0f;
  }

  float std = sqrt(s.variance);
  float z = (std > 1e-6f) ? (x - s.mean) / std : 0.0f;

  float diff = x - s.mean;
  s.mean += EMA_ALPHA * diff;
  s.variance = (1.0f - EMA_ALPHA) * (s.variance + EMA_ALPHA * diff * diff);
  s.count++;

  return z;
}

// ---------------- Audio: 5 stats, one per feature ----------------
static EMAStat statRMS, statZCR, statCentroid, statEntropy, statFlux;
static uint32_t audioWindowCount = 0;

// ---------------- IMU: 4 stats, one per feature ----------------
static EMAStat statIMUMean, statIMUStd, statIMUPeak, statIMURMS;
static uint32_t imuWindowCount = 0;

void anomalyDetectorInit() {
  emaStatInit(statRMS);
  emaStatInit(statZCR);
  emaStatInit(statCentroid);
  emaStatInit(statEntropy);
  emaStatInit(statFlux);

  emaStatInit(statIMUMean);
  emaStatInit(statIMUStd);
  emaStatInit(statIMUPeak);
  emaStatInit(statIMURMS);
}

static float maxAbs5(float a, float b, float c, float d, float e) {
  float m = fabs(a);
  m = max(m, fabs(b));
  m = max(m, fabs(c));
  m = max(m, fabs(d));
  m = max(m, fabs(e));
  return m;
}

AudioAnomalyResult anomalyDetectorUpdateAudio(const AudioFeatures &af) {
  AudioAnomalyResult r;
  r.zRMS      = emaStatUpdateAndZScore(statRMS, af.rms);
  r.zZCR      = emaStatUpdateAndZScore(statZCR, af.zcr);
  r.zCentroid = emaStatUpdateAndZScore(statCentroid, af.spectralCentroid);
  r.zEntropy  = emaStatUpdateAndZScore(statEntropy, af.spectralEntropy);
  r.zFlux     = emaStatUpdateAndZScore(statFlux, af.spectralFlux);
  r.maxAbsZ   = maxAbs5(r.zRMS, r.zZCR, r.zCentroid, r.zEntropy, r.zFlux);
  r.timestampMs = af.timestampMs;

  audioWindowCount++;
  r.isAnomaly = (audioWindowCount > ANOMALY_WARMUP_WINDOWS) && (r.maxAbsZ > getEffectiveThreshold(Z_SCORE_THRESHOLD));

  return r;
}

IMUAnomalyResult anomalyDetectorUpdateIMU(const IMUFeatures &imf) {
  IMUAnomalyResult r;
  r.zMean = emaStatUpdateAndZScore(statIMUMean, imf.mean);
  r.zStd  = emaStatUpdateAndZScore(statIMUStd, imf.std);
  r.zPeak = emaStatUpdateAndZScore(statIMUPeak, imf.peak);
  r.zRMS  = emaStatUpdateAndZScore(statIMURMS, imf.rms);
  r.maxAbsZ = maxAbs5(r.zMean, r.zStd, r.zPeak, r.zRMS, 0.0f); // reuse helper, 5th arg unused
  r.timestampMs = imf.timestampMs;

  imuWindowCount++;
  r.isAnomaly = (imuWindowCount > ANOMALY_WARMUP_WINDOWS) && (r.maxAbsZ > getEffectiveThreshold(Z_SCORE_THRESHOLD));

  return r;
}