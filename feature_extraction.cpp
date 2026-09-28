#include "feature_extraction.h"
#include "config.h"
#include <arduinoFFT.h>

// ---------------- Audio ring buffer (plain array, manual index math) ----------------
// Sized to the full 1s window. Allocated in PSRAM since 16000 int16 samples
// = 32KB, worth keeping off the main SRAM budget.
static int16_t *audioRing = nullptr;
static size_t audioWriteIdx = 0;
static size_t audioTotalPushed = 0;
static size_t audioHopCounter = 0;
static bool audioWindowReady = false;
static AudioFeatures audioFeaturesOut;

// Previous FFT frame's magnitude spectrum, for spectral flux.
static float prevMag[AUDIO_FFT_FRAME_SIZE / 2];
static bool havePrevMag = false;

// ---------------- IMU ring buffer (plain array) ----------------
static float imuRing[IMU_WINDOW_SAMPLES];
static size_t imuWriteIdx = 0;
static size_t imuTotalPushed = 0;
static size_t imuHopCounter = 0;
static bool imuWindowReady = false;
static IMUFeatures imuFeaturesOut;

// ---------------- FFT scratch buffers ----------------
static float vReal[AUDIO_FFT_FRAME_SIZE];
static float vImag[AUDIO_FFT_FRAME_SIZE];
static ArduinoFFT<float> fft(vReal, vImag, AUDIO_FFT_FRAME_SIZE, (float)AUDIO_SAMPLE_RATE);

void featureExtractionInit() {
  audioRing = (int16_t *)ps_malloc(AUDIO_WINDOW_SAMPLES * sizeof(int16_t));
  if (!audioRing) {
    Serial.println("PSRAM alloc for audio ring buffer failed - check PSRAM is enabled");
    while (true) delay(1000);
  }
  memset(audioRing, 0, AUDIO_WINDOW_SAMPLES * sizeof(int16_t));
  memset(imuRing, 0, sizeof(imuRing));
}

// ---------------- Audio ----------------

void featureExtractionPushAudio(const int16_t *samples, size_t count) {
  for (size_t i = 0; i < count; i++) {
    audioRing[audioWriteIdx] = samples[i];
    audioWriteIdx = (audioWriteIdx + 1) % AUDIO_WINDOW_SAMPLES;
  }
  audioTotalPushed += count;
  audioHopCounter += count;

  bool bufferFull = audioTotalPushed >= AUDIO_WINDOW_SAMPLES;
  if (bufferFull && audioHopCounter >= AUDIO_HOP_SAMPLES) {
    audioHopCounter -= AUDIO_HOP_SAMPLES;

    // ---- RMS + ZCR over the full window (oldest-to-newest) ----
    double sumSq = 0;
    size_t zeroCrossings = 0;
    int16_t prevSample = audioRing[audioWriteIdx]; // oldest sample in the ring
    size_t idx = audioWriteIdx;
    for (size_t i = 0; i < AUDIO_WINDOW_SAMPLES; i++) {
      int16_t s = audioRing[idx];
      sumSq += (double)s * s;
      if (i > 0 && ((s >= 0) != (prevSample >= 0))) {
        zeroCrossings++;
      }
      prevSample = s;
      idx = (idx + 1) % AUDIO_WINDOW_SAMPLES;
    }
    audioFeaturesOut.rms = sqrt(sumSq / AUDIO_WINDOW_SAMPLES);
    audioFeaturesOut.zcr = (float)zeroCrossings / AUDIO_WINDOW_SAMPLES;

    // ---- Spectral features from the most recent AUDIO_FFT_FRAME_SIZE samples ----
    size_t frameStart = (audioWriteIdx + AUDIO_WINDOW_SAMPLES - AUDIO_FFT_FRAME_SIZE) % AUDIO_WINDOW_SAMPLES;
    for (size_t i = 0; i < AUDIO_FFT_FRAME_SIZE; i++) {
      vReal[i] = (float)audioRing[(frameStart + i) % AUDIO_WINDOW_SAMPLES];
      vImag[i] = 0.0f;
    }

    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    fft.compute(FFTDirection::Forward);
    fft.complexToMagnitude();
    // vReal[0..N/2-1] now holds the magnitude spectrum.

    size_t nBins = AUDIO_FFT_FRAME_SIZE / 2;
    double magSum = 0;
    double weightedFreqSum = 0;
    for (size_t k = 0; k < nBins; k++) {
      float freq = (float)k * AUDIO_SAMPLE_RATE / AUDIO_FFT_FRAME_SIZE;
      magSum += vReal[k];
      weightedFreqSum += freq * vReal[k];
    }
    audioFeaturesOut.spectralCentroid = magSum > 0 ? (float)(weightedFreqSum / magSum) : 0.0f;

    // Spectral entropy: normalize magnitudes into a probability distribution,
    // then Shannon entropy over the bins (higher = noisier/flatter spectrum).
    double entropy = 0;
    if (magSum > 0) {
      for (size_t k = 0; k < nBins; k++) {
        double p = vReal[k] / magSum;
        if (p > 1e-9) {
          entropy -= p * log2(p);
        }
      }
    }
    audioFeaturesOut.spectralEntropy = (float)entropy;

    // Spectral flux: sum of squared differences vs the previous frame's spectrum.
    double flux = 0;
    if (havePrevMag) {
      for (size_t k = 0; k < nBins; k++) {
        double diff = vReal[k] - prevMag[k];
        flux += diff * diff;
      }
    }
    audioFeaturesOut.spectralFlux = (float)flux;
    for (size_t k = 0; k < nBins; k++) {
      prevMag[k] = vReal[k];
    }
    havePrevMag = true;

    audioFeaturesOut.timestampMs = millis();
    audioWindowReady = true;
  }
}

bool featureExtractionGetAudioFeatures(AudioFeatures &out) {
  if (!audioWindowReady) return false;
  out = audioFeaturesOut;
  audioWindowReady = false;
  return true;
}

// ---------------- IMU ----------------

void featureExtractionPushIMU(float accelMag, uint32_t timestampMs) {
  imuRing[imuWriteIdx] = accelMag;
  imuWriteIdx = (imuWriteIdx + 1) % IMU_WINDOW_SAMPLES;
  imuTotalPushed++;
  imuHopCounter++;

  bool bufferFull = imuTotalPushed >= IMU_WINDOW_SAMPLES;
  if (bufferFull && imuHopCounter >= IMU_HOP_SAMPLES) {
    imuHopCounter -= IMU_HOP_SAMPLES;

    double sum = 0, sumSq = 0;
    float peak = 0;
    for (size_t i = 0; i < IMU_WINDOW_SAMPLES; i++) {
      float v = imuRing[i];
      sum += v;
      sumSq += (double)v * v;
      if (fabs(v) > peak) peak = fabs(v);
    }
    float mean = sum / IMU_WINDOW_SAMPLES;
    float variance = (sumSq / IMU_WINDOW_SAMPLES) - (mean * mean);
    if (variance < 0) variance = 0; // guard against float rounding

    imuFeaturesOut.mean = mean;
    imuFeaturesOut.std = sqrt(variance);
    imuFeaturesOut.peak = peak;
    imuFeaturesOut.rms = sqrt(sumSq / IMU_WINDOW_SAMPLES);
    imuFeaturesOut.timestampMs = timestampMs;
    imuWindowReady = true;
  }
}

bool featureExtractionGetIMUFeatures(IMUFeatures &out) {
  if (!imuWindowReady) return false;
  out = imuFeaturesOut;
  imuWindowReady = false;
  return true;
}