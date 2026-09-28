#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#include <Arduino.h>

// Sets up the I2S peripheral for the INMP441 mic.
void audioCaptureInit();

// Blocking read of one chunk (AUDIO_CHUNK_SAMPLES) of 16-bit audio samples
// into outBuf. Returns the number of samples actually read.
// outTimestampMs is set to millis() at the moment the read completes.
size_t audioCaptureRead(int16_t *outBuf, size_t maxSamples, uint32_t *outTimestampMs);

#endif // AUDIO_CAPTURE_H