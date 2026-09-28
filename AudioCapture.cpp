#include "AudioCapture.h"
#include "config.h"
#include <driver/i2s.h>

// Scratch buffer for raw 32-bit I2S samples (INMP441 sends 24-bit audio
// left-justified in a 32-bit slot).
static int32_t rawBuf[AUDIO_CHUNK_SAMPLES];

void audioCaptureInit() {
  i2s_config_t i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = (i2s_bits_per_sample_t)AUDIO_BITS_PER_SAMPLE,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = AUDIO_CHUNK_SAMPLES,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pinConfig = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  i2s_driver_install(I2S_PORT, &i2sConfig, 0, NULL);
  i2s_set_pin(I2S_PORT, &pinConfig);
  i2s_zero_dma_buffer(I2S_PORT);
}

size_t audioCaptureRead(int16_t *outBuf, size_t maxSamples, uint32_t *outTimestampMs) {
  size_t samplesToRead = min(maxSamples, (size_t)AUDIO_CHUNK_SAMPLES);
  size_t bytesRead = 0;

  i2s_read(I2S_PORT, rawBuf, samplesToRead * sizeof(int32_t), &bytesRead, portMAX_DELAY);
  size_t samplesRead = bytesRead / sizeof(int32_t);

  // Timestamp taken at read completion. Because the read blocks on I2S DMA,
  // this marks roughly the end of the audio chunk, not its start - fine for
  // the coarse (ms-level) alignment check we're doing; note the systematic
  // ~chunk-duration offset (AUDIO_CHUNK_SAMPLES / AUDIO_SAMPLE_RATE) if you
  // need tighter alignment later.
  if (outTimestampMs) {
    *outTimestampMs = millis();
  }

  // Convert 32-bit I2S samples (24-bit audio, left-justified) down to 16-bit.
  for (size_t i = 0; i < samplesRead; i++) {
    outBuf[i] = (int16_t)(rawBuf[i] >> 16);
  }

  return samplesRead;
}