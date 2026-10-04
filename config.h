#ifndef CONFIG_H
#define CONFIG_H

// ---------- Audio (I2S / INMP441) ----------
#define I2S_WS_PIN          9
#define I2S_SCK_PIN         10
#define I2S_SD_PIN          8
#define I2S_PORT            I2S_NUM_0
#define AUDIO_SAMPLE_RATE   16000
#define AUDIO_CHUNK_SAMPLES 256      // samples per audio packet (~16ms @16kHz)
#define AUDIO_BITS_PER_SAMPLE 32     // INMP441 outputs 24-bit data in a 32-bit slot

// ---------- IMU (I2C / MPU-6050) ----------
#define IMU_SDA_PIN     12
#define IMU_SCL_PIN     11
#define MPU_ADDR        0x68
#define IMU_SAMPLE_HZ   100
#define IMU_PERIOD_MS   (1000 / IMU_SAMPLE_HZ)

// MPU-6500 registers (raw register access — no library matches the 6500's
// WHO_AM_I, so the Adafruit MPU6050 lib will not detect this chip)
#define MPU_REG_WHO_AM_I     0x75
#define MPU6500_WHO_AM_I_VAL 0x70
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_ACCEL_SCALE      16384.0f  // LSB/g at default +-2g range
#define MPU_GYRO_SCALE       131.0f    // LSB/(deg/s) at default +-250dps range

// ---------- Packet types (for MQTT / dashboard framing) ----------
#define TYPE_INIT   0x00
#define TYPE_AUDIO  0x01
#define TYPE_IMU    0x02
#define TYPE_EVENT  0x03

// ---------- Serial ----------
#define SERIAL_BAUD 115200

// ---------- Feature extraction ----------
#define WINDOW_MS        1000   // feature window length
#define HOP_MS           250    // feature hop (window slide) length

#define AUDIO_WINDOW_SAMPLES (AUDIO_SAMPLE_RATE * WINDOW_MS / 1000)  // 16000
#define AUDIO_HOP_SAMPLES    (AUDIO_SAMPLE_RATE * HOP_MS / 1000)     // 4000
#define AUDIO_FFT_FRAME_SIZE 512  // spectral features computed on this many
                                  // most-recent samples within the window,
                                  // not the full window (keeps FFT cost low)

#define IMU_WINDOW_SAMPLES (IMU_SAMPLE_HZ * WINDOW_MS / 1000)  // 100
#define IMU_HOP_SAMPLES    (IMU_SAMPLE_HZ * HOP_MS / 1000)     // 25

// ---------- Anomaly detection (EMA-weighted z-score) ----------
#define EMA_ALPHA          0.05f  // adaptation speed of the running baseline
#define Z_SCORE_THRESHOLD  3.0f   // |z| above this = anomalous
#define ANOMALY_WARMUP_WINDOWS 60 // windows before z-scores are trusted (~5s at 250ms hop)

// ---------- Fusion ----------
#define FUSION_STALE_MS 1000       // if the other modality hasn't reported in this long, ignore it
#define FUSION_STRONG_Z_MULTIPLIER 1.5f // single-sensor z above (threshold * this) = "moderate" not "weak"
#define FUSION_ELEVATED_Z 1.5f     // below full anomaly threshold, but "elevated" - used to catch
                                   // two co-elevated-but-individually-mild signals

// ---------- WiFi / MQTT ----------
// Fill these in with your own network + broker details.
#define WIFI_SSID       "hibah"
//#define WIFI_PASSWORD   "your-wifi-password"
#define MQTT_BROKER     "10.198.131.153"   // your broker's IP (e.g. Mosquitto on your PC/Pi)
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "envmonitor-esp32"
#define MQTT_TOPIC_EVENT "envmonitor/event"
#define MQTT_TOPIC_AUDIO "envmonitor/audio"
#define MQTT_TOPIC_IMU   "envmonitor/imu"

// ---------- Context: time-of-day ----------
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 19800   // India Standard Time = UTC+5:30 = 5*3600 + 30*60    
#define DAYLIGHT_OFFSET_SEC 0
#define NIGHT_START_HOUR 23        // 11pm
#define NIGHT_END_HOUR   6         // 6am
#define NIGHT_THRESHOLD_MULTIPLIER 0.8f // lower threshold at night = more sensitive

#define EVENT_HISTORY_SIZE 50
#define RECURRENCE_WINDOW_MS (30UL * 60UL * 1000UL)
#define RECURRENCE_COUNT_THRESHOLD 3
#define RECURRENCE_THRESHOLD_MULTIPLIER 0.8f

#endif // CONFIG_H