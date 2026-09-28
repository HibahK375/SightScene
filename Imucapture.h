#ifndef IMU_CAPTURE_H
#define IMU_CAPTURE_H

#include <Arduino.h>

struct IMUSample {
  float accelX, accelY, accelZ;  // g
  float gyroX, gyroY, gyroZ;     // deg/s
  uint32_t timestampMs;
};

// Sets up I2C, verifies WHO_AM_I, and wakes the MPU-6500 (raw register access).
void imuCaptureInit();

// Reads one sample from the MPU-6050 into sample. Returns true on success.
bool imuCaptureRead(IMUSample &sample);

#endif // IMU_CAPTURE_H