#include "IMUCapture.h"
#include "config.h"
#include <Wire.h>

void imuCaptureInit() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000);

  // Confirm we're actually talking to an MPU-6500 (WHO_AM_I = 0x70).
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1, (uint8_t)true);
  uint8_t whoAmI = Wire.read();
  if (whoAmI != MPU6500_WHO_AM_I_VAL) {
    Serial.printf("MPU-6500 not found (WHO_AM_I=0x%02X, expected 0x%02X) - check wiring\n",
                  whoAmI, MPU6500_WHO_AM_I_VAL);
    while (true) delay(1000);
  }

  // Wake the sensor (defaults to sleep mode on power-up).
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
}

bool imuCaptureRead(IMUSample &sample) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  // 14 bytes: accel (6) + temp (2) + gyro (6)
  uint8_t bytesReceived = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (bytesReceived != 14) {
    return false;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // discard temperature
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  sample.accelX = rawAx / MPU_ACCEL_SCALE;
  sample.accelY = rawAy / MPU_ACCEL_SCALE;
  sample.accelZ = rawAz / MPU_ACCEL_SCALE;
  sample.gyroX = rawGx / MPU_GYRO_SCALE;
  sample.gyroY = rawGy / MPU_GYRO_SCALE;
  sample.gyroZ = rawGz / MPU_GYRO_SCALE;
  sample.timestampMs = millis();

  return true;
}