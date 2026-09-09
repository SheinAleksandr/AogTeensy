#include <Arduino.h>
#include <Wire.h>

#include <SparkFun_BNO080_Arduino_Library.h>
BNO080 bno;
bool bnoOk = false;

#include "STM32_CAN.h"
STM32_CAN Can1(CAN1);

// ---------------------------
// Settings
// ---------------------------
static const uint32_t CAN_BAUD       = 500000;
static const uint32_t CAN_ID_IMU_EXT = 0x18FF51E5;
static const uint16_t IMU_RATE_HZ    = 50;
static const uint32_t SEND_PERIOD_MS = 1000UL / IMU_RATE_HZ;
static const uint8_t  BNO_ADDR       = 0x4B;

// Timeout: if no fresh IMU data, mark frame invalid
static const uint32_t IMU_TIMEOUT_MS = 200;

// ---------------------------
// Helpers
// ---------------------------
// Yaw from quaternion (Game Rotation Vector), result in degrees [-180, +180]
static float yawFromQuat(float qi, float qj, float qk, float qr)
{
  float t3 = 2.0f * (qr * qk + qi * qj);
  float t4 = 1.0f - 2.0f * (qj * qj + qk * qk);
  return atan2f(t3, t4) * 57.2957795f;
}

static inline int16_t clamp_i16_x10(float deg, float min_deg, float max_deg)
{
  if (deg < min_deg) deg = min_deg;
  if (deg > max_deg) deg = max_deg;
  int32_t v = (int32_t)lroundf(deg * 10.0f);
  if (v < -32768) v = -32768;
  if (v >  32767) v =  32767;
  return (int16_t)v;
}

// Pack CAN frame: bytes [6-7] = yaw angle x10 deg (int16, LE)
// Invalid marker: 0x7FFF in bytes [6-7]
static inline void pack_imu_can8(uint8_t data[8], float yaw_deg)
{
  data[0] = 0xFF; data[1] = 0xFF;   // heading  — not used
  data[2] = 0xFF; data[3] = 0x7F;   // roll     — not used
  data[4] = 0xFF; data[5] = 0x7F;   // pitch    — not used
  int16_t y = clamp_i16_x10(yaw_deg, -3000.0f, 3000.0f);
  data[6] = (uint8_t)(y & 0xFF);
  data[7] = (uint8_t)((uint16_t)y >> 8);
}

// ---------------------------
// CAN send
// ---------------------------
static void sendImuCan(float yawDeg, bool valid)
{
  uint8_t d[8];
  if (!valid) {
    d[0] = 0xFF; d[1] = 0xFF;
    d[2] = 0xFF; d[3] = 0x7F;
    d[4] = 0xFF; d[5] = 0x7F;
    d[6] = 0xFF; d[7] = 0x7F;
  } else {
    pack_imu_can8(d, yawDeg);
  }

  CAN_message_t msg;
  msg.id = CAN_ID_IMU_EXT;
  msg.flags.extended = 1;
  msg.len = 8;
  memcpy(msg.buf, d, 8);
  Can1.write(msg);
}

// ---------------------------
// Setup
// ---------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Init CAN...");
  Can1.begin();
  Can1.setBaudRate(CAN_BAUD);

  Serial.println("Init BNO08x...");
  bnoOk = bno.begin(BNO_ADDR, Wire);
  if (!bnoOk) {
    Serial.println("BNO08x not detected. Check wiring or try 0x4A.");
    // не зависаем — CAN уже работает, шлём невалидные фреймы
  } else {
    // Game Rotation Vector: абсолютный угол без магнитометра
    bno.enableGameRotationVector(1000 / IMU_RATE_HZ);
  }

  Serial.printf("Ready: yaw angle -> CAN 0x%08X @ %d Hz\n", CAN_ID_IMU_EXT, IMU_RATE_HZ);
}

// ---------------------------
// Loop
// ---------------------------
void loop() {
  static uint32_t tSend = 0;
  static float yawDeg = 0.0f;
  static uint32_t lastImuMs = 0;
  static bool hasData = false;

  if (bnoOk && bno.dataAvailable()) {
    float qi = bno.getQuatI();
    float qj = bno.getQuatJ();
    float qk = bno.getQuatK();
    float qr = bno.getQuatReal();
    yawDeg = yawFromQuat(qi, qj, qk, qr);
    lastImuMs = millis();
    hasData = true;
  }

  uint32_t now = millis();
  if ((uint32_t)(now - tSend) >= SEND_PERIOD_MS) {
    tSend += SEND_PERIOD_MS;
    if ((uint32_t)(now - tSend) > (5UL * SEND_PERIOD_MS)) {
      tSend = now;
    }

    bool valid = hasData && ((uint32_t)(now - lastImuMs) < IMU_TIMEOUT_MS);
    sendImuCan(yawDeg, valid);
  }
}
