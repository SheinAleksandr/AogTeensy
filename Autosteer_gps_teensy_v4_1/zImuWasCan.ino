// IMU WAS via CAN3 — интеграция угловой скорости в угол на Teensy
// STM32 шлёт: ID=0x18FF51E5 extended, bytes[6-7] = yaw rate int16 x10 deg/s
//
// Обнуление:
//   1. Автоматически при старте (AUTO_ZERO_MS после первого фрейма)
//   2. Кнопка "Zero WAS" в AOG (перехватывается через PGN252 в Autosteer.ino)
//   3. Автокоррекция в движении: едем прямо → угол медленно тянется к нулю

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Imu_Bus;

extern float gpsSpeed; // км/ч, из Autosteer.ino

static const uint32_t IMU_WAS_CAN_ID           = 0x18FF51E5;
static const uint32_t IMU_WAS_TIMEOUT_MS        = 200;    // нет фреймов → invalid
static const uint32_t AUTO_ZERO_MS              = 3000;   // автообнуление через 3с после старта

// Параметры автокоррекции в движении
static const float    AZ_SPEED_MIN_KMH          = 1.0f;   // минимальная скорость
static const float    AZ_YAW_RATE_MAX_DPS       = 0.8f;   // едем прямо если |yawRate| < этого
static const float    AZ_ANGLE_MAX_DEG          = 10.0f;  // применяем только если угол < этого
static const uint32_t AZ_STABLE_MS              = 400;    // сколько мс должны быть прямо
static const float    AZ_BETA                   = 0.05f;  // скорость коррекции (5% за тик)

static float    imuWasAngleDeg  = 0.0f;
static float    imuWasZeroDeg   = 0.0f;
static float    imuWasYawRate   = 0.0f;   // последний yaw rate, для автокоррекции
static uint32_t imuWasLastMs    = 0;
static uint32_t imuWasFirstMs   = 0;
static bool     autoZeroDone    = false;
static uint32_t azStableMs      = 0;      // накопленное время "прямо"
bool            imuWasCanValid  = false;

void ImuWasZero()
{
    imuWasZeroDeg = imuWasAngleDeg;
    azStableMs = 0;
    Serial.println("IMU WAS zeroed");
}

float GetImuWasAngleDeg()
{
    return imuWasAngleDeg - imuWasZeroDeg;
}

void ImuWasCan_Setup()
{
    Imu_Bus.begin();
    Imu_Bus.setBaudRate(500000);
    Serial.println("IMU WAS CAN3 ready (0x18FF51E5)");
}

static void updateAutoZero(uint32_t dtMs)
{
    if (!imuWasCanValid) { azStableMs = 0; return; }
    if (gpsSpeed < AZ_SPEED_MIN_KMH) { azStableMs = 0; return; }
    if (fabsf(imuWasYawRate) > AZ_YAW_RATE_MAX_DPS) { azStableMs = 0; return; }

    float angle = GetImuWasAngleDeg();
    if (fabsf(angle) > AZ_ANGLE_MAX_DEG) { azStableMs = 0; return; }

    azStableMs += dtMs;
    if (azStableMs >= AZ_STABLE_MS)
    {
        // медленно тянем ноль к текущему углу
        imuWasZeroDeg += AZ_BETA * angle;
    }
}

void ImuWasCan_Loop()
{
    CAN_message_t msg;
    while (Imu_Bus.read(msg))
    {
        if (!msg.flags.extended)       continue;
        if (msg.id != IMU_WAS_CAN_ID)  continue;
        if (msg.len < 8)                continue;

        int16_t yawRateX10 = (int16_t)((uint16_t)msg.buf[6] | ((uint16_t)msg.buf[7] << 8));

        if (yawRateX10 == (int16_t)0x7FFF) { imuWasCanValid = false; continue; }

        float yawRate = yawRateX10 * 0.1f; // deg/s
        imuWasYawRate = yawRate;

        uint32_t now = millis();
        if (imuWasFirstMs == 0) imuWasFirstMs = now;

        if (imuWasLastMs != 0)
        {
            float dt = (float)(now - imuWasLastMs) * 0.001f;
            if (dt > 0.0f && dt <= 0.2f)
            {
                imuWasAngleDeg += yawRate * dt;
                updateAutoZero((uint32_t)((now - imuWasLastMs)));
            }
        }

        imuWasLastMs  = now;
        imuWasCanValid = true;
    }

    // таймаут
    if (imuWasCanValid && imuWasLastMs != 0 &&
        (uint32_t)(millis() - imuWasLastMs) > IMU_WAS_TIMEOUT_MS)
    {
        imuWasCanValid = false;
        azStableMs = 0;
    }

    // автообнуление при старте
    if (!autoZeroDone && imuWasFirstMs != 0 &&
        (uint32_t)(millis() - imuWasFirstMs) >= AUTO_ZERO_MS)
    {
        ImuWasZero();
        autoZeroDone = true;
        Serial.println("IMU WAS auto-zeroed at startup");
    }
}
