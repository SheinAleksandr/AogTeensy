// IMU WAS via CAN3 — абсолютный угол рычага руля от STM32
// STM32 шлёт: ID=0x18FF51E5 extended, bytes[6-7] = yaw angle int16 x10 deg (LE)
//             0x7FFF = invalid
//
// Алгоритм (как TM171/BNO-скетчи):
//   steerAngle = Δheading_рычага − Δheading_кузова
//
// Обнуление: вручную из AOG + автокоррекция в движении (едем прямо → угол тянется к нулю)

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Imu_Bus;

extern float gpsSpeed;            // км/ч, из Autosteer.ino
extern float vehicleYawIntegDeg;  // накопленное рыскание кузова (Teensy BNO) с момента обнуления
extern bool  useBNO08x;           // есть ли BNO на тинси

static const uint32_t IMU_WAS_CAN_ID      = 0x18FF51E5;
static const uint32_t IMU_WAS_TIMEOUT_MS  = 200;    // нет фреймов → invalid

// Параметры автокоррекции в движении
static const float    AZ_SPEED_MIN_KMH   = 1.0f;   // минимальная скорость
static const float    AZ_YAW_RATE_MAX_DPS = 0.8f;  // едем прямо если |угловая скорость рычага| < этого
static const uint32_t AZ_STABLE_MS       = 400;    // сколько мс должны быть прямо
static const float    AZ_BETA            = 0.05f;  // скорость коррекции (5% за тик)

static float    imuWasAngleDeg  = 0.0f;  // накопленное изменение yaw рычага (Δsum)
static float    imuWasZeroDeg   = 0.0f;  // imuWasAngleDeg при обнулении
static float    imuWasRelRate   = 0.0f;  // скорость компенсированного угла (рычаг − кузов), °/с
static float    prevCanYawDeg   = 0.0f;  // предыдущий угол с CAN, для вычисления дельты
static bool     prevCanYawInit  = false;
static uint32_t imuWasLastMs    = 0;
static uint32_t azStableMs      = 0;
bool            imuWasCanValid  = false;

void ImuWasZero()
{
    imuWasZeroDeg      = imuWasAngleDeg;
    vehicleYawIntegDeg = 0.0f;  // компенсация считается от текущего курса
    azStableMs         = 0;
    Serial.println("IMU WAS zeroed");
}

float GetImuWasAngleDeg()
{
    float raw = imuWasAngleDeg - imuWasZeroDeg;
    return useBNO08x ? (raw - vehicleYawIntegDeg) : raw;
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
    if (fabsf(imuWasRelRate) > AZ_YAW_RATE_MAX_DPS) { azStableMs = 0; return; }

    azStableMs += dtMs;
    float angle = GetImuWasAngleDeg();
    if (azStableMs >= AZ_STABLE_MS)
    {
        imuWasZeroDeg += AZ_BETA * angle;
    }
}

void ImuWasCan_Loop()
{
    CAN_message_t msg;
    while (Imu_Bus.read(msg))
    {
        if (!msg.flags.extended)      continue;
        if (msg.id != IMU_WAS_CAN_ID) continue;
        if (msg.len < 8)              continue;

        int16_t yawX10 = (int16_t)((uint16_t)msg.buf[6] | ((uint16_t)msg.buf[7] << 8));

        if (yawX10 == (int16_t)0x7FFF) { imuWasCanValid = false; continue; }

        float canYawDeg = yawX10 * 0.1f; // абсолютный угол [-180..+180]

        uint32_t now = millis();

        if (!prevCanYawInit)
        {
            prevCanYawDeg  = canYawDeg;
            prevCanYawInit = true;
            imuWasLastMs   = now;
            imuWasCanValid = true;
            continue;
        }

        // дельта угла с обработкой перехода через ±180°
        float dYaw = canYawDeg - prevCanYawDeg;
        if (dYaw >  180.0f) dYaw -= 360.0f;
        if (dYaw < -180.0f) dYaw += 360.0f;

        prevCanYawDeg = canYawDeg;

        uint32_t dtMs = now - imuWasLastMs;
        float dt = dtMs * 0.001f;

        if (dt > 0.0f && dt <= 0.2f)
        {
            imuWasAngleDeg += dYaw;
            // скорость компенсированного угла: рыскание трактора отменяется вычитанием кузова
            float compAngle = GetImuWasAngleDeg();
            static float prevCompAngle = 0.0f;
            imuWasRelRate = (dt > 0.001f) ? ((compAngle - prevCompAngle) / dt) : 0.0f;
            prevCompAngle = compAngle;
            updateAutoZero(dtMs);
        }

        imuWasLastMs   = now;
        imuWasCanValid = true;
    }

    // таймаут
    if (imuWasCanValid && imuWasLastMs != 0 &&
        (uint32_t)(millis() - imuWasLastMs) > IMU_WAS_TIMEOUT_MS)
    {
        imuWasCanValid = false;
        azStableMs     = 0;
    }
}
