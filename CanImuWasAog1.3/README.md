# CanImuWasAog v1.3 — STM32 IMU WAS для AgOpenGPS

Скетч для STM32 (например Blue Pill / STM32F103C8).  
Читает абсолютный угол yaw с BNO085 (Game Rotation Vector) и отправляет по CAN.

## Протокол CAN

| Параметр | Значение |
|---|---|
| ID | `0x18FF51E5` (extended) |
| Скорость | 500 кбит/с |
| Частота | 50 Гц |
| Байты [6-7] | int16 × 0.1° (little-endian) |
| Невалидный фрейм | `0x7FFF` в байтах [6-7] |

## Зависимости (Arduino IDE)

- [SparkFun BNO080 Arduino Library](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library)
- [STM32_CAN](https://github.com/pazi88/STM32_CAN)

## Настройки

```cpp
BNO_ADDR  = 0x4B   // адрес BNO085 (или 0x4A)
IMU_RATE_HZ = 50   // частота отправки
```

## Файлы

- `CanImuWasAog1.3.ino` — основной скетч
- `hal_conf_extra.h` — включает модуль CAN в STM32duino (обязателен)

## Принцип работы

Угол рычага руля (STM32) вычитается из угла кузова (Teensy BNO) на стороне Teensy.  
Результат = чистый угол поворота колёс без влияния разворотов трактора.
