# Arduino_Electronic_Level_MPU5060
Arduino electronic level using MPU6050 IMU with complementary filter and LCD display.

Embedded system that measures tilt angles (Pitch and Roll) using the MPU-6050 IMU and displays the values on an LCD.

## Features
- Real-time tilt measurement
- Complementary filter sensor fusion
- LCD display of Pitch and Roll
- LED level indicator
- Buzzer alarm for critical tilt

## Hardware
- Arduino Uno (ATmega328p)
- MPU-6050 IMU
- 16x2 I2C LCD
- LEDs
- Active buzzer

## Architecture
MPU6050 → Arduino (sensor fusion) → LCD display + LEDs + buzzer
