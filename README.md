# IMU Bridge

Firmware for the Arduino Nano ESP32 to read data from IMU candidates and publish the data using Zenoh

## Supported IMUs

The table below reflects the ranges currently configured or decoded by the local drivers in `src/sensor/`.

| Device | Interface | Acceleration range (g) | Angular rate range (dps) | Max Frequency (MHz)
| --- | --- | --- | --- | --- |
| `ISM330DLC` | SPI | `±2/4/8/16` | `±125/250/500/1000/2000` | `10` |
| `BMI088` | SPI | `±3/6/12/24` | `±125/250/500/1000/2000` | `10` |
| `ICM-42688-P` | SPI | `±2/4/8/16` | `±15.6/31.2/62.5/125/250/500/1000/2000` | `24` |
| `ICM-45686` | SPI | `±2/4/8/16/32` | `±15.625/31.25/62.5/125/250/500/1000/2000/4000` | `24` |
