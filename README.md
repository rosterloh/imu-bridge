# IMU Bridge

Firmware for the Arduino Nano ESP32 to read data from IMU candidates and publish the data using Zenoh

## Supported IMUs

The table below reflects the ranges currently configured or decoded by the local drivers in `src/sensor/`.

| Device | Interface | Acceleration range (g) | Angular rate range (dps) | Max Frequency (MHz) |
| --- | --- | --- | --- | --- |
| `ISM330DLC` | SPI | `±2/4/8/16` | `±125/250/500/1000/2000` | `10` |
| `BMI088` | SPI | `±3/6/12/24` | `±125/250/500/1000/2000` | `10` |
| `ICM-42688-P` | SPI | `±2/4/8/16` | `±15.6/31.2/62.5/125/250/500/1000/2000` | `24` |
| `ICM-45686` | SPI | `±2/4/8/16/32` | `±15.625/31.25/62.5/125/250/500/1000/2000/4000` | `24` |

## Arduino Nano ESP32 Pinout

This drawing shows the pins currently used by the firmware in `src/board.rs`.

```text
     GPIO19 USB D-
     GPIO20 USB D+

            Arduino Nano
                ESP32
           +------------+
SCK GPIO48 D13        D12 GPIO47 MISO
           3V3        D11 GPIO38 MOSI
    GPIO46 B0         D10 GPIO21 CS0
    GPIO1  A0          D9 GPIO18 CS1
    GPIO2  A1          D8 GPIO17
    GPIO3  A2          D7 GPIO10
    GPIO4  A3          D6 GPIO9
SDA GPIO11 A4          D5 GPIO8
SCL GPIO12 A5          D4 GPIO7
    GPIO13 A6          D3 GPIO6
    GPIO14 A7          D2 GPIO5
           VBUS       GND
    GPIO0  B1         RST
           GND        RX0 GPIO44 D0
           VIN        TX1 GPIO43 D1
           +------------+

                      D14  GPIO46  LED0 (red)
                      D15  GPIO0   LED1 (green)
                      D16  GPIO45  LED2 (blue)
```
