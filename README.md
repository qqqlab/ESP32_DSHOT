# ESP32_DSHOT

ESP32 Arduino library for bidirectional DSHOT ESCs

This library uses the Remote Control Peripheral (RMT). The library uses the low level RMT API, because the RMT driver is not fast enough for bidrectional dshot, which has a 30 us pause between the outgoing and incoming frame.

|MCU|RMT Devices|Samples|
|-|-|-|
ESP32 | 8 R/W |128
ESP32-S2 | 4 R/W |128
ESP32-S3 | 4R 4W |96
ESP32-C3 | 2R 2W |96
ESP32-C6 | 2R 2W |96
ESP32-H2 | 2R 2W |96
ESP32-C2 (ESP8684)| no RMT|

The most atractive is the ESP32 which supports up to 8 DSHOT ESCs
