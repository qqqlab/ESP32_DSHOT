# ESP32_DSHOT

ESP32 Arduino library for bidirectional DSHOT ESCs

This libary supports up to 8 unidirectional or 4 bidirectional DSHOT ESCs on ESP32.

This library uses the Remote Control Peripheral (RMT). The low level RMT API is used because the RMT driver is not fast enough to grab bidrectional dshot data, which has a 30 us pause between the outgoing and incoming frame.

Tested on ESP32 with Adruino-ESP32 v2.0.14 (ESP-IDF v4.4). Not tested on other ESP32-XX variants, which probably need adaption of the code because of different RMT hardware.

## Overview of RMT on ESP32-XX variants

|MCU|RMT Devices|Samples|
|-|-|-|
ESP32 | 8 R/W |128
ESP32-S2 | 4 R/W |128
ESP32-S3 | 4R 4W |96
ESP32-C3 | 2R 2W |96
ESP32-C6 | 2R 2W |96
ESP32-H2 | 2R 2W |96
ESP32-C2 (ESP8684)| no RMT|
