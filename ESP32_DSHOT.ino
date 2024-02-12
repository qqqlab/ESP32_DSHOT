/*==========================================================================================
Demo Progam for: ESP32_DSHOT - ESP32 Arduino library for bidirectional DSHOT

Starts DSHOT300_BIDIR and DSHOT300 on pins 23 and 21 and cycles thottle.

Tested on ESP32 with Arduino-ESP32 v2.0.14 (ESP-IDF v4.4), might work on other ESP32 variants with modifications.

MIT License

Copyright (c) 2024 qqqlab - https://github.com/qqqlab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

#include "ESP32_DSHOT.h"

DSHOT dshot3b;
DSHOT dshot3;

void setup() {
  Serial.begin(115200);

  //setup the dshot instances: set pin and DSHOT speed
  dshot3b.begin(23, DSHOT::DSHOT300_BIDIR);
  dshot3.begin(21, DSHOT::DSHOT300);

  //arm the dshot instances
  DSHOT::arm(); 
}


int throttle = 0;
int dir = 1;

void loop() {
  dshot3b.set(throttle);
  dshot3.set(throttle);

  throttle+=dir;
  if(throttle>300) dir = -1;
  if(throttle<200) dir = 1;

  Serial.printf(" throttle=%d ",throttle);
  Serial.printf(" erpm_us=%d ",dshot3b.erpm_us);
  Serial.printf(" temp=%dC ",dshot3b.telem[1]);
  Serial.printf(" bidir_cnt=%d bidir_ok=%d",dshot3b.telem_cnt, dshot3b.telem_ok_cnt);
  Serial.println(); 

  delay(1);
}
