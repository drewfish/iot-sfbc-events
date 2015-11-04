/*
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2015 Drew Folta
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 * ----------------------------------------------------------------------------
 * hardware test
 * pins
 *      POS HW      USAGE
 *      1.1 ^RESET  pullup 10k
 *      1.2 ADC     TEST analog
 *      1.3 CH_PD   pullup 10k
 *      1.4 GPIO16  TEST digital
 *      1.5 GPIO14  TEST digital
 *      1.6 GPIO12  TEST digital
 *      1.7 GPIO13  TEST digital
 *      1.8 VCC     -
 *      2.1 TX      ftdi rx
 *      2.2 RX      ftdi tx
 *      2.3 GPIO4   TEST digital
 *      2.4 GPIO5   TEST digital
 *      2.5 GPIO0   pullup 10k -- wire to gnd to program
 *      2.6 GPIO2   -
 *      2.7 GPIO15  pulldown 10k
 *      2.8 GND     -
 * programming
 *      https://github.com/esp8266/esp8266-wiki/wiki/Boot-Process#esp-boot-modes
 *      pulldown gpio15
 *      tie gpio0 to gnd
 *      http://www.esp8266.com/viewtopic.php?f=5&t=3163&start=30
 */


const size_t  APINS_COUNT   = 1;
const uint8_t APINS[]       = {A0};
const size_t  DPINS_COUNT   = 6;
const uint8_t DPINS[]       = {4, 5, 12, 13, 14, 16};


void testAnalogRead() {
    Serial.println(F("ANALOG READ"));
    for (int pin = 0; pin < APINS_COUNT; pin++) {
        Serial.print(F("  "));
        Serial.print(analogRead(APINS[pin]), HEX);
    }
    Serial.println(F(""));
}


void testAnalogWrite() {
    Serial.println(F("ANALOG WRITE"));
    for (int x = 0; x < 256; x += 16) {
        for (int pin = 0; pin < DPINS_COUNT; pin++) {
            analogWrite(DPINS[pin], x);
        }
        delay(150);
    }    
    for (int x = 255; x >= 0; x -= 16) {
        for (int pin = 0; pin < DPINS_COUNT; pin++) {
            analogWrite(DPINS[pin], x);
        }
        delay(150);
    }    
    for (int pin = 0; pin < DPINS_COUNT; pin++) {
        analogWrite(DPINS[pin], 0);
    }
}


void testDigitalWrite() {
    Serial.println(F("DIGITAL WRITE"));
    for (int x = 0; x < 8; x++) {
        for (int pin = 0; pin < DPINS_COUNT; pin++) {
            digitalWrite(DPINS[pin], LOW);
        }
        delay(600);
        for (int pin = 0; pin < DPINS_COUNT; pin++) {
            digitalWrite(DPINS[pin], HIGH);
        }
        delay(600);
    }
}


void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    for (int pin = 0; pin < APINS_COUNT; pin++) {
        pinMode(APINS[pin], INPUT);
    }
    for (int pin = 0; pin < DPINS_COUNT; pin++) {
        pinMode(DPINS[pin], OUTPUT);
    }
    delay(2000);
}


void loop() {
    Serial.println(F("LOOP"));
    delay(1000);
//    testAnalogRead();
//    delay(1000);
    testAnalogWrite();
//    delay(1000);
//    testDigitalWrite();
}


