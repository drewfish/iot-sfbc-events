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
 * just spin the LEDs on the eval board
 *
 */

#include <ESP8266WiFi.h>


#define USE_SERIAL

const char* WIFI_SSID       = "foo";
const char* WIFI_PASS       = "bar";

const uint8_t LED_PINS[]    = {2, 0, 4, 5, 14};
const uint8_t RGBLED_PINS[] = {15, 12, 13}; // red, green, blue
const char* EVENTS_HOST     = "drewfish-iot.herokuapp.com";
const long  EVENTS_PORT     = 80;
const char* EVENTS_REQUEST  = "GET /sfbc/events HTTP/1.1\r\nHost: drewfish-iot.herokuapp.com\r\nConnection: close\r\n\r\n";
long RETRY_PAUSE    = 1000; // milliseconds
void (*STATE)(void) = NULL;
long ERROR_CODE     = 0;
long NOW            = 0;
long GROUND_STATUS  = 0;
long GROUND_TS      = 0;
long ANNEX_STATUS   = 0;
long ANNEX_TS       = 0;


struct Scanner {
    uint8_t m_pin = 0;
    bool m_bounce = false;
    bool m_forward = true;
    void start(bool bounce) {
        m_bounce = bounce;
        m_pin = 0;
        m_forward = true;
        digitalWrite(LED_PINS[m_pin], LOW);     // turn on first pin
    }
    void tick() {
        digitalWrite(LED_PINS[m_pin], HIGH);    // turn off current pin
        // move pin
        if (m_forward) {
            if (m_pin == 4) {
                if (m_bounce) {
                    m_pin = 3;
                    m_forward = false;
                } else {
                    m_pin = 0;
                }
            } else {
                m_pin++;
            }
        } else {
            // the only time we're moving backwards is when m_bounce
            if (m_pin == 0) {
                m_pin = 1;
                m_forward = true;
            } else {
                m_pin--;
            }
        }
        digitalWrite(LED_PINS[m_pin], LOW);     // turn on current pin
    }
    void stop() {
        digitalWrite(LED_PINS[m_pin], HIGH);    // turn off current pin
    }
} scanner;


void stateError() {
#ifdef USE_SERIAL
    Serial.print("STATE error ");
    Serial.println(ERROR_CODE, HEX);
#endif
    for (int pin = 0; pin < 5; pin++) {
        digitalWrite(LED_PINS[pin], LOW);
    }
    delay(500);
    for (int pin = 0; pin < 5; pin++) {
        digitalWrite(LED_PINS[pin], HIGH);
    }
    delay(500);
}


void stateWifiConnecting() {
#ifdef USE_SERIAL
    Serial.println("STATE wifi connecting");
#endif
    scanner.start(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(100);
    while (true) {
        uint8_t status = WiFi.status();
#ifdef USE_SERIAL
        Serial.print("--status ");
        Serial.println(status, HEX);
#endif
        switch (status) {
            case WL_CONNECTED:
                // success
                scanner.stop();
                STATE = stateStatusQuerying;
                return;

            case WL_DISCONNECTED:
            case WL_IDLE_STATUS:
                // keep polling
                scanner.tick();
                delay(300);
                break;

            case WL_NO_SSID_AVAIL:
            case WL_CONNECT_FAILED:
            case WL_CONNECTION_LOST:
                // retry
                scanner.stop();
                delay(RETRY_PAUSE);
                return;

            default:
                // error
                scanner.stop();
                STATE = stateError;
                ERROR_CODE = status;
                return;
        }
    }
}


void stateWifiDisconnecting() {
#ifdef USE_SERIAL
    Serial.println("STATE wifi disconnecting");
#endif
    WiFi.disconnect();
    delay(100);
    STATE = stateStatusDisplaying;
}


void stateStatusQuerying() {
#ifdef USE_SERIAL
    Serial.println("STATE status querying");
#endif
    scanner.start(false);
    WiFiClient client;
    if (! client.connect(EVENTS_HOST, EVENTS_PORT)) {
        scanner.stop();
#ifdef USE_SERIAL
        Serial.println("FAILED to connect");
#endif
        delay(RETRY_PAUSE);
        return;
    }
    client.print(EVENTS_REQUEST);
    while (! client.available()) {
        delay(100);
        scanner.tick();
        if (! client.connected()) {
            scanner.stop();
#ifdef USE_SERIAL
            Serial.println("ABORTED connection");
#endif
            STATE = stateError;
            ERROR_CODE = 0xF0;
            return;
        }
    }
    scanner.stop();
    client.find("\r\n\r\n");    // read until end of headers
    NOW = client.parseInt();
    GROUND_STATUS = client.parseInt();
    GROUND_TS = client.parseInt();
    ANNEX_STATUS = client.parseInt();
    ANNEX_TS = client.parseInt();
#ifdef USE_SERIAL
    Serial.print("now:");
    Serial.print(NOW);
    Serial.print("  status:");
    Serial.print(GROUND_STATUS ? "occupied" : "free");
    Serial.print("  ts:");
    Serial.print(GROUND_TS);
    Serial.print("  status:");
    Serial.print(ANNEX_STATUS ? "occupied" : "free");
    Serial.print("  ts:");
    Serial.println(ANNEX_TS);
#endif
    STATE = stateWifiDisconnecting;
}


void stateStatusDisplaying() {
#ifdef USE_SERIAL
    Serial.println("STATE status displaying");
    Serial.print("  ground:");
    Serial.print(GROUND_STATUS ? "occupied" : "free");
    Serial.print("  annex:");
    Serial.println(ANNEX_STATUS ? "occupied" : "free");
#endif
    analogWrite(RGBLED_PINS[0], GROUND_STATUS ? 1023 : 0);
    analogWrite(RGBLED_PINS[1], ANNEX_STATUS ? 1023 : 0);
    analogWrite(RGBLED_PINS[2], 0);
    scanner.start(true);
    long count = 60;
    while (count--) {
        delay(2000);    // two seconds
        scanner.tick();
    }
    scanner.stop();
    STATE = stateWifiConnecting;
}


void setup() {
#ifdef USE_SERIAL
    Serial.begin(115200);
#endif
    for (int pin = 0; pin < 5; pin++) {
        pinMode(LED_PINS[pin], OUTPUT);
        digitalWrite(LED_PINS[pin], HIGH);
    }
    for (int pin = 0; pin < 3; pin++) {
        pinMode(RGBLED_PINS[pin], OUTPUT);
        analogWrite(RGBLED_PINS[pin], 0);
    }
    randomSeed(analogRead(A0));
    STATE = stateWifiConnecting;
    delay(5000);    // give time to connect a serial console
}


void loop() {
    STATE();
}


