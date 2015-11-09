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
 * goals
 *      only show status of the ground floor
 *      tri-state:  free, soon-to-be-occupied, occupied
 * state matchine
 *      boot
 *          setup hardware
 *          ->connecting
 *      connecting
 *          establishes the WiFi connection
 *          ->querying if we connected
 *          ->connecting (retry) if we failed to connect
 *          ->error if something weird/bad happened
 *      querying
 *          makes the HTTP query
 *          ->disconnecting if we got a response
 *          ->querying (retry) if we failed to query
 *          ->error if something weird/bad happened
 *      disconnecting
 *          disconnects the WiFi connection
 *          ->displaying
 *      displaying
 *          updates UI to show state of the ground floor
 *          ->sleeping
 *      sleeping
 *          initiates sleep
 *          ->connecting to update schedule
 *      error
 *          blinks red
 *          terminal state -- powercycle to start again
 * pins
 *      POS HW      USAGE                   NOTES
 *      1.1 ^RESET  pullup 10k
 *      1.2 ADC     -
 *      1.3 CH_PD   pullup 10k
 *      1.4 GPIO16  wire to RESET           see "deep sleep" below
 *      1.5 GPIO14  RED: status error
 *      1.6 GPIO12  GREEN: status querying
 *      1.7 GPIO13  BLUE: status connecting
 *      1.8 VCC     -
 *      2.1 TX      -
 *      2.2 RX      -
 *      2.3 GPIO5   servo enable            n-channel mosfet, on=high
 *      2.4 GPIO4   servo position
 *      2.5 GPIO0   pullup 10k              see "deep sleep" below
 *      2.6 GPIO2   pullup 10k              see "deep sleep" below
 *      2.7 GPIO15  pulldown 10k
 *      2.8 GND     -
 * deep sleep
 *      pullup gpio0 & gpio2
 *      connect gpio16 to reset
 *      perhaps use lower ohm pullups to guard against high-RF environment
 *      can get stuck in "zombie sleep" mode
 *      http://www.esp8266.com/viewtopic.php?f=18&t=1418
 * programming
 *      tie gpio0 to GND to flash from serial
 */


#include <ESP8266WiFi.h>


#define USE_SERIAL

const char WIFI_SSID[]      = "foo";
const char WIFI_PASS[]      = "bar";
const char EVENTS_HOST[]    = "drewfish-iot.herokuapp.com";
const long EVENTS_PORT      = 80;
const char EVENTS_REQUEST[] = "GET /sfbc/events HTTP/1.1\r\nHost: drewfish-iot.herokuapp.com\r\nConnection: close\r\n\r\n";

const uint8_t PIN_SERVO_ENABLE      = 5;
const uint8_t PIN_SERVO_POSITION    = 4;
const uint8_t PINS_RGBLED[]         = {14, 12, 13}; // red, green, blue
void (*STATE)(void)         = NULL;
long ERROR_CODE             = 0;
unsigned long LAST_MS       = 0;    // (milliseconds) arduino millis() from last query
unsigned long LAST_WC       = 0;    // (seconds) wallclock time from last query
boolean GROUND_OCCUPIED     = false;
unsigned long GROUND_WC     = 0;    // (seconds) free: when next occupied -- occupied: when free

long S_SOON             = 2400;     // (seconds) size of "occupied soon" window
long MS_UPDATE          = 10000;    // how often to query the events webservice
long MS_BOOT_PAUSE      = 5000;     // give time to connect serial console
long MS_RETRY_PAUSE     = 1000;     // pause before trying again (connecting or querying)
long MS_STATUS_ERROR    = 500;      // blink cycle when there's an error
long MS_STATUS_CONNECTING = 300;    // blink cycle when connecting
long MS_STATUS_QUERYING = 100;      // blink cycle when querying
long MS_WIFI_SLEW       = 100;      // give Arduino WiFi API time to effect change
long MS_SERVO_SLEW      = 600;      // give servo time to move into position
long MS_SERVO_REFRESH   = 15;       // how often to remind servo that it's supposed to be doing something
long US_SERVO_PULSE[3]  = { 670, 1500, 2700 };  // microseconds for each position


// displays the operation status
struct StatusDevice {
    uint8_t m_pin;
    bool m_on;
    void setup() {
        m_on = false;
        for (uint8_t pin = 0; pin < 3; pin++) {
            pinMode(PINS_RGBLED[pin], OUTPUT);
            digitalWrite(PINS_RGBLED[pin], LOW);
        }
    }
    // 0=red 1=green 2=blue
    void on(uint8_t pin) {
        m_pin = pin;
        m_on = true;
        _update();
    }
    void toggle() {
        m_on = !m_on;
        _update();
    }
    void off() {
        m_on = false;
        _update();
    }
    void _update() {
        digitalWrite(PINS_RGBLED[m_pin], m_on ? HIGH : LOW);
    }
} devStatus;


// turn the flag
struct FlagDevice {
    // The Servo library seems to be tripping up the ESP8266WiFi library,
    // so we'll do the servo control by hand.
    void setup() {
        pinMode(PIN_SERVO_ENABLE, OUTPUT);
        pinMode(PIN_SERVO_POSITION, OUTPUT);
        digitalWrite(PIN_SERVO_ENABLE, LOW);
        digitalWrite(PIN_SERVO_POSITION, LOW);
        // calibrate
        move(2);
        move(0);
    }
    // 0=left 1=middle 2=right
    void move(uint8_t pos) {
        digitalWrite(PIN_SERVO_ENABLE, HIGH);
        for (int8_t i = (MS_SERVO_SLEW / MS_SERVO_REFRESH); i >= 0; i--) {
            digitalWrite(PIN_SERVO_POSITION, HIGH);
            delayMicroseconds(US_SERVO_PULSE[pos]);
            digitalWrite(PIN_SERVO_POSITION, LOW);
            delay(MS_SERVO_REFRESH);
        }
        digitalWrite(PIN_SERVO_ENABLE, LOW);
    }
} devFlag;


// returns the current wallclock time
// (uses LAST_MS and LAST_WC)
unsigned long getWallclock() {
    unsigned long now_ms = millis();
    unsigned long now_wc = LAST_WC;
    if (now_ms < LAST_MS) {
        // millis() overflowed
        now_wc += (0xFFFFFFFF - LAST_MS) / 1000;    // seconds from LAST_MS to overflow
        now_wc += now_ms / 1000;                    // seconds from overflow to now_ms
    } else {
        now_wc += (now_ms - LAST_MS) / 1000;
    }
    return now_wc;
}


void stateError() {
#ifdef USE_SERIAL
    Serial.print("STATE ERROR ");
    Serial.println(ERROR_CODE, HEX);
#endif
    devStatus.on(0);
    delay(MS_STATUS_ERROR);
    devStatus.off();
    delay(MS_STATUS_ERROR);
}


void stateConnecting() {
#ifdef USE_SERIAL
    Serial.println("STATE CONNECTING");
    // DEBUGGING -- WiFi.printDiag(Serial);
#endif
    devStatus.on(2);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(MS_WIFI_SLEW);
    while (true) {
        uint8_t status = WiFi.status();
#ifdef USE_SERIAL
        Serial.print("--status ");
        Serial.println(status, HEX);
#endif
        switch (status) {
            case WL_CONNECTED:
                // success
                devStatus.off();
                STATE = stateQuerying;
                return;

            case WL_DISCONNECTED:
            case WL_IDLE_STATUS:
                // keep polling
                devStatus.toggle();
                delay(MS_STATUS_CONNECTING);
                break;

            case WL_NO_SSID_AVAIL:
            case WL_CONNECT_FAILED:
            case WL_CONNECTION_LOST:
                // retry
                devStatus.off();
                delay(MS_RETRY_PAUSE);
                return;

            default:
                // error
                devStatus.off();
                STATE = stateError;
                ERROR_CODE = status;
                return;
        }
    }
}


void stateQuerying() {
#ifdef USE_SERIAL
    Serial.println("STATE QUERYING");
#endif
    devStatus.on(1);
    WiFiClient client;
    if (! client.connect(EVENTS_HOST, EVENTS_PORT)) {
#ifdef USE_SERIAL
        Serial.println("--FAILED to connect");
#endif
        devStatus.off();
        delay(MS_RETRY_PAUSE);
        return;
    }
    client.print(EVENTS_REQUEST);
    while (! client.available()) {
        delay(MS_STATUS_QUERYING);
        devStatus.toggle();
        if (! client.connected()) {
#ifdef USE_SERIAL
            Serial.println("--ABORTED connection");
#endif
            devStatus.off();
            STATE = stateError;
            ERROR_CODE = 0xF0;
            return;
        }
    }
    client.find("\r\n\r\n");    // read until end of headers
    LAST_MS = millis();
    LAST_WC = client.parseInt();
    GROUND_OCCUPIED = !!client.parseInt();
    GROUND_WC = client.parseInt();
#ifdef USE_SERIAL
    Serial.print("--now:");
    Serial.print(LAST_WC);
    Serial.print("  status:");
    Serial.print(GROUND_OCCUPIED ? "occupied" : "free");
    Serial.print("  next:");
    Serial.println(GROUND_WC);
#endif
    devStatus.off();
    STATE = stateDisconnecting;
}


void stateDisconnecting() {
#ifdef USE_SERIAL
    Serial.println("STATE DISCONNECTING");
#endif
    WiFi.disconnect();
    delay(MS_WIFI_SLEW);
    STATE = stateDisplaying;
}


void stateDisplaying() {
    unsigned long now_wc = getWallclock();
#ifdef USE_SERIAL
    Serial.println("STATE DISPLAYING");
    Serial.print("--now:");
    Serial.print(now_wc);
    Serial.print("  status:");
    Serial.print(GROUND_OCCUPIED ? "occupied" : "free");
    Serial.print("  next:");
    Serial.println(GROUND_WC);
#endif
    if (GROUND_OCCUPIED) {
#ifdef USE_SERIAL
        Serial.println("--occupied");
#endif
        devFlag.move(2);
    } else {
        bool soon = now_wc > (GROUND_WC - S_SOON);
        if (soon) {
#ifdef USE_SERIAL
            Serial.println("--soon");
#endif
            devFlag.move(1);
        } else {
#ifdef USE_SERIAL
            Serial.println("--free");
#endif
            devFlag.move(0);
        }
    }
    STATE = stateSleeping;
}


void stateSleeping() {
    unsigned long ms = MS_UPDATE;
#ifdef USE_SERIAL
    Serial.println("STATE SLEEPING");
    Serial.print("--sleeping ");
    Serial.print(ms / 1000);
    Serial.println(" seconds");
    Serial.flush();
#endif
    // TODO -- haven't gotten sleep to work yet
    // deepSleep() takes microseconds
    //ESP.deepSleep(ms * 1000);
    delay(ms);
    STATE = stateConnecting;
}


void stateDebuggingFlag() {
    Serial.println("STATE DEBUGGING FLAG");
    for (int pos = 0; pos < 3; pos++) {
        Serial.print("--pos ");
        Serial.println(pos);
        devStatus.on(pos);
        devFlag.move(pos);
        devStatus.off();
    }
    delay(2000);
}


void setup() {
    WiFi.mode(WIFI_OFF);
#ifdef USE_SERIAL
    Serial.begin(115200);
    // DEBUGGING -- Serial.setDebugOutput(true);
#endif
    delay(MS_BOOT_PAUSE);
    devStatus.setup();
    devFlag.setup();
    STATE = stateConnecting;
    // DEBUGGING -- STATE = stateDebuggingFlag;
}


void loop() {
    STATE();
}


