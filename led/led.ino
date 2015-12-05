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
 * arduino config
 *      board:  Generic ESP8266 Module
 *      programmer: AVRISP mkll
 *      serial:  115200
 * pins
 *      POS HW      USAGE                   NOTES
 *      1.1 ^RESET  pullup 10k
 *      1.2 ADC     -
 *      1.3 EN      pullup 10k
 *      1.4 GPIO16  -
 *      1.5 GPIO14  RED: status
 *      1.6 GPIO12  YELLOW: status
 *      1.7 GPIO13  -
 *      1.8 VCC     -
 *      2.1 TX      -
 *      2.2 RX      -
 *      2.3 GPIO5   -
 *      2.4 GPIO4   -
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


#undef USE_SERIAL

const char WIFI_SSID[]      = "foo";
const char WIFI_PASS[]      = "bar";
const char EVENTS_HOST[]    = "drewfish-iot.herokuapp.com";
const long EVENTS_PORT      = 80;
const char EVENTS_REQUEST[] = "GET /sfbc/events HTTP/1.1\r\nHost: drewfish-iot.herokuapp.com\r\nConnection: close\r\n\r\n";
const uint8_t PIN_RED       = 14;
const uint8_t PIN_YELLOW    = 12;

void (*STATE)(void)         = NULL;
long ERROR_CODE             = 0;
unsigned long LAST_MS       = 0;    // (milliseconds) arduino millis() from last query
unsigned long LAST_WC       = 0;    // (seconds) wallclock time from last query
boolean GROUND_OCCUPIED     = false;
unsigned long GROUND_WC     = 0;    // (seconds) free: when next occupied -- occupied: when free

static const long S_SOON            = 300;      // (seconds) size of "occupied soon" window
static const long S_UPDATE          = 3600;     // (seconds) how often to query the events webservice
static const long MS_BOOT_PAUSE     = 1000;     // give time to connect serial console
static const long MS_RETRY_PAUSE    = 1000;     // pause before trying again (connecting or querying)
static const long MS_STATUS_ERROR   = 500;      // blink cycle when there's an error
static const long MS_STATUS_CHECKING = 100;     // blink cycle when checking calendar
static const long MS_STATUS_SOON    = 1000;     // blink cycle when occupied soon
static const long MS_WIFI_SLEW      = 100;      // give Arduino WiFi API time to effect change


void stateError();
void stateConnecting();
void stateQuerying();
void stateDisconnecting();
void stateDisplaying();
void stateSleeping();


enum class Status {
    // calendar
    FREE = 0,
    SOON,
    BUSY,

    // device
    CHECKING,
    ERROR
};


// displays the status of the ground floor, and also this algorithm
struct StatusDevice {
    Status m_status;
    bool m_on;
    void setup() {
        m_status = Status::CHECKING;
        m_on = false;
        pinMode(PIN_RED, OUTPUT);
        pinMode(PIN_YELLOW, OUTPUT);
        digitalWrite(PIN_RED, LOW);
        digitalWrite(PIN_YELLOW, LOW);
    }
    void set(Status status) {
#ifdef USE_SERIAL
        Serial.print("--status ");
        Serial.println((uint8_t)status);
#endif
        m_on = (m_status != status) || !m_on;
        m_status = status;
        _update();
    }
    void toggle() {
        m_on = !m_on;
        _update();
    }
    void _update() {
        switch (m_status) {
            case Status::FREE:
                digitalWrite(PIN_RED, LOW);
                digitalWrite(PIN_YELLOW, LOW);
                break;
            case Status::SOON:
                digitalWrite(PIN_RED, LOW);
                digitalWrite(PIN_YELLOW, HIGH);
                break;
            case Status::BUSY:
                digitalWrite(PIN_RED, HIGH);
                digitalWrite(PIN_YELLOW, LOW);
                break;
            case Status::CHECKING:
                digitalWrite(PIN_RED, LOW);
                digitalWrite(PIN_YELLOW, m_on ? HIGH : LOW);
                break;
            case Status::ERROR:
                digitalWrite(PIN_RED, m_on ? HIGH : LOW);
                digitalWrite(PIN_YELLOW, LOW);
                break;
        }
    }
} devStatus;


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
    devStatus.set(Status::ERROR);
    delay(MS_STATUS_ERROR);
    while (1) {
        devStatus.toggle();
        delay(MS_STATUS_ERROR);
    }
}


void stateConnecting() {
#ifdef USE_SERIAL
    Serial.println("STATE CONNECTING");
    // DEBUGGING -- WiFi.printDiag(Serial);
#endif
    devStatus.set(Status::CHECKING);
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
            case WL_CONNECTED:          // status=3
                // success
                STATE = stateQuerying;
                return;

            case WL_DISCONNECTED:       // status=6
            case WL_IDLE_STATUS:        // status=0
                // keep polling
                devStatus.toggle();
                delay(MS_STATUS_CHECKING);
                break;

            case WL_NO_SSID_AVAIL:      // status=1
            case WL_CONNECT_FAILED:     // status=4
            case WL_CONNECTION_LOST:    // status=5
                // retry
                devStatus.toggle();
                delay(MS_RETRY_PAUSE);
                return;

            default:
                // status=2=WC_SCAN_COMPLETED
                // error
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
    devStatus.set(Status::CHECKING);
    WiFiClient client;
    if (! client.connect(EVENTS_HOST, EVENTS_PORT)) {
#ifdef USE_SERIAL
        Serial.println("--FAILED to connect");
#endif
        devStatus.toggle();
        delay(MS_RETRY_PAUSE);
        return;
    }
    client.print(EVENTS_REQUEST);
    while (! client.available()) {
        delay(MS_STATUS_CHECKING);
        devStatus.toggle();
        if (! client.connected()) {
#ifdef USE_SERIAL
            Serial.println("--ABORTED connection");
#endif
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
        devStatus.set(Status::BUSY);
        STATE = stateSleeping;
        return;
    }
    unsigned long soon_wc = GROUND_WC - S_SOON;
    if (now_wc > soon_wc) {
        int blink = (GROUND_WC - now_wc);   // seconds
        if (blink > S_UPDATE) {
            blink = S_UPDATE;
        }
#ifdef USE_SERIAL
        Serial.print("-- blink:");
        Serial.println(blink);
#endif
        blink *= 1000;  // milliseconds
        bool on = true;
        while (blink > 0) {
            devStatus.set(on ? Status::SOON : Status::FREE);
            delay(MS_STATUS_SOON);
            blink -= MS_STATUS_SOON;
            on = !on;
        }
        devStatus.set(Status::SOON);
        STATE = stateConnecting;
    } else {
        devStatus.set(Status::FREE);
        STATE = stateSleeping;
    }
}


void stateSleeping() {
#ifdef USE_SERIAL
    Serial.println("STATE SLEEPING");
#endif
    unsigned long s = S_UPDATE;
    if (GROUND_WC) {
        s = GROUND_WC - getWallclock();
        if (devStatus.m_status == Status::FREE) {
            s -= S_SOON;
        }
        if (s > S_UPDATE) {
            s = S_UPDATE;
        }
    }
#ifdef USE_SERIAL
    Serial.print("--now ");
    Serial.println(getWallclock());
    Serial.print("--sleeping ");
    Serial.print(s);
    Serial.println(" seconds");
#endif
    delay(s * 1000);
    STATE = stateConnecting;
}


void setup() {
#ifdef USE_SERIAL
    Serial.begin(115200);
    // DEBUGGING -- Serial.setDebugOutput(true);
#endif
    delay(MS_BOOT_PAUSE);
    devStatus.setup();
    STATE = stateConnecting;
}


void loop() {
    STATE();
}


