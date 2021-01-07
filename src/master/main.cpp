/*******************************************************************************
The MIT License

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*******************************************************************************/

#include <Arduino.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include <mms_motor.h>
#include <dc_motor.h>
#include <keypad.h>
#include "maxdome_protocol.h"
#include "dome.h"

#define LEN(a) ((int)(sizeof(a)/sizeof(*a)))

// Discard encoder pulses shorter than this duration (in milliseconds)
//#define DEBOUNCE_MS     10

// pin definitions
#define ENCODER1    2       // Encoder
#define ENCODER2    3       // Encoder
#define HOME_PIN    10      // Home sensor pin

#define KEYPAD      A4      // analog keypad input
#define BUTTON_CW   A4      // CW movement button (active low)
#define BUTTON_CCW  A5      // CCW movement button (Active low)

// pins of HC12 module (serial radio transceiver)
#define HC12_RX 4       // Receive Pin on HC12
#define HC12_TX 5       // Transmit Pin on HC12

// motor pins (if not using the Monster Moto Shield)
#define MOTOR_CW  8     // Move motor clockwise
#define MOTOR_CCW 9     // Move motor counterclockwise

enum KeyIDs {
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_ENTER,
};

#ifdef MONSTER_SHIELD
MMSMotor motor(0);
#else
DCMotor motor(MOTOR_CW, MOTOR_CCW);
#endif

#if NSHUTTERS > 0
#ifdef MONSTER_SHIELD
#error "MONSTER_SHIELD cannot be used when NSHUTTERS > 0"
#endif
// Create a Software Serial Port to communicate with the shutter controller
SoftwareSerial HC12(HC12_TX, HC12_RX);
Dome dome(&HC12, &motor, HOME_PIN);
#else
Dome dome(NULL, &motor, HOME_PIN);
#endif

MaxDomeProtocol protoc(&Serial, &dome);

// Encoder interrupt service routine
void encoderISR() {
#ifdef DEBOUNCE_MS
    // debounce encoder signal
    static unsigned long now, last;
    now = millis();
    if (now - last < DEBOUNCE_MS)
        return;
    last = now;
#endif

    if (digitalRead(ENCODER1) == digitalRead(ENCODER2))
        dome.tick(DIR_CCW);
    else
        dome.tick(DIR_CW);
}

// Handle keypad events
void keypadHandler(const KeyMsg &msg) {
    if (msg.event == KEY_EVT_PRESS) {
        switch (msg.key_id) {
        case KEY_RIGHT:
            dome.moveAzimuth(DIR_CW);
            break;
        case KEY_LEFT:
            dome.moveAzimuth(DIR_CCW);
            break;
        case KEY_ENTER:
            dome.abort();
            break;
        };
    } else  if (msg.event == KEY_EVT_RELEASE) {
        switch (msg.key_id) {
        case KEY_UP:
            dome.openShutter(SEL_BOTH);
            break;
        case KEY_DOWN:
            dome.closeShutter(SEL_BOTH);
            break;
        default:
            dome.stopAzimuth();
        }
    } else if (msg.event == KEY_EVT_HOLD_RELEASE) {
        switch (msg.key_id) {
        case KEY_UP:
            dome.openShutter(SEL_BOTH);
            break;
        case KEY_DOWN:
            dome.closeShutter(SEL_BOTH);
            break;
        default:
            dome.stopAzimuth();
        }
    } else if (msg.event == KEY_EVT_DOUBLE_CLICK) {
        switch (msg.key_id) {
        case KEY_ENTER:
            dome.home();
            break;
        default:
            dome.stopAzimuth();
        }
    }
}

#ifdef ANALOG_KEYPAD
// Analog keypad threshold values and key IDs (Escornabot keypad v2.1)
int key_thr[] = {597, 725, 794, 851, 953};
uint8_t key_ids[] = {KEY_UP, KEY_LEFT, KEY_DOWN, KEY_ENTER, KEY_RIGHT};
AnalogKeypad keypad(KEYPAD, LEN(key_thr), key_thr, key_ids, keypadHandler);
#else
// Digital keypad pins and key IDs
int key_pins[] = {BUTTON_CW, BUTTON_CCW};
uint8_t key_ids[] = {KEY_RIGHT, KEY_LEFT};
DigitalKeypad keypad(LEN(key_pins), key_pins, key_ids, keypadHandler);
#endif

//#pragma message "nkeys: " LEN(key_thr)

void setup() {
    wdt_disable();
    wdt_enable(WDTO_2S);
    pinMode(LED_BUILTIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER1), encoderISR, CHANGE);
    Serial.begin(MAXDOME_BAUDRATE);

#if NSHUTTERS > 0
    HC12.begin(9600);   // Open serial port to HC12
#endif
}


void loop() {
    protoc.readSerial();
    dome.update();
    keypad.update();
    wdt_reset();
}
