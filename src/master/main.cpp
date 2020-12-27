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

// Configuration
//#define NSHUTTERS 2   // Uncomment if the shutter controller is available
//#define MONSTER_SHIELD  // Uncomment if the motor driver is a Monster Moto Shield

// Discard encoder pulses shorter than this duration (in milliseconds)
//#define DEBOUNCE_MS     10

// pin definitions
#define ENCODER1    2       // Encoder
#define ENCODER2    3       // Encoder
#define HOME_PIN    10      // Home sensor pin
#define BUTTON_CW   11      // CW movement button (active low)
#define BUTTON_CCW  12      // CCW movement button (Active low)
#define KEYPAD      A4      // analog keypad input

// pins of HC12 module (serial radio transceiver)
#define HC12_RX 4       // Receive Pin on HC12
#define HC12_TX 5       // Transmit Pin on HC12

// motor pins (if not using the Monster Moto Shield)
#define MOTOR_CW  8     // Move motor clockwise
#define MOTOR_CCW 9     // Move motor counterclockwise

// Digital keypad pins
int key_pins[] = {BUTTON_CW, BUTTON_CCW};

// Analog keypad threshold values
int key_thresholds[] = {92, 303, 518, 820};

enum KeyIDs {
    KEY_CW,
    KEY_CCW,
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

// Manage keypad events
void keypadHandler(const KeyMsg &msg) {
    if (msg.event == KEY_EVT_PRESS) {
        switch (msg.key_id) {
        case KEY_CW:
            dome.moveAzimuth(DIR_CW);
            break;
        case KEY_CCW:
            dome.moveAzimuth(DIR_CW);
            break;
        default:
            dome.abort();
        }
    }
    else if (msg.event == KEY_EVT_RELEASE) {
        dome.stopAzimuth();
    }
}

#ifdef ANALOG_KEYPAD
AnalogKeypad keypad(KEYPAD, LEN(key_thresholds), key_thresholds, keypadHandler);
#else
DigitalKeypad keypad(key_pins, LEN(key_pins), keypadHandler);
#endif

//#pragma message "nkeys: " LEN(key_thresholds)

void setup() {
    wdt_disable();
    wdt_enable(WDTO_2S);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUTTON_CW, INPUT_PULLUP);
    pinMode(BUTTON_CCW, INPUT_PULLUP);

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
