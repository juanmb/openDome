/*******************************************************************************
openDome

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
#include <EEPROM.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include <monster_motor_shield.h>
#include <dc_motor.h>
#include "maxdome_protocol.h"
#include "dome.h"


// Configuration
//#define NSHUTTERS 2   // Uncomment if the shutter controller is available
//#define MOTOR_SHIELD  // Uncomment if the motor driver is a Monster Motor Shield
//#define USE_BUTTONS   // Uncomment if you want to move the dome with push buttons

#ifndef ENCODER_DIV
#define ENCODER_DIV     1   // Encoder divider ratio
#endif

#ifndef AZ_TIMEOUT
#define AZ_TIMEOUT      2000   // Azimuth movement timeout (in ms)
#endif

#ifndef AZ_TOLERANCE
#define AZ_TOLERANCE    4       // Azimuth target tolerance in encoder ticks
#endif

#ifndef AZ_SLOW_RANGE
// The motor will run at slower speed when the dome is at this number of ticks from the target
#define AZ_SLOW_RANGE   8
#endif

// Discard encoder pulses shorter than this duration (in milliseconds)
//#define DEBOUNCE_MS     10

// pin definitions
#define ENCODER1 2      // Encoder
#define ENCODER2 3      // Encoder
#define BUTTON_CW   11  // CW movement button (active low)
#define BUTTON_CCW  12  // CCW movement button (Active low)

#if NSHUTTERS > 0
#ifdef MOTOR_SHIELD
#error "HAS_SHUTTER and MOTOR_SHIELD cannot be defined at the same time"
#endif

// pins of HC12 module (serial radio transceiver)
#define HC12_RX 4       // Receive Pin on HC12
#define HC12_TX 5       // Transmit Pin on HC12
#endif

// motor pins (if not using the Monster Motor Shield)
#define MOTOR_CW  8     // Move motor clockwise
#define MOTOR_CCW 9     // Move motor counterclockwise

#define BAUDRATE 19200

#if NSHUTTERS > 0
// Create a Software Serial Port to communicate with the shutter controller
SoftwareSerial HC12(HC12_TX, HC12_RX);
#endif

#ifdef MOTOR_SHIELD
MMSMotor motor(0);
#else
DCMotor motor(MOTOR_CW, MOTOR_CCW);
#endif

Dome dome(&HC12, &motor);
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


// move the dome when the buttons are pressed
void read_buttons() {
    static int prev_cw_button = 0, prev_ccw_button = 0;
    int cw_button = !digitalRead(BUTTON_CW);
    int ccw_button = !digitalRead(BUTTON_CCW);

    if (cw_button != prev_cw_button) {
        if (cw_button)
            dome.moveAzimuth(DIR_CW);
        else
            dome.stopAzimuth();
    } else if (ccw_button != prev_ccw_button) {
        if (ccw_button)
            dome.moveAzimuth(DIR_CCW);
        else
            dome.stopAzimuth();
    }
    prev_cw_button = cw_button;
    prev_ccw_button = ccw_button;
}


void setup() {
    wdt_disable();
    wdt_enable(WDTO_2S);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUTTON_CW, INPUT_PULLUP);
    pinMode(BUTTON_CCW, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER1), encoderISR, CHANGE);

    DomeConf conf;
    EEPROM.get(0, conf);
    conf.nshutters = NSHUTTERS;
    conf.tolerance = AZ_TOLERANCE;
    conf.az_timeout = AZ_TIMEOUT;
    conf.encoder_div = ENCODER_DIV;
    conf.ticks_per_turn = 786;
    dome.setConf(conf);

    Serial.begin(BAUDRATE);

#if NSHUTTERS > 0
    HC12.begin(9600);   // Open serial port to HC12
#endif
}


void loop() {
    protoc.readSerial();
    dome.update();

#ifdef USE_BUTTONS
    read_buttons();
#endif
    wdt_reset();
}
