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

#include <avr/wdt.h>
#include <dc_motor.h>
#include <mms_motor.h>
#include <keypad.h>
#include "SerialCommand.h"
#include "shutter.h"

#define LEN(a) ((int)(sizeof(a)/sizeof(*a)))

// Pin definitions
#define SW_A1       12      // shutter closed switch (NC)
#define SW_A2       11      // shutter open switch (NO)
#define SW_B1       10      // flap closed switch (NC)
#define SW_B2       3       // flap open switch (NO)
#define SW_INT      2       // shutter interference detection switch (NC)
#define MOTOR_A1    8       // motor driver pin 1
#define MOTOR_A2    9       // motor driver pin 2
#define KEYPAD      A4      // analog keypad input
#define VBAT        A5      // battery voltage reading

// Digital keypad pins
int key_pins[] = {A4, A6, A7};  // 'abort', 'open' and 'close' buttons

// Analog keypad threshold values
int key_thresholds[] = {92, 303, 518, 820};

enum KeyIDs {
    KEY_ABORT,
    KEY_OPEN,
    KEY_CLOSE,
};

// Timeouts in ms
#ifndef COMMAND_TIMEOUT
// Max. time from last command. If this time is exceeded, the dome shutter will be closed.
#define COMMAND_TIMEOUT 60000
#endif

#ifndef SHUT_TIMEOUT
// Max. time the shutter takes to open/close
#define SHUT_TIMEOUT 75000
#endif

#ifndef FLAP_TIMEOUT
// Max. time the flap takes to open/close
#define FLAP_TIMEOUT 15000
#endif

// Detect mechanical interfence between the two shutters
bool checkFlapInter(State st) {
    return (st == ST_OPENING) && digitalRead(SW_INT);
}

// Detect mechanical interfence between the two shutters
bool checkShutInter(State st) {
    return (st == ST_CLOSING) && digitalRead(SW_INT) && !digitalRead(SW_B1);
}

#ifdef SINGLE_SHUTTER
// Single shutter with a generic motor driver
DCMotor motorA(MOTOR_A1, MOTOR_A2);
#else
// Two shutters controlled by a Monster Moto Shield
MMSMotor motorA(0);
MMSMotor motorB(1);
#endif

Shutter shutters[] = {
#ifdef SINGLE_SHUTTER
    Shutter(&motorA, SW_A1, SW_A2, SHUT_TIMEOUT),
#else
    Shutter(&motorA, SW_A1, SW_A2, SHUT_TIMEOUT, checkShutInter),
    Shutter(&motorB, SW_B1, SW_B2, FLAP_TIMEOUT, checkFlapInter),
#endif
};

SerialCommand sCmd;
unsigned long lastCmdTime = 0;


// Return the combined status of the shutters
State domeStatus() {
    bool closed = true;
    State st;

    for (int i = 0; i < LEN(shutters); i++) {
        st = shutters[i].getState();
        closed = closed && (st == ST_CLOSED);

        if (st == ST_ERROR)
            return ST_ERROR;
        else if (st == ST_OPENING)
            return ST_OPENING;
        else if (st == ST_CLOSING)
            return ST_CLOSING;
        else if (st == ST_OPEN)
            return ST_OPEN;
    }
    if (closed)
        return ST_CLOSED;
    return ST_ABORTED;
}

// Open main shutter only
void cmdOpenShutter() {
    lastCmdTime = millis();
    shutters[0].open();
}

// Open shutters
void cmdOpenBoth() {
    lastCmdTime = millis();
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].open();
}

// Close shutters
void cmdClose() {
    lastCmdTime = millis();
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].close();
}

void cmdAbort() {
    lastCmdTime = millis();
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].abort();
}

void cmdExit() {
    lastCmdTime = 0;
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].close();
}

void cmdStatus() {
    lastCmdTime = millis();
    State st = domeStatus();
    Serial.write('0' + st);
}

void cmdGetVBat() {
    lastCmdTime = millis();
    int val = analogRead(VBAT);
    char buffer[8];
    sprintf(buffer, "v%04d", val);
    Serial.write(buffer);
}

// Manage keypad events
void keypadHandler(const KeyMsg &msg) {
    if (msg.event == KEY_EVT_RELEASE) {
        switch (msg.key_id) {
        case KEY_OPEN:
            cmdOpenBoth();
            break;
        case KEY_CLOSE:
            cmdClose();
            break;
        default:
            cmdAbort();
        }
    }
}

#ifdef ANALOG_KEYPAD
AnalogKeypad keypad(KEYPAD, LEN(key_thresholds), key_thresholds, keypadHandler);
#else
DigitalKeypad keypad(key_pins, LEN(key_pins), keypadHandler);
#endif

void setup() {
    wdt_disable();
    wdt_enable(WDTO_1S);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SW_A1, INPUT_PULLUP);
    pinMode(SW_A2, INPUT_PULLUP);
    pinMode(SW_B1, INPUT_PULLUP);
    pinMode(SW_B2, INPUT_PULLUP);
    pinMode(SW_INT, INPUT_PULLUP);

    // Map serial commands to functions
    sCmd.addCommand("open", cmdOpenBoth);
    sCmd.addCommand("open1", cmdOpenShutter);
    sCmd.addCommand("close", cmdClose);
    sCmd.addCommand("abort", cmdAbort);
    sCmd.addCommand("exit", cmdExit);
    sCmd.addCommand("stat", cmdStatus);
    sCmd.addCommand("vbat", cmdGetVBat);

    Serial.begin(9600);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
}


void loop() {
    State st = domeStatus();

    // switch on the LED if there is an error
    digitalWrite(LED_BUILTIN, (st == ST_ERROR));

    // close the dome if the time since the last command is too long
    if ((lastCmdTime > 0) && ((millis() - lastCmdTime) > COMMAND_TIMEOUT)) {
        if (st != ST_CLOSED) {
            lastCmdTime = 0;
            for (int i = 0; i < LEN(shutters); i++)
                shutters[i].close();
        }
    }

    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].update();

    sCmd.readSerial();
    keypad.update();
    wdt_reset();
    delay(25);
}
