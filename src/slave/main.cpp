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
#include "SerialCommand.h"
#include "shutter.h"

// Pin definitions
#define SW_A1    12     // shutter closed switch (NC)
#define SW_A2    11     // shutter open switch (NO)
#define SW_B1    10     // flap closed switch (NC)
#define SW_B2    3      // flap open switch (NO)
#define SW_INT   2      // shutter interference detection switch (NC)
#define MOTOR_A1 8      // motor driver pin 1
#define MOTOR_A2 9      // motor driver pin 2
#define BTNX     A4     // analog input for reading the buttons
#define BTN1     A6     // 'open' button
#define BTN2     A7     // 'close' button
#define VBAT     A5     // battery voltage reading

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

// Number of ADC readings required to detect a pressed button
#define BUTTON_REPS 4

#define LEN(a) ((int)(sizeof(a)/sizeof(*a)))

enum {
    BTN_NONE,
    BTN_A_OPEN,
    BTN_A_CLOSE,
    BTN_B_OPEN,
    BTN_B_CLOSE,
};

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
// Two shutters controlled by a Monster Motor Shield
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


// Detect a pressed button by reading an analog input.
// Every button puts a different voltage at the input.
// A button is considered active after BUTTON_REPS succesive readings.
int readAnalogButtons(int pin) {
    static int btn_prev = 0, btn_count = 0;

    int buttonLimits[] = {92, 303, 518, 820};
    int val = analogRead(pin);

    int btn = BTN_NONE;
    for (int i = 0; i < LEN(buttonLimits); i++) {
        if (val < buttonLimits[i]) {
            btn = i + 1;
            break;
        }
    }

    if (btn && (btn == btn_prev))
        btn_count++;
    else
        btn_count = 0;
    btn_prev = btn;

    if (btn_count == BUTTON_REPS)
        return btn;
    return 0;
}

int readDigitalButtons() {
    static int btn1_prev = 1, btn2_prev = 1;
    int btn1 = digitalRead(BTN1);
    int btn2 = digitalRead(BTN2);
    int btn = BTN_NONE;

    if (!btn1 && btn1_prev)
        btn = BTN_A_OPEN;
    else if (!btn2 && btn2_prev)
        btn = BTN_A_CLOSE;

    btn1_prev = btn1;
    btn2_prev = btn2;
    return btn;
}

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

void setup() {
    wdt_disable();
    wdt_enable(WDTO_1S);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SW_A1, INPUT_PULLUP);
    pinMode(SW_A2, INPUT_PULLUP);
    pinMode(SW_B1, INPUT_PULLUP);
    pinMode(SW_B2, INPUT_PULLUP);
    pinMode(SW_INT, INPUT_PULLUP);
    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);

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
#ifdef ANALOG_BUTTONS
    int btn = readAnalogButtons(BTNX);
#else
    int btn = readDigitalButtons();
#endif

    switch (btn) {
    case BTN_A_OPEN:
        shutters[0].open();
        break;
    case BTN_A_CLOSE:
        shutters[0].close();
        break;
    case BTN_B_OPEN:
        if (LEN(shutters) > 1)
            shutters[1].open();
        break;
    case BTN_B_CLOSE:
        if (LEN(shutters) > 1)
            shutters[1].close();
        break;
    }

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
    wdt_reset();
    delay(25);
}
