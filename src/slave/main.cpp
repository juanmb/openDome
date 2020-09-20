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
#include <monster_motor_shield.h>
#include "SerialCommand.h"
#include "shutter.h"

// Pin definitions
#ifdef SINGLE_SHUTTER
#define SW_A1    5      // shutter closed switch (normally closed)
#define SW_A2    6      // shutter open switch (normally closed)
#else
#define SW_A1    12     // shutter closed switch (NC)
#define SW_A2    11     // shutter open switch (NO)
#define SW_B1    10     // flap closed switch (NC)
#define SW_B2    3      // flap open switch (NO)
#endif

#ifdef ANALOG_BUTTONS
#define BUTTONS  A4     // analog input for reading the buttons
#else
#define BTN_PIN1 11	// 'open' button
#define BTN_PIN2 12	// 'close' button
#endif

#define SW_INTER 2      // shutter interference detection switch (NC)
#define MOTOR_A1 8	// motor driver pin 1
#define MOTOR_A2 9	// motor driver pin 2
#define LED_PIN  13
#define VBAT_PIN A5     // battery voltage reading


#define BUTTON_REPS 4  // Number of ADC readings required to detect a pressed button

// Timeouts in ms
#ifndef COMMAND_TIMEOUT
#define COMMAND_TIMEOUT 60000   // Max. time from last command. If this time is
                                // exceeded, the dome shutter will be closed.
#endif
#ifndef SHUT_TIMEOUT
#define SHUT_TIMEOUT 75000   // Max. time the shutter takes to open/close
#endif
#ifndef FLAP_TIMEOUT
#define FLAP_TIMEOUT 15000   // Max. time the flap takes to open/close
#endif

#define LEN(a) ((int)(sizeof(a)/sizeof(*a)))

enum {
    BTN_NONE,
    BTN_A_OPEN,
    BTN_A_CLOSE,
    BTN_B_OPEN,
    BTN_B_CLOSE,
};

SerialCommand sCmd;
unsigned long lastCmdTime = 0;


#ifdef SINGLE_SHUTTER
// Single shutter with a generic motor driver
DCMotor motorA(MOTOR_A1, MOTOR_A2);
Shutter shutters[] = {
    Shutter(&motorA, SW_A1, SW_A2, SHUT_TIMEOUT),
};
#else

// Detect mechanical interfence between the two shutters
bool checkFlapInterference(State st)
{
    return (st == ST_OPENING) && digitalRead(SW_INTER);
}

// Detect mechanical interfence between the two shutters
bool checkShutInterference(State st)
{
    return (st == ST_CLOSING) && digitalRead(SW_INTER) && !digitalRead(SW_B1);
}

// Two shutters controlled by a Monster Motor Shield
MMSMotor motorA(0);
MMSMotor motorB(1);

Shutter shutters[] = {
    Shutter(&motorA, SW_A1, SW_A2, SHUT_TIMEOUT, checkShutInterference),
    Shutter(&motorB, SW_B1, SW_B2, FLAP_TIMEOUT, checkFlapInterference),
};
#endif

// Detect a pressed button by reading an analog input.
// Every button puts a different voltage at the input.
// A button is considered active after BUTTON_REPS succesive readings.
int readAnalogButtons(int pin)
{
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

#ifndef ANALOG_BUTTONS
int readButtons()
{
    static int btn1_prev = 1, btn2_prev = 1;
    int btn1 = digitalRead(BTN_PIN1);
    int btn2 = digitalRead(BTN_PIN2);
    int btn = BTN_NONE;

    if (!btn1 && btn1_prev)
        btn = BTN_A_OPEN;
    else if (!btn2 && btn2_prev)
        btn = BTN_A_CLOSE;

    btn1_prev = btn1;
    btn2_prev = btn2;
    return btn;
}
#endif

// Return the combined status of the shutters
State domeStatus()
{
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
void cmdOpenShutter()
{
    lastCmdTime = millis();
    shutters[0].open();
}

// Open shutters
void cmdOpenBoth()
{
    lastCmdTime = millis();
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].open();
}

// Close shutters
void cmdClose()
{
    lastCmdTime = millis();
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].close();
}

void cmdAbort()
{
    lastCmdTime = millis();
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].abort();
}

void cmdExit()
{
    lastCmdTime = 0;
    for (int i = 0; i < LEN(shutters); i++)
        shutters[i].close();
}

void cmdStatus()
{
    lastCmdTime = millis();
    State st = domeStatus();
    Serial.write('0' + st);
}

void cmdGetVBat()
{
    lastCmdTime = millis();
    int val = analogRead(VBAT_PIN);
    char buffer[8];
    sprintf(buffer, "v%04d", val);
    Serial.write(buffer);
}

void setup()
{
    wdt_disable();
    wdt_enable(WDTO_1S);

    pinMode(LED_PIN, OUTPUT);
    pinMode(SW_A1, INPUT_PULLUP);
    pinMode(SW_A2, INPUT_PULLUP);

#ifndef ANALOG_BUTTONS
    pinMode(BTN_PIN1, INPUT_PULLUP);
    pinMode(BTN_PIN2, INPUT_PULLUP);
#endif

    // Map serial commands to functions
    sCmd.addCommand("open", cmdOpenBoth);
    sCmd.addCommand("open1", cmdOpenShutter);
    sCmd.addCommand("close", cmdClose);
    sCmd.addCommand("abort", cmdAbort);
    sCmd.addCommand("exit", cmdExit);
    sCmd.addCommand("stat", cmdStatus);
    sCmd.addCommand("vbat", cmdGetVBat);

    Serial.begin(9600);

    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
}


void loop()
{
#ifdef ANALOG_BUTTONS
    int btn = readAnalogButtons(BUTTONS);
#else
    int btn = readButtons();
#endif

    switch(btn) {
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
    digitalWrite(LED_PIN, (st == ST_ERROR));

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
