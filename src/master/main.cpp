/*******************************************************************************
ArduinoDomeController
Azimuth control of an astronomical dome using Arduino


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
#include "serial_command.h"
#include "dome.h"


// Configuration
//#define NSHUTTERS 2   // Uncomment if the shutter controller is available
//#define MOTOR_SHIELD  // Uncomment if the motor driver is a Monster Motor Shield
//#define USE_BUTTONS   // Uncomment if you want to move the dome with push buttons

#ifndef ENCODER_DIV
#define ENCODER_DIV     1   // Encoder divider ratio
#endif

#ifndef AZ_TIMEOUT
#define AZ_TIMEOUT      60000   // Azimuth movement timeout (in ms)
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
#define MOTOR_CW 8      // Move motor clockwise
#define MOTOR_CCW 9     // Move motor counterclockwise

#define BAUDRATE 19200

// Message Destination
#define TO_MAXDOME  0x00
#define TO_COMPUTER 0x80

// Commands
#define ABORT_CMD   0x03 // Abort azimuth movement
#define HOME_CMD    0x04 // Move until 'home' position is detected
#define GOTO_CMD    0x05 // Go to azimuth position
#define SHUTTER_CMD 0x06 // Send a command to shutter
#define STATUS_CMD  0x07 // Retrieve status
#define TICKS_CMD   0x09 // Set the number of tick per revolution of the dome
#define ACK_CMD     0x0A // ACK (?)
#define SETPARK_CMD 0x0B // Set park coordinates and shutter closing policy
#define VBAT_CMD    0x0C // Read shutter's battery voltage

// Shutter commands
#define OPEN_SHUTTER        0x01
#define OPEN_UPPER_SHUTTER  0x02
#define CLOSE_SHUTTER       0x03
#define EXIT_SHUTTER        0x04 // Command sent to shutter on program exit
#define ABORT_SHUTTER       0x07


enum MotorSpeed { MOTOR_STOP, MOTOR_SLOW, MOTOR_FAST };

// MaxDome II azimuth status
enum MDAzimuthStatus {
    AS_IDLE = 1,
    AS_MOVING_CW,
    AS_MOVING_CCW,
    AS_IDLE2,
    AS_ERROR
};


#if NSHUTTERS > 0
// Create a Software Serial Port to communicate with the shutter controller
SoftwareSerial HC12(HC12_TX, HC12_RX);
#endif

SerialCommand sCmd(&Serial);
//
#ifdef MOTOR_SHIELD
MMSMotor motor(0);
#else
DCMotor motor(MOTOR_CW, MOTOR_CCW);
#endif

Dome dome(&HC12, &motor);


// Convert two bytes to a uint16_t value (big endian)
uint16_t bytesToInt(uint8_t *data) {
    uint16_t value1 = data[1];
    uint16_t value2 = data[0];
    return (value1 & 0xff) | ((value2 << 8) & 0xff00);
}

// Convert a uint16_t value to bytes (big endian)
void intToBytes(uint16_t value, uint8_t *data) {
    data[1] = value & 0xff;
    data[0] = (value >> 8) & 0xff;
}


void cmdAbort(uint8_t *cmd) {
    dome.abort();
    uint8_t resp[] = {START, 2, TO_COMPUTER | ABORT_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdHomeAzimuth(uint8_t *cmd) {
    dome.home();
    uint8_t resp[] = {START, 3, TO_COMPUTER | HOME_CMD, 0x01, 0x00};
    sCmd.sendResponse(resp, 5);
}

void cmdGotoAzimuth(uint8_t *cmd) {
    // direction field is ignored!
    uint16_t target = bytesToInt(cmd + 4);
    dome.gotoAzimuth(target);

    uint8_t resp[] = {START, 3, TO_COMPUTER | GOTO_CMD, 0x01, 0x00};
    sCmd.sendResponse(resp, 5);
}

void cmdShutterCommand(uint8_t *cmd) {
    switch (cmd[3]) {
    case OPEN_SHUTTER:
        dome.openShutter(SEL_BOTH);
        break;
    case OPEN_UPPER_SHUTTER:
        dome.openShutter(SEL_UPPER);
        break;
    case CLOSE_SHUTTER:
        dome.closeShutter(SEL_BOTH);
        break;
    case EXIT_SHUTTER:
        dome.exitShutters();
        break;
    case ABORT_SHUTTER:
        dome.abortShutters();
        break;
    }

    uint8_t resp[] = {START, 2, TO_COMPUTER | SHUTTER_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdStatus(uint8_t *cmd) {
    DomeStatus status = dome.getStatus();

    MDAzimuthStatus az_st;
    if (status.az_state == ST_IDLE)
        az_st = AS_IDLE;
    else if (status.az_state == ST_ERROR)
        az_st = AS_ERROR;
    else {
        if (status.dir == DIR_CW)
            az_st = AS_MOVING_CW;
        else
            az_st = AS_MOVING_CCW;
    }

    uint8_t resp[] = {
        START, 9, TO_COMPUTER | STATUS_CMD, status.sh_state,
        az_st, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00
    };

    intToBytes(status.pos, resp + 5);
    intToBytes(status.home_pos, resp + 7);

    sCmd.sendResponse(resp, 11);
}

void cmdSetPark(uint8_t *cmd) {
    DomeConf conf = dome.getConf();
    conf.park_on_shutter = cmd[3];
    conf.park_pos = bytesToInt(cmd + 4);
    dome.setConf(conf);
    EEPROM.put(0, conf);

    uint8_t resp[] = {START, 2, TO_COMPUTER | SETPARK_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdSetTicks(uint8_t *cmd) {
    DomeConf conf = dome.getConf();
    conf.ticks_per_turn = bytesToInt(cmd + 3);
    dome.setConf(conf);
    EEPROM.put(0, conf);

    uint8_t resp[] = {START, 2, TO_COMPUTER | TICKS_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdVBat(uint8_t *cmd) {
    int vbat = dome.getBatteryVolts() * 100;

    uint8_t resp[] = {START, 4, TO_COMPUTER | VBAT_CMD, 0x00, 0x00, 0x00};
    intToBytes(vbat, resp + 3);
    sCmd.sendResponse(resp, 6);
}

void cmdAck(uint8_t *cmd) {
    uint8_t resp[] = {START, 3, TO_COMPUTER | ACK_CMD, 0x03, 0x00};
    sCmd.sendResponse(resp, 5);
}

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

void setup() {
    wdt_disable();
    wdt_enable(WDTO_2S);

    sCmd.addCommand(ABORT_CMD, 2, cmdAbort);
    sCmd.addCommand(HOME_CMD, 2, cmdHomeAzimuth);
    sCmd.addCommand(GOTO_CMD, 5, cmdGotoAzimuth);
    sCmd.addCommand(SHUTTER_CMD, 3, cmdShutterCommand);
    sCmd.addCommand(STATUS_CMD, 2, cmdStatus);
    sCmd.addCommand(SETPARK_CMD, 5, cmdSetPark);
    sCmd.addCommand(TICKS_CMD, 4, cmdSetTicks);
    sCmd.addCommand(ACK_CMD, 2, cmdAck);
    sCmd.addCommand(VBAT_CMD, 2, cmdVBat);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUTTON_CW, INPUT_PULLUP);
    pinMode(BUTTON_CCW, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER1), encoderISR, CHANGE);

    DomeConf conf;
    EEPROM.get(0, conf);
    conf.nshutters = NSHUTTERS;
    conf.tolerance = AZ_TOLERANCE;
    conf.az_timeout = (uint8_t)AZ_TIMEOUT;
    conf.encoder_div = ENCODER_DIV;
    dome.setConf(conf);

    Serial.begin(BAUDRATE);

#if NSHUTTERS > 0
    HC12.begin(9600);   // Open serial port to HC12
#endif
}

// move the dome when the buttons are pressed
void read_buttons() {
    static int prev_cw_button = 0, prev_ccw_button = 0;
    int cw_button = !digitalRead(BUTTON_CW);
    int ccw_button = !digitalRead(BUTTON_CCW);

    if (cw_button != prev_cw_button) {
        if (cw_button) {
            digitalWrite(LED_BUILTIN, HIGH);
            dome.moveAzimuth(DIR_CW);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
            dome.stopAzimuth();
        }
    } else if (ccw_button != prev_ccw_button) {
        if (ccw_button) {
            digitalWrite(LED_BUILTIN, HIGH);
            dome.moveAzimuth(DIR_CCW);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
            dome.stopAzimuth();
        }
    }
    prev_cw_button = cw_button;
    prev_ccw_button = ccw_button;
}


void loop() {
    sCmd.readSerial();
    dome.update();

#ifdef USE_BUTTONS
    read_buttons();
#endif
    wdt_reset();
}
