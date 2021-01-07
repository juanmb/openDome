#include <Arduino.h>
#include "shutter.h"
#include <util/atomic.h>

#define MOTOR_OPEN  0
#define MOTOR_CLOSE 1
#define SPEED       1023


// Shutter constructor without interference detection switch.
Shutter::Shutter(MotorDriver *motor, int swClosed, int swOpen,
            unsigned long timeout) {
    this->motor = motor;
    this->swClosed = swClosed;    // normally closed (1 if shutter is closed)
    this->swOpen = swOpen;        // normally open (0 if shutter is fully open)
    runTimeout = timeout;
    interType = INTER_NO;
    initState();
}


// Shutter constructor with interference detection switch.
Shutter::Shutter(MotorDriver *motor, int swClosed, int swOpen,
        int swInter, InterType type, unsigned long timeout) {
    this->motor = motor;
    this->swClosed = swClosed;
    this->swOpen = swOpen;
    this->swInter = swInter;
    interType = type;
    runTimeout = timeout;
    initState();
}


void Shutter::initState() {
    nextAction = DO_NONE;
    if (digitalRead(swClosed))
        state = ST_CLOSED;
    else if (digitalRead(swOpen))
        state = ST_OPEN;
    else
        state = ST_ABORTED;
}


void Shutter::open() {
    nextAction = DO_OPEN;
}

void Shutter::close() {
    nextAction = DO_CLOSE;
}

void Shutter::abort() {
    nextAction = DO_ABORT;
}

State Shutter::getState() {
    return state;
}


// Shutter state machine
void Shutter::update() {
    Action action;
    static unsigned long t0;

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        action = nextAction;
        nextAction = DO_NONE;
    }

    switch (state) {
    case ST_CLOSED:
        if (action == DO_OPEN) {
            t0 = millis();
            state = ST_OPENING;
        }
        break;
    case ST_OPEN:
        if (action == DO_CLOSE) {
            t0 = millis();
            state = ST_CLOSING;
        }
        break;
    case ST_ABORTED:
    case ST_ERROR:
        if (action == DO_OPEN) {
            t0 = millis();
            state = ST_OPENING;
        } else if (action == DO_CLOSE) {
            t0 = millis();
            state = ST_CLOSING;
        }
        break;
    case ST_OPENING:
        if ((interType == INTER_INNER) && digitalRead(swInter))
            motor->brake();
        else
            motor->run(MOTOR_OPEN, SPEED);

        if (digitalRead(swOpen)) {
            state = ST_OPEN;
            motor->brake();
        } else if (action == DO_ABORT || action == DO_CLOSE) {
            state = ST_ABORTED;
            motor->brake();
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor->brake();
        }
        break;
    case ST_CLOSING:
        if ((interType == INTER_OUTER) && !digitalRead(swInter))
            motor->brake();
        else
            motor->run(MOTOR_CLOSE, SPEED);

        if (digitalRead(swClosed)) {
            state = ST_CLOSED;
            motor->brake();
        } else if (action == DO_ABORT || action == DO_OPEN) {
            state = ST_ABORTED;
            motor->brake();
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor->brake();
        }
        break;
    }
}
