#include <Arduino.h>
#include "dome.h"

#define HOME_SENSOR 10  // Home sensor (active low)

#define VBAT_FACTOR (5.0/1024.0)
#define VBAT_OFFSET 10.55


Dome::Dome(Stream *ss, MotorDriver *drv) {
    sstream = ss;
    driver = drv;

    conf = {.ticks_per_turn = 360};

    pinMode(HOME_SENSOR, INPUT_PULLUP);
}

// Obtain the direction of the shortest path to a target position
Direction Dome::getDirection(uint16_t target) {
    if (target > pos)
        return ((target - pos) > conf.ticks_per_turn / 2) ? DIR_CCW : DIR_CW;

    return ((pos - target) > conf.ticks_per_turn / 2) ? DIR_CW : DIR_CCW;
}

// Obtain the number of ticks to a target position
uint16_t Dome::getDistance(uint16_t target) {
    // obtain the absolute value of the difference
    uint16_t diff = (target > pos) ?  target - pos : pos - target;

    if (diff > conf.ticks_per_turn / 2)
        return conf.ticks_per_turn - diff;
    return diff;
}

void Dome::moveMotor(Direction dir) {
    current_dir = dir;
    digitalWrite(LED_BUILTIN, HIGH);
    driver->run(dir == DIR_CW, 1023);
}

void Dome::stopMotor() {
    digitalWrite(LED_BUILTIN, LOW);
    driver->stop();
}

// Obtain the state of the shutters
ShutterState Dome::getShutterState() {
    ShutterState st = SS_OPEN;

    if (conf.nshutters && sstream) {
        sstream->flush();
        sstream->println("stat");
        delay(100);

        char c = 0;
        while (sstream->available() > 0)
            c = sstream->read();

        st = SS_ERROR;
        if (c >= '0' && c <= ('0' + SS_ERROR))
            st = (ShutterState)(c - '0');
    }
    return st;
}

float Dome::getBatteryVolts() {
    int adc = 0;

    if (conf.nshutters && sstream) {
        char buffer[5];

        sstream->flush();
        sstream->println("vbat");
        delay(100);

        if (sstream->read() == 'v') {
            sstream->readBytes(buffer, 4);
            buffer[4] = 0;
            adc = atoi(buffer);
        }
    }

    // Convert ADC reading to voltage
    return (float)adc * VBAT_FACTOR + VBAT_OFFSET;
}

DomeStatus Dome::getStatus() {
    DomeStatus status = {
        .az_state = state,
        .sh_state = getShutterState(),
        .dir = current_dir,
        .pos = pos,
        .home_pos = home_pos,
    };
    return status;
}

void Dome::moveAzimuth(Direction dir) {
    //TODO
    //next_target = (dir == DIR_CW) ? target + 1 : target - 1;
    event = EVT_MOVE;
}

void Dome::gotoAzimuth(uint16_t tgt) {
    next_target = tgt;
    event = EVT_GOTO;
}

void Dome::stopAzimuth() {
    event = EVT_STOP;
}

void Dome::home() {
    event = EVT_HOME;
}

void Dome::park() {
    event = EVT_PARK;
}

void Dome::abort() {
    if (conf.nshutters && sstream)
        sstream->println("abort");  // abort shutters

    event = EVT_ABORT;
}

void Dome::openShutter(ShutterSel sel) {
    if (conf.park_on_shutter) {
        //TODO: open after parking is complete
        park();
    } else if (sstream) {
        sstream->print("open_");
        sstream->println(sel);
    }
}

void Dome::closeShutter(ShutterSel sel) {
    if (conf.park_on_shutter) {
        //TODO: close after parking is complete
        park();
    } else if (sstream) {
        sstream->print("close_");
        sstream->println(sel);
    }
}

void Dome::stopShutter(ShutterSel sel) {
    if (sstream) {
        sstream->print("stop_");
        sstream->println(sel);
    }
}

void Dome::abortShutters() {
    if (sstream)
        sstream->println("abort");
}

void Dome::exitShutters() {
    if (sstream)
        sstream->println("exit");
}

// Increment or decrement the azimuth position by 1 tick
void Dome::tick(Direction dir) {
    static uint8_t count = 0;

    if (dir == DIR_CCW) {
        count = (count == 0 ? conf.encoder_div - 1 : count - 1);
        if (count == 0)
            pos = (pos == 0 ? conf.ticks_per_turn - 1 : pos - 1);
    } else {
        count = (count + 1 == conf.encoder_div ? 0 : count + 1);
        if (count == 0)
            pos = (pos + 1 == conf.ticks_per_turn ? 0 : pos + 1);
    }
}

// Update state machine
void Dome::update() {
    static unsigned long t0;

    //TODO: timeout if dome is not rotating
    switch (state) {
    case ST_HOMING:
        if (event == EVT_STOP || event == EVT_ABORT) {
            stopMotor();
            state = ST_IDLE;
        } else if (!digitalRead(HOME_SENSOR)) {
            stopMotor();
            home_pos = 0;
            pos = 0;
            state = ST_IDLE;
        } else if (millis() - t0 > conf.az_timeout) {
            stopMotor();
            state = ST_ERROR;
        }
        break;

    case ST_MOVING:
        if (!digitalRead(HOME_SENSOR))
            home_pos = pos;     // store detected home position

        if (event == EVT_STOP || event == EVT_ABORT) {
            stopMotor();
            state = ST_IDLE;
        }
        break;

    case ST_GOING:
        if (!digitalRead(HOME_SENSOR))
            home_pos = pos;     // store detected home position

        if (event == EVT_STOP || event == EVT_ABORT) {
            stopMotor();
            state = ST_IDLE;
        } else if (getDistance(target) < conf.tolerance) {
            stopMotor();
            state = ST_IDLE;
        } else if (millis() - t0 > conf.az_timeout) {
            stopMotor();
            state = ST_ERROR;
        }
        break;

    case ST_PARKING:
        if (event == EVT_STOP || event == EVT_ABORT) {
            stopMotor();
            state = ST_IDLE;
        } else if (getDistance(target) < conf.tolerance) {
            stopMotor();
            state = ST_IDLE;
            closeShutter(SEL_BOTH); // close shutters after parking
        } else if (millis() - t0 > conf.az_timeout) {
            stopMotor();
            state = ST_ERROR;
        }
        break;

    case ST_ERROR:
    case ST_IDLE:
        if (event == EVT_HOME) {
            t0 = millis();
            state = ST_HOMING;
            moveMotor(getDirection(home_pos));
        } else if (event == EVT_MOVE) {
            state = ST_MOVING;
            moveMotor(getDirection(target));
        } else if (event == EVT_PARK) {
            t0 = millis();
            state = ST_PARKING;
            target = conf.park_pos;
            moveMotor(getDirection(target));
        } else if (event == EVT_GOTO) {
            t0 = millis();
            state = ST_GOING;
            target = next_target;
            moveMotor(getDirection(target));
        }
        break;
    }

    event = EVT_NONE;
}
