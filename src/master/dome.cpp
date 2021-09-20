#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include "dome.h"

#define VBAT_FACTOR (5.0/1024.0)
#define VBAT_OFFSET 10.55


Dome::Dome(Stream *ss, MotorDriver *drv, int home_pin) {
    sstream = ss;
    driver = drv;
    this->home_pin = home_pin;

    pinMode(home_pin, INPUT_PULLUP);

    // read the configuration from the EEPROM
    DomeConf conf;
    EEPROM.get(0, conf);

    // These values are defined in macros.
    // Values from the EEPROM are ignored
    conf.nshutters = NSHUTTERS;
    conf.tolerance = AZ_TOLERANCE;
    conf.az_timeout = AZ_TIMEOUT;
    conf.encoder_div = ENCODER_DIV;
    this->conf = conf;
}


void Dome::getConf(DomeConf *cfg) {
    memcpy(cfg, &conf, sizeof(DomeConf));
}

void Dome::setConf(DomeConf cfg) {
    conf = cfg;
    EEPROM.put(0, cfg);
};

// Obtain the direction of the shortest path to a target position
Direction Dome::getDirection(uint16_t target) {
    if (target > pos)
        return ((target - pos) > conf.ticks_per_turn / 2) ? DIR_CCW : DIR_CW;

    return ((pos - target) > conf.ticks_per_turn / 2) ? DIR_CW : DIR_CCW;
}

// Return the number of ticks between two positions
uint16_t getDistance(uint16_t pos, uint16_t target, uint16_t ticks_per_turn) {
    // obtain the absolute value of the difference
    uint16_t diff = (target > pos) ?  target - pos : pos - target;

    if (diff > ticks_per_turn / 2)
        return ticks_per_turn - diff;
    return diff;
}

// Return the number of ticks to a target position
uint16_t Dome::distanceTo(uint16_t target) {
    return getDistance(pos, target, conf.ticks_per_turn);
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

void Dome::getStatus(DomeStatus *status) {
    status->az_state = state;
    status->sh_state = getShutterState();
    status->dir = current_dir;
    status->pos = pos;
    status->home_pos = home_pos;
}

void Dome::moveAzimuth(Direction dir) {
    move_dir = dir;
    event = EVT_MOVE;
}

void Dome::gotoAzimuth(uint16_t tgt, Direction dir) {
    next_target = tgt;
    move_dir = dir;
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

void Dome::open(ShutterSel sel) {
    if (sstream) {
        switch (sel) {
        case SEL_UPPER:
            sstream->println("open1");
            break;
        default:
            sstream->println("open");
            break;
        }
    }
}

void Dome::openShutter(ShutterSel sel) {
    if (conf.park_on_shutter) {
        if (distanceTo(conf.park_pos) < conf.tolerance)
            open(sel);
        else {
            //after_park = AFTER_PARK_OPEN;
            park();
        }
    } else
        open(sel);
}

void Dome::close(ShutterSel sel) {
    if (sstream) {
        switch (sel) {
        case SEL_UPPER:
            sstream->println("close1");
            break;
        default:
            sstream->println("close");
            break;
        }
    }
}

void Dome::closeShutter(ShutterSel sel) {
    if (conf.park_on_shutter) {
        if (distanceTo(conf.park_pos) < conf.tolerance)
            close(sel);
        else {
            after_park = AFTER_PARK_CLOSE;
            park();
        }
    } else
        close(sel);
}

void Dome::stopShutter(ShutterSel sel) {
    if (sstream)
        sstream->println("stop");
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
    if (dir == DIR_CCW) {
        ticks++;
    } else {
        ticks--;
    }

    int32_t tmp = (ticks/ENCODER_DIV) % conf.ticks_per_turn;
    pos = tmp >=0 ? tmp : conf.ticks_per_turn + tmp;
}

void Dome::startAzTimeout() {
    t0 = millis();
    last_pos = pos;
}

// timeout if dome is not moving
bool Dome::checkAzTimeout() {
    unsigned long now = millis();

    if (now - t0 > conf.az_timeout) {
        if (getDistance(last_pos, pos, conf.ticks_per_turn) == 0)
            return true;
        t0 = now;
        last_pos = pos;
    }
    return false;
}

// Update state machine
void Dome::update() {
    switch (state) {
    case ST_HOMING:
        if (event == EVT_STOP || event == EVT_ABORT) {
            stopMotor();
            state = ST_IDLE;
        } else if (!digitalRead(home_pin)) {
            stopMotor();
            home_pos = 0;
            ticks = 0;
            pos = 0;
            state = ST_IDLE;
        } else if (checkAzTimeout()) {
            stopMotor();
            state = ST_ERROR;
        }
        break;

    case ST_MOVING:
        if (!digitalRead(home_pin))
            home_pos = pos;     // store detected home position

        if (event == EVT_STOP || event == EVT_ABORT) {
            stopMotor();
            state = ST_IDLE;
        }
        break;

    case ST_GOING:
        if (!digitalRead(home_pin))
            home_pos = pos;     // store detected home position

        if (event == EVT_STOP || event == EVT_ABORT) {
            stopMotor();
            state = ST_IDLE;
        } else if (distanceTo(target) < conf.tolerance) {
            stopMotor();
            state = ST_IDLE;
        } else if (checkAzTimeout()) {
            stopMotor();
            state = ST_ERROR;
        }
        break;

    case ST_PARKING:
        if (event == EVT_STOP || event == EVT_ABORT) {
            stopMotor();
            state = ST_IDLE;
        } else if (distanceTo(target) < conf.tolerance) {
            stopMotor();
            state = ST_IDLE;

            switch (after_park) {
            case AFTER_PARK_CLOSE:
                close(SEL_BOTH);
                break;
            case AFTER_PARK_OPEN:
                open(SEL_BOTH);
                break;
            }
        } else if (checkAzTimeout()) {
            stopMotor();
            state = ST_ERROR;
        }
        break;

    case ST_ERROR:
    case ST_IDLE:
        if (event == EVT_HOME) {
            startAzTimeout();
            state = ST_HOMING;
            moveMotor(getDirection(home_pos));
        } else if (event == EVT_MOVE) {
            state = ST_MOVING;
            moveMotor(move_dir);
        } else if (event == EVT_PARK) {
            startAzTimeout();
            state = ST_PARKING;
            target = conf.park_pos;
            moveMotor(getDirection(target));
        } else if (event == EVT_GOTO) {
            startAzTimeout();
            state = ST_GOING;
            target = next_target;
            moveMotor(move_dir);
        }
        break;
    }

    event = EVT_NONE;
}
