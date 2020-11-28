#include <inttypes.h>
#include <EEPROM.h>
#include "maxdome_protocol.h"

#define BAUDRATE 19200

#define START 0x01  // Start byte

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


// MaxDome II azimuth status
enum MDAzimuthStatus {
    AS_IDLE = 1,
    AS_MOVING_CW,
    AS_MOVING_CCW,
    AS_IDLE2,
    AS_ERROR
};


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


uint8_t getCRC(uint8_t *cmd, uint8_t length) {
    char crc = 0;

    for (int i = 1; i <= length; i++)
        crc -= cmd[i];
    return crc;
}


MaxDomeProtocol::MaxDomeProtocol(Stream *_stream, Dome *_dome) {
    stream = _stream;
    dome = _dome;
}

void MaxDomeProtocol::readSerial() {
    static uint8_t cmdId;
    static uint8_t cmdSize;

    while (stream->available()) {
        char c = stream->read();

        switch (bufPos) {
        case 0:
            // start byte
            if (c != START)
                continue;
            break;

        case 1:
            // command length
            if (c < 0x02 || c > MAX_CMD_SIZE)
                continue;

            cmdSize = c;
            break;

        case 2:
            // command id
            cmdId = c;
            break;
        }

        buffer[bufPos++] = c;

        if (bufPos == cmdSize + 2) {
            //TODO: check CRC
            execCommand(cmdId, buffer);
            bufPos = 0;
        }
    }
}

void MaxDomeProtocol::execCommand(uint8_t id, uint8_t *cmd) {
    switch (id) {
    case ABORT_CMD:
        cmdAbort(cmd);
        break;
    case HOME_CMD:
        cmdHomeAzimuth(cmd);
        break;
    case GOTO_CMD:
        cmdGotoAzimuth(cmd);
        break;
    case SHUTTER_CMD:
        cmdShutterCommand(cmd);
        break;
    case STATUS_CMD:
        cmdStatus(cmd);
        break;
    case SETPARK_CMD:
        cmdSetPark(cmd);
        break;
    case TICKS_CMD:
        cmdSetTicks(cmd);
        break;
    case ACK_CMD:
        cmdAck(cmd);
        break;
    case VBAT_CMD:
        cmdVBat(cmd);
        break;
    };
}

void MaxDomeProtocol::sendResponse(uint8_t *cmd, uint8_t length) {
    cmd[length - 1] = getCRC(cmd, length - 2);

    for (int i = 0; i < length; i++)
        stream->write(cmd[i]);
}


void MaxDomeProtocol::cmdAbort(uint8_t *cmd) {
    dome->abort();
    uint8_t resp[] = {START, 2, TO_COMPUTER | ABORT_CMD, 0x00};
    sendResponse(resp, 4);
}

void MaxDomeProtocol::cmdHomeAzimuth(uint8_t *cmd) {
    dome->home();
    uint8_t resp[] = {START, 3, TO_COMPUTER | HOME_CMD, 0x01, 0x00};
    sendResponse(resp, 5);
}

void MaxDomeProtocol::cmdGotoAzimuth(uint8_t *cmd) {
    // direction field is ignored!
    uint16_t target = bytesToInt(cmd + 4);
    dome->gotoAzimuth(target);

    uint8_t resp[] = {START, 3, TO_COMPUTER | GOTO_CMD, 0x01, 0x00};
    sendResponse(resp, 5);
}

void MaxDomeProtocol::cmdShutterCommand(uint8_t *cmd) {
    switch (cmd[3]) {
    case OPEN_SHUTTER:
        dome->openShutter(SEL_BOTH);
        break;
    case OPEN_UPPER_SHUTTER:
        dome->openShutter(SEL_UPPER);
        break;
    case CLOSE_SHUTTER:
        dome->closeShutter(SEL_BOTH);
        break;
    case EXIT_SHUTTER:
        dome->exitShutters();
        break;
    case ABORT_SHUTTER:
        dome->abortShutters();
        break;
    }

    uint8_t resp[] = {START, 2, TO_COMPUTER | SHUTTER_CMD, 0x00};
    sendResponse(resp, 4);
}

void MaxDomeProtocol::cmdStatus(uint8_t *cmd) {
    DomeStatus status = dome->getStatus();

    MDAzimuthStatus az_st;
    if (status.az_state == ST_IDLE)
        az_st = AS_IDLE;
    else if (status.az_state == ST_ERROR)
        az_st = AS_ERROR;
    else
        az_st = (status.dir == DIR_CW) ? AS_MOVING_CW : AS_MOVING_CCW;

    uint8_t resp[] = {
        START, 9, TO_COMPUTER | STATUS_CMD, status.sh_state, az_st, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    };

    //intToBytes(current_pos, resp + 5);
    //intToBytes(home_pos, resp + 7);

    sendResponse(resp, 11);
}

void MaxDomeProtocol::cmdSetPark(uint8_t *cmd) {
    DomeConf conf = dome->getConf();
    conf.park_on_shutter = cmd[3];
    conf.park_pos = bytesToInt(cmd + 4);
    dome->setConf(conf);
    EEPROM.put(0, conf);

    uint8_t resp[] = {START, 2, TO_COMPUTER | SETPARK_CMD, 0x00};
    sendResponse(resp, 4);
}

void MaxDomeProtocol::cmdSetTicks(uint8_t *cmd) {
    DomeConf conf = dome->getConf();
    conf.ticks_per_turn = bytesToInt(cmd + 3);
    dome->setConf(conf);
    EEPROM.put(0, conf);

    uint8_t resp[] = {START, 2, TO_COMPUTER | TICKS_CMD, 0x00};
    sendResponse(resp, 4);
}

void MaxDomeProtocol::cmdVBat(uint8_t *cmd) {
    int vbat = dome->getBatteryVolts() * 100;

    uint8_t resp[] = {START, 4, TO_COMPUTER | VBAT_CMD, 0x00, 0x00, 0x00};
    intToBytes(vbat, resp + 3);
    sendResponse(resp, 6);
}

void MaxDomeProtocol::cmdAck(uint8_t *cmd) {
    uint8_t resp[] = {START, 3, TO_COMPUTER | ACK_CMD, 0x03, 0x00};
    sendResponse(resp, 5);
}
