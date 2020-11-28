/*
#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "serial_command.h"
#include "dome.h"

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
#define OPEN_UPPER_SHUTTER	0x02
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

SerialCommand sCmd(&Serial);


// Convert two bytes to a uint16_t value (big endian)
uint16_t bytesToInt(uint8_t *data)
{
    uint16_t value1 = data[1];
    uint16_t value2 = data[0];
    return (value1 & 0xff) | ((value2 << 8) & 0xff00);
}

// Convert a uint16_t value to bytes (big endian)
void intToBytes(uint16_t value, uint8_t *data)
{
    data[1] = value & 0xff;
    data[0] = (value >> 8) & 0xff;
}


void cmdAbort(uint8_t *cmd)
{
	dome.abort();
    uint8_t resp[] = {START, 2, TO_COMPUTER | ABORT_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdHomeAzimuth(uint8_t *cmd)
{
	dome.home();
    uint8_t resp[] = {START, 3, TO_COMPUTER | HOME_CMD, 0x01, 0x00};
    sCmd.sendResponse(resp, 5);
}

void cmdGotoAzimuth(uint8_t *cmd)
{
	// direction field is ignored!
    uint16_t target = bytesToInt(cmd + 4);
	dome.gotoAzimuth(target);

    uint8_t resp[] = {START, 3, TO_COMPUTER | GOTO_CMD, 0x01, 0x00};
    sCmd.sendResponse(resp, 5);
}

void cmdShutterCommand(uint8_t *cmd)
{
    switch(cmd[3]) {
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

void cmdStatus(uint8_t *cmd)
{
	AzimuthState state = dome.getAzimuthState();

    MDAzimuthStatus az_st;
    if (state == ST_IDLE) {
        az_st = AS_IDLE;
    }
    else if (state == ST_ERROR) {
        az_st = AS_ERROR;
    }
    else {
        //if (current_dir == DIR_CW)
            //az_st = AS_MOVING_CW;
        //else
            //az_st = AS_MOVING_CCW;
    }

    uint8_t sh_st = (uint8_t)dome.getShutterState();
    uint8_t resp[] = {
		START, 9, TO_COMPUTER | STATUS_CMD, sh_st, az_st, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00
	};

    //intToBytes(current_pos, resp + 5);
    //intToBytes(home_pos, resp + 7);

    sCmd.sendResponse(resp, 11);
}

void cmdSetPark(uint8_t *cmd)
{
	DomeConf conf = dome.getConf();
    conf.park_on_shutter = cmd[3];
    conf.park_pos = bytesToInt(cmd + 4);
	dome.setConf(conf);
	EEPROM.put(0, conf);

    uint8_t resp[] = {START, 2, TO_COMPUTER | SETPARK_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdSetTicks(uint8_t *cmd)
{
	DomeConf conf = dome.getConf();
    conf.ticks_per_turn = bytesToInt(cmd + 3);
	dome.setConf(conf);
	EEPROM.put(0, conf);

    uint8_t resp[] = {START, 2, TO_COMPUTER | TICKS_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdVBat(uint8_t *cmd)
{
    int vbat = dome.getBatteryVolts() * 100;

    uint8_t resp[] = {START, 4, TO_COMPUTER | VBAT_CMD, 0x00, 0x00, 0x00};
    intToBytes(vbat, resp + 3);
    sCmd.sendResponse(resp, 6);
}

void cmdAck(uint8_t *cmd)
{
    uint8_t resp[] = {START, 3, TO_COMPUTER | ACK_CMD, 0x03, 0x00};
    sCmd.sendResponse(resp, 5);
}

void setup()
{
    sCmd.addCommand(ABORT_CMD, 2, cmdAbort);
    sCmd.addCommand(HOME_CMD, 2, cmdHomeAzimuth);
    sCmd.addCommand(GOTO_CMD, 5, cmdGotoAzimuth);
    sCmd.addCommand(SHUTTER_CMD, 3, cmdShutterCommand);
    sCmd.addCommand(STATUS_CMD, 2, cmdStatus);
    sCmd.addCommand(SETPARK_CMD, 5, cmdSetPark);
    sCmd.addCommand(TICKS_CMD, 4, cmdSetTicks);
    sCmd.addCommand(ACK_CMD, 2, cmdAck);
    sCmd.addCommand(VBAT_CMD, 2, cmdVBat);

	DomeConf conf;
	EEPROM.get(0, conf);
	conf.nshutters = NSHUTTERS;
	conf.tolerance = AZ_TOLERANCE;
	conf.az_timeout = (uint8_t)AZ_TIMEOUT;
	conf.encoder_div = ENCODER_DIV;
	dome.setConf(conf);

    Serial.begin(BAUDRATE);
}
*/
