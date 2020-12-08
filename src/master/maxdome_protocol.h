#ifndef _maxdome_protocol_h_
#define _maxdome_protocol_h_

#include <inttypes.h>
#include "dome.h"

#define MAXDOME_BAUDRATE 19200

#define MAX_CMD_SIZE 32
#define START       0x01  // Start byte

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
enum ShutterCommand {
    OPEN_BOTH = 1,
    OPEN_UPPER,
    CLOSE_BOTH,
    EXIT_SHUTTER,
    ABORT_SHUTTER = 7,
};

// MaxDome II azimuth status
enum MDAzimuthStatus {
    AS_IDLE = 1,
    AS_MOVING_CW,
    AS_MOVING_CCW,
    AS_IDLE2,
    AS_ERROR
};


class MaxDomeProtocol {
  public:
    MaxDomeProtocol(Stream *s, Dome *dome);
    void readSerial();

  private:
    Stream *stream;
    Dome *dome;

    void execCommand(uint8_t id, uint8_t *cmd);
    void sendResponse(uint8_t *, uint8_t);
    uint8_t buffer[MAX_CMD_SIZE];
    int cmdId;
    int cmdSize;
    int bufPos;

    void cmdAbort(uint8_t *cmd);
    void cmdHomeAzimuth(uint8_t *cmd);
    void cmdGotoAzimuth(uint8_t *cmd);
    void cmdShutterCommand(uint8_t *cmd);
    void cmdStatus(uint8_t *cmd);
    void cmdSetPark(uint8_t *cmd);
    void cmdSetTicks(uint8_t *cmd);
    void cmdAck(uint8_t *cmd);
    void cmdVBat(uint8_t *cmd);
};

#endif
