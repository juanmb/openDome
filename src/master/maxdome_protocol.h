#ifndef _maxdome_protocol_h_
#define _maxdome_protocol_h_

#include <inttypes.h>
#include "dome.h"

#define MAX_CMD_SIZE 32


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
