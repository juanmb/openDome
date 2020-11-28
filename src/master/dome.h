#ifndef _dome_h_
#define _dome_h_

#include <Arduino.h>
#include <inttypes.h>
#include "motor_driver.h"

//#define DEBOUNCE_MS     10    // Discard encoder pulses shorter than this duration
// (in milliseconds)

enum Direction {
    DIR_CW,
    DIR_CCW
};

// Azimuth state
enum AzimuthState {
    ST_IDLE,
    ST_MOVING,
    ST_GOING,
    ST_HOMING,
    ST_PARKING,
    ST_ERROR,
};

// Shutter state
enum ShutterState {
    SS_CLOSED = 0,
    SS_OPENING,
    SS_OPEN,
    SS_CLOSING,
    SS_ABORTED,
    SS_ERROR
};

enum AzimuthEvent {
    EVT_NONE,
    EVT_MOVE,
    EVT_GOTO,
    EVT_HOME,
    EVT_PARK,
    EVT_STOP,
    EVT_ABORT
};

enum ShutterSel {
    SEL_LOWER,
    SEL_UPPER,
    SEL_BOTH
};

typedef struct DomeConf {
    uint16_t ticks_per_turn;    // encoder ticks per dome rotation
    uint16_t tolerance;         // position tolerance in ticks
    uint16_t park_pos;          // parking postion
    uint8_t park_on_shutter;    // park dome before operating shutter
    uint8_t encoder_div;        // encoder divider
    uint8_t nshutters;          // number of shutters
    uint8_t az_timeout;         // timeout in seconds
} DomeConf;


typedef struct {
    AzimuthState az_state;
    ShutterState sh_state;
    Direction dir;
    uint16_t pos, home_pos;
} DomeStatus;


class Dome {
  public:
    Dome(Stream *ss, MotorDriver *drv);
    void setConf(DomeConf cfg) {
        conf = cfg;
    };
    DomeConf getConf() {
        return conf;
    };
    DomeStatus getStatus();

    void tick(Direction dir);  // call this method from the encoder ISR
    void update();          // Call this method periodically from the main loop

    void moveAzimuth(Direction dir);
    void stopAzimuth();
    void gotoAzimuth(uint16_t target);
    void home();
    void park();
    void abort();

    float getBatteryVolts();
    void openShutter(ShutterSel);
    void closeShutter(ShutterSel);
    void stopShutter(ShutterSel);
    void abortShutters();
    void exitShutters();

  private:
    void moveMotor(Direction dir);
    void stopMotor();
    Direction getDirection(uint16_t target);
    uint16_t getDistance(uint16_t target);
    ShutterState getShutterState();

    MotorDriver *driver;    // Azimuth motor driver
    Stream *sstream;        // Serial stream of shutters board

    DomeConf conf;
    AzimuthState state = ST_IDLE;
    AzimuthEvent event = EVT_NONE;
    bool home_reached = false;
    Direction current_dir;
    uint16_t home_pos = 0;
    uint16_t next_target = 0;
    uint16_t target = 0;
    uint16_t pos = 0;
};

#endif
