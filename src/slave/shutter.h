#ifndef _shutter_h_
#define _shutter_h_

#include "motor_driver.h"

// Lid states
enum State {
    ST_CLOSED,
    ST_OPENING,
    ST_OPEN,
    ST_CLOSING,
    ST_ABORTED,
    ST_ERROR,
};


enum Action {
    DO_NONE,
    DO_OPEN,
    DO_CLOSE,
    DO_ABORT,
};

enum InterType {
    INTER_NO,    // non-interfering type
    INTER_INNER, // inner shutter cannot open if the outer one is closed
    INTER_OUTER, // outer shutter cannot close if the inner one is open
};

// Define a pointer to a function for checking shutter/flap interference
typedef bool (*interFn)(State st);


class Shutter {
  public:
    // Non-interfering shutter constructor
    // swClosed and swOpen: limit switches. Normally close, active high.
    Shutter(MotorDriver *motor, int swClosed, int swOpen,
            unsigned long timeout);

    // Interfering shutter constructor
    // swClosed and swOpen: limit switches. Normally close, active high.
    // swInter: interference detection limit switch.
    // If type is INTER_INNER: swInter HIGH prevents shutter opening
    // If type is INTER_OUTER: swInter LOW prevents shutter closing
    Shutter(MotorDriver *motor, int swClosed, int swOpen, int swInter,
            InterType type, unsigned long timeout);

    void open();
    void close();
    void abort();
    void update();
    State getState();

  private:
    void initState();
    InterType interType;
    MotorDriver *motor;
    State state;
    Action nextAction;
    unsigned long runTimeout;
    int swClosed, swOpen, swInter;
};

#endif
