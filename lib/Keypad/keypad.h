#ifndef _keypad_h_
#define _keypad_h_

#include <stdint.h>

enum KeyEvent {
    KEY_EVT_NONE,
    KEY_EVT_PRESS,
    KEY_EVT_RELEASE,
    KEY_EVT_HOLD_RELEASE,
    KEY_EVT_DOUBLE_CLICK,
};

struct KeyMsg {
    KeyEvent event;
    uint8_t key_id;
};

// callback function pointer
typedef void(*KeyMsgCallback)(const KeyMsg &msg);


class DigitalKeypad {
  public:
    DigitalKeypad(uint8_t nkeys, int *pins, uint8_t *key_ids, KeyMsgCallback callback);
    void update();

  private:
    int *pins;
    uint8_t *key_ids;
    uint8_t nkeys;
    uint8_t last_key, last_released_key;
    long int last_t, last_release;
    KeyMsgCallback callback;
};


class AnalogKeypad {
  public:
    AnalogKeypad(int pin, uint8_t nkeys, int *thresholds, uint8_t *key_ids, KeyMsgCallback callback);
    void update();

  private:
    int pin;
    uint8_t *key_ids;
    uint8_t nkeys;
    uint8_t last_key, last_released_key;
    long int last_t, last_release;
    int *thresholds;
    KeyMsgCallback callback;
};

#endif
