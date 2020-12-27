#include <string.h>
#include <Arduino.h>
#include "keypad.h"

#define DEBOUNCE_MS 100     // debounce time in ms
#define HOLD_MS 2000        // 'hold' timeout in ms
#define DOUBLE_CLICK_MS 300 // double click timeout in ms


AnalogKeypad::AnalogKeypad(int pin, uint8_t nkeys, int *thresholds, KeyMsgCallback callback) {
    this->pin = pin;
    this->nkeys = nkeys;
    this->thresholds = thresholds;
    this->callback = callback;
    last_key = 255;
    last_released_key = 255;
    last_t = millis();
}

void AnalogKeypad::update() {
    int value = analogRead(pin);

    uint8_t key = 255;
    for (uint8_t i = 0; i < nkeys; i++) {
        if (value < thresholds[i]) {
            key = i;
            break;
        }
    }

    // no key changed
    if (key == last_key)
        return;

    long int t = millis();

    // debounce
    if ((t - last_t) < DEBOUNCE_MS)
        return;

    bool double_click = false;
    KeyMsg msg;
    if (key == 255) { // release event
        msg.key_id = last_key;
        if (t - last_t > HOLD_MS)
            msg.event = KEY_EVT_HOLD_RELEASE;
        else {
            msg.event = KEY_EVT_RELEASE;
            if ((last_released_key == last_key) && (t - last_release < DOUBLE_CLICK_MS))
                double_click = true;
            last_release = t;
            last_released_key = last_key;
        }
    } else {
        msg.key_id = key;
        msg.event = KEY_EVT_PRESS;
    }

    last_t = t;
    last_key = key;
    callback(msg);

    if (double_click) {
        msg.event = KEY_EVT_DOUBLE_CLICK;
        callback(msg);
    }
    return;
}

