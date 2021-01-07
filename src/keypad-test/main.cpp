// Read key events and print them to the serial port

#include <Arduino.h>
#include <keypad.h>

enum KeyIDs {
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_ENTER,
};

const char *key_names[] = {"up", "down", "left", "right", "enter"};

// Threshold values for Escornabot keypad v2.1
// https://github.com/escornabot/electronics/tree/master/Keypad/E_KEYPAD_2.1
int thresholds[] = {597, 725, 794, 851, 953};
uint8_t key_ids[] = {KEY_UP, KEY_LEFT, KEY_DOWN, KEY_ENTER, KEY_RIGHT};

void keyChanged(const KeyMsg &msg) {
    Serial.print("event=");
    Serial.print(msg.event);
    Serial.print(" key=");
    Serial.println(key_names[msg.key_id]);
}

AnalogKeypad pad(A4, 5, thresholds, key_ids, keyChanged);

void setup() {
    Serial.begin(9600);
}

void loop() {
    pad.update();
    delay(1);
}
