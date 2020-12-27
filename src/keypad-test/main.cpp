// Read key events and print them to the serial port

#include <Arduino.h>
#include <keypad.h>

// Threshold values for Escornabot keypad v2.1
// https://github.com/escornabot/electronics/tree/master/Keypad/E_KEYPAD_2.1
int thresholds[] = {597, 725, 794, 851, 953};


void keyChanged(const KeyMsg &msg) {
    Serial.print("event=");
    Serial.print(msg.event);
    Serial.print(" key=");
    Serial.println(msg.key_id);
}

AnalogKeypad pad(A0, 5, thresholds, keyChanged);

void setup() {
    Serial.begin(9600);
}

void loop() {
    pad.update();
    delay(1);
}
