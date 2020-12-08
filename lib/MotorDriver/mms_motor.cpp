/*******************************************************************************
Monster Moto Shield controller

https://www.sparkfun.com/products/10182
*******************************************************************************/

#include <Arduino.h>
#include "mms_motor.h"

// Arduino pins
#define EN1 A0
#define EN2 A1
#define CS1 A2
#define CS2 A3
#define INA1 7
#define INA2 4
#define INB1 8
#define INB2 9
#define PWM1 5
#define PWM2 6


/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {INA1, INA2}; // Clockwise input
int inBpin[2] = {INB1, INB2}; // Counter-clockwise input
int pwmpin[2] = {PWM1, PWM2}; // PWM input
int cspin[2] = {CS1, CS2}; // Current sense (analog inputs)
int enpin[2] = {EN1, EN2}; // Status of switches output (analog pin)


MMSMotor::MMSMotor(int n) {
    // _nmotor can be only 0 or 1
    _nmotor = (n > 0);
    pinMode(inApin[_nmotor], OUTPUT);
    pinMode(inBpin[_nmotor], OUTPUT);
    pinMode(pwmpin[_nmotor], OUTPUT);
    stop();
}

void MMSMotor::run(bool dir, int pwm) {
    digitalWrite(inApin[_nmotor], dir);
    digitalWrite(inBpin[_nmotor], !dir);
    analogWrite(pwmpin[_nmotor], pwm);
}

void MMSMotor::stop() {
    digitalWrite(inApin[_nmotor], LOW);
    digitalWrite(inBpin[_nmotor], LOW);
    analogWrite(pwmpin[_nmotor], 0);
}

void MMSMotor::brake() {
    digitalWrite(inApin[_nmotor], HIGH);
    digitalWrite(inBpin[_nmotor], HIGH);
    analogWrite(pwmpin[_nmotor], 0);
}

bool MMSMotor::isRunning() {
    return digitalRead(inApin[_nmotor]) || digitalRead(inBpin[_nmotor]);
}

int MMSMotor::readCurrent() {
    return analogRead(cspin[_nmotor]);
}
