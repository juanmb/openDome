/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#ifndef _dc_motor_h_
#define _dc_motor_h_

#include "motor_driver.h"

// Generic DC motor driver
class DCMotor : public MotorDriver {
public:
    DCMotor(int pin1, int pin2);
    void run(bool dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();
private:
    int pin1, pin2;
};

#endif
