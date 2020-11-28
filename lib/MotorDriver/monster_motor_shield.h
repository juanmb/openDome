/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#ifndef _monster_motor_shield_h_
#define _monster_motor_shield_h_

#include "dc_motor.h"


// Monster Motor Shield driver
class MMSMotor : public MotorDriver {
  public:
    MMSMotor(int nmotor);
    void run(bool dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();
  private:
    uint8_t _nmotor;
};

#endif
