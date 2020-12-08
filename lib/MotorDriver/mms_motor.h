/*******************************************************************************
Monster Moto Shield driver

https://www.sparkfun.com/products/10182
*******************************************************************************/

#ifndef _mms_motor_h_
#define _mms_motor_h_

#include "dc_motor.h"


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
