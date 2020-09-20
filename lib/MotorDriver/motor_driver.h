/*******************************************************************************
Monster Motor Shield library

https://www.sparkfun.com/products/10182

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
*******************************************************************************/

#ifndef _motor_driver_h_
#define _motor_driver_h_

#define DIR_CW  0x01
#define DIR_CCW 0x02


// Virtual Motor class
class MotorDriver {
public:
    virtual void run(bool dir, int pwm);
    virtual void stop();
    virtual void brake();
    virtual bool isRunning();
    virtual int readCurrent();
};

#endif
