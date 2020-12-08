/*******************************************************************************
Generic motor controller
*******************************************************************************/

#ifndef _motor_driver_h_
#define _motor_driver_h_


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
