#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PIDController{
private:
    float kp;
    float ki;
    float kd;
    // Error Terms
    float error;
    float previousError
    // Integral & Derivative
    float integral;
    float derivative;
    // Output Limits
    float minOutput;
    float maxOutput;
    // Integral Windup Limits
    float integralMin;
    float integralMax;

public:
    PIDController();
    PIDController(float Kp,float Ki,float Kd);

    void begin();
    float compute(float setpoint,float measurement,float dt)
    void reset();

    void setKp(float Kp);
    void setKi(float Ki);
    void setKd(float Kd);

    float getKp() const;
    float getKi() const;
    float getKd() const;

    void setOutputLimits(float minimum,float maximum);
    void setIntegralLimits(float minimum,float maximum);

    float getError() const;
    float getIntegral() const;
    float getDerivative() const;
};

#endif
