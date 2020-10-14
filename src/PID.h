#pragma once

#include <Romi32u4.h>

class PID
{
public:
    void pidInit(float fKP, float fKI, float fKD);
    void pidInit2(float fKP, float fKI, float fKD, float fEpsilonOuter);

    void reset(float currentValue);
    float pidCalculate(float target, float currentValue);
    float getKP();
    float getKI();
    float getKD();

    float kP;
    float kI;
    float kD;
    float epsilonOuter;

    unsigned long lastTime;
    float lastValue;
    float sigma;
    float lastTarget;


private:
    
    Romi32U4Motors motors;
    Romi32U4Encoders encoders;

};    
