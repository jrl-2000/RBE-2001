#include "PID.h"

void PID::pidInit(float fKP, float fKI, float fKD) {
    pidInit2(fKP, fKI, fKD, 999999);
}

void PID::pidInit2(float fKP, float fKI, float fKD, float fEpsilonOuter) {
    reset(0);

    kP = fKP;
    kI = fKI;
    kD = fKD;

    epsilonOuter = fEpsilonOuter;
}

float PID::pidCalculate(float target, float currentValue) {
    if (target!=lastTarget) {
        reset(currentValue);
        lastTarget = target;
    }
    unsigned long deltaTime = millis()-lastTime;
    lastTime = millis();

    float deltaValue = currentValue-lastValue;
    lastValue = currentValue;

    float slope = deltaValue/deltaTime;
    
    float error = target-currentValue;

    if (abs(error)<epsilonOuter) {
        sigma += error*deltaTime;
    }


    float output = error*kP + sigma*kI - slope*kD;

    return output;
}

void PID::reset(float currentValue) {
    lastValue = currentValue;
    lastTime = millis();
    sigma = 0;
    delay(5);
}

float PID::getKP(){
    return kP;
}
float PID::getKI(){
    return kI;
}
float PID::getKD(){
    return kD;
}