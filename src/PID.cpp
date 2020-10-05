#include "PID.h"

void PID::pidInit(float fKP, float fKI, float fKD) {
    reset();

    kP = fKP;
    kI = fKI;
    kD = fKD;
}

float PID::pidCalculate(float target, float currentValue) {
    if (target!=lastTarget) {
        reset();
        lastTarget = target;
    }

    unsigned long deltaTime = millis()-lastTime;
    lastTime = millis();

    float deltaValue = currentValue-lastValue;
    lastValue = currentValue;

    float slope = deltaValue/deltaTime;
    
    float error = target-currentValue;

    sigma += error*deltaTime;


    float output = error*kP + sigma*kI + slope*kD;
    return output;
}

void PID::reset() {
    lastValue = 0;
    lastTime = millis();
    sigma = 0;
    lastTarget = 0;
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