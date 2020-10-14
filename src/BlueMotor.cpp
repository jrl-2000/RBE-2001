#include "BlueMotor.h"
#include <Arduino.h>
#include "PID.h"


long count = 0;
int errorCount = 0;
int oldValue = 0;
char X = 5;
char encoderArray[4][4] = {
    {0,-1, 1, X},
    {1, 0, X, -1},
    {-1, X, 0, 1},
    {X, 1, -1, 0}
};
void isr();
BlueMotor::BlueMotor(){

}
void pwmSetup(){
    TCCR1A = 0xA8;
    TCCR1B = 0x11;
    ICR1 = 400;
    OCR1C = 0;
}
void BlueMotor::setup(){
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pwmSetup();
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA), isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isr, CHANGE);
    reset();
    oldValue = 0;
    armpid.pidInit(1.3,0,0);
}
void BlueMotor::resetPID() {
    armpid.reset(0);
}
void BlueMotor::turnToPosition(int target) {
    float effort = armpid.pidCalculate(target, getPositionDegrees());
    setEffortNoDB(effort);
    
    
}
void BlueMotor::setEffort(int effort){
    if(abs(effort) > 400){
        effort = (effort/abs(effort))*400;
    }
    setEffort(abs(effort), effort < 0);
}
void BlueMotor::setEffortNoDB(int effort){
    int deadband = 105;
    int newEffort = (abs(effort)>deadband)? abs((effort/400.0)*(400-deadband)+deadband):deadband;
    setEffort(newEffort, effort<0);
}
void BlueMotor::setEffort(int effort, bool clockwise){
    OCR1C = effort;
    digitalWrite(AIN1,clockwise);
    digitalWrite(AIN2,!clockwise);
    if(effort == 0){
        digitalWrite(AIN1,LOW);
        digitalWrite(AIN2,LOW);
    }
}
void BlueMotor::reset(){
    count = 0;
    errorCount = 0;
}
long BlueMotor::getPositionDegrees() {
    Serial.println(getPosition()*360/540);
    return getPosition()*360/540;
    
}
long BlueMotor::getPosition(){
    return count;
}
void isr(){
    int newValue = (digitalRead(3) << 1) | digitalRead(2);
    char value = encoderArray[oldValue][newValue];
    if(value == X){
        errorCount++;
    }else{
        count += value;
    }
    oldValue = newValue;
}