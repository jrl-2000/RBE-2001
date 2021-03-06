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
    armPID.pidInit(20,0,420);
}
void BlueMotor::resetPID() {
    armPID.reset(0.0);
}
void BlueMotor::turnToPosition(float targetDegrees) {
    float effort = -armPID.pidCalculate(targetDegrees, getPositionDegrees());
    // Serial.print("\t");
    // Serial.print(effort);
    setEffortNoDB(effort);
    // Serial.print("\t");
    // Serial.print(pid.getKP());
    // Serial.print("\t");
    // Serial.print(pid.getKI());
    // Serial.print("\t");
    // Serial.print(pid.getKD());
}
void BlueMotor::setEffort(int effort){
    if(abs(effort) > 400){
        effort = (effort/abs(effort))*400;
    }
    setEffort(abs(effort), effort < 0);
}
void BlueMotor::setEffortNoDB(int effort){
    int deadband = 120;
    int newEffort = (abs(effort)/400.0)*(400-deadband)+deadband;
    // Serial.print("\t");
    // Serial.print(((effort>=0)? ((effort==0)? 0: 1):-1)*newEffort);
    setEffort(((effort==0)? 0: 1)*newEffort, effort<0);
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
    return getPosition()*360/540;
}
long BlueMotor::getPosition(){
    return -count;
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