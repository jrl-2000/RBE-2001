#pragma once

#include <Romi32u4.h>
#include "PID.h"

class Chassis
{
public:

    void driveDistance(float inches);
    void turnAngle(float angle);
    void drive(float effort);
    void initialize();
    void stopAllMotors();

    float getX();
    float getY();
    void  setX(float newX);
    void  setY(float newY);

    void setAngle(float newAngle);

    float getAngleDegrees();
    float getAngle();
    void updatePosition();
      
    float getLeftEncoder();
    float getRightEncoder();

    bool moveToPoint(float targetX, float targetY);
    bool lineFollowToPoint(float targetX, float targetY, uint16_t SensorValues[]);

    float slewRateCalculate (float desiredRate);
    bool turnToAngle(float targetAngle);

    bool reversed;

    PID drivePowerPID;
    PID turnPowerPID;
    PID turnPID;

    const float wheelDiameter = 2.75;
    const float CPR = 1440;
    const float wheelTrack = 5.5;

    unsigned long lastSlewTime = millis();
    float lastSlewRate = 0;
    float maxAccel = 0.05;
    
    float Sl = wheelTrack/2; //distance from tracking center to middle of left wheel
    float Sr = wheelTrack/2; //distance from tracking center to middle of right wheel

    float x = 0;
    float y = 0;
    float angle = 0;;

    float lastLeftPos = 0;
    float lastRightPos = 0;

    float deltaTheta = 0;
    float thetaNew = 0;
    float thetaM = 0;

    float curLeft = 0;
    float curRight = 0;

    float leftAtReset = 0;
    float rightAtReset = 0;
    float thetaReset = 0;

    float deltaLeft = 0;
    float deltaRight = 0;

    float deltaLr = 0;
    float deltaRr = 0;

    float deltaX = 0;
    float deltaY = 0;


    float theta = 0;
    float radius = 0;

    unsigned long startTime = 0;


private:
    
    int repsAtTarget = -1;
    

    float modulus(float a, float b);
    
    Romi32U4Motors motors;
    Romi32U4Encoders encoders;

};    
