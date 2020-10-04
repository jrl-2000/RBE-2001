#pragma once

#include <Romi32u4.h>

class Chassis
{
public:

    void driveDistance(float inches);
    void turnAngle(float angle);
    void drive(float effort);
      


    const float wheelDiameter = 2.75;
    const int CPR = 1440;
    const float wheelTrack = 5.5;




private:
    
    Romi32U4Motors motors;
    Romi32U4Encoders encoders;

};    
