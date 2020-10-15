#pragma once
#include "PID.h"



class BlueMotor{
    public:
        BlueMotor();
        void setEffort(int effort);
        void setEffortNoDB(int effort);
        void turnToPosition(float targetDegrees);
        void resetPID();
        //void moveTo(long position);
        long getPositionDegrees();
        long getPosition();
        void reset();
        void setup();

        PID armPID;
    private:
        void setEffort(int effort, bool clockwise);
        const int tolerance = 3;
        const int PWMOutPin = 11;
        const int AIN2 = 4;
        const int AIN1 = 13;
        const int ENCA = 2;
        const int ENCB = 3;

};