#pragma once

#include <Romi32u4.h>

//45 degree side 12 cm
//25 degree side 8cm
//staging platform 6 cm


class Rangefinder {
public:
    void setup();
    void loop();
    static void UltrasonicISR();
    void resetPingSummation();
    float getAccurateDistance();
    float getDistanceCM();
private:
    float averagePingSummation = 0;
    float pingIterations = 0;
}; 
