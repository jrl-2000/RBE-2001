#pragma once

#include <Romi32u4.h>



class Rangefinder {
public:
 void setup();
 void loop();
 static void UltrasonicISR();
 float getDistanceCM();
}; 
