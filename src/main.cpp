#include <Arduino.h>
#include <Romi32U4.h>
#include "Chassis.h"
#include <Romi32U4Buttons.h>
#include "RangeFinder.h"
#include "BlueMotor.h"

Rangefinder rangeFinder;
BlueMotor arm;
Chassis baseBot;
Romi32U4ButtonA pb;
void setup() {
  rangeFinder.setup();
  arm.setup();
  Serial.begin(9600);
}
long positionOld = 0;
int currentEffort = 0;
unsigned long startTime = millis();
bool active = false;
void loop() {

}