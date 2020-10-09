#include <Arduino.h>
#include "Romi32U4.h"
#include "Chassis.h"
#include "Romi32U4Buttons.h"
#include "RangeFinder.h"
#include "BlueMotor.h"
#include "IRdecoder.h"
#include "RemoteConstants.h"
#include "servo32u4.h"
#include "PID.h"


Rangefinder rangeFinder;
BlueMotor arm;
Chassis chassis;
IRDecoder decoder;
Servo32U4 servo;

void checkRemote()
{
  if (decoder.getKeyCode() == remotePlayPause)
  {
    paused = !paused;
    Serial.println(paused ? "Paused" : "Running");
  }
}

void setup() {

  Serial.begin(9600);
  arm.setup();
  rangeFinder.setup();
  decoder.init();
  arm.reset();

}

void loop(){

}
