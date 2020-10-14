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
Romi32U4ButtonA pb;

//Team 12
//RBE 2001 A20 Final Project
//Brian Boxell, Jonathan Lopez, Nick Grumski

bool paused = false;

//State machine states
enum States
{
  CLOSE_JAW_START,
  LIFT_PLATE_45,
  BACK_UP_45,
  TURN_TO_PLATFORM_45,
  GO_TO_PLATFORM_45,
  LOWER_ARM_45,
  OPEN_JAW_45_PLATFORM,
  CLOSE_JAW_45_PLATFORM,
  RAISE_ARM_45_PLATFORM,
  MOVE_AWAY_FROM_PLATFORM_45,
  TURN_TO_45_ROOF,
  DRIVE_TO_ROOF_AND_RAISE_PLATE_45,
  DROP_PLATE_45,
  OPEN_JAW_45_END,
  BACK_UP_FROM_45,
  TURN_RIGHT_M,
  DRIVE_TO_CLEAR_ROOF_M,
  TURN_LEFT_M,
  DRIVE_TO_PLATFORM_LINE_M,
  TURN_LEFT_M2,
  DRIVE_TO_ROOF_LINE_M,
  TURN_TO_25_ROOF_1,
  DRIVE_TO_25_ROOF_AND_RAISE_ARM,
  CLOSE_JAW_25_START,
  LIFT_PLATE_25,
  BACK_UP_25,
  TURN_TO_PLATFORM_25,
  GO_TO_PLATFORM_25,
  LOWER_ARM_25,
  OPEN_JAW_25_PLATFORM,
  CLOSE_JAW_25_PLATFORM,
  RAISE_ARM_25_PLATFORM,
  MOVE_AWAY_FROM_PLATFORM_25,
  TURN_TO_25_ROOF_2,
  DRIVE_TO_ROOF_AND_RAISE_PLATE_25,
  DROP_PLATE_25,
  OPEN_JAW_FINAL,
  BACK_UP_FINAL,
  STOPPED
} state;


void checkRemote()
{
  if (decoder.getKeyCode() == remotePlayPause)
  {
    paused = !paused;
    Serial.println(paused ? "Paused" : "Running");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  arm.setup();
  rangeFinder.setup();
  decoder.init();
  arm.reset();
  servo.Init();
  servo.Attach();
  servo.SetMinMaxUS(900, 2100); // replace with our own us values
  pinMode(18, INPUT);
  state = CLOSE_JAW_START;
}

void doStateMachine()
{
  switch (state)
  {
  case CLOSE_JAW_START:
    
    state =  LIFT_PLATE_45;
    break;
  case LIFT_PLATE_45:

    state = BACK_UP_45;
    break;
  case BACK_UP_45:

    state = TURN_TO_PLATFORM_45;
    break;
  case TURN_TO_PLATFORM_45:
    
    state = GO_TO_PLATFORM_45;
    break;
  case GO_TO_PLATFORM_45:

    state = LOWER_ARM_45;
    break;
  case LOWER_ARM_45:

    state = OPEN_JAW_45_PLATFORM;
    break;
  case OPEN_JAW_45_PLATFORM:

    state = CLOSE_JAW_45_PLATFORM;
    break;
  case CLOSE_JAW_45_PLATFORM:

    state = RAISE_ARM_45_PLATFORM;
    break;
  case RAISE_ARM_45_PLATFORM:

    state = MOVE_AWAY_FROM_PLATFORM_45;
    break;
  case MOVE_AWAY_FROM_PLATFORM_45:

    state = TURN_TO_45_ROOF;
    break;
  case TURN_TO_45_ROOF:

    state = DRIVE_TO_ROOF_AND_RAISE_PLATE_45;
    break;
  case DRIVE_TO_ROOF_AND_RAISE_PLATE_45:

    state = DROP_PLATE_45;
    break;
  case DROP_PLATE_45:

    state = OPEN_JAW_45_END;
    break;
  case OPEN_JAW_45_END:

    state = BACK_UP_FROM_45;
    break;
  case BACK_UP_FROM_45:

    state = TURN_RIGHT_M;
    break;
  case TURN_RIGHT_M:

    state = DRIVE_TO_CLEAR_ROOF_M;
    break;
  case  DRIVE_TO_CLEAR_ROOF_M:

    state = TURN_LEFT_M;
    break;
  case TURN_LEFT_M:

    state = DRIVE_TO_PLATFORM_LINE_M;
    break;
  case DRIVE_TO_PLATFORM_LINE_M:

    state = TURN_LEFT_M2;
    break;
  case TURN_LEFT_M2:

    state = DRIVE_TO_ROOF_LINE_M;
    break;
  case DRIVE_TO_ROOF_LINE_M:

    state = TURN_TO_25_ROOF_1;
    break;
  case TURN_TO_25_ROOF_1:

    state = DRIVE_TO_25_ROOF_AND_RAISE_ARM;
    break;
  case DRIVE_TO_25_ROOF_AND_RAISE_ARM:

    state = CLOSE_JAW_25_START;
    break;
  case CLOSE_JAW_25_START:

    state = LIFT_PLATE_25;
    break;
  case LIFT_PLATE_25:

    state = BACK_UP_25;
    break;
  case BACK_UP_25:

    state = TURN_TO_PLATFORM_25;
    break;
  case TURN_TO_PLATFORM_25:

    state =  GO_TO_PLATFORM_25;
    break;
  case GO_TO_PLATFORM_25:

    state = LOWER_ARM_25;
    break;
  case LOWER_ARM_25:

    state = OPEN_JAW_25_PLATFORM;
    break;
  case OPEN_JAW_25_PLATFORM:

    state = CLOSE_JAW_25_PLATFORM;
    break;
  case CLOSE_JAW_25_PLATFORM:

    state = RAISE_ARM_25_PLATFORM;
    break;
  case RAISE_ARM_25_PLATFORM:

    state = MOVE_AWAY_FROM_PLATFORM_25;
    break;
  case MOVE_AWAY_FROM_PLATFORM_25:

    state = TURN_TO_25_ROOF_2;
    break;
  case TURN_TO_25_ROOF_2:

    state = DRIVE_TO_ROOF_AND_RAISE_PLATE_25;
    break;
  case DRIVE_TO_ROOF_AND_RAISE_PLATE_25:

    state = DROP_PLATE_25;
    break;
  case DROP_PLATE_25:

    state = OPEN_JAW_FINAL;
    break;
  case OPEN_JAW_FINAL:

    state = BACK_UP_FINAL;
    break;
  case BACK_UP_FINAL:

    state = STOPPED;
    break;
  case STOPPED:
    
    break;

  }


}

bool buttonPressed;
void loop(){
  // put your main code here, to run repeatedly:
  // put your main code here, to run repeatedly:
  rangeFinder.loop();
  //doStateMachine();
  Serial.println("\n\nX:\tY:\tAngle:\n");
  Serial.print(chassis.getX());
  Serial.print("\t");
  Serial.print(chassis.getY());
  Serial.print("\t");
  Serial.print(chassis.getAngleDegrees());

  if (pb.isPressed()) {
    buttonPressed = true;
  }
  if (buttonPressed) {
   // doStateMachine();
   if (chassis.moveToPoint(-10,-10)) {
      //state = TURN_TO_PLATFORM_45;
      chassis.stopAllMotors();
      buttonPressed = false;
    }
  }
  chassis.updatePosition();
  delay(5);
}
