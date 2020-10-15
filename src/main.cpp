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
#include "QTRSensors.h"

Rangefinder rangeFinder;
BlueMotor arm;
Chassis chassis;
IRDecoder decoder;
Servo32U4 servo;
Romi32U4ButtonC pb;
QTRSensors qtr;

//arm targets
float target25pickup = 300; 
float target45dropoff = 569;
float target45pickup = 960;
float target25dropoff = 130;
float targetStagingPlatform = 1900;

//Team 12
//RBE 2001 A20 Final Project
//Brian Boxell, Jonathan Lopez, Nick Grumski

bool paused = false;
float armTarget = 0;

//State machine states
enum States
{
  MOVE_LIFT_FIRST_POSITION,
  CORRECTION_0,
  DRIVE_TO_REMOVE_45,
  STRAIGHTEN_1,
  CLOSE_JAW_START,
  FEEDBACK_1,
  LIFT_PLATE_45,
  BACK_UP_45,
  TURN_TO_PLATFORM_45,
  CORRECTION_1,
  GO_TO_PLATFORM_45,
  STRAIGHTEN_2,
  LOWER_ARM_45,
  FEEDBACK_2,
  OPEN_JAW_45_PLATFORM,
  FEEDBACK_3,
  CLOSE_JAW_45_PLATFORM,
  RAISE_ARM_45_PLATFORM,
  MOVE_AWAY_FROM_PLATFORM_45,
  TURN_TO_45_ROOF,
  CORRECTION_2,
  DRIVE_TO_ROOF_AND_RAISE_PLATE_45,
  STRAIGHTEN_3,
  DROP_PLATE_45,
  FEEDBACK_4,
  OPEN_JAW_45_END,
  BACK_UP_FROM_45,
  TURN_RIGHT_M,
  DRIVE_TO_CLEAR_ROOF_M,
  TURN_RIGHT_M2,
  CORRECTION_3,
  TURN_LEFT_M,
  DRIVE_TO_PLATFORM_LINE_M,
  TURN_LEFT_M2,
  DRIVE_TO_ROOF_LINE_M,
  TURN_TO_25_ROOF_1,
  CORRECTION_4,
  DRIVE_TO_25_ROOF_AND_RAISE_ARM,
  FEEDBACK_5,
  CLOSE_JAW_25_START,
  LIFT_PLATE_25,
  BACK_UP_25,
  TURN_TO_PLATFORM_25,
  CORRECTION_5,
  GO_TO_PLATFORM_25,
  STRAIGHTEN_4,
  LOWER_ARM_25,
  FEEDBACK_6,
  OPEN_JAW_25_PLATFORM,
  FEEDBACK_7,
  CLOSE_JAW_25_PLATFORM,
  RAISE_ARM_25_PLATFORM,
  MOVE_AWAY_FROM_PLATFORM_25,
  TURN_TO_25_ROOF_2,
  CORRECTION_6,
  DRIVE_TO_ROOF_AND_RAISE_PLATE_25,
  DROP_PLATE_25,
  FEEDBACK_8,
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
  state = MOVE_LIFT_FIRST_POSITION;
  chassis.initialize();
  chassis.setY(0);
}
float distance = 0;
float distanceError = 0;
int waitIterations = 0;
void doStateMachine()
{
  switch (state)
  {
  case MOVE_LIFT_FIRST_POSITION:
    armTarget = target45pickup;
    if (waitIterations > 20) {
      state = DRIVE_TO_REMOVE_45;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case DRIVE_TO_REMOVE_45:
    servo.Write(1300);
    if (chassis.moveToPoint(0,9.25)){
      chassis.stopAllMotors();
      state = CLOSE_JAW_START;
    }
    break;
  case STRAIGHTEN_1:
    if (chassis.turnToAngle(0)){
      chassis.stopAllMotors();
      state = CLOSE_JAW_START;
    }
    break;
  case CLOSE_JAW_START:
    servo.Write(1900);
    state =  LIFT_PLATE_45;
    break;
  case LIFT_PLATE_45:
    armTarget = target45pickup-200;
    state = BACK_UP_45;
    break;
  case BACK_UP_45:
    if (chassis.moveToPoint(0,-1)){
      chassis.stopAllMotors();
      state = TURN_TO_PLATFORM_45;
    }
    break;
  case TURN_TO_PLATFORM_45:
    if (chassis.turnToAngle(94)) {
      chassis.stopAllMotors();
      state = CORRECTION_1;
    }
    break;
  case CORRECTION_1:
    if (waitIterations % 4 == 2) {
      distance = rangeFinder.getAccurateDistance()/2.54;
      if (distance>0) {
        distanceError = distance - 10.5;
        chassis.setX(chassis.getX()-distanceError);
        state = GO_TO_PLATFORM_45;
        waitIterations = 0;
      }
      else {
        waitIterations ++;
      }
    }
    else {
      waitIterations ++;
    }
    break;
  case GO_TO_PLATFORM_45:
    //armTarget = targetStagingPlatform;
    if (chassis.moveToPoint(9.5,-2.2)){
      chassis.stopAllMotors();
      state = STRAIGHTEN_2;
    }
    break;
  case STRAIGHTEN_2:
    if (chassis.turnToAngle(95)){
      chassis.stopAllMotors();
      state = LOWER_ARM_45;
    }
    break;
  case LOWER_ARM_45:
    armTarget = targetStagingPlatform;
    if (waitIterations > 250) {
      state = OPEN_JAW_45_PLATFORM;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case OPEN_JAW_45_PLATFORM:
    if (waitIterations > 150) {
      state = CLOSE_JAW_45_PLATFORM;
      waitIterations = 0;
    }
    else {
      servo.Write(1300);
      waitIterations ++;
    }
    break;
  case CLOSE_JAW_45_PLATFORM:
    servo.Write(1900);
     if (waitIterations > 100) {
      state = RAISE_ARM_45_PLATFORM;  
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case RAISE_ARM_45_PLATFORM:
    armTarget = target45dropoff;
    state = MOVE_AWAY_FROM_PLATFORM_45;
    break;
  case MOVE_AWAY_FROM_PLATFORM_45:
    if (chassis.moveToPoint(-0.5,0)){
      chassis.stopAllMotors();     
      state = TURN_TO_45_ROOF;
    }
    break;
  case TURN_TO_45_ROOF:
    if (chassis.turnToAngle(0)) {
      chassis.stopAllMotors();
      state = CORRECTION_2;
    }
    break;
  case CORRECTION_2:
    if (waitIterations % 4 == 3) {
      distance = rangeFinder.getAccurateDistance()/2.54;
      if (distance>0) {
        distanceError = distance - 11;
        chassis.setY(chassis.getY()-distanceError);
        state = DRIVE_TO_ROOF_AND_RAISE_PLATE_45;
        waitIterations = 0;
      }
      else {
        waitIterations ++;
      }
    }
    else {
      waitIterations ++;
    }
    break;
  case DRIVE_TO_ROOF_AND_RAISE_PLATE_45:
    if (chassis.moveToPoint(0,8.75)){
      chassis.stopAllMotors();    
      state = DROP_PLATE_45;
    }
    break;
  case STRAIGHTEN_3:
    if (chassis.turnToAngle(0)){
      chassis.stopAllMotors();
      state = DROP_PLATE_45;
    }
    break;
  case DROP_PLATE_45:
    armTarget = target45dropoff+50;
    if (waitIterations > 100) {
      state = OPEN_JAW_45_END;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case OPEN_JAW_45_END:
    servo.Write(1300);
    state = BACK_UP_FROM_45;
    break;
  case BACK_UP_FROM_45:
    if (chassis.moveToPoint(6,-1)){
      chassis.stopAllMotors();
     
      state = TURN_RIGHT_M;
    }
    break;
  case TURN_RIGHT_M:
    if (chassis.turnToAngle(76)) {
      chassis.stopAllMotors();
      state = DRIVE_TO_CLEAR_ROOF_M;
    }
    break;
  case  DRIVE_TO_CLEAR_ROOF_M:
    if (chassis.moveToPoint(7.5,33)){
      chassis.stopAllMotors();           
      state = TURN_RIGHT_M2;
    }
    break;
  case TURN_RIGHT_M2:
    if (chassis.turnToAngle(90)) {
      chassis.stopAllMotors();
      state = CORRECTION_3;

    }

    break;
  case CORRECTION_3:
    if (waitIterations % 4 == 3) {
      distance = rangeFinder.getAccurateDistance()/2.54;
      if (distance>0) {
        distanceError = distance - 3.25;
        chassis.setY(chassis.getY()-distanceError);
        state = TURN_LEFT_M2;
        waitIterations = 0;
      }
      else {
        waitIterations ++;
      }
    }
    else {
      waitIterations ++;
    }
    break;
  case TURN_LEFT_M2:
    if (chassis.turnToAngle(-90)) {
      chassis.stopAllMotors();
      state = DRIVE_TO_ROOF_LINE_M;

    }

    break;
  case DRIVE_TO_ROOF_LINE_M:
    if (chassis.moveToPoint(0,33)){
      chassis.stopAllMotors();           
      state = TURN_TO_25_ROOF_1;
    }
    break;
  case TURN_TO_25_ROOF_1:
    armTarget = target25pickup;
    if (chassis.turnToAngle(172)) {
      chassis.stopAllMotors();
      state = CORRECTION_4;
    }
    break;
  case CORRECTION_4:
    if (waitIterations % 4 == 3) {
      distance = rangeFinder.getAccurateDistance()/2.54;
      if (distance>0) {
        distanceError = distance - 8;
        chassis.setY(chassis.getY()-distanceError);
        state = DRIVE_TO_25_ROOF_AND_RAISE_ARM;
        waitIterations = 0;
      }
      else {
        waitIterations ++;
      }
    }
    else {
      waitIterations ++;
    }
    break;
  case DRIVE_TO_25_ROOF_AND_RAISE_ARM:
    if (chassis.moveToPoint(1,23)){
      chassis.stopAllMotors();           
      state = CLOSE_JAW_25_START;
    }
    break;
  case CLOSE_JAW_25_START:

    state = LIFT_PLATE_25;
    break;
  case LIFT_PLATE_25:

    state = BACK_UP_25;
    break;
  case BACK_UP_25:
    if (chassis.moveToPoint(2,33)){
      chassis.stopAllMotors();           
      state = TURN_TO_PLATFORM_25;
    }
    break;
  case TURN_TO_PLATFORM_25:
    if (chassis.turnToAngle(82)){
      chassis.stopAllMotors();           
      state =  GO_TO_PLATFORM_25;
    }
    break;
  case CORRECTION_5:
    distance = rangeFinder.getDistanceCM()/2.54;
    distanceError = distance - 10.75;
    chassis.setX(chassis.getX()-distanceError);
    state = GO_TO_PLATFORM_25;
    break;
  case GO_TO_PLATFORM_25:
    //armTarget = targetStagingPlatform;
    if (chassis.moveToPoint(9.8,34)){
      chassis.stopAllMotors();
      state = STRAIGHTEN_4;
    }
    break;
  case STRAIGHTEN_4:
    if (chassis.turnToAngle(86)){
      chassis.stopAllMotors();
      state = LOWER_ARM_25;
    }
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
    if (chassis.moveToPoint(1,32)){
      chassis.stopAllMotors();           
      state = TURN_TO_25_ROOF_2;
    }
    break;
  case TURN_TO_25_ROOF_2:
    if (chassis.turnToAngle(174)){
      chassis.stopAllMotors();
      state = CORRECTION_6;
    }
    break;
  case CORRECTION_6:
    distance = rangeFinder.getDistanceCM()/2.54;
    distanceError = distance - 12;
    chassis.setY(chassis.getY()-distanceError);
    state = DRIVE_TO_ROOF_AND_RAISE_PLATE_25;
    break;
  case DRIVE_TO_ROOF_AND_RAISE_PLATE_25:
    if (chassis.moveToPoint(2,23)){
      chassis.stopAllMotors();           
      state = DROP_PLATE_25;
    }
    break;
  case DROP_PLATE_25:

    state = OPEN_JAW_FINAL;
    break;
  case OPEN_JAW_FINAL:

    state = BACK_UP_FINAL;
    break;
  case BACK_UP_FINAL:
    if (chassis.moveToPoint(1,33)){
      chassis.stopAllMotors();           
      state = STOPPED;
    }
    break;
  case STOPPED:
    
    break;

  }


}

// void loop(){

  
//   //arm.turnToPosition(target45pickup);
//   //arm.getPositionDegrees();
//   int value = analogRead(18);
//   Serial.println(value);
//   servo.Write(1300);

  
//   delay(1000);
// }


bool buttonPressed = false;
void loop(){
  // put your main code here, to run repeatedly:
  // put your main code here, to run repeatedly:
  rangeFinder.loop();
  //doStateMachine();
  // Serial.println("\n\nX:\tY:\tAngle:\n");
  // Serial.print(chassis.getX());
  // Serial.print("\t");
  // Serial.print(chassis.getY());
  // Serial.print("\t");
  // Serial.print(chassis.getAngleDegrees());

  checkRemote();
  if (pb.isPressed()) {
    buttonPressed = true;
    delay(800);
  }
  if (buttonPressed) {
    //servo.Write(1900);
    arm.turnToPosition(armTarget);
    //Serial.println(arm.getPositionDegrees());
    doStateMachine();
    // if (chassis.turnToAngle(90)) {
    // //if (chassis.moveToPoint(10,21.5)) {
    //   //state = TURN_TO_PLATFORM_45;
    //   chassis.stopAllMotors();
    //   buttonPressed = false;
    // }
  }
  chassis.updatePosition();
  delay(5);
}



