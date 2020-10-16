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
float target25pickup = 580; 
float target45dropoff = 569;
float target45pickup = 1030;
float target25dropoff = 200;
float targetStagingPlatform = 1969;

//Team 12
//RBE 2001 A20 Final Project
//Brian Boxell, Jonathan Lopez, Nick Grumski

bool paused = false;
float armTarget = 0;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

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
  POSITION_FOR_CROSS,
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
  servo.SetMinMaxUS(1000, 1800); 
  pinMode(18, INPUT);
  state = MOVE_LIFT_FIRST_POSITION;
  chassis.initialize();
  chassis.setY(4.4);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A2, A3}, SensorCount);
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
    if (waitIterations > 350) {
      state = DRIVE_TO_REMOVE_45;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case DRIVE_TO_REMOVE_45:
    servo.Write(1300);
    if (chassis.lineFollowToPoint(0,8, sensorValues)){
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
    chassis.setX(0);
    state =  LIFT_PLATE_45;
    break;
  case LIFT_PLATE_45:
    armTarget = 380;
    if (waitIterations > 350) {
      state = DRIVE_TO_REMOVE_45;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    state = BACK_UP_45;
    break;
  case BACK_UP_45:
    if (chassis.moveToPoint(0,-0.5)){
      chassis.stopAllMotors();
      state = TURN_TO_PLATFORM_45;
    }
    break;
  case TURN_TO_PLATFORM_45:
    if (chassis.turnToAngle(90)) {
      chassis.stopAllMotors();
      state = GO_TO_PLATFORM_45;
      chassis.setY(0);
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
    chassis.setY(0);
    if (chassis.lineFollowToPoint(10.5,0,sensorValues)){
      chassis.stopAllMotors();
      chassis.setY(0);
      waitIterations = 0;
      state = LOWER_ARM_45;

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
    if (waitIterations > 2000) {
      state = OPEN_JAW_45_PLATFORM;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case OPEN_JAW_45_PLATFORM:
    if (waitIterations > 1500) {
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
     if (waitIterations > 1000) {
      state = RAISE_ARM_45_PLATFORM;  
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case RAISE_ARM_45_PLATFORM:
    armTarget = target45dropoff;
    //chassis.setAngle(90);
    state = MOVE_AWAY_FROM_PLATFORM_45;
    break;
  case MOVE_AWAY_FROM_PLATFORM_45:
    if (chassis.moveToPoint(0,0.5)){
      chassis.stopAllMotors();     
      state = TURN_TO_45_ROOF;
    }
    break;
  case TURN_TO_45_ROOF:
    if (chassis.turnToAngle(0)) {
      chassis.stopAllMotors();
      state = DRIVE_TO_ROOF_AND_RAISE_PLATE_45;
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
    chassis.setX(0);
    if (chassis.lineFollowToPoint(0,9.5, sensorValues)){
      chassis.stopAllMotors();    
      state = DROP_PLATE_45;
      chassis.setX(0);
      chassis.setAngle(0);
    }
    break;
  case STRAIGHTEN_3:
    if (chassis.turnToAngle(0)){
      chassis.stopAllMotors();
      state = DROP_PLATE_45;
    }
    break;
  case DROP_PLATE_45:
    armTarget = target45dropoff+80;
    servo.Write(1300);

    if (waitIterations > 100) {
      state = OPEN_JAW_45_END;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case OPEN_JAW_45_END:
    armTarget = target45dropoff+300;
    state = BACK_UP_FROM_45;
    break;
  case BACK_UP_FROM_45:
    if (chassis.moveToPoint(0,4)){
      chassis.stopAllMotors();
     
      state = POSITION_FOR_CROSS;
    }
    break;
  case POSITION_FOR_CROSS:
    if (chassis.moveToPoint(5.5,-4)){
      chassis.stopAllMotors();
     
      state = TURN_RIGHT_M;
    }
    break;
  case TURN_RIGHT_M:
    if (chassis.turnToAngle(80)) {
      chassis.stopAllMotors();
      state = DRIVE_TO_CLEAR_ROOF_M;
    }
    break;
  case  DRIVE_TO_CLEAR_ROOF_M:
    if (chassis.moveToPoint(7,33)){
      chassis.stopAllMotors();           
      state = TURN_LEFT_M2;
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
      chassis.setAngle(-90);
      chassis.setY(32);
    }

    break;
  case DRIVE_TO_ROOF_LINE_M:
    chassis.setY(32);
    if (chassis.lineFollowToPoint(-1.5,32,sensorValues)) {
      chassis.stopAllMotors();           
      state = TURN_TO_25_ROOF_1;
      chassis.setAngle(-90);
      chassis.setAngle(-32);
    }
    break;
  case TURN_TO_25_ROOF_1:
    armTarget = target25pickup;
    if (chassis.turnToAngle(175)) {
      chassis.stopAllMotors();
      state = DRIVE_TO_25_ROOF_AND_RAISE_ARM;
      chassis.setX(0);
      chassis.setAngle(-180);
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
    chassis.setX(0);
    if (chassis.lineFollowToPoint(0,27.25,sensorValues)){
      chassis.stopAllMotors();           
      state = CLOSE_JAW_25_START;
      chassis.setX(0);
    }
    break;
  case CLOSE_JAW_25_START:
    servo.Write(1900);
    if (waitIterations > 120) {
      state = OPEN_JAW_45_END;
      state = LIFT_PLATE_25;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case LIFT_PLATE_25:
    armTarget = target25pickup -200;
    state = BACK_UP_25;
    chassis.setAngle(179.0);
    break;
  case BACK_UP_25:
    if (chassis.moveToPoint(0,35.25)){
      chassis.stopAllMotors();           
      state = TURN_TO_PLATFORM_25;
    }
    break;
  case TURN_TO_PLATFORM_25:
    if (chassis.turnToAngle(82.5)){
      chassis.stopAllMotors();           
      state =  GO_TO_PLATFORM_25;
      chassis.setY(32);
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
    chassis.setY(32);
    if (chassis.lineFollowToPoint(8.5,32,sensorValues)){
      chassis.stopAllMotors();
      state = LOWER_ARM_25;
    }
    break;
  case STRAIGHTEN_4:
    if (chassis.turnToAngle(86)){
      chassis.stopAllMotors();
      state = LOWER_ARM_25;
    }
    break;
  case LOWER_ARM_25:
    armTarget = targetStagingPlatform;
    if (waitIterations > 2000) {
      state = OPEN_JAW_25_PLATFORM;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case OPEN_JAW_25_PLATFORM:
    if (waitIterations > 1500) {
      state = CLOSE_JAW_25_PLATFORM;
      waitIterations = 0;
    }
    else {
      servo.Write(1300);
      waitIterations ++;
    }
    break;
  case CLOSE_JAW_25_PLATFORM:
    servo.Write(1900);
     if (waitIterations > 1000) {
      state = RAISE_ARM_25_PLATFORM;  
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case RAISE_ARM_25_PLATFORM:
    armTarget = 600;
    chassis.setY(32);
    chassis.setAngle(90);
    state = MOVE_AWAY_FROM_PLATFORM_25;
    break;
  case MOVE_AWAY_FROM_PLATFORM_25:
    if (chassis.moveToPoint(1.25,25.25)){
      chassis.stopAllMotors();           
      state = TURN_TO_25_ROOF_2;
    }
    break;
  case TURN_TO_25_ROOF_2:
    armTarget = target25dropoff;
    if (chassis.turnToAngle(178)){
      chassis.stopAllMotors();
      state = DRIVE_TO_ROOF_AND_RAISE_PLATE_25;
    }
    break;
  case CORRECTION_6:
    distance = rangeFinder.getDistanceCM()/2.54;
    distanceError = distance - 12;
    chassis.setY(chassis.getY()-distanceError);
    state = DRIVE_TO_ROOF_AND_RAISE_PLATE_25;
    break;
  case DRIVE_TO_ROOF_AND_RAISE_PLATE_25:
    chassis.setX(0);
    //chassis.setAngle(-179);
    if (chassis.lineFollowToPoint(0,16.6,sensorValues)){
      chassis.stopAllMotors();           
      state = DROP_PLATE_25;
    }
    break;
  case DROP_PLATE_25:
    servo.Write(1300);
    state = OPEN_JAW_FINAL;
    break;
  case OPEN_JAW_FINAL:
    armTarget = target25dropoff+80;
    state = BACK_UP_FINAL;
    break;
  case BACK_UP_FINAL:
    if (chassis.moveToPoint(0,33)){
      chassis.stopAllMotors();           
      state = STOPPED;
    }
    break;
  case STOPPED:
    
    break;

  }


}

void doStateMachine2()
{
  switch (state)
  {
  case MOVE_LIFT_FIRST_POSITION:
    armTarget = target25pickup;
    if (waitIterations > 100) {
      state = DRIVE_TO_REMOVE_45;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case DRIVE_TO_REMOVE_45:
    servo.Write(1200);
    if (chassis.lineFollowToPoint(0,8.25, sensorValues)){
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
    chassis.setX(0);
    state =  LIFT_PLATE_45;
    break;
  case LIFT_PLATE_45:
    armTarget = target45pickup-200;
    state = BACK_UP_45;
    break;
  case BACK_UP_45:
    if (chassis.moveToPoint(0,0)){
      chassis.stopAllMotors();
      state = TURN_TO_PLATFORM_45;
    }
    break;
  case TURN_TO_PLATFORM_45:
    if (chassis.turnToAngle(-90)) {
      chassis.stopAllMotors();
      state = GO_TO_PLATFORM_45;
      chassis.setY(0);
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
    chassis.setY(0);
    if (chassis.lineFollowToPoint(-10.5,0,sensorValues)){
      chassis.stopAllMotors();
      chassis.setY(0);
      waitIterations = 0;
      state = LOWER_ARM_45;

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
    if (waitIterations > 2000) {
      state = OPEN_JAW_45_PLATFORM;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case OPEN_JAW_45_PLATFORM:
    if (waitIterations > 1500) {
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
     if (waitIterations > 1000) {
      state = RAISE_ARM_45_PLATFORM;  
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case RAISE_ARM_45_PLATFORM:
    armTarget = 600;
    //chassis.setAngle(90);
    state = MOVE_AWAY_FROM_PLATFORM_45;
    break;
  case MOVE_AWAY_FROM_PLATFORM_45:
    if (chassis.moveToPoint(0.5,0.5)){
      chassis.stopAllMotors();     
      state = TURN_TO_45_ROOF;
    }
    break;
  case TURN_TO_45_ROOF:
     armTarget = target25dropoff;
    if (chassis.turnToAngle(0)) {
      chassis.stopAllMotors();
      state = DRIVE_TO_ROOF_AND_RAISE_PLATE_45;
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
    chassis.setX(0);
    if (chassis.lineFollowToPoint(0,10.625, sensorValues)){
      chassis.stopAllMotors();    
      state = DROP_PLATE_45;
      chassis.setX(0);
      chassis.setAngle(0);
    }
    break;
  case STRAIGHTEN_3:
    if (chassis.turnToAngle(0)){
      chassis.stopAllMotors();
      state = DROP_PLATE_45;
    }
    break;
  case DROP_PLATE_45:
    armTarget = target25dropoff+80;
    servo.Write(1300);

    if (waitIterations > 1000) {
      state = OPEN_JAW_45_END;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case OPEN_JAW_45_END:
    armTarget = target25dropoff+80;
    state = BACK_UP_FROM_45;
    break;
  case BACK_UP_FROM_45:
    if (chassis.moveToPoint(0,4)){
      chassis.stopAllMotors();
     
      state = STOPPED;
    }
    break;
  case POSITION_FOR_CROSS:
    if (chassis.moveToPoint(6,-4)){
      chassis.stopAllMotors();
     
      state = TURN_RIGHT_M;
    }
    break;
  case TURN_RIGHT_M:
    if (chassis.turnToAngle(80)) {
      chassis.stopAllMotors();
      state = DRIVE_TO_CLEAR_ROOF_M;
    }
    break;
  case  DRIVE_TO_CLEAR_ROOF_M:
    if (chassis.moveToPoint(7,33)){
      chassis.stopAllMotors();           
      state = TURN_LEFT_M2;
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
      chassis.setAngle(-90);
      chassis.setY(32);
    }

    break;
  case DRIVE_TO_ROOF_LINE_M:
    chassis.setY(32);
    if (chassis.lineFollowToPoint(-1.5,32,sensorValues)) {
      chassis.stopAllMotors();           
      state = TURN_TO_25_ROOF_1;
      chassis.setAngle(-90);
      chassis.setAngle(-32);
    }
    break;
  case TURN_TO_25_ROOF_1:
    armTarget = target25pickup;
    if (chassis.turnToAngle(175)) {
      chassis.stopAllMotors();
      state = DRIVE_TO_25_ROOF_AND_RAISE_ARM;
      chassis.setX(0);
      chassis.setAngle(-180);
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
    chassis.setX(0);
    if (chassis.lineFollowToPoint(0,27.75,sensorValues)){
      chassis.stopAllMotors();           
      state = CLOSE_JAW_25_START;
      chassis.setX(0);
    }
    break;
  case CLOSE_JAW_25_START:
    servo.Write(1900);
    if (waitIterations > 20) {
      state = OPEN_JAW_45_END;
      state = LIFT_PLATE_25;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case LIFT_PLATE_25:
    armTarget = target25pickup -200;
    state = BACK_UP_25;
    chassis.setAngle(179.0);
    break;
  case BACK_UP_25:
    if (chassis.moveToPoint(0,35.25)){
      chassis.stopAllMotors();           
      state = TURN_TO_PLATFORM_25;
    }
    break;
  case TURN_TO_PLATFORM_25:
    if (chassis.turnToAngle(90)){
      chassis.stopAllMotors();           
      state =  GO_TO_PLATFORM_25;
      chassis.setY(32);
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
    chassis.setY(32);
    if (chassis.lineFollowToPoint(8.75,32,sensorValues)){
      chassis.stopAllMotors();
      state = LOWER_ARM_25;
    }
    break;
  case STRAIGHTEN_4:
    if (chassis.turnToAngle(86)){
      chassis.stopAllMotors();
      state = LOWER_ARM_25;
    }
    break;
  case LOWER_ARM_25:
    armTarget = targetStagingPlatform;
    if (waitIterations > 2000) {
      state = OPEN_JAW_25_PLATFORM;
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case OPEN_JAW_25_PLATFORM:
    if (waitIterations > 1500) {
      state = CLOSE_JAW_25_PLATFORM;
      waitIterations = 0;
    }
    else {
      servo.Write(1300);
      waitIterations ++;
    }
    break;
  case CLOSE_JAW_25_PLATFORM:
    servo.Write(1900);
     if (waitIterations > 1000) {
      state = RAISE_ARM_25_PLATFORM;  
      waitIterations = 0;
    }
    else {
      waitIterations ++;
    }
    break;
  case RAISE_ARM_25_PLATFORM:
    armTarget = target45dropoff;
    chassis.setY(32);
    chassis.setAngle(90);
    state = MOVE_AWAY_FROM_PLATFORM_25;
    break;
  case MOVE_AWAY_FROM_PLATFORM_25:
    if (chassis.moveToPoint(1.25,25.25)){
      chassis.stopAllMotors();           
      state = TURN_TO_25_ROOF_2;
    }
    break;
  case TURN_TO_25_ROOF_2:
    armTarget = target25dropoff;
    if (chassis.turnToAngle(178)){
      chassis.stopAllMotors();
      state = DRIVE_TO_ROOF_AND_RAISE_PLATE_25;
    }
    break;
  case CORRECTION_6:
    distance = rangeFinder.getDistanceCM()/2.54;
    distanceError = distance - 12;
    chassis.setY(chassis.getY()-distanceError);
    state = DRIVE_TO_ROOF_AND_RAISE_PLATE_25;
    break;
  case DRIVE_TO_ROOF_AND_RAISE_PLATE_25:
    chassis.setX(0);
    if (chassis.lineFollowToPoint(0,18.25,sensorValues)){
      chassis.stopAllMotors();           
      state = DROP_PLATE_25;
    }
    break;
  case DROP_PLATE_25:
    servo.Write(1300);
    state = OPEN_JAW_FINAL;
    break;
  case OPEN_JAW_FINAL:
    armTarget = target25dropoff+80;
    state = BACK_UP_FINAL;
    break;
  case BACK_UP_FINAL:
    if (chassis.moveToPoint(0,33)){
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

//servo code
// void loop()
// {
// //  for (int us = 1500; us < 2000; us += 50)
// //  {
//  servo.Write(1300);
// //  if (us % 100 == 0)
// //  {
//  int value = analogRead(18);
//  Serial.print(2000);
//  Serial.print(" ");
//  Serial.println(value);
//  //}
//  delay(4000);
//  //}
// }


bool buttonPressed = false;
void loop(){
  // put your main code here, to run repeatedly:
  // put your main code here, to run repeatedly:
 // rangeFinder.loop();
  //doStateMachine();
  // Serial.println("\n\nX:\tY:\tAngle:\n");
  // Serial.print(chassis.getX());
  // Serial.print("\t");
  // Serial.print(chassis.getY());
  // Serial.print("\t");
  //  Serial.print(chassis.getAngleDegrees());

  checkRemote();
  qtr.read(sensorValues);
  if (pb.isPressed()) {
    buttonPressed = true;
    delay(800);
  }
  if (buttonPressed) {
    //servo.Write(1100);
    //Serial.println(sensorValues[0]);
    doStateMachine();
    arm.turnToPosition(armTarget);

    // if (chassis.turnToAngle(90)) {
    // //if (chassis.moveToPoint(10,21.5)) {
    //   //state = TURN_TO_PLATFORM_45;
    //   chassis.stopAllMotors();
    //   buttonPressed = false;
    // }
  }
    Serial.println(arm.getPositionDegrees());

  chassis.updatePosition();
 // delay(5);
}

//line follower code
//tests the values

// QTRSensors qtr;

// const uint8_t SensorCount = 2;
// uint16_t sensorValues[SensorCount];

// void setup()
// {
//   // configure the sensors
//   qtr.setTypeAnalog();
//   qtr.setSensorPins((const uint8_t[]){A2, A3}, SensorCount);


//   Serial.begin(9600);
// }


// void loop()
// {
//   // read raw sensor values
//   qtr.read(sensorValues);

//   // print the sensor values as numbers from 0 to 1023, where 0 means maximum
//   // reflectance and 1023 means minimum reflectance
//   for (uint8_t i = 0; i < SensorCount; i++)
//   {
//     Serial.print(sensorValues[i]);
//     Serial.print('\t');
//   }
//   Serial.println();

//   delay(250);
// }
