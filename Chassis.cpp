
#include <Chassis.h>
const float pi = 3.1415926535897932384626433;


void Chassis::turnAngle(float degrees)
{
  //Encoder clicks per a certain angle
  float getEncoderAngle = (degrees * CPR * wheelTrack) / (wheelDiameter * 360);
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();

while (encoders.getCountsRight() < getEncoderAngle || encoders.getCountsLeft() > -1 * getEncoderAngle)
{
  motors.setEfforts(-50,  50);
} 
  motors.setEfforts(0, 0);
}


void Chassis::driveDistance(float inches)
{
  //Encoder clicks per certain distance
  float encoderClicks = (CPR * inches) / (wheelDiameter * pi);
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();

  while (encoders.getCountsLeft() <= encoderClicks || encoders.getCountsRight() <= encoderClicks)
  {
    motors.setEfforts(100, 100);
  }
  
  motors.setEfforts(0, 0);
}

void Chassis::drive(float effort)
{
  motors.setEfforts(effort, effort);
}
