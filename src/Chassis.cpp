
#include <Chassis.h>

void Chassis::initialize() {
  drivePowerPID.pidInit(70,0,0);
  turnPowerPID.pidInit(140,0,0);
  turnPID.pidInit(1.8,0,0);
}

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
  float encoderClicks = (CPR * inches) / (wheelDiameter * M_PI);
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

float Chassis::getLeftEncoder() {
  return encoders.getCountsLeft()/CPR;
}

float Chassis::getRightEncoder() {
  return encoders.getCountsRight()/CPR;
}

void Chassis::updatePosition() {
  curLeft = encoders.getCountsLeft()/CPR;
  curRight = encoders.getCountsRight()/CPR; //step 1

  deltaLeft = (curLeft - lastLeftPos)*(2*M_PI)*(wheelDiameter/2);
  deltaRight = (curRight - lastRightPos)*(2*M_PI)*(wheelDiameter/2); //step 2

  lastLeftPos = curLeft;
  lastRightPos = curRight; //step 3

  deltaLr = (curLeft - leftAtReset)*(2*M_PI)*(wheelDiameter/2); //step 4
  deltaRr = (curRight - rightAtReset)*(2*M_PI)*(wheelDiameter/2);

  thetaNew = (thetaReset + (deltaLr - deltaRr)/(Sl + Sr)); //step 5

  deltaTheta = thetaNew - angle; //step 6

  if (deltaTheta == 0) {
    deltaY = deltaRight;
  }

  else {
    deltaX = 0; //step 8
    deltaY = (2*sin(deltaTheta/2))*(deltaRight/deltaTheta +Sr);
  }

  thetaM = angle + deltaTheta/2; //step 9

  theta = atan2f(deltaY, deltaX);
  radius = sqrt(deltaX*deltaX + deltaY*deltaY);
  theta = theta-thetaM;                          //step 10
  deltaX = radius*cos(theta);
  deltaY = radius*sin(theta);

  angle = thetaNew;
  x = x + deltaX; //step 11
  y = y + deltaY;
}

float Chassis::getX() {
  return x;
}

float Chassis::getY() {
  return y;
}

float Chassis::getAngleDegrees() {
  return getAngle()*180/M_PI;
}
float Chassis::getAngle() {
  return (modulus(angle+M_PI, 2*M_PI))-M_PI;
  //return angle;
}
float Chassis::modulus(float a, float b) {
  while(a<0) {
    a+=b;
  }
  while(a>b) {
    a-=b;
  }
  return a;
}
bool Chassis::turnToAngle(float targetAngle) {
  bool atPoint = false;

  float referenceAngle = 0;
  if (targetAngle-getAngleDegrees()>180) {
    referenceAngle = getAngleDegrees()+360;
  }
  else if (targetAngle-getAngleDegrees()<-180) {
    referenceAngle = getAngleDegrees()-360;
  }
  else {
    referenceAngle = getAngleDegrees();
  }
  float turnPower = turnPID.pidCalculate(targetAngle, referenceAngle);
  
  motors.setEfforts(turnPower,-turnPower);

  if (abs(getAngleDegrees()-targetAngle)<1.5) {
    atPoint = true;
  }
  return atPoint;
}

bool Chassis::moveToPoint(float targetX, float targetY) {
	bool atPoint = false;
	float targetAngle =0;
	float power =0;
	float turnPower =0;
  if (millis()-lastSlewTime>10){
    lastSlewTime = millis()-5;
  }
  power = -drivePowerPID.pidCalculate(0, sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)));
  power = slewRateCalculate(power);
  //Serial.println(power);
 // if (targetY-getY())
  targetAngle = -1*(atan2f((targetY-getY()),(targetX-getX()))-M_PI/2);

  Serial.print(targetAngle*180/M_PI);

  float projection = ((targetX-getX())*sin(getAngle())+(targetY-getY())*cos(getAngle()));

  if (projection < 0) {
    power = -power;
    targetAngle = modulus((targetAngle+M_PI)+M_PI, 2*M_PI)-M_PI;
  }

  float referenceAngle = 0;
  if (targetAngle-getAngle()>M_PI) {
    referenceAngle = getAngle()+(2*M_PI);
  }
  else if (targetAngle-getAngle()<-M_PI) {
    referenceAngle = getAngle()-(2*M_PI);
  }
  else {

     referenceAngle = getAngle();
   }
  turnPower = turnPowerPID.pidCalculate(targetAngle, referenceAngle);
  Serial.println("\nTarget Angle: ");

  // Serial.println("\nAngle");
  // Serial.print(getAngleDegrees());
  // Serial.println("\nTurnPower");
  // Serial.print(turnPower);

 // power = 0;
  motors.setEfforts((power + turnPower)*0.35, (power - turnPower)*0.35);

  if (sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)) < 0.5) {
    atPoint = true;
  }
  return atPoint;
}

float Chassis::slewRateCalculate (float desiredRate) {
		float deltaTime = millis()-lastSlewTime;
		float desiredAccel = (desiredRate -lastSlewRate)/deltaTime;
		float addedRate;
		float newRate;

		if (fabs(desiredAccel) < maxAccel || (desiredAccel<0 && desiredRate>0) || (desiredAccel>0 && desiredRate<0)) {
		    addedRate = desiredAccel*deltaTime;
		    newRate = addedRate+lastSlewRate;
		}
		else {
		    addedRate = ((desiredAccel>0)? 1: -1)*maxAccel*deltaTime;
        newRate = addedRate+lastSlewRate;
		}
	  lastSlewTime = lastSlewTime+deltaTime;
	  lastSlewRate = newRate;

		float returnVal = newRate;
		return returnVal;
}

void Chassis::stopAllMotors(){
  motors.setEfforts(0,0);
}