
#include <Chassis.h>

void Chassis::initialize() {
  drivePowerPID.pidInit(150,0,1400);
  turnPowerPID.pidInit(120,0,0);
    turnPID.pidInit2(14,0,420,5);

  reversed = true;
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
  x = x + ((reversed)? -1: 1)*deltaX; //step 11
  y = y + ((reversed)? -1: 1)*deltaY;
}

float Chassis::getX() {
  return x;
}

float Chassis::getY() {
  return y;
}
void Chassis::setX(float newX) {
  x = newX;
}
void Chassis::setY(float newY) {
  y = newY;
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
  if (repsAtTarget == -1) {
    startTime = millis();
  }

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
  if (fabs(turnPower)>160) {
    turnPower = ((turnPower>0)? 1: -1) * 160;
  }
  
  motors.setEfforts(turnPower*0.5,-turnPower*0.5);

  if (abs(getAngleDegrees()-targetAngle)<0.5) {
    repsAtTarget++;
  }
  else {
    repsAtTarget = 0;
  }
  if (repsAtTarget > 10 || millis()-startTime>2000) {
    atPoint = true;
    repsAtTarget = -1;
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
  power = ((power<0)? -1: 1)*(abs(power)>200)? 200: abs(power);
  power = ((reversed)? -1: 1)*power;
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
  motors.setEfforts((power + turnPower)*0.65, (power - turnPower)*0.65);

  if (sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)) < 1) {
    repsAtTarget++;
  }
  else {
    repsAtTarget = 0;
  }
  if (repsAtTarget > 2) {
    atPoint = true;
    repsAtTarget = -1;
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

bool Chassis::lineFollowToPoint(float targetX, float targetY, uint16_t SensorValues[]) {
  bool atPoint = false;
	float power =0;
	float turnPower =0;
  if (millis()-lastSlewTime>10){
    lastSlewTime = millis()-5;
  }
  power = -drivePowerPID.pidCalculate(0, sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)));
  power = slewRateCalculate(power);
  power = ((power<0)? -1: 1)*(abs(power)>200)? 200: abs(power);
  power = ((reversed)? -1: 1)*power;

  float projection = ((targetX-getX())*sin(getAngle())+(targetY-getY())*cos(getAngle()));

  if (projection < 0) {
    power = -power;
  }

 // power = -150;

  int leftSensor = SensorValues[1];
  int rightSensor = SensorValues[0];

  turnPower = 0.12*(leftSensor-rightSensor);

  motors.setEfforts((power + turnPower)*0.65, (power - turnPower)*0.65);

  if (sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)) < 0.7) {
    repsAtTarget++;
  }
  else {
    repsAtTarget = 0;
  }
  if (repsAtTarget > 3) {
    atPoint = true;
    repsAtTarget = -1;
  }
  return atPoint;
}
void Chassis::stopAllMotors(){
  motors.setEfforts(0,0);
}

void Chassis::setAngle(float newAngle) {
  angle = newAngle;
}