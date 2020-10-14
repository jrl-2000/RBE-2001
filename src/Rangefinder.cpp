#include <Rangefinder.h>

static const int triggerPin = 12;
static const int echoPin = 0;
unsigned long duration;
unsigned long startEchoTime;

void Rangefinder::setup()
{
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(echoPin), UltrasonicISR, CHANGE);
}

void Rangefinder::UltrasonicISR()
  {
    if (digitalRead(0) == HIGH)
    startEchoTime = micros();
  
  else if (digitalRead(0) == LOW)  {
    duration =  (micros() - startEchoTime);

  } 
}
  
void Rangefinder::loop()
{
  // trigger the ping
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  this->UltrasonicISR();
  this->getDistanceCM();
  delay(20);

} 

float Rangefinder::getDistanceCM()
{
  // measure the echo
  //long duration = pulseIn(echoPin, HIGH); blocks
  
  float distance = duration * (0.034 / 2); // Speed of sound wave divided by 2
  return distance;
}

float Rangefinder::getAccurateDistance() {
  
}



