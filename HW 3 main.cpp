//Jonathan Lopez
//RBE 2001 A01 A Term 2020
//HW 3  Part B Ultrasonic Sensor
// 9/20/2020-9/22/2020
//Ultrasonic Range Finder code
//Takes a distance and the robot turns around when an object is sensed and in that range
//In this case that distance is 20cm


#include <Arduino.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include <time.h>
//#include <chrono>


Chassis romi;
Rangefinder rangefinder;

//setup values and pins
const float targetDistance = 20.0; //20cm
static const int triggerPin = 12;
static const int echoPin = 0;


void setup() {
    // put your setup code here, to run once:
    rangefinder.setup();
    Serial.begin(9600);
}

void loop() {
    // put your main code here, to run repeatedly:
    //try using epoch as the ref time
    //unsigned __int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    //Arduino has a millis() 

    int counter = 0;
    unsigned long startTime;
    if (counter == 0) {
        startTime = millis();
        counter = counter +1;
    }
    
    rangefinder.loop();
    romi.drive(50); // drive with 50 effort

    if (rangefinder.getDistanceCM() < targetDistance){
     delay(50);
     romi.turnAngle(180);
     delay(50); //turn around if sensed a object within 20cm aaway
    }

    //updates with the new time
    unsigned long presentTime = millis();
    unsigned long diffTime = presentTime - startTime;
    
    //checks to see if 100ms has passed and if it has it sends out new ping
    if (diffTime >= 100) { 
        //ping code here
        rangefinder.loop(); //could be aprob with this statement
        startTime = presentTime;
    }
}

//loops back again
