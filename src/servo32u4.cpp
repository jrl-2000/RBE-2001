/*
 * A minimal class to control a servo on pin 5.
 * 
 * It uses output compare on Timer3 to control the pulse. The 16-bit Timer3 is set up (in Init()) 
 * with a pre-scaler of 8, TOP of 39999 + 1 => 20 ms
 * 
 * OCR3A controls the pulse on pin 5 -- THE SERVO MUST BE ON PIN 5! 
 * 
 * Defaults to a range of 1000 - 2000 us, but can be customized.
 */ 

#include <servo32u4.h>

void Servo32U4::Init(void)
{
    cli();
    //set up Timer3 for 20ms rollover
    
    TCCR3A = 0x02; //WGM
    TCCR3B = 0x1A; //WGM + CS = 8
    ICR3 = 39999; //20ms
    OCR3A = 3000; //default to 1500us

    sei();

    pinMode(5, OUTPUT); //don't forget to enable the pin as OUTPUT
}

void Servo32U4::Attach(void) //MUST USE PIN 6
{
    cli();
    TCCR3A = 0x82; //set up OCR3A
    sei();
}

void Servo32U4::Detach(void)
{
    cli();
    TCCR3A = 0x02; //cancel OCR3A
    sei();
}

void Servo32U4::Write(uint16_t microseconds)
{
    microseconds = constrain(microseconds, usMin, usMax);
    //prescaler is 8, so 1 timer count = 0.5 us
    OCR3A = microseconds << 1; //times 2
}

uint16_t Servo32U4::SetMinMaxUS(uint16_t min, uint16_t max)
{
    usMin = min;
    usMax = (max > min) ? max : min; //in case they're mixed up, just constrain to min

    return usMax - usMin; //return the range, in case the user wants to do a sanity check
}
