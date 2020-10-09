/*
 * A minimal class to control a servo on pin 6.
 * 
 * It uses output compare on Timer4 to control the pulse. The 10-bit Timer4 is set up in Init()
 * with a pre-scaler of 256, so 16 us resolution for the servo position (which is pretty fine, actually).
 * 
 * Upon request, I could figure out how to have 11-bit accuracy, or 8 us.
 * 
 * OCR4D controls the pulse on pin 6 -- THE SERVO MUST BE ON PIN 6! 
 * 
 * OCR4D takes a 10-bit value, but two MSBs are written using a common register (TC4H). 
 * But since a value of 256 corresponds to 4096 us, the high bits will always be 00.
 * 
 * Defaults to a range of 1000 - 2000 us, but can be customized.
 */ 

#include <Arduino.h>

class Servo32U4
{
private:
    uint16_t usMin = 1000;
    uint16_t usMax = 2000;

    uint8_t feedbackPin = -1;
public:
    static void Init(void);
    static void Attach(void); //MUST USE PIN 6
    void Detach(void);
    void Write(uint16_t microseconds);
    uint16_t SetMinMaxUS(uint16_t min, uint16_t max);
};
