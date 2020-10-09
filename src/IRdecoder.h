#pragma once
#include <Arduino.h>
#include <Romi32u4.h>
/*
 * A class to interpret IR remotes with NEC encoding. NEC encoding sends four bytes:
 * 
 * [device ID, ~divice ID, key code, ~key code]
 * 
 * Sending the inverse allow for easy error checking (and reduces saturation in the receiver).
 * 
 * Codes are send in little endian; this library reverses upon reception, so the first bit received
 * is in the LSB of currCode. That means that the key code is found in bits [23..16] of currCode
 * 
 * https://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol
 * 
 * This does not interpret the codes into which key was pressed. That needs to be 
 * mapped on a remote by remote basis.
 */

class IRDecoder
{
private:
  enum IR_STATE
  {
    IR_READY,    //idle, returns to this state after you request a code
    IR_PREAMBLE, //received the start burst, waiting for first bit
    IR_REPEAT,   //received repeat code (part of NEC protocol); last code will be returned
    IR_ACTIVE,   //have some bits, but not yet complete
    IR_COMPLETE, //a valid code has been received
    IR_ERROR
  }; //an error occurred; won't return a valid code

  IR_STATE state = IR_READY; //a simple state machine for managing reception

  volatile uint32_t lastReceiveTime = 0; //not really used -- could be used to sunset codes

  volatile uint32_t currCode = 0; //the most recently received valid code
  volatile uint8_t index = 0;     //for tracking which bit we're on

  volatile uint32_t fallingEdge = 0;
  volatile uint32_t risingEdge = 0;

  volatile uint32_t lastRisingEdge = 0; //used for tracking spacing between rising edges, i.e., bit value
public:
  //volatile uint16_t bits[32];  //I used this for debugging; obsolete

public:
  void init(void);           //call this in the setup()
  void handleIRsensor(void); //ISR

  uint32_t getCode(void) //returns the most recent valid code; returns zero if there was an error
  {
    if (state == IR_COMPLETE || state == IR_REPEAT)
    {
      state = IR_READY;
      return currCode;
    }
    else
      return -1;
  }

  int16_t getKeyCode(void) //returns the most recent key code; returns 0 on error (not sure if 0 can be a code or not!!!)
  {
    if (state == IR_COMPLETE || state == IR_REPEAT)
    {
      state = IR_READY;
      return (uint8_t)(currCode >> 16);
    }
    else
      return -1;
  }
};

extern IRDecoder decoder;
