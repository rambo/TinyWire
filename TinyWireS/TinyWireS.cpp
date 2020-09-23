/*
  TinyWireS.cpp - a wrapper class for Don Blake's usiTwiSlave routines.
  Provides TWI/I2C Slave functionality on ATtiny processers in Arduino environment.
  1/23/2011 BroHogan -  brohoganx10 at gmail dot com

  **** See TinyWireS.h for Credits and Usage information ****

  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or any later version.
  This program is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
  PARTICULAR PURPOSE.  See the GNU General Public License for more details.
*/

extern "C" {
  #include <inttypes.h>
  #include "usiTwiSlave.h"
  #include <avr/interrupt.h>
  }

#include "TinyWireS.h"
#include "Arduino.h"

// Constructors ////////////////////////////////////////////////////////////////

USI_TWI_S::USI_TWI_S(){
}

#define ACTIVE 1
#define IDLE 0

static uint32_t busActiveStart;   // start time when bus becomes active
static bool lastBusState = IDLE;  // 1 = active, 0 = idle
static bool thisBusState = IDLE;  // 1 = active, 0 = idle
static uint16_t i2c_timeout = 25; // i2c timeout, ms
static uint8_t slave_addr;

// Public Methods //////////////////////////////////////////////////////////////

void USI_TWI_S::begin(uint8_t slaveAddr, uint16_t tout){ // initialize I2C lib
  slave_addr = slaveAddr; // save
  if (tout) i2c_timeout = tout;
  usiTwiSlaveInit(slave_addr);
}

void USI_TWI_S::send(uint8_t data){  // send it back to master
  usiTwiTransmitByte(data);
}

uint8_t USI_TWI_S::available(){ // the bytes available that haven't been read yet
  return usiTwiAmountDataInReceiveBuffer();
  //return usiTwiDataInReceiveBuffer(); // This is wrong as far as the Wire API is concerned since it returns boolean and not amount
}

uint8_t USI_TWI_S::receive(){ // returns the bytes received one at a time
  return usiTwiReceiveByte();
}

// sets function called on master write (slave read)
void USI_TWI_S::onReceive( void (*function)(uint8_t) )
{
  usi_onReceiverPtr = function;
}

// sets function called on master read (slave write)
void USI_TWI_S::onRequest( void (*function)(void) )
{
  usi_onRequestPtr = function;
}

void USI_TWI_S::stateCheck()
{
  // check for i2c stop condition, handle if needed
  if (USISR & ( 1 << USIPF )) usiTwiHandleSTOP();

  // --- monitor bus state, reset if frozen ---
  if (i2c_timeout) {
    thisBusState = usiTwiIsBusActive();

    if ( (thisBusState == ACTIVE) && (lastBusState == ACTIVE) &&
         ( (millis() - busActiveStart) > i2c_timeout) ) {

      // timeout, so do a complete reset of i2c slave
      usiTwiSlaveInit(slave_addr);
      thisBusState = IDLE;

    } else if ( (lastBusState == IDLE) && (thisBusState == ACTIVE) ) {
      // idle to active state, so mark start
      busActiveStart = millis();
    }

    lastBusState = thisBusState;    // remember last bus state
  }
}

bool USI_TWI_S::busActive(){
    return usiTwiIsBusActive();
}

bool USI_TWI_S::sleepMode(bool sleepFlag){

  // wait until bus idle, prior to sleep
  if (sleepFlag && TinyWireS.busActive()) {
    busActiveStart = millis();
    while (TinyWireS.busActive()) {
      tws_delay(1);
      if (i2c_timeout &&
          ( (millis() - busActiveStart) > i2c_timeout) ) return false;
    }
  }

  // set/clear flag
  if (!sleepFlag || !TinyWireS.busActive()) {
    usiTwiSetSleep(sleepFlag);
    return true;
  }
}

// Implement a delay loop that checks i2c state
void tws_delay(unsigned long ms)
{
    uint32_t start = millis();
    while (millis() < (start + ms)) TinyWireS.stateCheck();
}

// version of delay loop using microseconds
void tws_delay_micros(unsigned long us)
{
    uint32_t start = micros();
    while (micros() < (start + us)) TinyWireS.stateCheck();
}

// Preinstantiate Objects //////////////////////////////////////////////////////

USI_TWI_S TinyWireS = USI_TWI_S();
