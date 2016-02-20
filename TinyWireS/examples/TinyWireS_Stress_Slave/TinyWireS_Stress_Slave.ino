// ---------------------------------
// Stress test program/example for TinyWireS I2C library.
// Run this slave program on the Attiny.
// Run the other master program on the Arduino Uno R3.
// ---------------------------------
// // Written by Scott Hartog, 2/6/2016, to stress test the TinyWireS library.
// https://github.com/rambo/TinyWire
//
// This project uses the Tiny85 as an I2C slave.
//
// The slave program using TinyWireS, running on a Attiny85, receives
// N bytes of random data in a single receiveEvent() callback and
// stores that data in a global buffer. It then responds the first requestEvent()
// callback with that same data. The requestEvent() callback overwrites the data
// buffer with zeros after responding so it will only respond correctly to the
// first requestEvent() callback after each receiveEvent() callback. Subsequent
// requestEvent() will respond with 0xff for all data bytes.
//
//
// SETUP:
// AtTiny Pin 5 (PB0/SDA) = I2C SDA 
//     connect to SDA on master with external pull-up (~4.7K)
// AtTiny Pin 7 (PB0/SCL) = I2C SCL 
//     connect to SCL on master with external pull-up (~4.7K)
// AtTiny Pin 1 (PB5/!RST)
//     connect to reset on master (or just pull-up)
//
// Please see credits and usage for usiTwiSlave and TinyWireS in the .h files of 
// those libraries.

#include <avr/sleep.h>
#include <avr/wdt.h>
#include "TinyWireS.h"                  // wrapper class for I2C slave routines

#define I2C_SLAVE_ADDR  0x26            // i2c slave address (38, 0x26)

// turns on code that makes the Tiny85 sleep between transactions
// This is optional. The Tiny85 current drops from
// about 2mA to about 20uA when the CPU is put into
// PowerDown sleep mode.
#define USE_CPU_SLEEP

// global buffer to store data sent from the master.
uint8_t master_data[16];
// global variable to number of bytes sent from the master.
uint8_t master_bytes;

// Gets called when the ATtiny receives an i2c write slave request
// This routine runs from the usiTwiSlave interrupt service routine (ISR)
// so interrupts are disabled while it runs.
void receiveEvent(uint8_t num_bytes)
{
  uint8_t i;

  // save the number of bytes sent from the master
  master_bytes = num_bytes;

  // store the data from the master into the data buffer
  for (i = 0; i < master_bytes; i++)
    master_data[i] = TinyWireS.receive();
    
}

// Gets called when the ATtiny receives an i2c read slave request
// This routine runs from the usiTwiSlave interrupt service routine (ISR)
// so interrupts are disabled while it runs.
void requestEvent()
{
  uint8_t i;
  
  // send the data buffer back to the master
  for (i = 0; i < master_bytes; i++)
    TinyWireS.send(master_data[i]);

  // corrupt the byte values in the data buffer
  // so that any subsequent call won't match
  for (i = 0; i < master_bytes; i++)
    master_data[i] += 0x5a;

  // corrupt length of the request, but don't make it zero
  
  // if the usiTwiSlave.c is working fine, then this number is completely irrelevant
  // because the requestEvent() callback will not be called again until
  // after the next receiveEvent() callback, so the master_data and
  // master_bytes variables will be overwritten by that call.

  // If the usiTwiSlave.c has the issue of calling the requestFrom() callback
  // for each byte sent, the buffer will accumulate by this amount *for each byte
  // in the original request*. (This problem is fixed in the recent version.)
  // 
  // Making it zero will obscure the 1-byte send issue in the usiTwiSlave.c
  // that is being tested.
  // Making it small will allow a few requests to succeed before the tx buffer
  // overflows and the usiTwiSlave.c hangs on the "while ( tmphead == txTail );"
  // line
  master_bytes = 2; 
}

void setup()
{
  //pinMode(1,OUTPUT);   // This pin can be used for rudimentary debug
  
  // initialize the TinyWireS and usiTwiSlave libraries
  TinyWireS.begin(I2C_SLAVE_ADDR);      // init I2C Slave mode

  // register the onReceive() callback function
  TinyWireS.onReceive(receiveEvent);
  
  // register the onRequest() callback function
  TinyWireS.onRequest(requestEvent);

 // disable the watchdog timer so that it doesn't
  // cause power-up, code is from datasheet
  // Clear WDRF in MCUSR – MCU Status Register
  // MCUSR provides information on which reset source caused an MCU Reset.
  MCUSR = 0x00;
  // WDTCR - Watchdog Timer Control Register
  // Write logical one to WDCE and WDE (must be done before disabling)
  WDTCR |= ( _BV(WDCE) | _BV(WDE) );
  // Turn off WDT
  WDTCR = 0x00;

#ifdef USE_CPU_SLEEP
  // enable power down sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // sleep mode
  sleep_enable();
#endif

  sei();                                // enable interrupts

}

void loop()
{

#ifdef USE_CPU_SLEEP
  // optionally put the CPU to sleep. It will be woken by a USI interrupt
  // when it sees a "start condition" on the I2C bus. Everything interesting
  // happens in the usiTwiSlave ISR.
  sleep_cpu();
#endif

}
