// ---------------------------------
// Stress test program/example for TinyWireS I2C library.
// Run this slave program on the Attiny.
// Run the other master program on the Arduino Uno R3.
// ---------------------------------
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

#include "TinyWireS_orig.h"                  // wrapper class for I2C slave routines

#define I2C_SLAVE_ADDR  0x26            // i2c slave address (38, 0x26)

// global buffer to store data sent from the master.
uint8_t master_data[16];
// global variable to number of bytes sent from the master.
uint8_t master_bytes;

// Gets called when the ATtiny receives an i2c write slave request
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
void requestEvent()
{
  uint8_t i;
  
  // send the data buffer back to the master
  for (i = 0; i < master_bytes; i++)
    TinyWireS.send(master_data[i]);

  // corrupt the byte values in the data buffer
  // so that subsequent call won't match
  for (i = 0; i < master_bytes; i++)
    master_data[i] += 0x5a;
  master_bytes = 5;  // corrupt length, but dont' make it zero
                     // Making it zero will obscure the 1-byte
                     // send issue in the library.
}

void setup()
{
  // initialize the TinyWireS and usiTwiSlave libraries
  TinyWireS.begin(I2C_SLAVE_ADDR);      // init I2C Slave mode

  // register the onReceive() callback function
  TinyWireS.onReceive(receiveEvent);
  
  // register the onRequest() callback function
  TinyWireS.onRequest(requestEvent);

  // blink
  pinMode(1,OUTPUT);
  pinMode(3,OUTPUT);
  digitalWrite(1,HIGH);
  delay(200);
  digitalWrite(1,LOW);
  delay(200);
  digitalWrite(3,HIGH);
  delay(200);
  digitalWrite(3,LOW);
  delay(200);
}

void loop()
{
  // This needs to be here
  TinyWireS_stop_check();  
  // otherwise empty loop
}

