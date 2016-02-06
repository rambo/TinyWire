// ---------------------------------
// Stress test program/example for TinyWireS I2C library.
// Run this master program on the Arduino Uno R3.
// Run the other slave program on the Attiny.
// ---------------------------------
// Written by Scott Hartog, 2/6/2016
// This is the I2C master program which runs on on a regular Arduino
// (not a AtTiny). This program uses the regular Wire library from the Arduino IDE.
//
// It performs these steps in a loop:
//    1. picks a random number of bytes between 1 and 12
//    2. sends that many bytes of random data to the AtTiny slave within
//       a single Wire.beginTransmission() / Wire.write() / Wire.endTransmission() set
//    3. reads that same number of bytes back with a single Wire.requestFrom() call
//    4. compares the received data to the originally transmitted data
//    5. displays the number of requests, number of requests with mismatches,
//       and enough of the data so that the operator can tell it's working.
//
#include <Wire.h>

// BREADBOARD SETUP:
// Arduino Uno R3 (D18/SDA) = I2C SDA 
//     connect to SDA on slave with external pull-up (~4.7K)
// Arduino Uno R3 (D19/SCL) = I2C SCL 
//     connect to SCL on slave with external pull-up (~4.7K)
// Arduino Uno R3 (D2)
//     connect to !RST on slave
//     Can alternatively connect !RST on slave to the Ardiuno "!RESET" pin

#define I2C_SLAVE_ADDR  0x26            // i2c slave address (38, 0x26)

#define SLAVE_RESET_PIN 2

uint16_t count = 0;       // total number of passes so far
uint16_t error_count = 0; // total errors encountered so far

char c_buf[64]; // for creating messages

void setup()
{
  // set pin modes 
  pinMode(SLAVE_RESET_PIN,OUTPUT);

  // init the serial port
  Serial.begin(115200);

  // print some useful pinnout info for the Arduino
  //Serial.println(String("SCL:")+String(SCL)+String(", SDA:")+String(SDA));
  //Serial.println(String("MOSI:")+String(MOSI)+String(", SCK:")+String(SCK));

  // init the Wire object (for I2C)
  Wire.begin(); 
  
  // reset the slave
  digitalWrite(SLAVE_RESET_PIN, LOW);
  delay(10);
  digitalWrite(SLAVE_RESET_PIN, HIGH);
  
  // wait for slave to finish any init sequence
  delay(2000);
}

void loop()
{
  uint8_t i;
  uint8_t req_rtn;       // num bytes returned by requestFrom() call
  uint8_t rand_byte_count;
  uint8_t out_rand[16];  // data written from master
  uint8_t in_rand[16];   // data read back from slave

  bool mismatch;

  // count total number of request
  count++;

  // compute random number of bytes for this pass
  rand_byte_count = random(12) + 1;

  // force the first three requests to be small so that the tx buffer doesn't overflow
  // instantly and the user can see at least one successful transaction and some
  // mismtaches before the usiTwiSlave.c library hangs on the line "while ( tmphead == txTail );".
  if (count <= 3) rand_byte_count = 2;

  // generate, save, and send N random byte values
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  for (i = 0; i < rand_byte_count; i++)
    Wire.write(out_rand[i] = random(256));
  Wire.endTransmission();
  
  //delay (10);  // optional delay if required by slave (like sample ADC)

  // read N bytes from slave
  req_rtn = Wire.requestFrom(I2C_SLAVE_ADDR, (int)rand_byte_count);      // Request N bytes from slave
  for (i = 0; i < req_rtn; i++)
    in_rand[i] = Wire.read();

  // compare in/out data values
  mismatch = false;
  for (i = 0; i < rand_byte_count; i++)
    mismatch = mismatch || (out_rand[i] != in_rand[i]);

  // increment the error counter if the number of byte variables don't match or
  // if the data itself doesn't match
  if (mismatch || (rand_byte_count != req_rtn)) error_count++;

  // The rest of the program just displays the results
  
  // display total requests so far and error count so far
  snprintf(c_buf, sizeof(c_buf), "req: %3d,err: %3d", count, error_count);
  Serial.println(c_buf);

  // display the random byte count, the number of bytes read back, and "MATCH"/"MISMATCH"
  snprintf(c_buf, sizeof(c_buf), "size: %2d/%2d,%s", rand_byte_count, req_rtn, rand_byte_count != req_rtn?"MISMATCH  <<--- !!!":"MATCH");
  Serial.println(c_buf);
  
  // display whether the data compare matched or mismatched
  snprintf(c_buf, sizeof(c_buf), "data: %s", mismatch?"MISMATCH  <<--- !!!":"MATCH");
  Serial.println(c_buf);

  // send up to three tx/rx bytes so that random data can be
  // visually verified
  if (rand_byte_count >= 1)
  {
    snprintf(c_buf, sizeof(c_buf), "rand[0]: %02x/%02x", out_rand[0], in_rand[0]);
    Serial.println(c_buf);
  }

  if (rand_byte_count >= 2)
  {
    snprintf(c_buf, sizeof(c_buf), "rand[1]: %02x/%02x", out_rand[1], in_rand[1]);
    Serial.println(c_buf);
  }

  if (rand_byte_count >= 3)
  {
    snprintf(c_buf, sizeof(c_buf), "rand[2]: %02x/%02x", out_rand[2], in_rand[2]);
    Serial.println(c_buf);
  }

  // delay 1 second so user can watch results
  delay(1000);
}
