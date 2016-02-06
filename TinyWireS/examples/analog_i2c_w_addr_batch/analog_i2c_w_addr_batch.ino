/*
Read an analog value on pin A3 (sensor pin)

Listens on the i2c bus (slave mode) at address I2C_SLAVE_ADDR

When:
request is received on the i2c bus a address 1
  -> the analog value is sent back.
request at address 2 
  -> turns on the led at ledPin
request at address 3 
  -> turns off the led

This has been tested on the i2c bus of the raspberry pi:
i2cdump -y -r 1-1 1 3 w  # reads analog value
i2cdump -y -r 2-2 1 3 w  # led on
i2cdump -y -r 3-3 1 3 w  # led off
 */

#include <TinyWireS.h>                  // wrapper class for I2C slave routines

#define I2C_SLAVE_ADDR  0x3            // i2c slave address (2)

// pin 5 is SDA
// pini 7 is SCL
void delayms(uint16_t milliseconds);  //forward declaration to the delay function
void delayus(uint16_t microseconds);  //forward declaration to the delay function

uint8_t sensorPin = A3;    // select the input pin for the potentiometer
uint8_t ledPin = 1;      // select the pin for the LED (pin 6, PB1)
typedef union {
  unsigned short v;
  unsigned char b[2];
} sensor_t;
volatile sensor_t sensorValue={0}; // variable to store the value coming from the sensor
volatile byte reg_number;

uint8_t value;

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
  TinyWireS.begin(I2C_SLAVE_ADDR);      // init I2C Slave mode
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent); // register event
}

void requestEvent()
{
  uint8_t val=0;
  if (reg_number==1){
    TinyWireS.send(sensorValue.b[0]); // respond with message of 2 bytes at once
    TinyWireS.send(sensorValue.b[1]); 
  }
}


void receiveEvent(uint8_t howMany)
{
   if (howMany > 16) // 16 is default queue length
      return;
   for (uint8_t i=0;i<howMany;i++)
      reg_number = TinyWireS.receive();
}


void loop() {
  // read the value from the sensor:
  sensorValue.v = analogRead(sensorPin);    
  
  tws_delay(100);
  if (reg_number==2) 
    digitalWrite(ledPin,HIGH);
  if (reg_number==3) 
    digitalWrite(ledPin,LOW);

}

