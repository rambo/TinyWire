/*
Read an analog value on pin A3 (sensor pin)

Listens on the i2c bus (slave mode) at address I2C_SLAVE_ADDR

When a request is received on the i2c bus a address 1, the analog value is sent back.
request at address 2 turns on the led at ledPin
request at address 3 turns off the led

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
int8_t nextIndex=0; //to scan the byte values in sensor_t
uint8_t bug=0;

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
  if (reg_number==1 && TinyWireS.sendBatchNow()){
    TinyWireS.send(sensorValue.b[0]); // respond with message of 2 bytes, one at a time
    TinyWireS.send(sensorValue.b[1]); // respond with message of 2 bytes, one at a time
  }
}


void receiveEvent(uint8_t howMany)
{
   if (howMany > 100) // Also insane number
      return;
   for (uint8_t i=0;i<howMany;i++)
      reg_number = TinyWireS.receive();
}


void loop() {
  // read the value from the sensor:
  sensorValue.v = analogRead(sensorPin);    
  
  tws_delay(100);
  if (reg_number==1) 
    nextIndex=0;
  if (reg_number==2) 
    digitalWrite(ledPin,HIGH);
  if (reg_number==3) 
    digitalWrite(ledPin,LOW);

}

void delayus(uint16_t us)
{
#if F_CPU >= 16000000L
    // for the 16 MHz clock on most Arduino boards

    // for a one-microsecond delay, simply return.  the overhead
    // of the function call yields a delay of approximately 1 1/8 us.
    if (--us == 0)
        return;

    // the following loop takes a quarter of a microsecond (4 cycles)
    // per iteration, so execute it four times for each microsecond of
    // delay requested.
    us <<= 2;

    // account for the time taken in the preceeding commands.
    us -= 2;
#else
    // for the 8 MHz internal clock on the ATmega168

    // for a one- or two-microsecond delay, simply return.  the overhead of
    // the function calls takes more than two microseconds.  can't just
    // subtract two, since us is unsigned; we'd overflow.
    if (--us == 0)
        return;
    if (--us == 0)
        return;

    // the following loop takes half of a microsecond (4 cycles)
    // per iteration, so execute it twice for each microsecond of
    // delay requested.
    us <<= 1;

    // partially compensate for the time taken by the preceeding commands.
    // we can't subtract any more than this or we'd overflow w/ small delays.
    us--;
#endif

    // busy wait
    __asm__ __volatile__ (
        "1: sbiw %0,1" "\n\t" // 2 cycles
        "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
    );
}//end delayMicroseconds



void delayms(uint16_t milliseconds)
{
    for(uint16_t i = 0; i < milliseconds; i++)
    {
        delayus(1000);
    }
}//end delay

