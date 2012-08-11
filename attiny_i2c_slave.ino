/**
 * Pin notes by Suovula (see also http://hlt.media.mit.edu/?p=1229)
 *
// I2C
arduino pin 0 = not(OC1A) = PORTB <- _BV(0) = SOIC pin 5 (I2C SDA, PWM)
arduino pin 2 =           = PORTB <- _BV(2) = SOIC pin 7 (I2C SCL, Analog 1)
// Timer1 -> PWM
arduino pin 1 =     OC1A  = PORTB <- _BV(1) = SOIC pin 6 (PWM)
arduino pin 3 = not(OC1B) = PORTB <- _BV(3) = SOIC pin 2 (Analog 3)
arduino pin 4 =     OC1B  = PORTB <- _BV(4) = SOIC pin 3 (Analog 2)
 */
#define I2C_SLAVE_ADDRESS 0x4 // the 7-bit address (remember to change this)
// Get this from https://github.com/rambo/TinyWire
#include <TinyWireS.h>
// The default buffer size, Can't recall the scope of defines right now
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif


volatile uint8_t i2c_regs[] =
{
    10, // pwm
    150, // blink on delay/4
    150, // blink off delay/4
};

void setup()
{
    // TODO: Tri-state this and wait for input voltage to stabilize 
    pinMode(3, OUTPUT); // OC1B-, Arduino pin 3, ADC
    digitalWrite(3, LOW); // Note that this makes the led turn on, it's wire this way to allow for the voltage sensing above.


    pinMode(1, OUTPUT); // OC1A, also The only HW-PWM -pin supported by the tiny core analogWrite

    // This *should* enable pull-ups but it doesn't seem to work
    digitalWrite(0, HIGH);
    digitalWrite(2, HIGH);

    TinyWireS.begin(I2C_SLAVE_ADDRESS);

    
    // Whatever other setup routines ?
    
    digitalWrite(3, HIGH);
}

void receiveEvent(uint8_t howMany)
{
    if (howMany < 1)
    {
        // Sanity-check
        return;
    }
    if (howMany > TWI_RX_BUFFER_SIZE)
    {
        // Also insane number
        return;
    }
    //TinyWireS.send(howMany);
    /*
    if (howMany < 2)
    {
        // We're only interested when we know we can suppose the first byte is register address
        return;
    }

    byte reg_addr = TinyWireS.receive();
    */
    byte reg_addr = 0;
    byte max_reg = reg_addr + howMany;
    
    for (byte i = reg_addr; i < max_reg; i++)
    {
        i2c_regs[i] = TinyWireS.receive();
        /*
        switch (i)
        {
            case 0x0:
            {
            }
        }
        */
    }
}

void loop()
{
    // Poor-mans event handling (tinywire lib does not yet trigger the event right away), though I still wonder if we can still get two triggers during one I2C transaction (which will mess things up)
    uint8_t i2c_available = TinyWireS.available();
    if (i2c_available > 0)
    {
        receiveEvent(i2c_available);
    }
    analogWrite(1, i2c_regs[0]);
    i2c_regs[0] = i2c_regs[0]+10; // See if the loop is still runnign when I2C hangs

    digitalWrite(3, LOW); // Note that this makes the led turn on, it's wire this way to allow for the voltage sensing above.
    delay(i2c_regs[1]*4);
    digitalWrite(3, HIGH);
    delay(i2c_regs[2]*4);
}
