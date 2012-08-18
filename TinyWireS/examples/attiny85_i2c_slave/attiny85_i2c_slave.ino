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
    0xDE, 
    0xAD, 
    0xBE, 
    0xEF, 
};


volatile byte reg_position;
void requestEvent()
{  
    TinyWireS.send(i2c_regs[reg_position]);
    // Increment the reg position on each read, and loop back to zero
    reg_position = (reg_position+1) % sizeof(i2c_regs);
}


void blinkn(uint8_t blinks)
{
    digitalWrite(3, HIGH);
    while(blinks--)
    {
        digitalWrite(3, LOW);
        delay(50);
        digitalWrite(3, HIGH);
        delay(100);
    }
}

/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) does 
 */
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

    reg_position = TinyWireS.receive();
    howMany--;
    if (!howMany)
    {
        // This write was only to set the buffer for next read
        return;
    }
    while(howMany--)
    {
        i2c_regs[reg_position%sizeof(i2c_regs)] = TinyWireS.receive();
        reg_position++;
    }
}


void setup()
{
    // TODO: Tri-state this and wait for input voltage to stabilize 
    pinMode(3, OUTPUT); // OC1B-, Arduino pin 3, ADC
    digitalWrite(3, LOW); // Note that this makes the led turn on, it's wire this way to allow for the voltage sensing above.

    pinMode(1, OUTPUT); // OC1A, also The only HW-PWM -pin supported by the tiny core analogWrite

    /**
     * Reminder: taking care of pull-ups is the masters job
     */

    TinyWireS.begin(I2C_SLAVE_ADDRESS);
    TinyWireS.onReceive(receiveEvent);
    TinyWireS.onRequest(requestEvent);

    
    // Whatever other setup routines ?
    
    digitalWrite(3, HIGH);
}

void loop()
{
    /**
     * This is the only way we can detect stop condition (http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=984716&sid=82e9dc7299a8243b86cf7969dd41b5b5#984716)
     * it needs to be called in a very tight loop in order not to miss any.
     * It will call the function registered via TinyWireS.onReceive(); if there is data in the buffer on stop.
     */
    TinyWireS_stop_check();
}
