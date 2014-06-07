/**
 * Example sketch for writing to and reading from a slave in transactional manner
 *
 * NOTE: You must not use delay() or I2C communications will fail, use tws_delay() instead (or preferably some smarter timing system)
 *
 * On write the first byte received is considered the register addres to modify/read
 * On each byte sent or read the register address is incremented (and it will loop back to 0)
 *
 * You can try this with the Arduino I2C REPL sketch at https://github.com/rambo/I2C/blob/master/examples/i2crepl/i2crepl.ino 
 * If you have bus-pirate remember that the older revisions do not like the slave streching the clock, this leads to all sorts of weird behaviour
 * Examples use bus-pirate semantics (like the REPL)
 *
 * The basic idea is:
 *  1. Choose your ADC channel (0-X), use "byte ch = 1;" for example.
 *  2. Combine the channel and conversion start flag to single calue: byte start_on_ch = (ch | _BV(7)); // This is 0x81
 *  3. Write start_on_ch to the first register on the attiny [ 8 0  81 ]
 *  4. Come back later and check the first register [ 8 0 [ r ], if the value is same as ch then the conversion is complete, you can now read the value
 *  5. read the value [ 8 2 [ r r ] (first one is low, second high byte)
 *
 * You need to have at least 8MHz clock on the ATTiny for this to work (and in fact I have so far tested it only on ATTiny85 @8MHz using internal oscillator)
 * Remember to "Burn bootloader" to make sure your chip is in correct mode 
 */


/**
 * Pin notes by Suovula, see also http://hlt.media.mit.edu/?p=1229
 *
 * DIP and SOIC have same pinout, however the SOIC chips are much cheaper, especially if you buy more than 5 at a time
 * For nice breakout boards see https://github.com/rambo/attiny_boards
 *
 * Basically the arduino pin numbers map directly to the PORTB bit numbers.
 *
// I2C
arduino pin 0 = not(OC1A) = PORTB <- _BV(0) = SOIC pin 5 (I2C SDA, PWM)
arduino pin 2 =           = PORTB <- _BV(2) = SOIC pin 7 (I2C SCL, Analog 1)
// Timer1 -> PWM
arduino pin 1 =     OC1A  = PORTB <- _BV(1) = SOIC pin 6 (PWM)
arduino pin 3 = not(OC1B) = PORTB <- _BV(3) = SOIC pin 2 (Analog 3)
arduino pin 4 =     OC1B  = PORTB <- _BV(4) = SOIC pin 3 (Analog 2)
 */
#define I2C_SLAVE_ADDRESS 0x4 // the 7-bit address (remember to change this when adapting this example)
// Get this from https://github.com/rambo/TinyWire
#include <TinyWireS.h>
// The default buffer size, though we cannot actually affect it by defining it in the sketch
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif
// For the ADC_xxx helpers
#include <core_adc.h>

// The "registers" we expose to I2C
volatile uint8_t i2c_regs[] =
{
    0x0, // Status register, writing (1<<7 & channel) will start a conversion on that channel, the flag will be set low when conversion is done.
    0x1, // Averaging count, make this many conversions in row and average the result (well, actually it's a rolling average since we do not want to have the possibility of integer overflows)
    0x0, // low byte 
    0x0, // high byte
};
const byte reg_size = sizeof(i2c_regs);
// Tracks the current register pointer position
volatile byte reg_position;
// Tracks wheter to start a conversion cycle
volatile boolean start_conversion;
// Counter to track where we are averaging
byte avg_count;
// Some temp value holders
int avg_temp1;
int avg_temp2;

/**
 * This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the 
 * send-buffer when using this callback
 */
void requestEvent()
{  
    TinyWireS.send(i2c_regs[reg_position]);
    // Increment the reg position on each read, and loop back to zero
    reg_position++;
    if (reg_position >= reg_size)
    {
        reg_position = 0;
    }
}

/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
 * so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,
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
        i2c_regs[reg_position] = TinyWireS.receive();
        if (   reg_position == 0 // If it was the first register
            && bitRead(i2c_regs[0], 7) // And the highest bit is set
            && !ADC_ConversionInProgress() // and we do not actually have a conversion running already
            )
        {
            start_conversion = true;
        }
        reg_position++;
        if (reg_position >= reg_size)
        {
            reg_position = 0;
        }
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
     * it needs to be called in a very tight loop in order not to miss any (REMINDER: Do *not* use delay() anywhere, use tws_delay() instead).
     * It will call the function registered via TinyWireS.onReceive(); if there is data in the buffer on stop.
     */
    TinyWireS_stop_check();

    // Thus stuff is basically copied from wiring_analog.c
    if (start_conversion)
    {
        //Avoid doubled starts
        start_conversion = false;
        byte adcpin = (i2c_regs[0] & 0x7f); // Set the channel from the control reg, dropping the highest bit.
#if defined( CORE_ANALOG_FIRST )
        if ( adcpin >= CORE_ANALOG_FIRST ) adcpin -= CORE_ANALOG_FIRST; // allow for channel or pin numbers
#endif
        // NOTE: These handy helpers (ADC_xxx) are only present in the tiny-core, for other cores you need to check their wiring_analog.c source.
        ADC_SetInputChannel( (adc_ic_t)adcpin ); // we need to typecast
        ADC_StartConversion();
        // Reset these variables
        avg_count = 0;
        avg_temp2 = 0;
    }
    
    if (   bitRead(i2c_regs[0], 7) // We have conversion flag up
        && !ADC_ConversionInProgress()) // But the conversion is complete
    {
        // So handle it
        avg_temp1 = ADC_GetDataRegister();
        // Rolling average
        if (avg_count)
        {
            avg_temp2 = (avg_temp2+avg_temp1)/2;
        }
        else
        {
            avg_temp2 = avg_temp1;
        }
        avg_count++;
        if (avg_count >= i2c_regs[1])
        {
            // All done, set the bytes to registers
            cli();
            i2c_regs[2] = lowByte(avg_temp2);
            i2c_regs[3] = highByte(avg_temp2);
            sei();
            // And clear the conversion flag so the master knows we're ready
            bitClear(i2c_regs[0], 7);
        }
        else
        {
            // Re-trigger conversion
            ADC_StartConversion();
        }
    }

}
