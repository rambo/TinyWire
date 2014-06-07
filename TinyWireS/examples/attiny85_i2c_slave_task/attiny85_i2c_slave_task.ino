/**
 * Example sketch for writing to and reading from a slave in transactional manner, it will also blink a led attached to pin 3 (which is the SOIC pin 2)
 * (provided you're using one of my ATTiny85 boards from https://github.com/rambo/attiny_boards with the led soldered) 
 *
 * NOTE: You must not use delay() or I2C communications will fail, use tws_delay() instead (or preferably some smarter timing system, like the Task library used in this example)
 *
 * On write the first byte received is considered the register addres to modify/read
 * On each byte sent or read the register address is incremented (and it will loop back to 0)
 *
 * You can try this with the Arduino I2C REPL sketch at https://github.com/rambo/I2C/blob/master/examples/i2crepl/i2crepl.ino 
 * If you have bus-pirate remember that the older revisions do not like the slave streching the clock, this leads to all sorts of weird behaviour
 *
 * By default this blinks the SOS morse pattern and then has long on/off time to indicate end of pattern, send [ 8 0 32 ] (using the REPL/bus-pirate 
 * semantics) to make the delay per bit smaller (and thus blinking faster). The pattern lenght is calculated from the register size, it would be fairly
 * trivial to make it yet another variable changeable via I2C.
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
// The default buffer size, Can't recall the scope of defines right now
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif
// Get this library from http://bleaklow.com/files/2010/Task.tar.gz 
// and read http://bleaklow.com/2010/07/20/a_very_simple_arduino_task_manager.html for background and instructions
#include <Task.h>
#include <TaskScheduler.h>

// The led is connected so that the tiny sinks current
#define LED_ON LOW
#define LED_OFF HIGH

// The I2C registers
volatile uint8_t i2c_regs[] =
{
    150, // Delay between each position (ms, remeber that this isa byte so 255 is max)
    B10101000, // SOS pattern 
    B01110111, 
    B01110001,
    B01010000, 
    B00000000,
    B11111111, // Long on and off to mark end of pattern
    B00000000,
};
// Tracks the current register pointer position
volatile byte reg_position;
const byte reg_size = sizeof(i2c_regs);


/**
 * BEGIN: PatternBlinker task based on the Task library Blinker example
 */
// Timed task to blink a LED.
const byte pattern_lenght = (sizeof(i2c_regs)-1) * 8; // bits (first is the speed, rest is the pattern)
class PatternBlinker : public TimedTask
{
public:
    // Create a new blinker for the specified pin and rate.
    PatternBlinker(uint8_t _pin);
    virtual void run(uint32_t now);
private:
    uint8_t pin;      // LED pin.
    uint8_t pattern_position; // Used to calcuate the register and bit offset
};

PatternBlinker::PatternBlinker(uint8_t _pin)
: TimedTask(millis()),
  pin(_pin)
{
    pinMode(pin, OUTPUT);     // Set pin for output.
}

void PatternBlinker::run(uint32_t now)
{
    // Start by setting the next runtime
    incRunTime(i2c_regs[0]);

    // Written out for clear code, the complier might optimize it to something more efficient even without it being unrolled into one line
    byte reg = i2c_regs[1+(pattern_position/8)]; // Get the register where the bit pattern position is stored
    byte shift_amount = 7 - (pattern_position % 7); // To have "natural" left-to-right pattern flow.
    bool state = (reg >> shift_amount) & 0x1;
    if (state) {
        digitalWrite(pin, LED_ON);
    } else {
        digitalWrite(pin, LED_OFF);
    }
    // Calculate the next pattern position
    pattern_position = (pattern_position+1) % pattern_lenght;
}
/**
 * END: PatternBlinker task copied from the Task library example
 */
/**
 * BEGIN: I2C Stop flag checker
 *
 * This task needs to run almost all the time due to the USI I2C implementation limitations
 *
 * So I2CStopCheck_YIELD_TICKS below is used to specify how often the task is run, not it's every 4 ticks
 */
#define I2CStopCheck_YIELD_TICKS 4
class I2CStopCheck : public Task
{
public:
    I2CStopCheck();
    virtual void run(uint32_t now);
    virtual bool canRun(uint32_t now);
private:
    uint8_t yield_counter; // Incremented on each canRun call, used to yield to other tasks.
};

I2CStopCheck::I2CStopCheck()
: Task()
{
}

// We can't just return true since then no other task could ever run (since we have the priority)
bool I2CStopCheck::canRun(uint32_t now)
{
    yield_counter++;
    bool ret = false;
    if (yield_counter == I2CStopCheck_YIELD_TICKS)
    {
        ret = true;
        yield_counter = 0;
    }
    return ret;
}

void I2CStopCheck::run(uint32_t now)
{
    TinyWireS_stop_check();
}
/**
 * END: I2C Stop flag checker
 */

// Create the tasks.
PatternBlinker blinker(3);
I2CStopCheck checker;

// Tasks are in priority order, only one task is run per tick
Task *tasks[] = { &checker, &blinker, };
TaskScheduler sched(tasks, NUM_TASKS(tasks));


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
    digitalWrite(3, LED_ON); // Note that this makes the led turn on, it's wire this way to allow for the voltage sensing above.

    pinMode(1, OUTPUT); // OC1A, also The only HW-PWM -pin supported by the tiny core analogWrite

    /**
     * Reminder: taking care of pull-ups is the masters job
     */

    TinyWireS.begin(I2C_SLAVE_ADDRESS);
    TinyWireS.onReceive(receiveEvent);
    TinyWireS.onRequest(requestEvent);

    
    // Whatever other setup routines ?
    
    digitalWrite(3, LED_OFF);
}

void loop()
{
    // Run the scheduler - never returns.
    sched.run();
}
