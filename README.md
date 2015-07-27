# Arduino TinyWire Slave Library

Originals from <http://www.arduino.cc/playground/Code/USIi2c>

Modified to support ATtiny44/84

## NOTE about reliable communication

Since (most of) the ATTinys lack TWI module for implementing all the nitty-gritty of I2C in hardware
they will have to do some clock-stretching (at least if run at 8MHz, you may get away with more on higher clock speeds)
as specified in the I2C protocol. However some (especially "bit-banged") master implementations do not
support clock-streching (looking at you Bus pirate 3.x and [RPI][rpibug]), you will not get reliable communication
unless your master supports the full I2C protocol spec. There is a [library][pigpio] which can bit-bang I2C correctly on RPI, use that instead of the [buggy hardware][rpibug] (thanks to @brendanarnold for [that tip][pigtip]).

You can use my [Arduino based I2C master][i2crepl] to test your TinyWire code, this uses Bus Pirate semantics
with Arduinos TWI hardware that is known to implement I2C properly.

[i2crepl]: https://github.com/rambo/I2C/blob/master/examples/i2crepl/i2crepl.ino
[rpibug]: http://www.advamation.com/knowhow/raspberrypi/rpi-i2c-bug.html
[pigpio]: http://abyz.co.uk/rpi/pigpio/python.html#bb_i2c_zip
[pigtip]: https://github.com/rambo/TinyWire/issues/14#issuecomment-125325081

## delayMicroseconds

See this issue: <https://github.com/rambo/TinyWire/issues/8> (remember to declare the function void if you copy-paste it verbatim)
