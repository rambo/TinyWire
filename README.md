# Arduino TinyWire Slave Library

This version of the library has a few enhancements to help with improving the stability and reliability of comms if you have a USI TWI device (i.e. ATtiny85) that likes to sleep but sits on a busy i2c bus...for example sharing the bus with a 128x64 OLED.

Interference with the sleep process can quite often result in a frozen bus (SDA/SCL held low indefinitely). The following additions and notes try to address some of these issues.

## Timeout
TinyWireS.begin() can now accept a timeout (uint16), in milliseconds, as the second argument, for example:

```
#define I2C_addr 0x4D
const uint16_t timeout = 25;  // 25ms

TinyWireS.begin(I2C_addr,timeout);
```

The timeout is used in the following ways:

* In waiting for an idle bus prior to sleep. For example,

  ```
  // put MCU to sleep
  if (TinyWireS.sleepMode(true)) {

    // safe to sleep
    ...your sleep commands;

    // zzz

    // woken by something
    ...initial wake commands;

   // now awake, re-initialise (clears sleep flag)
   // via TinyWireS.begin(I2C_addr,timeout), or cancelling
   // sleep does the same thing
   TinyWireS.sleepMode(false);
  }
  ```

* State activity monitoring. TinyWireS.stateCheck() replaces TinyWireS_stop_check(). The new routine checks for the 'stop' condition as well as a bus freeze, and should be called regularly within your main loop, for example,

  ```
  void loop () {

  // run me as often as possible!
  TinyWireS.stateCheck(); // check and handle i2c 'stop' condition and frozen bus

  ...do some other stuff;
  }
  ```
 
 * A 25ms timeout is set by default. If you want to disable the timeout just set the second argument to 0 or false within TinyWireS.begin.

  ## Bus Active

  A function called TinyWireS.busActive() was written to return the current bus activity state (boolean, true = active, false = idle). This is used within the TinyWireS.stateCheck() and TinyWireS.sleepMode() routines, but can be called from your code.

  ## Multibyte Master 'requestFrom'

  At some point in the past this functionality was apparently lost. Anyway, this version brings that back. You should be able to do things like the following from your 'master' side,

  ```
  // request n bytes from the slave
  if (Wire.requestFrom(I2C_addr,n)) {
    while (Wire.available()) {
      uint8_t c = Wire.read(); // receive a byte
      Serial.print(" 0x");Serial.println(c,HEX);
    }
  }
  ```

  ## Additional notes

 * Waking your MCU can sometimes take a few milliseconds, so be patient and expect this in your code. For example, say we want some data from an ATtiny85. We first set the register start position and then we do a multibyte 'requestFrom'. The master side might look something like this,

  ```
  unsigned long timeout_time = 100+millis(); // 100ms timeout
  byte pos = 0;                              // registry address for data retreivel
  byte n = 6;                                // n bytes from slave

  // set registry start position on the slave
  Wire.beginTransmission(I2C_addr);   // begin
  Wire.write(pos);                    // set the reg start address
  err = Wire.endTransmission(false);  // send but suppress stop

  // wait for successful transmission (sensor could be asleep so we need to give some time to wakeup)
  while (err && (millis() < timeout_time)) {
    err = Wire.endTransmission(false);
    delay(1);
  }

  // now request n bytes from slave
  if (!err && Wire.requestFrom(I2C_addr,n)) {
    while (Wire.available()) {
      uint8_t c = Wire.read(); // receive a byte
      Serial.print(" 0x");Serial.println(c,HEX);
    }
  } else {
    // print error
    Serial.print("ERROR: ");Serial.println(err);
  }
  ```
