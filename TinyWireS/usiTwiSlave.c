/********************************************************************************

USI TWI Slave driver.

Created by Donald R. Blake. donblake at worldnet.att.net
Adapted by Jochen Toppe, jochen.toppe at jtoee.com

---------------------------------------------------------------------------------

Created from Atmel source files for Application Note AVR312: Using the USI Module
as an I2C slave.

This program is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

---------------------------------------------------------------------------------

Change Activity:

    Date       Description
   ------      -------------
  16 Mar 2007  Created.
  27 Mar 2007  Added support for ATtiny261, 461 and 861.
  26 Apr 2007  Fixed ACK of slave address on a read.
  04 Jul 2007  Fixed USISIF in ATtiny45 def
  12 Dev 2009  Added callback functions for data requests
  06 Feb 2016  Minor change to allow mutli-byte requestFrom() from master.
  10 Feb 2016  Simplied RX/TX buffer code and allowed use of full buffer.
  13 Feb 2016  Made USI_RECEIVE_CALLBACK() callback fully interrupt-driven
  12 Dec 2016  Added support for ATtiny167
  23 Dec 2017  Fixed repeated restart (which broke when making receive callback
                interrupt-driven)

********************************************************************************/


/********************************************************************************
                                    includes
********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "usiTwiSlave.h"
//#include "../common/util.h"


/********************************************************************************
                            device dependent defines
********************************************************************************/

#if defined( __AVR_ATtiny167__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined( __AVR_ATtiny2313__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB5
#  define PORT_USI_SCL        PB7
#  define PIN_USI_SDA         PINB5
#  define PIN_USI_SCL         PINB7
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined(__AVR_ATtiny84__) | \
     defined(__AVR_ATtiny44__)
#  define DDR_USI             DDRA
#  define PORT_USI            PORTA
#  define PIN_USI             PINA
#  define PORT_USI_SDA        PORTA6
#  define PORT_USI_SCL        PORTA4
#  define PIN_USI_SDA         PINA6
#  define PIN_USI_SCL         PINA4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny25__ ) | \
     defined( __AVR_ATtiny45__ ) | \
     defined( __AVR_ATtiny85__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny26__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_STRT_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny261__ ) | \
      defined( __AVR_ATtiny461__ ) | \
      defined( __AVR_ATtiny861__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATmega165__ ) | \
     defined( __AVR_ATmega325__ ) | \
     defined( __AVR_ATmega3250__ ) | \
     defined( __AVR_ATmega645__ ) | \
     defined( __AVR_ATmega6450__ ) | \
     defined( __AVR_ATmega329__ ) | \
     defined( __AVR_ATmega3290__ )
#  define DDR_USI             DDRE
#  define PORT_USI            PORTE
#  define PIN_USI             PINE
#  define PORT_USI_SDA        PE5
#  define PORT_USI_SCL        PE4
#  define PIN_USI_SDA         PINE5
#  define PIN_USI_SCL         PINE4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined( __AVR_ATmega169__ )
#  define DDR_USI             DDRE
#  define PORT_USI            PORTE
#  define PIN_USI             PINE
#  define PORT_USI_SDA        PE5
#  define PORT_USI_SCL        PE4
#  define PIN_USI_SDA         PINE5
#  define PIN_USI_SCL         PINE4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

// These macros make the stop condition detection code more readable.
#define USI_PINS_SCL_SDA ( ( 1 << PIN_USI_SDA ) | ( 1 << PIN_USI_SCL ) )
#define USI_PINS_SDA     ( 1 << PIN_USI_SDA )
#define USI_PINS_SCL     ( 1 << PIN_USI_SCL )

/********************************************************************************

                        functions implemented as macros

********************************************************************************/

#define SET_USI_TO_SEND_ACK( ) \
{ \
  /* prepare ACK, ack is a zero */ \
  USIDR = 0; \
  /* set SDA as output */ \
  DDR_USI |= ( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR = \
       ( 0 << USI_START_COND_INT ) | \
       ( 1 << USIOIF ) | ( 1 << USIPF ) | \
       ( 1 << USIDC )| \
       /* set USI counter to shift 1 bit */ \
       ( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_READ_ACK( ) \
{ \
  /* set SDA as input */ \
  DDR_USI &= ~( 1 << PORT_USI_SDA ); \
  /* prepare ACK */ \
  USIDR = 0; \
  /* clear all interrupt flags, except Start Cond */ \
  USISR = \
       ( 0 << USI_START_COND_INT ) | \
       ( 1 << USIOIF ) | \
       ( 1 << USIPF ) | \
       ( 1 << USIDC ) | \
       /* set USI counter to shift 1 bit */ \
       ( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_TWI_START_CONDITION_MODE( ) \
{ \
  USICR = \
       /* enable Start Condition Interrupt, disable Overflow Interrupt */ \
       ( 1 << USISIE ) | ( 0 << USIOIE ) | \
       /* set USI in Two-wire mode, no USI Counter overflow hold */ \
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) | \
       /* Shift Register Clock Source = External, positive edge */ \
       /* 4-Bit Counter Source = external, both edges */ \
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) | \
       /* no toggle clock-port pin */ \
       ( 0 << USITC ); \
  USISR = \
        /* clear all interrupt flags, except Start Cond */ \
        ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
        ( 1 << USIDC ) | ( 0x0 << USICNT0 ); \
}

#define SET_USI_TO_SEND_DATA( ) \
{ \
  /* set SDA as output */ \
  DDR_USI |=  ( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR    =  \
       ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
       ( 1 << USIDC) | \
       /* set USI to shift out 8 bits */ \
       ( 0x0 << USICNT0 ); \
}

#define SET_USI_TO_READ_DATA( ) \
{ \
  /* set SDA as input */ \
  DDR_USI &= ~( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR    = \
       ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | \
       ( 1 << USIPF ) | ( 1 << USIDC ) | \
       /* set USI to shift out 8 bits */ \
       ( 0x0 << USICNT0 ); \
}

#define USI_RECEIVE_CALLBACK() \
{ \
    if (usi_onReceiverPtr) \
    { \
        if (usiTwiAmountDataInReceiveBuffer()) \
        { \
            usi_onReceiverPtr(usiTwiAmountDataInReceiveBuffer()); \
        } \
    } \
}

#define USI_REQUEST_CALLBACK() \
{ \
    if(usi_onRequestPtr) usi_onRequestPtr(); \
}

/********************************************************************************

                                   typedef's

********************************************************************************/

typedef enum
{
  USI_SLAVE_CHECK_ADDRESS                = 0x00,
  USI_SLAVE_SEND_DATA                    = 0x01,
  USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA = 0x02,
  USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA   = 0x03,
  USI_SLAVE_REQUEST_DATA                 = 0x04,
  USI_SLAVE_GET_DATA_AND_SEND_ACK        = 0x05
} overflowState_t;



/********************************************************************************

                                local variables

********************************************************************************/

static uint8_t                  slaveAddress;
static uint8_t                  sleep_enable_bit;
static uint8_t                  in_transaction;
static volatile overflowState_t overflowState;


static uint8_t          rxBuf[ TWI_RX_BUFFER_SIZE ];
static volatile uint8_t rxHead;
static volatile uint8_t rxTail;
static volatile uint8_t rxCount;

static uint8_t          txBuf[ TWI_TX_BUFFER_SIZE ];
static volatile uint8_t txHead;
static volatile uint8_t txTail;
static volatile uint8_t txCount;



/********************************************************************************

                                local functions

********************************************************************************/

// flushes the TWI buffers

static void flushTwiBuffers( void )
{
  rxTail = 0;
  rxHead = 0;
  rxCount = 0;
  txTail = 0;
  txHead = 0;
  txCount = 0;
} // end flushTwiBuffers



/********************************************************************************

                                public functions

********************************************************************************/

// initialise USI for TWI slave mode

void usiTwiSlaveInit( uint8_t ownAddress )
{
  // initialize the TX and RX buffers to empty
  flushTwiBuffers( );

  slaveAddress = ownAddress;

  // In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL
  // low when a start condition is detected or a counter overflow (only
  // for USIWM1, USIWM0 = 11).  This inserts a wait state.  SCL is released
  // by the ISRs (USI_START_vect and USI_OVERFLOW_vect).

  // Set SCL and SDA as output
  DDR_USI |= ( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA );

  // set SCL high
  PORT_USI |= ( 1 << PORT_USI_SCL );

  // set SDA high
  PORT_USI |= ( 1 << PORT_USI_SDA );

  // Set SDA as input
  DDR_USI &= ~( 1 << PORT_USI_SDA );

  USICR =
       // enable Start Condition Interrupt
       ( 1 << USISIE ) |
       // disable Overflow Interrupt
       ( 0 << USIOIE ) |
       // set USI in Two-wire mode, no USI Counter overflow hold
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
       // Shift Register Clock Source = external, positive edge
       // 4-Bit Counter Source = external, both edges
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
       // no toggle clock-port pin
       ( 0 << USITC );

  // clear all interrupt flags and reset overflow counter

  USISR = ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC );

  // The 'in_transaction' variable remembers if the usiTwiSlave driver is in the middle of
  // an i2c transaction. Initialize it to zero
  in_transaction = 0;

} // end usiTwiSlaveInit


bool usiTwiDataInTransmitBuffer(void)
{

  // return 0 (false) if the receive buffer is empty
  return txCount;

} // end usiTwiDataInTransmitBuffer


// put data in the transmission buffer, wait if buffer is full

void usiTwiTransmitByte( uint8_t data )
{

  // wait for free space in buffer
  while ( txCount == TWI_TX_BUFFER_SIZE) ;

  // store data in buffer
  txBuf[ txHead ] = data;
  txHead = ( txHead + 1 ) & TWI_TX_BUFFER_MASK;
  txCount++;

} // end usiTwiTransmitByte


// return a byte from the receive buffer, wait if buffer is empty

uint8_t usiTwiReceiveByte( void )
{
  uint8_t rtn_byte;

  // wait for Rx data
  while ( !rxCount );

  rtn_byte = rxBuf [ rxTail ];
  // calculate buffer index
  rxTail = ( rxTail + 1 ) & TWI_RX_BUFFER_MASK;
  rxCount--;

  // return data from the buffer.
  return rtn_byte;

} // end usiTwiReceiveByte


uint8_t usiTwiAmountDataInReceiveBuffer(void)
{
    return rxCount;
}


/********************************************************************************

                            USI Start Condition ISR

********************************************************************************/

ISR( USI_START_VECTOR )
{
  uint8_t usi_pins;
  // http://www.atmel.com/webdoc/AVRLibcReferenceManual/group__avr__interrupts.html

  // Notes about ISR. The compiler in the Arduino IDE handles some of the
  // basic ISR plumbing (unless the "ISR_NAKED" attribute is applied).
  //   * The AVR processor resets the SREG.I bit when jumping into an ISR
  //   * The compiler automatically adds code to save SREG
  //   * < user's ISR code goes here >
  //   * The compiler automatically adds code to restore SREG
  //   * The compiler automatically uses the RETI instruction to return from the ISR.
  //     The RETI instruction enables interrupts after the return from ISR.
  // The compiler behavior can be altered with attributes into the ISR declaration;
  // however, the description above is the default.

  // cli() call is not necessary. Processor disables interrupts when
  // jumping to an ISR

  // no need to save the SREG. The compiler does this automatically when using the
  // ISR construct without modifying attributes.

  if ( !in_transaction )
  {
    // remeber the sleep enable bit when entering the ISR
    sleep_enable_bit = MCUCR & ( 1 << SE );

    // clear the sleep enable bit to prevent the CPU from entering sleep mode while executing this ISR.
    MCUCR &= ~( 1 << SE );
  }

  // set default starting conditions for new TWI package
  overflowState = USI_SLAVE_CHECK_ADDRESS;

  // set SDA as input
  DDR_USI &= ~( 1 << PORT_USI_SDA );

  // the start condition is that the master pulls SDA low.

  // wait for SCL to go low to ensure the Start Condition has completed (the
  // start detector will hold SCL low ) - if a Stop Condition arises then leave
  // the interrupt to prevent waiting forever - don't use USISR to test for Stop
  // Condition as in Application Note AVR312 because the Stop Condition Flag is
  // going to be set from the last TWI sequence

  // while SCL is high and SDA is low
  while  ( ( usi_pins = PIN_USI & USI_PINS_SCL_SDA ) == USI_PINS_SCL );

  // if SDA line was low at SCL edge, then start condition occurred
  if ( !( usi_pins & USI_PINS_SDA ) )
  {
    // a Stop Condition did not occur
    
    // Execute callback if this is a repeated start
    if (in_transaction)
    {
        USI_RECEIVE_CALLBACK();
    }

    USICR =
         // keep Start Condition Interrupt enabled to detect RESTART
         ( 1 << USISIE ) |
         // enable Overflow Interrupt
         ( 1 << USIOIE ) |
         // set USI in Two-wire mode, hold SCL low on USI Counter overflow
         ( 1 << USIWM1 ) | ( 1 << USIWM0 ) |
         // Shift Register Clock Source = External, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );
         
    //remember that the USI is in a valid i2c transaction
    in_transaction = 1;

  }
  else
  {
    // a Stop Condition did occur

    USICR =
         // enable Start Condition Interrupt
         ( 1 << USISIE ) |
         // disable Overflow Interrupt
         ( 0 << USIOIE ) |
         // set USI in Two-wire mode, no USI Counter overflow hold
         ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
         // Shift Register Clock Source = external, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );

    //no longer in valid i2c transaction
    in_transaction = 0;
    // restore the sleep enable bit
    MCUCR |= sleep_enable_bit;

  } // end if

  USISR =
       // clear interrupt flags - resetting the Start Condition Flag will
       // release SCL
       ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) |
       ( 1 << USIPF ) |( 1 << USIDC ) |
       // set USI to sample 8 bits (count 16 external SCL pin toggles)
       ( 0x0 << USICNT0);

  // no need to restore the SREG. The compiler does this automatically when using the
  // ISR construct without modifying attributes.

  // The compiler automatically uses an RETI instruction to return when using the
  // ISR construct without modifying attributes.

} // end ISR( USI_START_VECTOR )



/********************************************************************************

                                USI Overflow ISR

Handles all the communication.

Only disabled when waiting for a new Start Condition.

********************************************************************************/

ISR( USI_OVERFLOW_VECTOR )
{
  uint8_t finished;
  uint8_t usi_pins;

  // http://www.atmel.com/webdoc/AVRLibcReferenceManual/group__avr__interrupts.html

  // Notes about ISR. The compiler in the Arduino IDE handles some of the
  // basic ISR plumbing.
  //   * The AVR processor resets the SREG.I bit when jumping into an ISR
  //   * The compiler automatically adds code to save the SREG
  //   * < user's ISR code goes here >
  //   * The compiler automatically adds code to restore the SREG
  //   * The compiler automatically uses the RETI instruction to return from the ISR.
  //     The RETI insturction enables interrupts after the return from ISR.
  // The compiler behavior can be altered with attributes into the ISR declaration;
  // however, the description above is the default.

  // cli() call is not necessary. Processor disables interrupts when
  // jumping to an ISR

  // no need to save the SREG. The compiler does this automatically when using the
  // ISR construct without modifying attributes.

  // The ISR is only ever entered because the ISR(USI_START_VECTOR) interrupt
  // routine ran first. That routine saved the sleep mode and disabled sleep.

  // Most of the time this routine exits, it has setup the USI to shift in/out bits
  // and is expected to re-entered because of the USI overflow interrupt. Track whether or
  // not the transaction is completely finished.
  finished = 0;


  switch ( overflowState )
  {

    // Address mode: check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK,
    // else reset USI
    case USI_SLAVE_CHECK_ADDRESS:
      if ( ( USIDR == 0 ) || ( ( USIDR >> 1 ) == slaveAddress) )
      {
        if ( USIDR & 0x01 )
        {
          overflowState = USI_SLAVE_SEND_DATA;
        }
        else
        {
          overflowState = USI_SLAVE_REQUEST_DATA;
        } // end if

        // ack the start frame
        // sets up the USI to pull SDA low and clock one bit (two edges)
        SET_USI_TO_SEND_ACK( );
      }
      else
      {
        SET_USI_TO_TWI_START_CONDITION_MODE( );
        finished = 1;
      }
      break;

    // master-read / slave-send: check reply and goto USI_SLAVE_SEND_DATA if OK,
    // else reset USI
    case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
      // Execute request callback for each byte requested, as this is the intended
      // behavior of this library
      USI_REQUEST_CALLBACK();
      if ( USIDR )
      {
        // if NACK, the master does not want more data
        SET_USI_TO_TWI_START_CONDITION_MODE( );
        finished = 1;
        break;
      }
      // from here we just drop straight into USI_SLAVE_SEND_DATA if the
      // master sent an ACK

    // copy data from buffer to USIDR and set USI to shift byte
    // next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
    case USI_SLAVE_SEND_DATA:
      // Get data from Buffer
      if ( txCount )
      {
        USIDR = txBuf[ txTail ];
        txTail = ( txTail + 1 ) & TWI_TX_BUFFER_MASK;
        txCount--;

        overflowState = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
        SET_USI_TO_SEND_DATA( );
      }
      else
      {
        // the buffer is empty
        SET_USI_TO_READ_ACK( ); // This might be neccessary sometimes see http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=805227#805227
        SET_USI_TO_TWI_START_CONDITION_MODE( );
      } // end if
      break;

    // set USI to sample reply from master
    // next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
    case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
      overflowState = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
      SET_USI_TO_READ_ACK( );
      break;

    // master-send / slave-receive: set USI to sample data from master, next
    // USI_SLAVE_GET_DATA_AND_SEND_ACK
    case USI_SLAVE_REQUEST_DATA:
      overflowState = USI_SLAVE_GET_DATA_AND_SEND_ACK;
      SET_USI_TO_READ_DATA( );

      // with the SET_USI_TO_READ_DATA() macro call above, the USI has
      // been setup to catch the next byte if the master sends one.
      // while that's going on, look for a stop condition here which
      // is when the SDA line goes high after the SCL line;

      // wait until SCL goes high
      while  ( ! ( ( usi_pins = PIN_USI & USI_PINS_SCL_SDA ) & USI_PINS_SCL ) );

      // if SDA line was high at SCL edge, then not a stop condition
      if ( usi_pins & USI_PINS_SDA )
        break;

      // wait until SCL goes low or SDA goes high
      while  ( ( usi_pins = PIN_USI & USI_PINS_SCL_SDA ) == USI_PINS_SCL );

      // if both SCL and SDA are high, then stop condition occurred
      if ( usi_pins == USI_PINS_SCL_SDA )
      {
        USI_RECEIVE_CALLBACK();
        SET_USI_TO_TWI_START_CONDITION_MODE( );
        finished = 1;
      }

      break;

    // copy data from USIDR and send ACK
    // next USI_SLAVE_REQUEST_DATA
    case USI_SLAVE_GET_DATA_AND_SEND_ACK:
      // put data into buffer
      // check buffer size
      if ( rxCount < TWI_RX_BUFFER_SIZE )
      {
        rxBuf[ rxHead ] = USIDR;
        rxHead = ( rxHead + 1 ) & TWI_RX_BUFFER_MASK;
        rxCount++;
      } else {
        // overrun
        // drop data
      }
      // next USI_SLAVE_REQUEST_DATA
      overflowState = USI_SLAVE_REQUEST_DATA;
      SET_USI_TO_SEND_ACK( );
      break;

  } // end switch

  if (finished)
  {
    //no longer in valid i2c transaction
    in_transaction = 0;
    // restore the sleep enable bit
    // note that this allows sleep -- it does not cause sleep
    MCUCR |= sleep_enable_bit;
  }

  // no need to restore the SREG. The compiler does this automatically when using the
  // ISR construct without modifying attributes.

  // The compiler automatically uses an RETI instruction to return when using the
  // ISR construct without modifying attributes.

} // end ISR( USI_OVERFLOW_VECTOR )
