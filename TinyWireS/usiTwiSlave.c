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
  06 Feb 2015  Minor change to allow mutli-byte requestFrom() from master.
  10 Feb 2015  Simplied RX/TX buffer code and allowed use of full buffer.
  12 Dec 2016  Added support for ATtiny167

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



/********************************************************************************

                        functions implemented as macros

********************************************************************************/

#define SET_USI_TO_SEND_ACK( ) \
{ \
  /* prepare ACK */ \
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
  /* set SDA as input */ \
  DDR_USI &= ~( 1 << PORT_USI_SDA ); \
  /* clear bus active flag */ \
  busActive = false; \
  \
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
        /* clear all interrupt flags */ \
        ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC ); \
}

#define SET_USI_TO_TWI_OVERFLOW_MODE( ) \
{ \
    USICR = \
         /* keep Start Condition Interrupt enabled to detect RESTART */ \
         /* enable overflow interrupt */ \
         ( 1 << USISIE ) | ( 1 << USIOIE ) | \
         /* set USI in Two-wire mode, hold SCL low on USI Counter overflow */ \
         ( 1 << USIWM1 ) | ( 1 << USIWM0 ) | \
         /* Shift Register Clock Source = External, positive edge */ \
         /* 4-Bit Counter Source = external, both edges */ \
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) | \
         /* no toggle clock-port pin */ \
         ( 0 << USITC ); \
     USISR = \
          /* clear interrupt flags - resetting the Start Condition Flag will */ \
          /* release SCL */ \
          ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) | \
          ( 1 << USIPF ) | ( 1 << USIDC ) | \
          /* set USI to sample 8 bits (count 16 external SCL pin toggles) */ \
          ( 0x0 << USICNT0); \
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
            busy = true; \
            usi_onReceiverPtr(usiTwiAmountDataInReceiveBuffer()); \
            busy = false; \
        } \
    } \
}

#define ONSTOP_USI_RECEIVE_CALLBACK() \
{ \
    USI_RECEIVE_CALLBACK(); \
    SET_USI_TO_TWI_START_CONDITION_MODE(); \
}


#define USI_REQUEST_CALLBACK() \
{ \
    if (usi_onRequestPtr) \
    { \
        busy = true; \
        usi_onRequestPtr(); \
        busy = false; \
    } \
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

// Define a timeout to wait for a properly latched start condition
// 250 or so clock cycles should be more than enough!
static volatile uint8_t waitCnt, waitCnt_limit = 250;

// Flag to ensure we finish processing the last request and aren't rudely
// interrupted by the next one (i.e. fast master, slow slave scenario)
// This should also be set prior to sleeping, and cleared once awake, as this
// prevents entering an overflow state just before the MCU sleeps.
static volatile bool busy;

// Flag to check for bus activty.
// Can be useful to check prior to sleeping. Should help to prevent going to
// sleep while waiting for a 'send ACK' or 'send data' to be clocked out.
static volatile bool busActive;

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

static
void
flushTwiBuffers(
  void
)
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

void
usiTwiSlaveInit(
  uint8_t ownAddress
)
{

  flushTwiBuffers( );

  slaveAddress = ownAddress;

  busy = false;

  // In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL
  // low when a start condition is detected or a counter overflow (only
  // for USIWM1, USIWM0 = 11).  This inserts a wait state.  SCL is released
  // by the ISRs (USI_START_vect and USI_OVERFLOW_vect).

  // Set SCL and SDA as output
  DDR_USI |= ( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA );

  // set SCL and SDA high
  PORT_USI |= ( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA );

  // setup for start condition detection
  SET_USI_TO_TWI_START_CONDITION_MODE();

} // end usiTwiSlaveInit


bool usiTwiDataInTransmitBuffer(void)
{
  return txCount;

} // end usiTwiDataInTransmitBuffer


// put data in the transmission buffer, wait if buffer is full

void
usiTwiTransmitByte(
  uint8_t data
)
{
  // wait for free space in buffer
  while (txCount == TWI_TX_BUFFER_SIZE);

  // store data in buffer
  txBuf[ txHead ] = data;
  txHead = ( txHead + 1 ) & TWI_TX_BUFFER_MASK;
  txCount++;

} // end usiTwiTransmitByte

// return a byte from the receive buffer, wait if buffer is empty

uint8_t
usiTwiReceiveByte(
  void
)
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

void usiTwiHandleSTOP(void)
{
  ONSTOP_USI_RECEIVE_CALLBACK();
}

bool usiTwiIsBusActive(void)
{
  return busActive;
}

void usiTwiSetSleep(bool bFlag)
{
  if (bFlag) {
    // set busy flag while sleeping
    busy = true;

    // release both SCL and SDA
    DDR_USI &= ~( (1 << PIN_USI_SCL) | (1 << PIN_USI_SDA) ); // set as inputs
    PORT_USI |= (1 << PIN_USI_SCL) | (1 << PIN_USI_SDA);     // enable pullups

  } else {
    // when we wake we need to reset the interface
    usiTwiSlaveInit(slaveAddress);
  }
}

/********************************************************************************

                            USI Start Condition ISR

********************************************************************************/

ISR( USI_START_VECTOR )
{
  busActive = true;

  // if not busy proceed
  if (!busy) {

    // set default starting conditions for new TWI package
    overflowState = USI_SLAVE_CHECK_ADDRESS;

    // wait for SCL to go low to ensure the Start Condition has completed (the
    // start detector will hold SCL low ) - if a Stop Condition arises then leave
    // the interrupt to prevent waiting forever - don't use USISR to test for Stop
    // Condition as in Application Note AVR312 because the Stop Condition Flag is
    // going to be set from the last TWI sequence

    waitCnt = 0; // reset timeout counter
    while (
      // SCL is high
      ( PIN_USI & ( 1 << PIN_USI_SCL ) ) &&
      // and SDA is low
      !( PIN_USI & ( 1 << PIN_USI_SDA ) ) &&
      // and we haven't timed out
      (waitCnt < waitCnt_limit)
    ) waitCnt++;

    if ( !( PIN_USI & ( 1 << PIN_USI_SCL ) ) && (waitCnt < waitCnt_limit) )
    { // valid start so prepare to send/receive data
      SET_USI_TO_TWI_OVERFLOW_MODE();
    }
    else
    { // bad start or timeout occurred so reset
      SET_USI_TO_TWI_START_CONDITION_MODE();
    } // end if

  } else {
    // pass through if busy
    // but make sure to reset start condition to release SCL
    USISR |= ( 1 << USI_START_COND_INT );
  }

} // end ISR( USI_START_VECTOR )



/********************************************************************************

                                USI Overflow ISR

Handles all the communication.

Only disabled when waiting for a new Start Condition.

********************************************************************************/

ISR( USI_OVERFLOW_VECTOR )
{

  switch ( overflowState )
  {

    // Address mode: check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK,
    // else reset USI
    case USI_SLAVE_CHECK_ADDRESS:
      // handle valid address (broadcast (0) or slave)
      if ( ( USIDR == 0 ) || ( ( USIDR >> 1 ) == slaveAddress ) ) {

        // First check we don't have data sitting in receive buffer to process.
        // This situation can arise from a no 'stop' between 'starts' type
        // scenario (start->write->start->read/write), or a missed 'stop'.
        USI_RECEIVE_CALLBACK();

        // determine mode
        if ( USIDR & 0x01 )
        { // --- master read mode ---
          USI_REQUEST_CALLBACK();   // put initial data in tx buffer
          overflowState = USI_SLAVE_SEND_DATA;
        } else
        { // --- master write mode ---
          overflowState = USI_SLAVE_REQUEST_DATA;
        }
        SET_USI_TO_SEND_ACK( );

      } else {
          // no address match so reset
          SET_USI_TO_TWI_START_CONDITION_MODE( );
      }
      break;

    // Master read data mode: check reply and goto USI_SLAVE_SEND_DATA if OK,
    // else reset USI
    case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
      if ( USIDR || (USISR & ( 1 << USIPF )) )
      {
        // if NACK or stop, the master does not want more data, so reset
        SET_USI_TO_TWI_START_CONDITION_MODE( );
        return;
      } else {
        // ACK received, so put some more data in the tx buffer
        USI_REQUEST_CALLBACK();
      }

      // from here we just drop straight into USI_SLAVE_SEND_DATA if the
      // master sent an ACK

    // copy data from buffer to USIDR and set USI to shift byte
    // next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
    case USI_SLAVE_SEND_DATA:
      // write data from tx Buffer into USIDR
      if ( txCount )
      {
        USIDR = txBuf[ txTail ];
        txTail = ( txTail + 1 ) & TWI_TX_BUFFER_MASK;
        txCount--;
      }
      else
      {
        // the buffer is empty
        SET_USI_TO_TWI_START_CONDITION_MODE( );
        return;
      } // end if
      overflowState = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
      SET_USI_TO_SEND_DATA( );
      break;

    // set USI to sample reply from master
    // next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
    case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
      overflowState = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
      SET_USI_TO_READ_ACK( );
      break;

    // Master write data mode: set USI to sample data from master, next
    // USI_SLAVE_GET_DATA_AND_SEND_ACK
    case USI_SLAVE_REQUEST_DATA:

      overflowState = USI_SLAVE_GET_DATA_AND_SEND_ACK;
      SET_USI_TO_READ_DATA( );
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
      }

      // next USI_SLAVE_REQUEST_DATA
      overflowState = USI_SLAVE_REQUEST_DATA;
      SET_USI_TO_SEND_ACK( );
      break;

  } // end switch

} // end ISR( USI_OVERFLOW_VECTOR )
