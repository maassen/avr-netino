/*
  pins_arduino.h - Pin definition functions for Arduino (arduino.cc)
  Part of AVR-Netino - http://code.google.com/p/avr-netino/

  Copyright (c) 2011 Michael Maassen
  
  2011-01-04:	port to AVR-Net-IO by M.Maassen <mic.maassen@gmail.com>

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version, or
  
  under the terms of the  Common Development and Distribution License (CDDL) 
  version 1, see http://www.opensource.org/licenses/cddl1.php
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id$
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h		0x20110327 /* Date */

/* Hopfully this will be setable in future arduino ide's boards.txt */
#ifndef BOARD_DEF
#define BOARD_DEF	"board.def"
#endif

//#define PACKED_PINS		/* store pin definitions in a compact way */
//#define DISABLE_PWM		/* no analog output to save flash mem */
//#define CHECK_PIN_RANGE	/* check pin number to available pins */

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/pgmspace.h>

  /* this enum maps avr port pins to pin numbers */
enum pins_by_port { 
#define pinDef(P,B,T,F,...) pins_port_##P##B,
#include BOARD_DEF
#undef pinDef
  NOT_A_PIN,			/* number of pins */
  NOT_A_PORT = NOT_A_PIN
}; 

#ifdef PACKED_PINS
  /* 
     In the original, the TIMER* values are just distinguish number for comparism.
     I define them to the pin numbers to save flash mem. It'll only work, if 
     all existing TIMERS are used in BOARD_DEF!
     The following def's are only to have distinguish NOT_ON_TIMER names for
     the enum members.
 */
#define NOT_ON_TIMER(P,B) _NOT_ON_TIMER_##P##B
#define TIMER0A(P,B) TIMER0A
#define TIMER0B(P,B) TIMER0B
#define TIMER1A(P,B) TIMER1A
#define TIMER1B(P,B) TIMER1B
#define TIMER2(P,B)  TIMER2
#define TIMER2A(P,B) TIMER2A
#define TIMER2B(P,B) TIMER2B
#define TIMER3A(P,B) TIMER3A
#define TIMER3B(P,B) TIMER3B
#define TIMER3C(P,B) TIMER3C
#define TIMER4A(P,B) TIMER4A 
#define TIMER4B(P,B) TIMER4B
#define TIMER4C(P,B) TIMER4C
#define TIMER5A(P,B) TIMER5A
#define TIMER5B(P,B) TIMER5B
#define TIMER5C(P,B) TIMER5C

enum timer_names {
#define pinDef(P,B,T,F,...) T(P,B),
#include BOARD_DEF
#undef pinDef

#undef NOT_ON_TIMER		
#undef TIMER0A
#undef TIMER0B
#undef TIMER1A
#undef TIMER1B
#undef TIMER2
#undef TIMER2A
#undef TIMER2B
#undef TIMER3A
#undef TIMER3B
#undef TIMER3C
#undef TIMER4A
#undef TIMER4B
#undef TIMER4C
#undef TIMER5A
#undef TIMER5B
#undef TIMER5C
  NOT_ON_TIMER,		
#if defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  /* the ATmega1284(p) have 4 timers, but only 3 have pwm outputs */
  TIMER3A,
  TIMER3B,
#endif
};
#else
enum timer_names {
  NOT_ON_TIMER,
  TIMER0A,
  TIMER0B,
  TIMER1A,
  TIMER1B,
  TIMER2,
  TIMER2A,
  TIMER2B,

  TIMER3A,
  TIMER3B,
  TIMER3C,
  TIMER4A,
  TIMER4B,
  TIMER4C,
  TIMER5A,
  TIMER5B,
  TIMER5C,
};
#endif /* PACKED_PINS */

  /* this enum maps avr pin functions to pin numbers */
enum pins_function { 
#define pinDef(P,B,T,F,...) pins_##F,
#include BOARD_DEF
#undef pinDef
};

  /* this enum maps board pins usage to pin numbers */
enum pins_usage { 
#define pinDef(P,B,T,F,U,...) pins_##U,
#include BOARD_DEF
#undef pinDef
};

/* this is for additional board constants */
#define All_Constants
#define defConstant(N,V)  N = V, 
  enum pins_constants {
#include BOARD_DEF
  };
#undef defConstant
#undef All_Constants

  /* this is for compatiblity to arduino */
enum arduino_compat {
  MISO = pins_MISO, 
  MOSI = pins_MOSI,
  SCK = pins_SCK,
  SS = pins_SS,
};

/************************************************************
 * port access
 ************************************************************/
#ifdef PORT_ADDR_TYPE
/* defined in Makefile */
#elif defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
#define PORT_ADDR_TYPE uint16_t
#define GET_PORT_ADDR(A,P) ( (volatile uint8_t *)( pgm_read_word( A + (P))) ) 
#else
#define PORT_ADDR_TYPE uint8_t
#define GET_PORT_ADDR(A,P) ( (volatile uint8_t *)( pgm_read_byte( A + (P))) ) 
#endif

#ifndef PINS_DATA_TYPE
#define PINS_DATA_TYPE uint8_t
#define GET_PIN_DATA(A,P) ( pgm_read_byte( A + P ) )
#endif

extern const PORT_ADDR_TYPE PROGMEM port_to_mode_PGM[];
extern const PORT_ADDR_TYPE PROGMEM port_to_input_PGM[];
extern const PORT_ADDR_TYPE PROGMEM port_to_output_PGM[];
extern const PORT_ADDR_TYPE PROGMEM port_to_pcmask_PGM[];

#ifdef PACKED_PINS
extern const PINS_DATA_TYPE PROGMEM digital_pin_def_PGM[];

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
#define digPinToPort(P)	   (GET_PIN_DATA(digital_pin_def_PGM,(P)) >> 4)
#define digPinToBitMask(P) _BV(GET_PIN_DATA(digital_pin_def_PGM,(P)) & 7)
#define digPinToTimer(P)   (GET_PIN_DATA(digital_pin_def_PGM,(P))&8 ? (P)+1 : NOT_ON_TIMER) 

#else /* PACKED_PINS */

extern const PINS_DATA_TYPE PROGMEM digital_pin_to_port_PGM[];
extern const PINS_DATA_TYPE PROGMEM digital_pin_to_bit_mask_PGM[];
extern const PINS_DATA_TYPE PROGMEM digital_pin_to_timer_PGM[];

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
#define digPinToPort(P)	   GET_PIN_DATA(digital_pin_to_port_PGM,(P) )
#define digPinToBitMask(P) GET_PIN_DATA(digital_pin_to_bit_mask_PGM,(P))
#define digPinToTimer(P)   GET_PIN_DATA(digital_pin_to_timer_PGM,(P) )
#endif /* PACKED_PINS */

#ifdef CHECK_PIN_RANGE
#define ASSERT_PIN(PIN,OK,ELS)	((P)<NOT_A_PIN)?(OK):(ELS)
#else
#define ASSERT_PIN(PIN,OK,ELS)  (OK)
#endif

#define analogInPinToBit(P) ASSERT_PIN(P,P,0)

/* the following can be defined in the makefile or in board.def */
#ifndef portOutputRegister
#define portOutputRegister(P) GET_PORT_ADDR(port_to_output_PGM,(P))
#endif
#ifndef portInputRegister
#define portInputRegister(P)  GET_PORT_ADDR(port_to_input_PGM, (P))
#endif
#ifndef portModeRegister
#define portModeRegister(P)   GET_PORT_ADDR(port_to_mode_PGM,  (P))
#endif
#ifndef portPCMaskRegister
#define portPCMaskRegister(P) GET_PORT_ADDR(port_to_pcmask_PGM,  (P))
#endif

#ifndef digitalPinToPort
#define digitalPinToPort(P)	ASSERT_PIN(P,digPinToPort(P),NOT_A_PORT)
#endif
#ifndef digitalPinToBitMask
#define digitalPinToBitMask(P)	ASSERT_PIN(P,digPinToBitMask(P),0)
#endif
#ifdef digitalPinToTimer
  /* defined in outer space */
#elif defined DISABLE_PWM
  /* the compilers optimization will see, that with this constant definition
     no timer code will be executed and removes it */
#define digitalPinToTimer(P)	NOT_ON_TIMER
#else
#define digitalPinToTimer(P)	ASSERT_PIN(P,digPinToTimer(P),NOT_ON_TIMER)
#endif

/* these *Fast routines are for quick hi/lo setting only
   the pin has to be put to output before, and not used as pwm.
   it is suggested for chip select or bit bang protocols */
#define PinSetFast(P) \
  (*(portOutputRegister(digitalPinToPort(P))) |= digitalPinToBitMask(P))
#define PinClrFast(P) \
  (*(portOutputRegister(digitalPinToPort(P))) &= (uint8_t)~digitalPinToBitMask(P))
  
#ifdef __cplusplus
}
#endif

#endif
