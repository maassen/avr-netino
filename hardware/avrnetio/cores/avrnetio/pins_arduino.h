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
#define Pins_Arduino_h		0x20110119 /* Date: 2011-01-19 */

/* Hopfully this will be setable in future arduino ide's boards.txt */
#ifndef BOARD_DEF
#define BOARD_DEF	"board.def"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/pgmspace.h>

#define NOT_A_PIN 0
#define NOT_A_PORT 0

#define NOT_ON_TIMER 0
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER2  5
#define TIMER2A 6
#define TIMER2B 7

#define TIMER3A 8
#define TIMER3B 9
#define TIMER3C 10
#define TIMER4A 11
#define TIMER4B 12
#define TIMER4C 13
#define TIMER5A 14
#define TIMER5B 15
#define TIMER5C 16

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

  /* this is for compatiblity to arduino */
enum arduino_compat {
  MISO = pins_MISO, 
  MOSI = pins_MOSI,
  SCK = pins_SCK,
  SS = pins_SS,
};

/* this is for additional board constants */
#define All_Constants
#define defConstant(N,V)  N = V, 
  enum pins_constants {
#include BOARD_DEF
  };
#undef defConstant
#undef All_Constants

#define PORT_ADDR_TYPE uint8_t
#define PINS_DATA_TYPE uint8_t
extern const PORT_ADDR_TYPE PROGMEM port_to_mode_PGM[];
extern const PORT_ADDR_TYPE PROGMEM port_to_input_PGM[];
extern const PORT_ADDR_TYPE PROGMEM port_to_output_PGM[];
#define GET_PORT_ADDR(A,P) ( (volatile uint8_t *)( pgm_read_byte( A + (P))) ) 

extern const PINS_DATA_TYPE PROGMEM digital_pin_to_port_PGM[];
extern const PINS_DATA_TYPE PROGMEM digital_pin_to_bit_mask_PGM[];
extern const PINS_DATA_TYPE PROGMEM digital_pin_to_timer_PGM[];
#define GET_PIN_DATA(A,P) ( pgm_read_byte( A + P ) )


// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.

#define digitalPinToPort(P)    GET_PIN_DATA(digital_pin_to_port_PGM,(P) )
#define digitalPinToBitMask(P) GET_PIN_DATA(digital_pin_to_bit_mask_PGM,(P))
#define digitalPinToTimer(P)   GET_PIN_DATA(digital_pin_to_timer_PGM,(P) )

#define analogInPinToBit(P) (P)

#define portOutputRegister(P) GET_PORT_ADDR(port_to_output_PGM,(P))
#define portInputRegister(P)  GET_PORT_ADDR(port_to_input_PGM, (P))
#define portModeRegister(P)   GET_PORT_ADDR(port_to_mode_PGM,  (P))

/* this gives the count of pins */
extern const uint8_t num_digital_pins;

#ifdef __cplusplus
}
#endif

#endif
