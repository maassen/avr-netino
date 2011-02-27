/*
  pins_arduino.c - pin definitions for the Arduino board
  Part of Arduino / Wiring Lite

  Copyright (c) 2005 David A. Mellis

  2011-01-04:	port to AVR-Net-IO and usage of Xmacros in board.def 
		by M.Maassen <mic.maassen@gmail.com>

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

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

#include <avr/io.h>
#include "wiring_private.h"
#include "pins_arduino.h"

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

enum ports {
#ifdef PORTA
  PA,
#endif
#ifdef PORTB
  PB,
#endif
#ifdef PORTC
  PC,
#endif
#ifdef PORTD
  PD,
#endif
};

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const PORT_ADDR_TYPE PROGMEM port_to_mode_PGM[] = {
#ifdef PORTA
	&DDRA,
#endif 
#ifdef PORTB
	&DDRB,
#endif
#ifdef PORTC
	&DDRC,
#endif
#ifdef PORTD
	&DDRD,
#endif
};

const PORT_ADDR_TYPE PROGMEM port_to_output_PGM[] = {
#ifdef PORTA
	&PORTA,
#endif
#ifdef 	PORTB
	&PORTB,
#endif
#ifdef 	PORTC
	&PORTC,
#endif
#ifdef 	PORTD
	&PORTD,
#endif
};

const PORT_ADDR_TYPE PROGMEM port_to_input_PGM[] = {
#ifdef 	PORTA
	&PINA,
#endif
#ifdef 	PORTB
	&PINB,
#endif
#ifdef 	PORTC
	&PINC,
#endif
#ifdef 	PORTD
	&PIND,
#endif
};

const PINS_DATA_TYPE PROGMEM digital_pin_to_port_PGM[] = {
#define pinDef(p,B,T,...)	(P##p),
#include BOARD_DEF
#undef pinDef
};

const PINS_DATA_TYPE PROGMEM digital_pin_to_bit_mask_PGM[] = {
#define pinDef(P,B,T,...)	_BV(B),
#include BOARD_DEF
#undef pinDef
};

const PINS_DATA_TYPE PROGMEM digital_pin_to_timer_PGM[] =
{
#define pinDef(P,B,T,...)	(T),
#include BOARD_DEF
#undef pinDef
};

/* this gives the count of pins */
const uint8_t num_digital_pins = sizeof(digital_pin_to_port_PGM);

