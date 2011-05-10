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
#ifdef PORTE
  PE,
#endif
#ifdef PORTF
  PF,
#endif
#ifdef PORTG
  PG,
#endif
#ifdef PORTH
  PH,
#endif
#ifdef PORTJ
  PJ,
#endif
#ifdef PORTK
  PK,
#endif
#ifdef PORTL
  PL,
#endif
  NO_PORT
};

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const PORT_ADDR_TYPE PROGMEM port_to_mode_PGM[] = {
#ifdef PORTA
  (PORT_ADDR_TYPE)(int)&DDRA,
#endif 
#ifdef PORTB
  (PORT_ADDR_TYPE)(int)&DDRB,
#endif
#ifdef PORTC
  (PORT_ADDR_TYPE)(int)&DDRC,
#endif
#ifdef PORTD
  (PORT_ADDR_TYPE)(int)&DDRD,
#endif
#ifdef PORTE
  (PORT_ADDR_TYPE)(int)&DDRE,
#endif
#ifdef PORTF
  (PORT_ADDR_TYPE)(int)&DDRF,
#endif
#ifdef PORTG
  (PORT_ADDR_TYPE)(int)&DDRG,
#endif
#ifdef PORTH
  (PORT_ADDR_TYPE)(int)&DDRH,
#endif
#ifdef PORTJ
  (PORT_ADDR_TYPE)(int)&DDRJ,
#endif
#ifdef PORTK
  (PORT_ADDR_TYPE)(int)&DDRK,
#endif
#ifdef PORTL
  (PORT_ADDR_TYPE)(int)&DDRL,
#endif
};

const PORT_ADDR_TYPE PROGMEM port_to_output_PGM[] = {
#ifdef PORTA
  (PORT_ADDR_TYPE)(int)	&PORTA,
#endif
#ifdef 	PORTB
  (PORT_ADDR_TYPE)(int)	&PORTB,
#endif
#ifdef 	PORTC
  (PORT_ADDR_TYPE)(int)	&PORTC,
#endif
#ifdef 	PORTD
  (PORT_ADDR_TYPE)(int)	&PORTD,
#endif
#ifdef 	PORTE
  (PORT_ADDR_TYPE)(int)	&PORTE,
#endif
#ifdef 	PORTF
  (PORT_ADDR_TYPE)(int)	&PORTF,
#endif
#ifdef 	PORTG
  (PORT_ADDR_TYPE)(int)	&PORTG,
#endif
#ifdef 	PORTH
  (PORT_ADDR_TYPE)(int)	&PORTH,
#endif
#ifdef 	PORTJ
  (PORT_ADDR_TYPE)(int)	&PORTJ,
#endif
#ifdef 	PORTK
  (PORT_ADDR_TYPE)(int)	&PORTK,
#endif
#ifdef 	PORTL
  (PORT_ADDR_TYPE)(int)	&PORTL,
#endif
};

const PORT_ADDR_TYPE PROGMEM port_to_input_PGM[] = {
#ifdef 	PORTA
  (PORT_ADDR_TYPE)(int)	&PINA,
#endif
#ifdef 	PORTB
  (PORT_ADDR_TYPE)(int)	&PINB,
#endif
#ifdef 	PORTC
  (PORT_ADDR_TYPE)(int)	&PINC,
#endif
#ifdef 	PORTD
  (PORT_ADDR_TYPE)(int)	&PIND,
#endif
#ifdef 	PORTE
  (PORT_ADDR_TYPE)(int)	&PINE,
#endif
#ifdef 	PORTF
  (PORT_ADDR_TYPE)(int)	&PINF,
#endif
#ifdef 	PORTG
  (PORT_ADDR_TYPE)(int)	&PING,
#endif
#ifdef 	PORTH
  (PORT_ADDR_TYPE)(int)	&PINH,
#endif
#ifdef 	PORTJ
  (PORT_ADDR_TYPE)(int)	&PINJ,
#endif
#ifdef 	PORTK
  (PORT_ADDR_TYPE)(int)	&PINK,
#endif
#ifdef 	PORTL
  (PORT_ADDR_TYPE)(int)	&PINL,
#endif
};

/* PCINT mask */
const PORT_ADDR_TYPE PROGMEM port_to_pcmask_PGM[] = {
#ifdef 	PCMSK
  (PORT_ADDR_TYPE)(int)	&PCMSK,
#elif defined PCMSK0
  (PORT_ADDR_TYPE)(int)	&PCMSK0,
#endif
#ifdef 	PCMSK1
  (PORT_ADDR_TYPE)(int)	&PCMSK1,
#endif
#ifdef 	PCMSK2
  (PORT_ADDR_TYPE)(int)	&PCMSK2,
#endif
#ifdef PCMSK3	
  (PORT_ADDR_TYPE)(int)	&PCMSK3,
#endif
};

#ifdef PACKED_PINS
const PINS_DATA_TYPE PROGMEM digital_pin_def_PGM[] = {
#define pinDef(p,B,T,...)	((P##p)<<4) | (((T)==NOT_ON_TIMER)?0:8) | ((PIN##p##B)&7),
#include BOARD_DEF
#undef pinDef
};

#else /* PACKED_PINS */
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
#endif /* PACKED_PINS */
