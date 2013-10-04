/*
  pins_arduino.h - Pin definition functions for Arduino (arduino.cc)
  Part of AVR-Netino - http://code.google.com/p/avr-netino/

  Copyright (c) 2011 Michael Maassen
  
  2011-01-04:	port to AVR-Net-IO by M.Maassen <mic.maassen@gmail.com>
  2012-01-21:   port to Arduino-1.0
  2013-10-03:   port to Arduino-1.0.5

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
#define Pins_Arduino_h		0x20131003	/* Date 2013-10-03 */

// we define AVRNETINO to flag avrnetino core
// with additional constants for libraries
#ifndef AVR_Netino 
#define AVR_Netino	0x20131003	/* Date 2012-10-03 */
#endif

/* Hopfully this will be setable in future arduino ide's boards.txt */
#ifndef BOARD_DEF
#define BOARD_DEF	"board.def"
#endif

#include <avr/pgmspace.h>

  /* this enum maps avr port pins to pin numbers */
enum pins_by_port { 
#define pinDef(P,B,T,F,...) pins_port_##P##B,
#include BOARD_DEF
#undef pinDef
  pins_count_digital,			/* number of pins */
}; 

/* this enum is to get the number of analog channels, by increasing Ax 
 * and maps adc channels to analog input numbers */
enum analog_pin_order { 
#define anaDef(a,p,c,...) pins_adc_ch##c,
#include BOARD_DEF
#undef anaDef
  pins_count_analog,			/* number of analog inputs */
}; 

  /* this enum maps analog names A0,A1,... to pin numbers */
enum analog_by_pin { 
#define anaDef(a,p,c,...) A##a = p,
#include BOARD_DEF
#undef anaDef
}; 


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
  SDA = pins_SDA,
  SCL = pins_SCL,

#define Arduino_Constants
#define defConstant(N,V)  N = V,
#define defClass(C,N) N
#include BOARD_DEF
#undef defClass
#undef defConstant
#undef Arduino_Constants
};

#define ArduinoDefs_Digital
#define ArduinoDefs_Analog
#define ArduinoDefs_PWM
#define ArduinoDefs_PCINT
#define ArduinoDefs_USB
#include BOARD_DEF
#undef ArduinoDefs_USB
#undef ArduinoDefs_PCINT
#undef ArduinoDefs_PWM
#undef ArduinoDefs_Analog
#undef ArduinoDefs_Digital

#ifndef NUM_DIGITAL_PINS
#define NUM_DIGITAL_PINS            pins_count_digital
#endif

#ifndef NUM_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS           pins_count_analog
#endif

#ifndef analogPinToChannel
extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  (((P) < NUM_ANALOG_INPUTS) ? pgm_read_byte( analog_pin_to_channel_PGM+(P)):-1)
#define MAP_ANALOG_CHANNEL
#endif

#ifndef analogInputToDigitalPin
extern const int8_t PROGMEM analog_input_to_digital_pin_PGM[];
#define analogInputToDigitalPin(P)  (((P) < NUM_ANALOG_INPUTS) ? pgm_read_byte(analog_input_to_digital_pin_PGM+(P)): -1)
#define MAP_ANALOG_PIN
#endif

#ifndef digitalPinHasPWM
extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];
#define digitalPinHasPWM(P)  ( pgm_read_byte(digital_pin_to_timer_PGM  + (P) ) != NOT_ON_TIMER)
#endif

#ifndef digitalPinToPCICR
extern const int8_t PROGMEM digital_pin_to_pcint_PGM[];
#ifdef PCINT
#define digitalPinToPCINT(P)  ( pgm_read_byte(digital_pin_to_pcint_PGM  + (P) ) 
#define MAP_PCINT
#define digitalPinToPCICR(p)    (digitalPinToPCINT(p) >= 0) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (digitalPinToPCINT(p) >> 3)
#ifdef PCMSK2
#define digitalPinToPCMSK(p)    ((digitalPinToPCINT(p)&16) ? (&PCMSK2) : ((digitalPinToPCINT(p)&8) ? (&PCMSK1) : (&PCMSK0)))
#elif defined(PCMSK1)
#define digitalPinToPCMSK(p)    ((digitalPinToPCINT(p)&8) ? (&PCMSK1) : (&PCMSK0))
#else
#define digitalPinToPCMSK(p)    (&PCMSK0)
#endif
#define digitalPinToPCMSKbit(p) (_BV(digitalPinToPCINT(p)&7))
#else	/* PCINT */
/* MCU has no pin change interrupts */
#define digitalPinToPCICR(p)    ((uint8_t *)0)
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ((uint8_t *)0)
#define digitalPinToPCMSKbit(p) 0
#endif	/* PCINT */
#endif	/* digitalPinToPCICR */

#ifdef ARDUINO_MAIN
// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
#ifdef PORTA
  (uint16_t) &DDRA,
#else
	NOT_A_PORT,
#endif 
#ifdef PORTB
  (uint16_t) &DDRB,
#else
	NOT_A_PORT,
#endif
#ifdef PORTC
  (uint16_t) &DDRC,
#else
	NOT_A_PORT,
#endif
#ifdef PORTD
  (uint16_t) &DDRD,
#else
	NOT_A_PORT,
#endif
#ifdef PORTE
  (uint16_t) &DDRE,
#endif
#ifdef PORTF
  (uint16_t) &DDRF,
#endif
#ifdef PORTG
  (uint16_t) &DDRG,
#endif
#ifdef PORTH
  (uint16_t) &DDRH,
#endif
#ifdef PORTJ
  (uint16_t) &DDRJ,
#endif
#ifdef PORTK
  (uint16_t) &DDRK,
#endif
#ifdef PORTL
  (uint16_t) &DDRL,
#endif
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
#ifdef PORTA
  (uint16_t) 	&PORTA,
#else
	NOT_A_PORT,
#endif
#ifdef 	PORTB
  (uint16_t) 	&PORTB,
#else
	NOT_A_PORT,
#endif
#ifdef 	PORTC
  (uint16_t) 	&PORTC,
#else
	NOT_A_PORT,
#endif
#ifdef 	PORTD
  (uint16_t) 	&PORTD,
#else
	NOT_A_PORT,
#endif
#ifdef 	PORTE
  (uint16_t) 	&PORTE,
#endif
#ifdef 	PORTF
  (uint16_t) 	&PORTF,
#endif
#ifdef 	PORTG
  (uint16_t) 	&PORTG,
#endif
#ifdef 	PORTH
  (uint16_t) 	&PORTH,
#endif
#ifdef 	PORTJ
  (uint16_t) 	&PORTJ,
#endif
#ifdef 	PORTK
  (uint16_t) 	&PORTK,
#endif
#ifdef 	PORTL
  (uint16_t) 	&PORTL,
#endif
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
#ifdef 	PORTA
  (uint16_t) 	&PINA,
#else
	NOT_A_PORT,
#endif
#ifdef 	PORTB
  (uint16_t) 	&PINB,
#else
	NOT_A_PORT,
#endif
#ifdef 	PORTC
  (uint16_t) 	&PINC,
#else
	NOT_A_PORT,
#endif
#ifdef 	PORTD
  (uint16_t) 	&PIND,
#else
	NOT_A_PORT,
#endif
#ifdef 	PORTE
  (uint16_t) 	&PINE,
#endif
#ifdef 	PORTF
  (uint16_t) 	&PINF,
#endif
#ifdef 	PORTG
  (uint16_t) 	&PING,
#endif
#ifdef 	PORTH
  (uint16_t) 	&PINH,
#endif
#ifdef 	PORTJ
  (uint16_t) 	&PINJ,
#endif
#ifdef 	PORTK
  (uint16_t) 	&PINK,
#endif
#ifdef 	PORTL
  (uint16_t) 	&PINL,
#endif
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
#define pinDef(p,B,T,...)	(P##p),
#include BOARD_DEF
#undef pinDef
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
#define pinDef(P,B,T,...)	_BV(B),
#include BOARD_DEF
#undef pinDef
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
#define pinDef(P,B,T,...)	(T),
#include BOARD_DEF
#undef pinDef
};

#ifdef MAP_PCINT
const int8_t PROGMEM digital_pin_to_pcint_PGM[] = {
#define pinDef(P,B,T,F,U,I...)	(I),
#include BOARD_DEF
#undef pinDef
};
#endif	/* MAP_PCINT */

#ifdef MAP_ANALOG_CHANNEL
const uint8_t PROGMEM analog_pin_to_channel_PGM[] = {
#define anaDef(A,P,C,...)	(C),
#include BOARD_DEF
#undef anaDef
};
#endif	/* MAP_ANALOG_CHANNEL */

#ifdef MAP_ANALOG_PIN
const int8_t PROGMEM analog_input_to_digital_pin_PGM[] = {
#define anaDef(A,P,C,...)	(P),
#include BOARD_DEF
#undef anaDef
};
#endif	/* MAP_ANALOG_PIN */


#endif	/* ARDUINO_MAIN */

#endif	/* Pins_Arduino_h */
