/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

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

#ifndef Pins_Arduino_h
#define Pins_Arduino_h		0x20120110

#define AVR_NET_IO		0x20120110

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + 8 : -1)

#if defined(COM21)
#if defined(COM0B1)
#define digitalPinHasPWM(p)         ((p) == 20 || (p) == 21 || (p) == 23 || (p) == 27 || (p) == 28)
#else
#define digitalPinHasPWM(p)         ((p) == 20 || (p) == 21 || (p) == 23 || (p) == 27)
#endif	/* COM0B1 */
#else
#if defined(COM0B1)
#define digitalPinHasPWM(p)         ((p) == 20 || (p) == 21 || (p) == 22 || (p) == 23 || (p) == 27 || (p) == 28)
#else
#define digitalPinHasPWM(p)         ((p) == 20 || (p) == 21 || (p) == 22 || (p) == 23 || (p) == 27)
#endif	/* COM0B1 */
#endif	/* COM21 */

const static uint8_t SS   = 28;
const static uint8_t MOSI = 29;
const static uint8_t MISO = 30;
const static uint8_t SCK  = 31;

const static uint8_t SDA = 1;
const static uint8_t SCL = 0;
const static uint8_t LED_BUILTIN = 18;

const static uint8_t A0 = 8;
const static uint8_t A1 = 9;
const static uint8_t A2 = 10;
const static uint8_t A3 = 11;
const static uint8_t A4 = 12;
const static uint8_t A5 = 13;
const static uint8_t A6 = 14;
const static uint8_t A7 = 15;
#ifdef PCICR
#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 31) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 2 : (((p) <= 15) ? 0 : (((p) <= 23) ? 3 : 1)))
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK2) : (((p) <= 15) ? (&PCMSK0) : (((p) <= 23) ? (&PCMSK3) : (((p) <= 31) ? (&PCMSK1) :((uint8_t *)0)))))
#define digitalPinToPCMSKbit(p) ((p) & 7)
#else
/* no PCINT (mega32) */
#define digitalPinToPCICR(p)    ((uint8_t *)0)
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ((uint8_t *)0)
#define digitalPinToPCMSKbit(p) 0
#endif /* PCICR */

#ifdef ARDUINO_MAIN
/***********************************************************
 * Board:	AVR-Net-IO	by Pollin.de
  * On the Arduino board, digital pins are also used
 * for the analog output (software PWM).  Analog input
 * pins are a separate set.

 * ATMEL ATMEGA32 & 644(P) / AVR-NetIO
 *
 *                   +---\/---+
 * INT0 (D 24) PB0  1|        |40  PA0 (AI 0 / D8)
 * INT1 (D 25) PB1  2|        |39  PA1 (AI 1 / D9)
 * INT2 (D 26) PB2  3|        |38  PA2 (AI 2 / D10)
 *  PWM (D 27) PB3  4|        |37  PA3 (AI 3 / D11)
 *PWM+SS(D 28) PB4  5|        |36  PA4 (AI 4 / D12)
 * MOSI (D 29) PB5  6|        |35  PA5 (AI 5 / D13)
 * MISO (D 30) PB6  7|        |34  PA6 (AI 6 / D14)
 *  SCK (D 31) PB7  8|        |33  PA7 (AI 7 / D15)
 *             RST  9|        |32  AREF
 *             VCC 10|        |31  GND 
 *             GND 11|        |30  AVCC
 *           XTAL2 12|        |29  PC7 (D  7)
 *           XTAL1 13|        |28  PC6 (D  6)
 *  RX0 (D 16) PD0 14|        |27  PC5 (D  5) TDI
 *  TX0 (D 17) PD1 15|        |26  PC4 (D  4) TDO
 *  RX1 (D 18) PD2 16|        |25  PC3 (D  3) TMS
 *  TX1 (D 19) PD3 17|        |24  PC2 (D  2) TCK
 *  PWM (D 20) PD4 18|        |23  PC1 (D  1) SDA
 *  PWM (D 21) PD5 19|        |22  PC0 (D  0) SCL
 *  PWM+(D 22) PD6 20|        |21  PD7 (D 23) PWM
 *                   +--------+
 *
 * PWM+ 644 only
 *
 * D0 ..D11 are on SubD25
 * D12..D15 are on screw connectors
 * D16..D17 are RS232 
 * D18..D24,D26 are on EXT. connectors

 ***********************************************************/

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
/* 0 */
	PC,	// PC0: SCL - J3_2
	PC,	// PC1: SDA - J3_3
	PC,	// PC2: TCK - J3_4
	PC,	// PC3: TMS - J3_5
	PC,	// PC4: TDO - J3_6
	PC,	// PC5: TDI - J3_7
	PC,	// PC6: TOSC1 - J3_8
	PC,	// PC7: TOSC2 - J3_9
/* 8 */
	PA,	// PA0: ADC0 - A0
	PA,	// PA1: ADC1 - A1
	PA,	// PA2: ADC2 - A2
	PA,	// PA3: ADC3 - A3
	PA,	// PA4: ADC4 - ADC_1
	PA,	// PA5: ADC5 - ADC_2
	PA,	// PA6: ADC6 - ADC_3
	PA,	// PA7: ADC7 - ADC_4
/* 16 */
	PD,	// PD0: RXD - RS232_RxD
	PD,	// PD1: TXD - RS232_TxD
	PD,	// PD2: INT0 - LED_1
	PD,	// PD3: INT1 - RFM12_IRQ
	PD,	// PD4: OC1B - LED_2
	PD,	// PD5: OC1A - RFM12_CS
#ifdef COM21
	PD,	// PD6: ICP1 - LED_3
	PD,	// PD7: OC2 - SDcard_INS
#else
	PD,	// PD6: OC2B - LED_3
	PD,	// PD7: OC2A - SDcard_INS
#endif
/* 24 */
	PB,	// PB0: T0 - IR_Rx
	PB,	// PB1: T1 - JUMP_PROG
	PB,	// PB2: INT2 - ENC28J60_IRQ
	PB,	// PB3: OC0 - SDcard_CS
#ifdef COM0B1
	PB,	// PB4: SS - ENC28J60_CS
#else
	PB,	// PB4: SS - ENC28J60_CS
#endif
	PB,	// PB5: MOSI - SPI_MOSI
	PB,	// PB6: MISO - SPI_MISO
	PB,	// PB7: SCK - SPI_SCK
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
/* 0 */
	_BV( 0 ),	// PC0: SCL - J3_2
	_BV( 1 ),	// PC1: SDA - J3_3
	_BV( 2 ),	// PC2: TCK - J3_4
	_BV( 3 ),	// PC3: TMS - J3_5
	_BV( 4 ),	// PC4: TDO - J3_6
	_BV( 5 ),	// PC5: TDI - J3_7
	_BV( 6 ),	// PC6: TOSC1 - J3_8
	_BV( 7 ),	// PC7: TOSC2 - J3_9
/* 8 */
	_BV( 0 ),	// PA0: ADC0 - A0
	_BV( 1 ),	// PA1: ADC1 - A1
	_BV( 2 ),	// PA2: ADC2 - A2
	_BV( 3 ),	// PA3: ADC3 - A3
	_BV( 4 ),	// PA4: ADC4 - ADC_1
	_BV( 5 ),	// PA5: ADC5 - ADC_2
	_BV( 6 ),	// PA6: ADC6 - ADC_3
	_BV( 7 ),	// PA7: ADC7 - ADC_4
/* 16 */
	_BV( 0 ),	// PD0: RXD - RS232_RxD
	_BV( 1 ),	// PD1: TXD - RS232_TxD
	_BV( 2 ),	// PD2: INT0 - LED_1
	_BV( 3 ),	// PD3: INT1 - RFM12_IRQ
	_BV( 4 ),	// PD4: OC1B - LED_2
	_BV( 5 ),	// PD5: OC1A - RFM12_CS
#ifdef COM21
	_BV( 6 ),	// PD6: ICP1 - LED_3
	_BV( 7 ),	// PD7: OC2 - SDcard_INS
#else
	_BV( 6 ),	// PD6: OC2B - LED_3
	_BV( 7 ),	// PD7: OC2A - SDcard_INS
#endif
/* 24 */
	_BV( 0 ),	// PB0: T0 - IR_Rx
	_BV( 1 ),	// PB1: T1 - JUMP_PROG
	_BV( 2 ),	// PB2: INT2 - ENC28J60_IRQ
	_BV( 3 ),	// PB3: OC0 - SDcard_CS
#ifdef COM0B1
	_BV( 4 ),	// PB4: SS - ENC28J60_CS
#else
	_BV( 4 ),	// PB4: SS - ENC28J60_CS
#endif
	_BV( 5 ),	// PB5: MOSI - SPI_MOSI
	_BV( 6 ),	// PB6: MISO - SPI_MISO
	_BV( 7 ),	// PB7: SCK - SPI_SCK
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
/* 0 */
	 NOT_ON_TIMER,	// PC0: SCL - J3_2
	 NOT_ON_TIMER,	// PC1: SDA - J3_3
	 NOT_ON_TIMER,	// PC2: TCK - J3_4
	 NOT_ON_TIMER,	// PC3: TMS - J3_5
	 NOT_ON_TIMER,	// PC4: TDO - J3_6
	 NOT_ON_TIMER,	// PC5: TDI - J3_7
	 NOT_ON_TIMER,	// PC6: TOSC1 - J3_8
	 NOT_ON_TIMER,	// PC7: TOSC2 - J3_9
/* 8 */
	 NOT_ON_TIMER,	// PA0: ADC0 - A0
	 NOT_ON_TIMER,	// PA1: ADC1 - A1
	 NOT_ON_TIMER,	// PA2: ADC2 - A2
	 NOT_ON_TIMER,	// PA3: ADC3 - A3
	 NOT_ON_TIMER,	// PA4: ADC4 - ADC_1
	 NOT_ON_TIMER,	// PA5: ADC5 - ADC_2
	 NOT_ON_TIMER,	// PA6: ADC6 - ADC_3
	 NOT_ON_TIMER,	// PA7: ADC7 - ADC_4
/* 16 */
	 NOT_ON_TIMER,	// PD0: RXD - RS232_RxD
	 NOT_ON_TIMER,	// PD1: TXD - RS232_TxD
	 NOT_ON_TIMER,	// PD2: INT0 - LED_1
	 NOT_ON_TIMER,	// PD3: INT1 - RFM12_IRQ
	 TIMER1B,	// PD4: OC1B - LED_2
	 TIMER1A,	// PD5: OC1A - RFM12_CS
#ifdef COM21
	 NOT_ON_TIMER,	// PD6: ICP1 - LED_3
	 TIMER2,	// PD7: OC2 - SDcard_INS
#else
	 TIMER2B,	// PD6: OC2B - LED_3
	 TIMER2A,	// PD7: OC2A - SDcard_INS
#endif
/* 24 */
	 NOT_ON_TIMER,	// PB0: T0 - IR_Rx
	 NOT_ON_TIMER,	// PB1: T1 - JUMP_PROG
	 NOT_ON_TIMER,	// PB2: INT2 - ENC28J60_IRQ
	 TIMER0A,	// PB3: OC0 - SDcard_CS
#ifdef COM0B1
	 TIMER0B,	// PB4: SS - ENC28J60_CS
#else
	 NOT_ON_TIMER,	// PB4: SS - ENC28J60_CS
#endif
	 NOT_ON_TIMER,	// PB5: MOSI - SPI_MOSI
	 NOT_ON_TIMER,	// PB6: MISO - SPI_MISO
	 NOT_ON_TIMER,	// PB7: SCK - SPI_SCK
};

#endif

#endif
