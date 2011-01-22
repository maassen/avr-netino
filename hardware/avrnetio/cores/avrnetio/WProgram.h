/***********************************************************
 * WProgram for Pollin AVR-Net-IO
 * based on Arduino-0022
 * (C) 2011 Michael Maassen <mic.maassen@gmail.com>
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version, or
 *
 *  under the terms of the  Common Development and Distribution License (CDDL) 
 * version 1, see http://www.opensource.org/licenses/cddl1.php
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA  02111-1307  USA
 *
 * $Id$
 *
 **********************************************************/
#ifndef WProgram_h
#define WProgram_h

/* Hopfully this will be setable in future arduino ide's boards.txt */
#ifndef BOARD_DEF
#define BOARD_DEF	"board.def"
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <avr/interrupt.h>

#include "wiring.h"

#ifdef __cplusplus

#include "WCharacter.h"
#include "WString.h"
#include "HardwareSerial.h"

uint16_t makeWord(uint16_t w);
uint16_t makeWord(byte h, byte l);

#define word(...) makeWord(__VA_ARGS__)

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
void noTone(uint8_t _pin);

// WMath prototypes
long random(long);
long random(long, long);
void randomSeed(unsigned int);
long map(long, long, long, long, long);

extern "C" {
#endif // __cplusplus

#define pinDef(P,B,T,F,U) U,
  enum pin_usage {
#include BOARD_DEF
  };
#undef pinDef

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // WProgram_h
