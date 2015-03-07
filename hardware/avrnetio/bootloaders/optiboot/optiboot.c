/**********************************************************/
/* Optiboot bootloader for Arduino                        */
/*                                                        */
/* http://optiboot.googlecode.com                         */
/*                                                        */
/* 2011-02-06 Modifications for AVR-Net-IO (avr-netino)   */
/* by M.Maassen <mic.maassen@gmail.com>                   */
/* added ATmega32 and ATmega644(P)                        */
/* put all board specific stuff including LED definitions */
/* in the Makefile via LED_B/LED_P and remove pin_def.h   */
/*                                                        */
/* http://avr-netino.googlecode.com                       */
/*                                                        */
/* Heavily optimised bootloader that is faster and        */
/* smaller than the Arduino standard bootloader           */
/*                                                        */
/* Enhancements:                                          */
/*   Fits in 512 bytes, saving 1.5K of code space         */
/*   Background page erasing speeds up programming        */
/*   Higher baud rate speeds up programming               */
/*   Written almost entirely in C                         */
/*   Customisable timeout with accurate timeconstant      */
/*   Optional virtual UART. No hardware UART required.    */
/*   Optional virtual boot partition for devices without. */
/*                                                        */
/* What you lose:                                         */
/*   Implements a skeleton STK500 protocol which is       */
/*     missing several features including EEPROM          */
/*     programming and non-page-aligned writes            */
/*   High baud rate breaks compatibility with standard    */
/*     Arduino flash settings                             */
/*                                                        */
/* Fully supported:                                       */
/*   ATmega168 based devices  (Diecimila etc)             */
/*   ATmega328P based devices (Duemilanove etc)           */
/*                                                        */
/* Alpha test                                             */
/*   ATmega1280 based devices (Arduino Mega)              */
/*                                                        */
/* Work in progress:                                      */
/*   ATmega644P based devices (Sanguino/AVR-Net-IO)       */
/*   ATmega644/32 based devices (AVR-Net-IO)              */
/*   ATtiny84 based devices (Luminet)                     */
/*                                                        */
/* Does not support:                                      */
/*   USB based devices (eg. Teensy)                       */
/*                                                        */
/* Assumptions:                                           */
/*   The code makes several assumptions that reduce the   */
/*   code size. They are all true after a hardware reset, */
/*   but may not be true if the bootloader is called by   */
/*   other means or on other hardware.                    */
/*     No interrupts can occur                            */
/*     UART and Timer 1 are set to their reset state      */
/*     SP points to RAMEND                                */
/*                                                        */
/* Code builds on code, libraries and optimisations from: */
/*   stk500boot.c          by Jason P. Kyle               */
/*   Arduino bootloader    http://arduino.cc              */
/*   Spiff's 1K bootloader http://spiffie.org/know/arduino_1k_bootloader/bootloader.shtml */
/*   avr-libc project      http://nongnu.org/avr-libc     */
/*   Adaboot               http://www.ladyada.net/library/arduino/bootloader.html */
/*   AVR305                Atmel Application Note         */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/**********************************************************/


/**********************************************************/
/*                                                        */
/* Optional defines:                                      */
/*                                                        */
/**********************************************************/
/*                                                        */
/* BIG_BOOT:                                              */
/* Build a 1k bootloader, not 512 bytes. This turns on    */
/* extra functionality.                                   */
/*                                                        */
/* BAUD_RATE:                                             */
/* Set bootloader baud rate.                              */
/*                                                        */
/* LUDICROUS_SPEED:                                       */
/* 230400 baud :-)                                        */
/*                                                        */
/* SOFT_UART:                                             */
/* Use AVR305 soft-UART instead of hardware UART.         */
/*                                                        */
/* LED_START_FLASHES:                                     */
/* Number of LED flashes on bootup.                       */
/*                                                        */
/* LED_DATA_FLASH:                                        */
/* Flash LED when transferring data. For boards without   */
/* TX or RX LEDs, or for people who like blinky lights.   */
/*                                                        */
/* LED,LED_DDR,LED_PORT|LED_PIN                           */
/* or for short: LED_B LED_P                              */
/* These must be defined, if LED_START_FLASHES or         */
/* LED_DATA_FLASH is set.                                 */
/*                                                        */
/* BL,BL_PIN or BL_P and BL_B:                            */
/* specify PIN port and Bit to force/skip bootloader      */
/*                                                        */
/* HAVE_PIN_DEFS to use pin_defs.h                        */
/*                                                        */
/* SUPPORT_EEPROM:                                        */
/* Support reading and writing from EEPROM. This is not   */
/* used by Arduino, so off by default.                    */
/*                                                        */
/* TIMEOUT_MS:                                            */
/* Bootloader timeout period, in milliseconds.            */
/* 500,1000,2000,4000,8000 supported.                     */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Version Numbers!                                       */
/*                                                        */
/* Arduino Optiboot now includes this Version number in   */
/* the source and object code.                            */
/*                                                        */
/* Version 3 was released as zip from the optiboot        */
/*  repository and was distributed with Arduino 0022.     */
/* Version 4 starts with the arduino repository commit    */
/*  that brought the arduino repository up-to-date with   */
/* the optiboot source tree changes since v3.             */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Edit History:					  */
/*							  */
/* 4.4 WestfW: add initialization of address to keep      */
/*             the compiler happy.  Change SC'ed targets. */
/*             Return the SW version via READ PARAM       */
/* 4.3 WestfW: catch framing errors in getch(), so that   */
/*             AVRISP works without HW kludges.           */
/*  http://code.google.com/p/arduino/issues/detail?id=368n*/
/* 4.2 WestfW: reduce code size, fix timeouts, change     */
/*             verifySpace to use WDT instead of appstart */
/* 4.1 WestfW: put version number in binary.		  */
/**********************************************************/

// on some avrs (ie. 164p/324p) we need to save some space
#ifdef BIG_BOOT
#define STK_UNIVERSAL_SIGNATURE
#endif	/* BIG_BOOT */

#define OPTIBOOT_MAJVER 4
#define OPTIBOOT_MINVER 4
#if defined (OPTIBOOT_MAJVER) && defined (OPTIBOOT_MINVER) 
#define MAKESTR(a) #a
#define MAKEVER(a, b) MAKESTR(a*256+b)

asm("  .section .version\n"
    "optiboot_version:  .word " MAKEVER(OPTIBOOT_MAJVER, OPTIBOOT_MINVER) "\n"
    "  .section .text\n");
#endif

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

// <avr/boot.h> uses sts instructions, but this version uses out instructions
// This saves cycles and program memory.
#include "boot.h"


// We don't use <avr/wdt.h> as those routines have interrupt overhead we don't need.

#ifdef HAVE_PIN_DEFS
#include "pin_defs.h"
#endif

#include "stk500.h"

#define _REG(x,y) _CAT(x,y)
#define _CAT(x,y)  x##y
#if defined ( LED_B ) && defined ( LED_P )
#define LED_DDR  _REG(DDR,LED_P)
#define LED_PORT _REG(PORT,LED_P)
#if defined(__AVR_ATmega168P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__)
#define LED_PIN  _REG(PIN,LED_P)
#endif	/* can toggle pin with write ti PINx */
#define LED      _REG(PIN,_REG(LED_P,LED_B))
#define LED_PGM  _REG(PIN,_REG(LED_P,LED_B))
#endif

#ifndef LED_START_FLASHES
#ifdef  NUM_LED_FLASHES
#define LED_START_FLASHES NUM_LED_FLASHES
#else
#define LED_START_FLASHES 0
#endif
#endif

#if ! defined( LED_DATA_FLASH ) && defined( LED_PGM )
#define LED_DATA_FLASH
#endif

#if defined(BL_P) && defined(BL_B)
#define BL_PIN  _REG(PIN,BL_P)
#define BL      _REG(PIN,_REG(BL_P,BL_B))
#endif

#ifdef LUDICROUS_SPEED
#define BAUD_RATE 230400L
#endif


/* set the UART baud rate defaults */
#ifndef BAUD_RATE
#if F_CPU >= 8000000L
#define BAUD_RATE   115200L // Highest rate Avrdude win32 will support
#elif F_CPU >= 1000000L
#define BAUD_RATE   9600L   // 19200 also supported, but with significant error
#elif F_CPU >= 128000L
#define BAUD_RATE   4800L   // Good for 128kHz internal RC
#else
#define BAUD_RATE 1200L     // Good even at 32768Hz
#endif
#endif

/* Switch in soft UART for hard baud rates */
#if (F_CPU/BAUD_RATE) > 280 // < 57600 for 16MHz
#ifndef SOFT_UART
#define SOFT_UART
#endif
#endif

/* Ports for soft UART */
#ifdef SOFT_UART
#ifndef UART_P
#define UART_P	D
#endif
#ifndef UART_PORT
#define UART_PORT   _REG(PORT,UART_P)
#endif
#ifndef UART_PIN
#define UART_PIN    _REG(PIN,UART_P)
#endif
#ifndef UART_DDR
#define UART_DDR    _REG(DDR,UART_P)
#endif
#ifndef UART_TX_BIT
#define UART_TX_BIT 1
#endif
#ifndef UART_RX_BIT
#define UART_RX_BIT 0
#endif
#endif

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#ifdef WDE3
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))
#endif

#if TIMEOUT_MS == 500
#define WATCHDOG_DFT    WATCHDOG_500MS
#elif TIMEOUT_MS == 1000
#define WATCHDOG_DFT    WATCHDOG_1S
#elif TIMEOUT_MS == 2000
#define WATCHDOG_DFT    WATCHDOG_2S
#elif TIMEOUT_MS == 3000
#define WATCHDOG_DFT    WATCHDOG_4S
#elif TIMEOUT_MS == 8000
#define WATCHDOG_DFT    WATCHDOG_8S
#else
#define WATCHDOG_DFT    WATCHDOG_1S
#endif

/* Watch dog register names for old ATmegas */
#if !defined ( MCUSR ) && defined ( MCUCSR )
#define MCUSR  MCUCSR
#endif
#if !defined ( WDTCSR ) && defined ( WDTCR )
#define WDTCSR WDTCR		/* m32 */
#endif
#if !defined ( WDCE ) && defined ( WDTOE )
#define WDCE WDTOE
#endif

/* USART register names for old ATmegas */
#if !defined ( UCSR0A ) && defined ( UCSRA ) 
#define	UCSR0A	 UCSRA
#endif

#if !defined ( UDR0 ) && defined ( UDR )
#define	UDR0	 UDR
#endif

#if !defined ( UDRE0 ) && defined ( UDRE )
#define	UDRE0	 UDRE
#endif

#if !defined ( RXC0 ) && defined ( RXC )
#define	RXC0	 RXC
#endif

#if !defined ( FE0 ) && defined ( FE )
#define	FE0	 FE
#endif

#if !defined ( TIFR1 ) && defined ( TIFR )
#define	TIFR1	 TIFR
#endif

/* Function Prototypes */
/* The main function is in init9, which removes the interrupt vector table */
/* we don't need. It is also 'naked', which means the compiler does not    */
/* generate any entry or exit code itself. */
int main(void) __attribute__ ((naked)) __attribute__ ((section (".init9")));
void putch(char);
uint8_t getch(void);
static inline void getNch(uint8_t); /* "static inline" is a compiler hint to reduce code size */
void verifySpace();
#if LED_START_FLASHES > 0
static inline void flash_led(uint8_t);
#endif
uint8_t getLen();
static inline void watchdogReset();
void watchdogConfig(uint8_t x);
#ifdef SOFT_UART
void uartDelay() __attribute__ ((naked));
#endif
void appStart() __attribute__ ((naked));


/************************************************************
 * the following is at least true for:
 * ATmega48A/48PA/88A/88PA/168A/168PA/328/328P
 * ATmega164A/164PA/324A/324PA/644A/644PA/1284/1284P
 * ATmega640/1280/1281/2560/2561
 * ATmega8/32A/16(L)162(V)
 ************************************************************/
#ifdef NRWWSTART
/* defined in Makefile */
#elif FLASHEND == 0x1FFF		/* 8k */
#define NRWWSTART (0x1800)
#elif FLASHEND == 0x3FFF		/* 16k */
#define NRWWSTART (0x3800)
#elif FLASHEND == 0x7FFF		/* 32k */
#define NRWWSTART (0x7000)
#elif FLASHEND == 0xFFFF		/* 64k */
#define NRWWSTART (0xE000)
#elif FLASHEND == 0x1FFFF		/* 128k */
#define NRWWSTART (0x1E000)
#elif FLASHEND == 0x3FFFF		/* 256k */
#define NRWWSTART (0x3E000)
#else
#define NRWWSTART (0x0000)	/* disable background page erase */
#endif

#ifdef RAMSTART
/* defined in Makefile */
#elif defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
#define RAMSTART (0x200)
#else
#define RAMSTART (0x100)
#endif


/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
#define buff    ((uint8_t*)(RAMSTART))
#ifdef VIRTUAL_BOOT_PARTITION
#define rstVect (*(uint16_t*)(RAMSTART+SPM_PAGESIZE*2+4))
#define wdtVect (*(uint16_t*)(RAMSTART+SPM_PAGESIZE*2+6))
#endif

/* main program starts here */
int main(void) {
  uint8_t ch;

  /*
   * Making these local and in registers prevents the need for initializing
   * them, and also saves space because code no longer stores to memory.
   * (initializing address keeps the compiler happy, but isn't really
   *  necessary, and uses 4 bytes of flash.)
   */
  register uint16_t address = 0;
  register uint8_t  length;

  // After the zero init loop, this is the first code to run.
  //
  // This code makes the following assumptions:
  //  No interrupts will execute
  //  SP points to RAMEND
  //  r1 contains zero
  //
  // If not, uncomment the following instructions:
  // cli();
  asm volatile ("cli");


  /*
   * Since we've supressed the normal startup code, we have to initialize
   * A1 (__zero_reg__) so that it will contain 0 as expected.  Apparently
   * there is no guarantee that GP regs are clear on either PWRUP or RST.
   */
  asm volatile ("clr __zero_reg__");
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__) 
  SP=RAMEND;  // This is done by hardware reset on newer AVRs
#endif

  // Adaboot no-wait mod
  ch = MCUSR;
  MCUSR = 0;
#ifdef BL
  if (!(ch & _BV(EXTRF)) && (BL_PIN & _BV(BL))) appStart();
#else
  if (!(ch & _BV(EXTRF))) appStart();
#endif	/* BL */

#if (LED_START_FLASHES > 0) && defined(TCCR1B) 
  // Set up Timer 1 for timeout counter
  TCCR1B = _BV(CS12) | _BV(CS10); // div 1024
#endif
#ifndef SOFT_UART
#ifdef UCSRA
  UCSRA = _BV(U2X); //Double speed mode USART
  UCSRB = _BV(RXEN) | _BV(TXEN);  // enable Rx & Tx
#ifdef URSEL
  UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);  // config USART; 8N1
#else
  UCSRC = _BV(UCSZ1) | _BV(UCSZ0);  // config USART; 8N1
#endif	/* URSEL */
  UBRRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#else
  UCSR0A = _BV(U2X0); //Double speed mode USART0
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);
  UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);
  UBRR0L = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#endif /* UCSRA */
#endif /* SOFT_UART */

  // Set up watchdog to trigger after 500ms
  watchdogConfig(WATCHDOG_DFT);

  /* Set LED pin as output */
#if defined(LED_DATA_FLASH) || (LED_START_FLASHES > 0)
  LED_DDR |= _BV(LED);
#endif

#ifdef SOFT_UART
  /* Set TX pin as output and hi */
  UART_PORT |= _BV(UART_TX_BIT);
  UART_DDR |= _BV(UART_TX_BIT);
#endif

#if LED_START_FLASHES > 0
  /* Flash onboard LED to signal entering of bootloader */
  flash_led(LED_START_FLASHES * 2);
#endif

  /* Forever loop */
  for (;;) {
    /* get character from UART */
    ch = getch();

    if(ch == STK_GET_PARAMETER) {
#if defined(OPTIBOOT_MAJVER) && defined (OPTIBOOT_MINVER)
      unsigned char which = getch();
      verifySpace();
      if (which == 0x82) {
	/*
	 * Send optiboot version as "minor SW version"
	 */
	putch(OPTIBOOT_MINVER);
      } else if (which == 0x81) {
	  putch(OPTIBOOT_MAJVER);
      } else {
	/*
	 * GET PARAMETER returns a generic 0x03 reply for
         * other parameters - enough to keep Avrdude happy
	 */
	putch(0x03);
      }
#else
      getNch(1);
      putch(0x03);
#endif	/* OPTIBOOT_VER */
    }
    else if(ch == STK_SET_DEVICE) {
      // SET DEVICE is ignored
      getNch(20);
    }
    else if(ch == STK_SET_DEVICE_EXT) {
      // SET DEVICE EXT is ignored
      getNch(5);
    }
    else if(ch == STK_LOAD_ADDRESS) {
      // LOAD ADDRESS
      uint16_t newAddress;
      newAddress = getch();
      newAddress = (newAddress & 0xff) | (getch() << 8);
#ifdef RAMPZ
      // Transfer top bit to RAMPZ
      RAMPZ = (newAddress & 0x8000) ? 1 : 0;
#endif
      newAddress += newAddress; // Convert from word address to byte address
      address = newAddress;
      verifySpace();
    }
    else if(ch == STK_UNIVERSAL) {
      // UNIVERSAL command is ignored
#ifdef STK_UNIVERSAL_SIGNATURE
      uint8_t u1,u2;
      u1 =  getch();
      getch();
      u2 =  getch();
      getNch(1);
      if (u1 == 0x30) {
	if (u2 == 0) {
	  putch(SIGNATURE_0);
	} else if (u2 == 1) {
	  putch(SIGNATURE_1); 
	} else {
	  putch(SIGNATURE_2);
	} 
      } else {
	putch(0x00);
      }
#else
      getNch(4);
      putch(0x00);
#endif
    }
    /* Write memory, length is big endian and is in bytes */
    else if(ch == STK_PROG_PAGE) {
      // PROGRAM PAGE - we support flash programming only, not EEPROM
      uint8_t *bufPtr;
      uint16_t addrPtr;

      getch();			/* getlen() */
      length = getch();
      getch();

      // If we are in RWW section, immediately start page erase
      if (address < NRWWSTART) __boot_page_erase_short((uint16_t)(void*)address);

      // While that is going on, read in page contents
      bufPtr = buff;
      do *bufPtr++ = getch();
      while (--length);

      // If we are in NRWW section, page erase has to be delayed until now.
      // Todo: Take RAMPZ into account
      if (address >= NRWWSTART) __boot_page_erase_short((uint16_t)(void*)address);

      // Read command terminator, start reply
      verifySpace();

      // If only a partial page is to be programmed, the erase might not be complete.
      // So check that here
      boot_spm_busy_wait();

#ifdef VIRTUAL_BOOT_PARTITION
      if ((uint16_t)(void*)address == 0) {
        // This is the reset vector page. We need to live-patch the code so the
        // bootloader runs.
        //
        // Move RESET vector to WDT vector
        uint16_t vect = buff[0] | (buff[1]<<8);
        rstVect = vect;
        wdtVect = buff[8] | (buff[9]<<8);
        vect -= 4; // Instruction is a relative jump (rjmp), so recalculate.
        buff[8] = vect & 0xff;
        buff[9] = vect >> 8;

        // Add jump to bootloader at RESET vector
#ifdef BOOT_SEC
	// BOOT_START = boot loader start address in bytes
	buff[0] = (BOOT_SEC>>1) & 0xff;
	buff[1] = 0xc0 | (BOOT_SEC>>9);
#else
        buff[0] = 0x7f;
        buff[1] = 0xce; // rjmp 0x1d00 instruction
#endif
      }
#endif

      // Copy buffer into programming buffer
      bufPtr = buff;
      addrPtr = (uint16_t)(void*)address;
      ch = SPM_PAGESIZE / 2;
      do {
        uint16_t a;
        a = *bufPtr++;
        a |= (*bufPtr++) << 8;
        __boot_page_fill_short((uint16_t)(void*)addrPtr,a);
        addrPtr += 2;
      } while (--ch);

      // Write from programming buffer
      __boot_page_write_short((uint16_t)(void*)address);
      boot_spm_busy_wait();

#if defined(RWWSRE)
      // Reenable read access to flash
      boot_rww_enable();
#endif

    }
    /* Read memory block mode, length is big endian.  */
    else if(ch == STK_READ_PAGE) {
      // READ PAGE - we only read flash
      getch();			/* getlen() */
      length = getch();
      getch();

      verifySpace();
#ifdef VIRTUAL_BOOT_PARTITION
      do {
        // Undo vector patch in bottom page so verify passes
        if (address == 0)       ch=rstVect & 0xff;
        else if (address == 1)  ch=rstVect >> 8;
        else if (address == 8)  ch=wdtVect & 0xff;
        else if (address == 9) ch=wdtVect >> 8;
        else ch = pgm_read_byte_near(address);
        address++;
        putch(ch);
      } while (--length);
#else
#ifdef __AVR_ATmega1280__
//      do putch(pgm_read_byte_near(address++));
//      while (--length);
      do {
        uint8_t result;
        __asm__ ("elpm %0,Z\n":"=r"(result):"z"(address));
        putch(result);
        address++;
      }
      while (--length);
#else
      do putch(pgm_read_byte_near(address++));
      while (--length);
#endif
#endif
    }

    /* Get device signature bytes  */
    else if(ch == STK_READ_SIGN) {
      // READ SIGN - return what Avrdude wants to hear
      verifySpace();
      putch(SIGNATURE_0);
      putch(SIGNATURE_1);
      putch(SIGNATURE_2);
    }
    else if (ch == 'Q') {
      // Adaboot no-wait mod
      watchdogConfig(WATCHDOG_16MS);
      verifySpace();
    }
    else {
      // This covers the response to commands like STK_ENTER_PROGMODE
      verifySpace();
    }
    putch(STK_OK);
  }
}

void putch(char ch) {
#ifndef SOFT_UART
  while (!(UCSR0A & _BV(UDRE0)));
  UDR0 = ch;
#else
  __asm__ __volatile__ (
    "   com %[ch]\n" // ones complement, carry set
    "   sec\n"
    "1: brcc 2f\n"
    "   cbi %[uartPort],%[uartBit]\n"
    "   rjmp 3f\n"
    "2: sbi %[uartPort],%[uartBit]\n"
    "   nop\n"
    "3: rcall uartDelay\n"
    "   rcall uartDelay\n"
    "   lsr %[ch]\n"
    "   dec %[bitcnt]\n"
    "   brne 1b\n"
    :
    :
      [bitcnt] "d" (10),
      [ch] "r" (ch),
      [uartPort] "I" (_SFR_IO_ADDR(UART_PORT)),
      [uartBit] "I" (UART_TX_BIT)
    :
      "r25"
  );
#endif
}

uint8_t getch(void) {
  uint8_t ch;

#ifdef LED_DATA_FLASH
#ifndef LED_PIN
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif

#ifdef SOFT_UART
  __asm__ __volatile__ (
    "1: sbic  %[uartPin],%[uartBit]\n"  // Wait for start edge
    "   rjmp  1b\n"
    "   rcall uartDelay\n"          // Get to middle of start bit
    "2: rcall uartDelay\n"              // Wait 1 bit period
    "   rcall uartDelay\n"              // Wait 1 bit period
    "   clc\n"
    "   sbic  %[uartPin],%[uartBit]\n"
    "   sec\n"
    "   dec   %[bitCnt]\n"
    "   breq  3f\n"
    "   ror   %[ch]\n"
    "   rjmp  2b\n"
    "3:\n"
    :
      [ch] "=r" (ch)
    :
      [bitCnt] "d" (9),
      [uartPin] "I" (_SFR_IO_ADDR(UART_PIN)),
      [uartBit] "I" (UART_RX_BIT)
    :
      "r25"
);
  watchdogReset();
#else
  while(!(UCSR0A & _BV(RXC0)))
    ;
  if (!(UCSR0A & _BV(FE0))) {
      /*
       * A Framing Error indicates (probably) that something is talking
       * to us at the wrong bit rate.  Assume that this is because it
       * expects to be talking to the application, and DON'T reset the
       * watchdog.  This should cause the bootloader to abort and run
       * the application "soon", if it keeps happening.  (Note that we
       * don't care that an invalid char is returned...)
       */
    watchdogReset();
  }
  
  ch = UDR0;
#endif

#ifdef LED_DATA_FLASH
#ifndef LED_PIN 
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif

  return ch;
}

#ifdef SOFT_UART
// AVR350 equation: #define UART_B_VALUE (((F_CPU/BAUD_RATE)-23)/6)
// Adding 3 to numerator simulates nearest rounding for more accurate baud rates
#define UART_B_VALUE (((F_CPU/BAUD_RATE)-20)/6)
#if UART_B_VALUE > 255
#error Baud rate too slow for soft UART
#endif

void uartDelay() {
  __asm__ __volatile__ (
    "ldi r25,%[count]\n"
    "1:dec r25\n"
    "brne 1b\n"
    "ret\n"
    ::[count] "M" (UART_B_VALUE)
  );
}
#endif

void getNch(uint8_t count) {
  do getch(); while (--count);
  verifySpace();
}

void verifySpace() {
  if (getch() != CRC_EOP) {
    watchdogConfig(WATCHDOG_16MS);    // shorten WD timeout
    while (1)			      // and busy-loop so that WD causes
      ;				      //  a reset and app start.
  }
  putch(STK_INSYNC);
}

#if LED_START_FLASHES > 0
void flash_led(uint8_t count) {
  do {
#if defined(TCCR1B)
    TCNT1 = -(F_CPU/(1024*16));
    TIFR1 = _BV(TOV1);
    while(!(TIFR1 & _BV(TOV1)));
#ifndef LED_PIN 
    LED_PORT ^= _BV(LED);
#else
    LED_PIN |= _BV(LED);
#endif
#else  /* TCCR1B */
    uint32_t abc;
    for(abc=0;abc<(F_CPU/50);abc++) {
      LED_PORT |= _BV(LED);
    }
    for(abc=0;abc<(F_CPU/50);abc++) {
      LED_PORT &= ~_BV(LED);
    }
#endif  /* TCCR1B */
    watchdogReset();
  } while (--count);
}
#endif

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset() {
  __asm__ __volatile__ (
    "wdr\n"
  );
}

void watchdogConfig(uint8_t x) {
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = x;
}

void appStart() {
  watchdogConfig(WATCHDOG_OFF);
  __asm__ __volatile__ (
#ifdef VIRTUAL_BOOT_PARTITION
    // Jump to WDT vector
    "ldi r30,4\n"
    "clr r31\n"
#else
    // Jump to RST vector
    "clr r30\n"
    "clr r31\n"
#endif
    "ijmp\n"
  );
}
