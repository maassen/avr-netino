# Makefile for ATmegaBOOT
# E.Lins, 18.7.2005
# 
# 5-2-2011 	restructured by M.Maassen <mic.maassen@gmail.com>
# 
# $Id$
#
# Instructions
#
# To make bootloader .hex file:
# make diecimila
# make lilypad
# make ng
# etc...
#
# To burn bootloader .hex file:
# make diecimila_isp
# make lilypad_isp
# make ng_isp
# etc...
MAKEFILES = $(lastword $(MAKEFILE_LIST))

# avrdude:
AVRPRG=avrdude

# program name should not be changed...
#PROGRAM    = ATmegaBOOT_168
#PROGRAM	   = optiboot
PROGRAM    = $(basename $(wildcard *.c))

MCU_DEF = $(shell echo $(MCU_TARGET)|awk '{$$0=toupper($$0);sub("MEGA","mega");sub("TINY","tiny");print "__AVR_" $$0 "__"}')

PROTOCOL   = stk500

# enter the parameters for the avrdude isp tool
ISPTOOL	   = stk500v2
ISPTOOL	   = avrispmkII
#ISPTOOL	   = usbasp
ISPPORT	   = usb
ISPSPEED   = -b 115200

# uncomment the following line, if you want the variant folder be first in 
# include search order
#BUILD_EXTRA = -I{runtime.hardware.path}/avr/variants/{build.variant}

# the efuse should really be 0xf8; since, however, only the lower
# three bits of that byte are used on the atmega168, avrdude gets
# confused if you specify 1's for the higher bits, see:
# http://tinker.it/now/2007/02/24/the-tale-of-avrdude-atmega168-and-extended-bits-fuses/
#
# similarly, the lock bits should be 0xff instead of 0x3f (to
# unlock the bootloader section) and 0xcf instead of 0x0f (to
# lock it), but since the high two bits of the lock byte are
# unused, avrdude would get confused.
LOCK = 3f
ULOCK = 0f

# some ATtiny cannot read lock bits
ISPUNLOCK = $(AVRPRG) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
-e -u -V -U lock:w:0x$(strip $(ULOCK)):m 
ISPFUSES  = $(AVRPRG) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
$(if $(EFUSE), -U efuse:w:0x$(strip $(EFUSE)):m) \
-U hfuse:w:0x$(strip $(HFUSE)):m -U lfuse:w:0x$(strip $(LFUSE)):m
ISPFLASH  = $(AVRPRG) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
-D -U flash:w:$(PROGRAM)_$(TARGET).hex
ISPLOCK   = $(AVRPRG) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
 -V -U lock:w:0x$(strip $(LOCK)):m

STK500 = "C:\Program Files\Atmel\AVR Tools\STK500\Stk500.exe"
STK500-1 = $(STK500) -e -d$(MCU_TARGET) -pf -vf -if$(PROGRAM)_$(TARGET).hex \
-lFF -LFF -f$(HFUSE)$(LFUSE) -EF8 -ms -q -cUSB -I200kHz -s -wt
STK500-2 = $(STK500) -d$(MCU_TARGET) -ms -q -lCF -LCF -cUSB -I200kHz -s -wt


SRC        = $(PROGRAM).c
OBJ        = 
#$(PROGRAM).o

DEFS       = '-DBOOT_SEC=$(LDSECTION)'
LIBS       =

ifneq ($(windir),)
ifeq ($(ARDUINO_DIR),)
ARDUINO_DIR = ..\..\..\..
endif
# we are on windows
CC         = $(ARDUINO_DIR)\hardware\tools\avr\bin\avr-gcc
OBJSIZE    = $(ARDUINO_DIR)\hardware\tools\avr\bin\avr-size
OBJCOPY    = $(ARDUINO_DIR)\hardware\tools\avr\bin\avr-objcopy
OBJDUMP    = $(ARDUINO_DIR)\hardware\tools\avr\bin\avr-objdump
AWK        = $(ARDUINO_DIR)\hardware\tools\avr\utils\bin\gawk
else
# good old un*x system :-)
ifneq ($(TOOLROOT),)
GCCROOT = $(TOOLROOT)/avr/bin/
AVRDUDE_CONF = -C$(TOOLROOT)/avr/etc/avrdude.conf
else
GCCROOT = ""
AVRDUDE_CONF = "" 
endif
CC         = $(GCCROOT)avr-gcc
OBJSIZE    = $(GCCROOT)avr-size
OBJCOPY    = $(GCCROOT)avr-objcopy
OBJDUMP    = $(GCCROOT)avr-objdump
AWK        = awk
endif

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) -DBAUD_RATE=$(BAUD) $(DEFS)

ifeq ($(PROGRAM),optiboot)
OPTI_BAUD  = 115200
OPTIMIZE = -Os -fno-inline-small-functions -fno-split-wide-types -mshort-calls
# for avrdude 5.10 and newer optiboot works only with programmer arduino
# otherwise device signature cannot be read
PROTOCOL = arduino
override LDFLAGS       = -Wl,--section-start=.text=$(LDSECTION) -Wl,--relax -nostartfiles -Wl,--gc-sections -nostdlib

else ifeq ($(PROGRAM),stk500v2)
OPTI_BAUD  = 115200
OPTIMIZE = -I. -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -mno-tablejump -Wstrict-prototypes -std=gnu99 -DAPP_END=$(LDSECTION)
PROTOCOL = $(PROGRAM)
override LDFLAGS       = -Wl,--section-start=.text=$(LDSECTION) -nostartfiles -nostdlib

else ifeq ($(PROGRAM),usbasp)
override LDFLAGS       = -Wl,--section-start=.text=$(LDSECTION) -Wl,--relax -Wl,--gc-sections
OPTIMIZE = -Os -fno-move-loop-invariants -fno-tree-scev-cprop -fno-inline-small-functions -I.
OBJ       += usbdrv/usbdrvasm.o usbdrv/oddebug.o
PROTOCOL = $(PROGRAM)

else
override LDFLAGS       = -Wl,--section-start=.text=$(LDSECTION)
OPTIMIZE   = -O2
endif

# AVR-Net-IO with different CPUs
ANIO = anio32 anio644 anio644p anio1284p anio16 anio164p anio324p
# standard ARDUINO boards with ATmega*8 CPUs at 16MHz
ARDUINOx8 = duemilanove diecimila ng ng8 ng88 
# small ARDUINO boards with ATmega*8 CPUs using internal RC oszillator
ARDUINOx8RC = lilypad atm48
# ARDUINO Mega boards
ARDUINO_MEGA = mega
# all arduino
ARDUINO = $(ARDUINOx8) $(ARDUINOx8RC) $(ARDUINO_MEGA)
# pure ATtiny chips
TINY = tiny25 tiny45 tiny44
TINYx13 = tiny2313
# all tinys
ATTINY = luminet $(TINY) $(TINYx13)
# Carrera Digital
SCAR = bbox132
# all boards
ALL = $(ANIO) $(ARDUINO) $(ATTINY) $(SCAR)  

IHEX_AWK = 'BEGIN {c=0;s=0;v=""}; {c+=1;s+=$$0;v=sprintf("%s%02X",v,$$0)}; END {s+=c+A%256+int(A/256)+T; printf (":%02X%04X%02X%s%02X\n",c,A,T,v,(256-(s%256))) };'
RJMP_AWK = '{x=strtonum($$0)/2-1+0xC000;printf("%i %i",int(x/256),x%256)}'

anio: $(ANIO)
	echo "menu.cpu=Processor" > $@_board.txt
	awk '/remove for/ {exit(0)}; {print $0}' $(<:%=$(PROGRAM)_%.brd.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.cpu.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.h) > $@_bootloader_settings.h

arduino8: $(ARDUINOx8)
	echo "menu.cpu=Processor" > $@_board.txt
	awk '/remove for/ {exit(0)}; {print $0}' $(<:%=$(PROGRAM)_%.brd.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.cpu.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.h) > $@_bootloader_settings.h

RC: $(ARDUINOx8RC)
	echo "menu.cpu=Processor" > $@_board.txt
	awk '/remove for/ {exit(0)}; {print $0}' $(<:%=$(PROGRAM)_%.brd.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.cpu.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.h) > $@_bootloader_settings.h

tiny: $(TINY)
	echo "menu.cpu=Processor" > $@_board.txt
	awk '/remove for/ {exit(0)}; {print $0}' $(<:%=$(PROGRAM)_%.brd.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.cpu.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.h) > $@_bootloader_settings.h

t2313: $(TINYx13)
	echo "menu.cpu=Processor" > $@_board.txt
	awk '/remove for/ {exit(0)}; {print $0}' $(<:%=$(PROGRAM)_%.brd.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.cpu.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.h) > $@_bootloader_settings.h

scar: $(SCAR)
	echo "menu.cpu=Processor" > $@_board.txt
	awk '/remove for/ {exit(0)}; {print $0}' $(<:%=$(PROGRAM)_%.brd.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.cpu.txt) >> $@_board.txt
	cat $(^:%=$(PROGRAM)_%.h) > $@_bootloader_settings.h

all: $(ALL)

#all:
#	for i in $(ALL) ; do $(MAKE) $$i ; done

############################################################
# defaults
############################################################
$(all) $(all:%=%_isp): core=avrnetio 

############################################################
# Arduino w/ ATmega*8
############################################################
$(ARDUINOx8) $(ARDUINOx8:%=%_isp): AVR_FREQ = 16000000L
$(ARDUINOx8) $(ARDUINOx8:%=%_isp): CFLAGS +=  '-DLED_B=5' '-DLED_P=B' '-DNUM_LED_FLASHES=1'
$(ARDUINOx8) $(ARDUINOx8:%=%_isp): BOARD = ARDUINO
$(ARDUINOx8) $(ARDUINOx8:%=%_isp): variant = standard
$(ARDUINOx8) $(ARDUINOx8:%=%_isp): BRD=anioX8

# Arduino w/ ATmega168@16MHz auto-reset 
diecimila diecimila_isp: TARGET = diecimila
diecimila diecimila_isp: MCU_TARGET = atmega168
diecimila diecimila_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' 
diecimila diecimila_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),19200)
ifeq ($(PROGRAM),optiboot)
diecimila diecimila_isp: LDSECTION  = 0x3e00
diecimila diecimila_isp: EFUSE = 04 # FC
else
diecimila diecimila_isp: LDSECTION  = 0x3800
diecimila diecimila_isp: EFUSE = 00
endif
diecimila diecimila_isp: LFUSE = FF
diecimila diecimila_isp: HFUSE = DD

# Arduino w/ ATmega328@16MHz auto-reset 
duemilanove duemilanove_isp: TARGET = duemilanove
duemilanove duemilanove_isp: MCU_TARGET = atmega328p
duemilanove duemilanove_isp: CFLAGS+= '-DMAX_TIME_COUNT=F_CPU>>4' 
duemilanove duemilanove_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
ifeq ($(PROGRAM),optiboot)
duemilanove duemilanove_isp: LDSECTION  = 0x7e00
duemilanove duemilanove_isp: HFUSE = DE
else
duemilanove duemilanove_isp: LDSECTION  = 0x7800
duemilanove duemilanove_isp: HFUSE = DA
endif
duemilanove duemilanove_isp: LFUSE = FF
duemilanove duemilanove_isp: EFUSE = FD # 05

# Arduino w/ ATmega168@16MHz w/o auto-reset 
ng ng_isp: TARGET = ng
ng ng_isp: MCU_TARGET = atmega168
ng ng_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1'
ng ng_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),19200)
ifeq ($(PROGRAM),optiboot)
ng ng_isp: LDSECTION  = 0x3e00
ng ng_isp: EFUSE = FC # 04
else
ng ng_isp: LDSECTION  = 0x3800
ng ng_isp: EFUSE = 00
endif
ng ng_isp: LFUSE = FF
ng ng_isp: HFUSE = DD

# Arduino w/ ATmega8@16MHz w/o auto-reset 
ng8 ng8_isp: TARGET = ng8
ng8 ng8_isp: MCU_TARGET = atmega8
ng8 ng8_isp: AVR_FREQ = 16000000L
ng8 ng8_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' 
ng8 ng8_isp: CFLAGS += '-DLED_B=5' '-DLED_P=B'
ng8 ng8_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),19200)
ifeq ($(PROGRAM),optiboot)
ng8 ng8_isp: LDSECTION  = 0x1e00
ng8 ng8_isp: HFUSE = DC
else
ng8 ng8_isp: LDSECTION  = 0x1800
ng8 ng8_isp: HFUSE = D8
endif
ng8 ng8_isp: LFUSE = BF

# Arduino w/ ATmega88@16MHz w/o auto-reset 
ng88 ng88_isp: TARGET = ng88
ng88 ng88_isp: MCU_TARGET = atmega88
ng88 ng88_isp: AVR_FREQ = 16000000L
ng88 ng88_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' 
ng88 ng88_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),19200)
ifeq ($(PROGRAM),optiboot)
ng88 ng88_isp: LDSECTION  = 0x1e00
ng88 ng88_isp: EFUSE = FC # 04
else
ng88 ng88_isp: LDSECTION  = 0x1800
ng88 ng88_isp: EFUSE = F8 # 00
endif
ng88 ng88_isp: LFUSE = FF
ng88 ng88_isp: HFUSE = DD

############################################################
# Arduino 8 MHz (RC)
############################################################
# Arduino w/ ATmega168@8MHz w/o auto-reset 
$(ARDUINOx8RC) $(ARDUINOx8RC:%=%_isp): BOARD = ARDUINO
$(ARDUINOx8RC) $(ARDUINOx8RC:%=%_isp): variant = standard
lilypad lilypad_isp: TARGET = lilypad
lilypad lilypad_isp: MCU_TARGET = atmega168
lilypad lilypad_isp: AVR_FREQ = 8000000L
lilypad lilypad_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=3' 
lilypad lilypad_isp: CFLAGS += '-DLED_B=5' '-DLED_P=B'
lilypad lilypad_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),19200)
ifeq ($(PROGRAM),optiboot)
lilypad lilypad_isp: LDSECTION  = 0x3e00
lilypad lilypad_isp: EFUSE = FC # 04
else
lilypad lilypad_isp: LDSECTION  = 0x3800
lilypad lilypad_isp: EFUSE = 00
endif
lilypad lilypad_isp: LFUSE = E2
lilypad lilypad_isp: HFUSE = DD

atm48 atm48_isp: TARGET = atm48
atm48 atm48_isp: MCU_TARGET = atmega48
atm48 atm48_isp: AVR_FREQ = 8000000L
atm48 atm48_isp: LOCK = 0xff
atm48 atm48_isp: ULOCK = 0xff
atm48 atm48_isp: LDSECTION  = 0x0dc0
atm48 atm48_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' 
atm48 atm48_isp: CFLAGS += '-DVIRTUAL_BOOT_PARTITION' '-DBOOT_SEC=$(LDSECTION)'
#atm48 atm48_isp: CFLAGS += '-DLED_B=5' '-DLED_P=B' '-DNUM_LED_FLASHES=3' 
atm48 atm48_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),19200)
ifeq ($(PROGRAM),optiboot)
atm48 atm48_isp: EFUSE = FE # 00
else
atm48 atm48_isp: EFUSE = FE # 00
endif
atm48 atm48_isp: LFUSE = E2
atm48 atm48_isp: HFUSE = DF

############################################################
# Arduino Megas
############################################################
# Arduino w/ ATmega1280@16MHz auto-reset 
$(ARDUINO_MEGA) $(ARDUINO_MEGA:%=%_isp): BOARD = ARDUINO_MEGA
$(ARDUINO_MEGA) $(ARDUINO_MEGA:%=%_isp): variant = mega
mega mega_isp: TARGET = mega
mega mega_isp: MCU_TARGET = atmega1280
mega mega_isp: AVR_FREQ = 16000000L 
mega mega_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=0' '-DBIGBOOT'
mega mega_isp: CFLAGS += '-DLED_B=7' '-DLED_P=B' '-DUART_P=E'
mega mega_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
ifeq ($(PROGRAM),optiboot)
mega mega_isp: LDSECTION  = 0x1Fc00
mega mega_isp: HFUSE = DE
else
mega mega_isp: LDSECTION  = 0x1F000
mega mega_isp: HFUSE = DA
endif
mega mega_isp: LFUSE = FF
mega mega_isp: EFUSE = F5

############################################################
# AVR-Net-IO w/ ATmega16/32/*4
############################################################
$(ANIO) $(ANIO:%=%_isp): AVR_FREQ = 16000000L
$(ANIO) $(ANIO:%=%_isp): BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
$(ANIO) $(ANIO:%=%_isp): CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
$(ANIO) $(ANIO:%=%_isp): #CFLAGS += '-DLED_B=1' '-DLED_P=B' # led at BL jumper
$(ANIO) $(ANIO:%=%_isp): CFLAGS += '-DLED_B=6' '-DLED_P=D' # LED3
$(ANIO) $(ANIO:%=%_isp): CFLAGS += '-DBL_B=1' '-DBL_P=B' # BL JMP
$(ANIO) $(ANIO:%=%_isp): LDFLAGS +=
$(ANIO) $(ANIO:%=%_isp): BOARD = AVR_NET_IO
$(ANIO) $(ANIO:%=%_isp): variant = avrnetio
$(ANIO) $(ANIO:%=%_isp): BRD=anio

### AVR-Net-IO w/ ATmega32@16MHz auto-reset
anio32 anio32_isp: TARGET = anio32
anio32 anio32_isp: MCU_TARGET = atmega32
ifeq ($(PROGRAM),optiboot)
anio32 anio32_isp: LDSECTION  = 0x7e00
anio32 anio32_isp: HFUSE = DE
else ifeq ($(PROGRAM),stk500v2)
anio32 anio32_isp: LDSECTION  = 0x7c00
anio32 anio32_isp: HFUSE = DC
else
anio32 anio32_isp: LDSECTION  = 0x7800
anio32 anio32_isp: HFUSE = DA
endif
anio32 anio32_isp: LFUSE = BF

### AVR-Net-IO w/ ATmega16@16MHz auto-reset
anio16 anio16_isp: TARGET = anio16
anio16 anio16_isp: MCU_TARGET = atmega16
ifeq ($(PROGRAM),optiboot)
anio16 anio16_isp: LDSECTION  = 0x3e00
anio16 anio16_isp: HFUSE = DC
else
anio16 anio16_isp: LDSECTION  = 0x3800
anio16 anio16_isp: HFUSE = D8
endif
anio16 anio16_isp: LFUSE = BF

### AVR-Net-IO w/ ATmega164p@16MHz auto-reset
anio164p anio164p_isp: TARGET = anio164p
anio164p anio164p_isp: MCU_TARGET = atmega164p
ifeq ($(PROGRAM),optiboot)
anio164p anio164p_isp: LDSECTION  = 0x3e00
anio164p anio164p_isp: HFUSE = DC
else
anio164p anio164p_isp: LDSECTION  = 0x3800
anio164p anio164p_isp: HFUSE = D8
endif
anio164p anio164p_isp: LFUSE = FF
anio164p anio164p_isp: EFUSE = FD # 05
# this seems to be missing in header files
anio164p anio164p_isp: CFLAGS += -'DSIGNATURE_0=0x1e' -'DSIGNATURE_1=0x94' -'DSIGNATURE_2=0x0a'

### AVR-Net-IO w/ ATmega324p@16MHz auto-reset
anio324p anio324p_isp: TARGET = anio324p
anio324p anio324p_isp: MCU_TARGET = atmega324p
ifeq ($(PROGRAM),optiboot)
anio324p anio324p_isp: LDSECTION  = 0x7e00
anio324p anio324p_isp: HFUSE = DE
else
anio324p anio324p_isp: LDSECTION  = 0x7800
anio324p anio324p_isp: HFUSE = DA
endif
anio324p anio324p_isp: LFUSE = FF
anio324p anio324p_isp: EFUSE = FD # 05
# this seems to be missing in header files
anio324p anio324p_isp: CFLAGS += -'DSIGNATURE_0=0x1e' -'DSIGNATURE_1=0x95' -'DSIGNATURE_2=0x08'

### AVR-Net-IO w/ ATmega644@16MHz auto-reset
anio644 anio644_isp: TARGET = anio644
anio644 anio644_isp: MCU_TARGET = atmega644
ifeq ($(PROGRAM),optiboot)
anio644 anio644_isp: CFLAGS +=  -DBIG_BOOT
anio644 anio644_isp: LDSECTION  = 0xfc00
anio644 anio644_isp: HFUSE = DE
else
anio644 anio644_isp: LDSECTION  = 0xF800
anio644 anio644_isp: HFUSE = DC
endif
anio644 anio644_isp: LFUSE = FF
anio644 anio644_isp: EFUSE = FD # 05

### AVR-Net-IO w/ ATmega644p@16MHz auto-reset
anio644p anio644p_isp: TARGET = anio644p
anio644p anio644p_isp: MCU_TARGET = atmega644p
ifeq ($(PROGRAM),optiboot)
anio644p anio644p_isp: CFLAGS += -DBIG_BOOT
anio644p anio644p_isp: LDSECTION  = 0xfc00
anio644p anio644p_isp: HFUSE = DE
else
anio644p anio644p_isp: LDSECTION  = 0xF800
anio644p anio644p_isp: HFUSE = DC
endif
anio644p anio644p_isp: LFUSE = FF
anio644p anio644p_isp: EFUSE = FD # 05

### AVR-Net-IO w/ ATmega1284p@16MHz auto-reset
anio1284p anio1284p_isp: TARGET = anio1284p
anio1284p anio1284p_isp: MCU_TARGET = atmega1284p
ifeq ($(PROGRAM),optiboot)
anio1284p anio1284p_isp: CFLAGS += -DBIG_BOOT
anio1284p anio1284p_isp: LDSECTION  = 0x1fc00
anio1284p anio1284p_isp: HFUSE = DE
else
anio1284p anio1284p_isp: CFLAGS += '-DHAVE_EEPROM_BYTE_ACCESS=0' -Os
anio1284p anio1284p_isp: LDSECTION  = 0x1F800
anio1284p anio1284p_isp: HFUSE = DC
endif
anio1284p anio1284p_isp: LFUSE = FF
anio1284p anio1284p_isp: EFUSE = FD # 05

############################################################
# ATtiny (1MHz clocked platforms; These are capable of 9600 baud)
############################################################
luminet luminet_isp: TARGET = luminet
luminet luminet_isp: MCU_TARGET = attiny84
luminet luminet_isp: AVR_FREQ = 1000000L
luminet luminet_isp: CFLAGS += '-DUART_P=A' #'-DLED_B=2' '-DLED_P=A' 
luminet luminet_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),9600)
ifeq ($(PROGRAM),optiboot)
luminet luminet_isp: CFLAGS += '-DSOFT_UART' '-DBAUD_RATE=9600'
luminet luminet_isp: CFLAGS += '-DVIRTUAL_BOOT_PARTITION'
luminet luminet_isp: BAUD = 9600
luminet luminet_isp: LDSECTION  = 0x1d80 # 0x1d00
else
luminet luminet_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),9600)
endif
luminet luminet_isp: HFUSE = DF # Brownout disabled
luminet luminet_isp: LFUSE = 62 # 1MHz internal oscillator, slowly rising power
luminet luminet_isp: EFUSE = FE # Self-programming enable

############################################################
# ATtiny (8MHz clocked platforms)
############################################################
$(TINY) $(TINY:%=%_isp): AVR_FREQ = 8000000L
$(TINY) $(TINY:%=%_isp): CFLAGS += '-DSOFT_UART' #'-DBAUD_RATE=9600'
$(TINY) $(TINY:%=%_isp): CFLAGS += '-DVIRTUAL_BOOT_PARTITION'
$(TINY) $(TINY:%=%_isp): CFLAGS += '-DRAMSTART=0x60'
$(TINY) $(TINY:%=%_isp): BAUD = 9600
$(TINY) $(TINY:%=%_isp): BOARD = ATtiny
$(TINY) $(TINY:%=%_isp): variant = tiny
$(TINY) $(TINY:%=%_isp): BRD = attiny

tiny25 tiny25_isp: TARGET = tiny25
tiny25 tiny25_isp: MCU_TARGET = attiny25
tiny25 tiny25_isp: CFLAGS += '-DUART_P=B'
ifeq ($(PROGRAM),optiboot)
tiny25 tiny25_isp: LDSECTION  = 0x580 
else
endif
tiny25 tiny25_isp: HFUSE = DF # Brownout disabled
tiny25 tiny25_isp: LFUSE = 62 # 1MHz internal oscillator, slowly rising power
tiny25 tiny25_isp: EFUSE = FE # Self-programming enable
tiny25 tiny25_isp: PREDAT = $(shell echo $(LDSECTION)|$(AWK) $(RJMP_AWK))

tiny45 tiny45_isp: TARGET = tiny45
tiny45 tiny45_isp: MCU_TARGET = attiny45
tiny45 tiny45_isp: CFLAGS += '-DUART_P=B' 
tiny45 tiny45_isp: CFLAGS += '-DLED_P=B' '-DLED_B=4' '-DNUM_LED_FLASHES=1' 
# 
ifeq ($(PROGRAM),optiboot)
tiny45 tiny45_isp: LDSECTION  = 0xd00 
#tiny45 tiny45_isp: LDSECTION  = 0x0 
else
endif
#tiny45 tiny45_isp: HFUSE = DF # Brownout disabled
#tiny45 tiny45_isp: LFUSE = 62 # 1MHz internal oscillator, slowly rising power
#tiny45 tiny45_isp: EFUSE = FE # Self-programming enable
tiny45 tiny45_isp: HFUSE = D5 # Brownout 2.7V + SPIEN + EESAVE
tiny45 tiny45_isp: LFUSE = E2 # 8MHz internal oscillator, slowly rising power
tiny45 tiny45_isp: EFUSE = FE # Self-programming enable
tiny45 tiny45_isp: PREDAT = $(shell echo $(LDSECTION)|$(AWK) $(RJMP_AWK))

tiny44 tiny44_isp: TARGET = tiny44
tiny44 tiny44_isp: MCU_TARGET = attiny44
tiny44 tiny44_isp: CFLAGS += '-DUART_P=B'
ifeq ($(PROGRAM),optiboot)
tiny44 tiny44_isp: LDSECTION  = 0xd80 
else
endif
tiny44 tiny44_isp: HFUSE = DF # Brownout disabled
tiny44 tiny44_isp: LFUSE = 62 # 1MHz internal oscillator, slowly rising power
tiny44 tiny44_isp: EFUSE = FE # Self-programming enable

############################################################
# ATtiny (8MHz clocked platforms)
############################################################
$(TINYx13) $(TINYx13:%=%_isp): AVR_FREQ = 8000000L
$(TINYx13) $(TINYx13:%=%_isp): CFLAGS += '-DVIRTUAL_BOOT_PARTITION'
$(TINYx13) $(TINYx13:%=%_isp): CFLAGS += '-DRAMSTART=0x60'
$(TINYx13) $(TINYx13:%=%_isp): BAUD = 57600
$(TINYx13) $(TINYx13:%=%_isp): BOARD = ATtiny2313
$(TINYx13) $(TINYx13:%=%_isp): variant = tiny2313
$(TINYx13) $(TINYx13:%=%_isp): BRD = attinyX13
$(TINYx13) $(TINYx13:%=%_isp): ULOCK = 3f

tiny2313 tiny2313_isp: TARGET = tiny2313
tiny2313 tiny2313_isp: MCU_TARGET = attiny2313
ifeq ($(PROGRAM),optiboot)
tiny2313 tiny2313_isp: LDSECTION  = 0x5E0 
else
endif
tiny2313 tiny2313_isp: HFUSE = DF # Brownout disabled
tiny2313 tiny2313_isp: LFUSE = E4 # 8MHz internal oscillator, slowly rising power
tiny2313 tiny2313_isp: EFUSE = FE # Self-programming enable
tiny2313 tiny2313_isp: PREDAT = $(shell echo $(LDSECTION)|$(AWK) $(RJMP_AWK))

############################################################
# Carrera Black Box D132 (30344) w/ ATmega16
############################################################
bbox132 bbox132_isp: AVR_FREQ = 8000000L
bbox132 bbox132_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
bbox132 bbox132_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
bbox132 bbox132_isp: CFLAGS += '-DTIMEOUT_MS=500' 
bbox132 bbox132_isp: CFLAGS += '-DLED_B=3' '-DLED_P=C' # LED5 (=PC3)
bbox132 bbox132_isp: CFLAGS += '-DBL_B=1' '-DBL_P=B' # Start (=PB1)
bbox132 bbox132_isp: LDFLAGS +=
bbox132 bbox132_isp: BOARD = BLACK_BOX_D132
bbox132 bbox132_isp: variant = bbox132
bbox132 bbox132_isp: BRD=bbox132
bbox132 bbox132_isp: BUILD_EXTRA += -DTIMER0_CLK_SRC=2

bbox132 bbox132_isp: TARGET = bbox132
bbox132 bbox132_isp: MCU_TARGET = atmega16
ifeq ($(PROGRAM),optiboot)
bbox132 bbox132_isp: LDSECTION  = 0x3e00
bbox132 bbox132_isp: HFUSE = CC
else
bbox132 bbox132_isp: LDSECTION  = 0x3800
bbox132 bbox132_isp: HFUSE = C8	# same as Carrera = 2kB Bootloader
endif
bbox132 bbox132_isp: LFUSE = 2F



############################################################
# machine room
############################################################
$(ALL):%:$(PROGRAM)_%.hex $(PROGRAM)_%.boards.txt $(PROGRAM)_%.brd.txt $(PROGRAM)_%.cpu.txt $(PROGRAM)_%.h
$(ALL:%=%_isp):%_isp: % isp

isp: $(TARGET)
	$(ISPUNLOCK)
	$(ISPFUSES)
	$(ISPFLASH)
	$(ISPLOCK)

isp-stk500: $(PROGRAM)_$(TARGET).hex
	$(STK500-1)
	$(STK500-2)

%.o: %.S
	$(CC) $(CFLAGS) -o $@ -c $^

%.elf: $(SRC) $(OBJ)
	$(CC) $(CFLAGS) -c  $<
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(^:%.c=%.o) $(LIBS)
	$(OBJSIZE) $@

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex *boards.txt *~

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
	$(if $(PREDAT),(echo $(PREDAT)|$(AWK) -v RS=" " $(IHEX_AWK); cat $@) > $@.tmp && mv $@.tmp $@)

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

reverse = $(if $(1),$(call reverse,$(wordlist 2,$(words $(1)),$(1)))) $(firstword $(1))
ldir = $(subst /, ,$(PWD))
rdir = $(call reverse,$(ldir))
bldr = $(word 1,$(rdir))
mntr = $(word 3,$(rdir))
ifeq ($(mntr),avr)
mntr = $(word 4,$(rdir))
endif
ifeq ($(core),)
core = $(mntr)
endif

### arduino 1.0.x ##########################################
%.boards.txt: %.hex
	@echo "##################################################" > $@
	@echo "# $(TARGET): $(core) /w $(MCU_TARGET) ($(SRC))" >> $@
	@echo "##################################################" >> $@
	@echo "$(TARGET).name=$(BOARD) w/ $(MCU_TARGET) ($(SRC))" >> $@
	@echo "$(TARGET).build.mcu=$(MCU_TARGET)" >> $@
	@echo "$(TARGET).build.f_cpu=$(AVR_FREQ)" >> $@
	@echo "$(TARGET).build.core=$(core)" >> $@
	@echo $(if $(variant),,'#') "$(TARGET).build.variant=$(variant)" >> $@
	@echo "$(TARGET).upload.protocol=$(PROTOCOL)" >> $@
	@echo -n "$(TARGET).upload.maximum_size=" >> $@
	@$(AWK) 'BEGIN {print $(LDSECTION);}' >> $@
	@echo "$(TARGET).upload.speed=$(BAUD)" >> $@
	@echo "$(TARGET).bootloader.path=$(bldr)" >> $@
	@echo "$(TARGET).bootloader.file=$<" >> $@
	@$(OBJSIZE) $(<:%.hex=%.elf) | $(AWK) '{print "# " $$0;}' >> $@
	@echo "$(TARGET).bootloader.low_fuses=0x$(LFUSE)" >> $@
	@echo "$(TARGET).bootloader.high_fuses=0x$(HFUSE)" >> $@
	@echo $(if $(EFUSE),,'#') "$(TARGET).bootloader.extended_fuses=0x$(EFUSE)" >> $@
	@echo "$(TARGET).bootloader.unlock_bits=0x$(ULOCK)" >> $@
	@echo "$(TARGET).bootloader.lock_bits=0x$(LOCK)" >> $@
	@echo "### for ChipKit mpide:" >> $@
	@echo "# $(TARGET).platform=avr" >> $@
	@echo "# $(TARGET).board=$(BOARD)" >> $@
	@echo "##################################################" >> $@

### arduino 1.5.x ##########################################
%.brd.txt: %.hex
	@echo "##################################################" > $@
	@echo "# $(BRD): $(BOARD) - $(core) " >> $@
	@echo "##################################################" >> $@
	@echo "$(BRD).name=$(BOARD) ($(core))" >> $@

	@echo "$(BRD).bootloader.tool=$(AVRPRG)" >> $@
	@echo "$(BRD).upload.tool=$(AVRPRG)" >> $@
	@echo "$(BRD).upload.speed=$(BAUD)" >> $@
	@echo "$(BRD).upload.protocol=$(PROTOCOL)" >> $@

	@echo "$(BRD).build.core=$(core)" >> $@	
	@echo "$(BRD).build.board=$(BOARD)" >> $@
	@echo "$(BRD).build.f_cpu=$(AVR_FREQ)" >> $@
	@echo $(if $(variant),,'#') "$(BRD).build.variant=$(variant)" >> $@
	@echo $(if $(BUILD_EXTRA),,'#') "$(BRD).build.extra_flags=$(BUILD_EXTRA)" >> $@
	@echo "# CPU stuff ---------------- (remove for cpu menu)" >> $@
	@echo "$(BRD).build.mcu=$(MCU_TARGET)" >> $@

	@echo -n "$(BRD).upload.maximum_size=" >> $@
	@$(AWK) 'BEGIN {print $(LDSECTION);}' >> $@
	# DEBUG: how to get ram size from cpu type? I use 1/16 of flash
	@echo -n "$(BRD).upload.maximum_data_size=" >> $@
	@$(AWK) 'BEGIN {print lshift(1,int(log($(LDSECTION))/log(2)+.5)-4);}' >> $@

	@echo "$(BRD).bootloader.low_fuses=0x$(LFUSE)" >> $@
	@echo "$(BRD).bootloader.high_fuses=0x$(HFUSE)" >> $@
	@echo $(if $(EFUSE),,'#') "$(BRD).bootloader.extended_fuses=0x$(EFUSE)" >> $@
	@echo "$(BRD).bootloader.unlock_bits=0x$(ULOCK)" >> $@
	@echo "$(BRD).bootloader.lock_bits=0x$(LOCK)" >> $@
	@echo "$(BRD).bootloader.file=$(bldr)/$<" >> $@
	@$(OBJSIZE) $(<:%.hex=%.elf) | $(AWK) '{print "# " $$0;}' >> $@
	@echo "#.BootLoader ($(SRC)) compile options:" >> $@
	@echo "#.CFLAGS $(CFLAGS)" >> $@
	@echo "#.LDFLAGS $(LDFLAGS)" >> $@
	@echo "##################################################" >> $@

%.cpu.txt: %.hex
	@echo "#-------------------------------------------------" > $@
	@echo "# $(TARGET): $(core) /w $(MCU_TARGET) ($(SRC))" >> $@
	@echo "#-------------------------------------------------" >> $@
	@echo "$(BRD).menu.cpu.$(TARGET)=$(MCU_TARGET) ($(SRC))" >> $@
	@echo "$(BRD).menu.cpu.$(TARGET).build.mcu=$(MCU_TARGET)" >> $@
	@echo "$(BRD).menu.cpu.$(TARGET).build.f_cpu=$(AVR_FREQ)" >> $@

	@echo "$(BRD).menu.cpu.$(TARGET).upload.protocol=$(PROTOCOL)" >> $@
	@echo -n "$(BRD).menu.cpu.$(TARGET).upload.maximum_size=" >> $@
	@$(AWK) 'BEGIN {print $(LDSECTION);}' >> $@
	# DEBUG: how to get ram size from cpu type? I use 1/16 of flash
	@echo -n "$(BRD).menu.cpu.$(TARGET).upload.maximum_data_size=" >> $@
	@$(AWK) 'BEGIN {print lshift(1,int(log($(LDSECTION))/log(2)+.5)-4);}' >> $@
	@echo "$(BRD).menu.cpu.$(TARGET).upload.speed=$(BAUD)" >> $@

	@echo "$(BRD).menu.cpu.$(TARGET).bootloader.file=$(bldr)/$<" >> $@
	@$(OBJSIZE) $(<:%.hex=%.elf) | $(AWK) '{print "# " $$0;}' >> $@
	@echo "$(BRD).menu.cpu.$(TARGET).bootloader.low_fuses=0x$(LFUSE)" >> $@
	@echo "$(BRD).menu.cpu.$(TARGET).bootloader.high_fuses=0x$(HFUSE)" >> $@
	@echo "$(BRD).menu.cpu.$(TARGET).bootloader.$(if $(EFUSE),extended_fuses=0x$(EFUSE),tool=$(AVRPRG)NoEFuse)" >> $@
	@echo "$(BRD).menu.cpu.$(TARGET).bootloader.lock_bits=0x$(LOCK)" >> $@
	@echo "$(BRD).menu.cpu.$(TARGET).bootloader.unlock_bits=0x$(ULOCK)" >> $@
	@echo "#-------------------------------------------------" >> $@

%.h: %.hex
	@echo "/***********************************************************" > $@
	@echo " * $(BOARD) - $(TARGET) - $(MCU_TARGET)" >> $@
	@echo " **********************************************************/" >> $@
	@echo "#ifndef BOOTLOADER_SETTINGS_H" >> $@
	@echo "#if defined($(MCU_DEF))" >> $@
	@echo "#define BOOTLOADER_SETTINGS_H" $$(date +"0X%Y%m%d") >> $@
	@echo $(CFLAGS) | awk 'BEGIN {FS="="; RS=" "; OFS=" "}; /^-D/ {$$1="#define BOOTLOADER_" substr($$1,3); print }' >> $@
	@echo '#define BOOTLOADER_PROGRAM "$(PROGRAM)"' >> $@
	@echo '#define BOOTLOADER_TOOL "$(AVRPRG)"' >> $@
	@echo '#define BOOTLOADER_TARGET "$(TARGET)"' >> $@
	@echo '#define BOOTLOADER_PROTOCOL "$(PROTOCOL)"' >> $@
	@echo '#define BOOTLOADER_LOW_FUSES 0x$(LFUSE)' >> $@
	@echo '#define BOOTLOADER_HIGH_FUSES 0x$(HFUSE)' >> $@
	@echo $(if $(EFUSE),'#define BOOTLOADER_EXTENDED_FUSES 0x$(EFUSE)',) >> $@
	@echo '#define BOOTLOADER_UNLOCK_BITS 0x$(ULOCK) ' >> $@
	@echo '#define BOOTLOADER_LOCK_BITS 0x$(LOCK) ' >> $@
	@echo '#define BOOTLOADER_FILE "$(bldr)/$<" ' >> $@
	@echo '#define BOOTLOADER_BOARD_NAME "$(BOARD) ($(core))"' >> $@
	@echo '#define BOOTLOADER_BOARD_ID "$(BRD)"' >> $@
	@echo '#define BOOTLOADER_BUILD_CORE "$(core)" ' >> $@
	@echo '#define BOOTLOADER_BUILD_BOARD "$(BOARD)" ' >> $@
	@echo $(if $(variant),'#define BOOTLOADER_BUILD_VARIANT "$(variant)"',) >> $@
	@echo $(if $(BUILD_EXTRA),'#define BOOTLOADER_BUILD_EXTRA_FLAGS "$(BUILD_EXTRA)"',) >> $@
	@echo '#define BOOTLOADER_BUILD_MCU "$(MCU_TARGET)" ' >> $@
	@echo '' >> $@
	@echo '#define BOOTLOADER_PORTPIN(P,B) BOOTLOADER_pins_port(P,B)' >> $@
	@echo '#define BOOTLOADER_pins_port(P,B) pins_port_##P##B' >> $@
	@echo '#if defined(BOOTLOADER_LED_P) && defined(BOOTLOADER_LED_B)' >> $@
	@echo '#define BOOTLOADER_LED BOOTLOADER_PORTPIN(BOOTLOADER_LED_P,BOOTLOADER_LED_B)' >> $@
	@echo "#endif" >> $@
	@echo '#if defined(BOOTLOADER_BL_P) && defined(BOOTLOADER_BL_B)' >> $@
	@echo '#define BOOTLOADER_SWITCH BOOTLOADER_PORTPIN(BOOTLOADER_BL_P,BOOTLOADER_BL_B)' >> $@
	@echo "#endif" >> $@
	@echo "#endif /* $(MCU_TARGET) */" >> $@
	@echo "#endif /* BOOTLOADER_SETTINGS_H */" >> $@



BOARDS = $(wildcard *.boards.txt)
boards.txt: $(BOARDS)
ifeq (,$(BOARDS))
	echo run make '<TARGET>' first
else
	cat $^ > $@
endif
