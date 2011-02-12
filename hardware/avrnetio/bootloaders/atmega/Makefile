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

# program name should not be changed...
#PROGRAM    = ATmegaBOOT_168
#PROGRAM	   = optiboot
PROGRAM    = $(basename $(wildcard *.c))

OPTI_BAUD  = 115200

# enter the parameters for the avrdude isp tool
ISPTOOL	   = stk500v2
ISPTOOL	   = avrispmkII
ISPPORT	   = usb
ISPSPEED   = -b 115200

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

ISPFUSES    = avrdude -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
-e -u -U lock:w:0x$(strip $(ULOCK)):m \
$(if $(EFUSE), -U efuse:w:0x$(strip $(EFUSE)):m) \
-U hfuse:w:0x$(strip $(HFUSE)):m -U lfuse:w:0x$(strip $(LFUSE)):m
ISPFLASH    = avrdude -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
-U flash:w:$(PROGRAM)_$(TARGET).hex -U lock:w:0x$(strip $(LOCK)):m

STK500 = "C:\Program Files\Atmel\AVR Tools\STK500\Stk500.exe"
STK500-1 = $(STK500) -e -d$(MCU_TARGET) -pf -vf -if$(PROGRAM)_$(TARGET).hex \
-lFF -LFF -f$(HFUSE)$(LFUSE) -EF8 -ms -q -cUSB -I200kHz -s -wt
STK500-2 = $(STK500) -d$(MCU_TARGET) -ms -q -lCF -LCF -cUSB -I200kHz -s -wt


SRC        = $(PROGRAM).c
OBJ        = $(PROGRAM).o
ifeq ($(PROGRAM),optiboot)
OPTIMIZE = -Os -fno-inline-small-functions -fno-split-wide-types -mshort-calls
else
OPTIMIZE   = -O2
endif

DEFS       = 
LIBS       =

CC         = avr-gcc
AWK        = awk

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) -DBAUD_RATE=$(BAUD) $(DEFS)

ifeq ($(PROGRAM),optiboot)
override LDFLAGS       = -Wl,--section-start=.text=$(LDSECTION) -Wl,--relax -nostartfiles -Wl,--gc-sections -nostdlib
else
override LDFLAGS       = -Wl,--section-start=.text=$(LDSECTION)
endif

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

ALL = anio32 anio644 anio644p diecimila ng ng8 ng88 lilypad duemilanove mega
anio: anio32 anio644 anio644p 
all: $(ALL)


# Arduino w/ ATmega168@16MHz auto-reset 
diecimila diecimila_isp: TARGET = diecimila
diecimila diecimila_isp: MCU_TARGET = atmega168
diecimila diecimila_isp: AVR_FREQ = 16000000L 
diecimila diecimila_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
diecimila diecimila_isp: CFLAGS +=  '-DLED_B=5' '-DLED_P=B'
ifeq ($(PROGRAM),optiboot)
diecimila diecimila_isp: BAUD = $(OPTI_BAUD)
diecimila diecimila_isp: LDSECTION  = 0x3e00
diecimila diecimila_isp: EFUSE = FC # 04
else
diecimila diecimila_isp: BAUD = 19200
diecimila diecimila_isp: LDSECTION  = 0x3800
diecimila diecimila_isp: EFUSE = 00
endif
diecimila diecimila_isp: LFUSE = FF
diecimila diecimila_isp: HFUSE = DD

# Arduino w/ ATmega168@16MHz w/o auto-reset 
ng ng_isp: TARGET = ng
ng ng_isp: MCU_TARGET = atmega168
ng ng_isp: AVR_FREQ = 16000000L
ng ng_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=3' 
ng ng_isp: CFLAGS += '-DLED_B=5' '-DLED_P=B'
ifeq ($(PROGRAM),optiboot)
ng ng_isp: BAUD = $(OPTI_BAUD)
ng ng_isp: LDSECTION  = 0x3e00
ng ng_isp: EFUSE = FC # 04
else
ng ng_isp: BAUD = 19200
ng ng_isp: LDSECTION  = 0x3800
ng ng_isp: EFUSE = 00
endif
ng ng_isp: LFUSE = FF
ng ng_isp: HFUSE = DD

# Arduino w/ ATmega8@16MHz w/o auto-reset 
ng8 ng8_isp: TARGET = ng8
ng8 ng8_isp: MCU_TARGET = atmega8
ng8 ng8_isp: AVR_FREQ = 16000000L
ng8 ng8_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=3' 
ng8 ng8_isp: CFLAGS += '-DLED_B=5' '-DLED_P=B'
ifeq ($(PROGRAM),optiboot)
ng8 ng8_isp: BAUD = $(OPTI_BAUD)
ng8 ng8_isp: LDSECTION  = 0x1e00
ng8 ng8_isp: HFUSE = DC
else
ng8 ng8_isp: BAUD = 19200
ng8 ng8_isp: LDSECTION  = 0x1800
ng8 ng8_isp: HFUSE = D8
endif
ng8 ng8_isp: LFUSE = BF

# Arduino w/ ATmega8@16MHz w/o auto-reset 
ng88 ng88_isp: TARGET = ng88
ng88 ng88_isp: MCU_TARGET = atmega8
ng88 ng88_isp: AVR_FREQ = 16000000L
ng88 ng88_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=3'
ng88 ng88_isp: CFLAGS += '-DLED_B=5' '-DLED_P=B'
ifeq ($(PROGRAM),optiboot)
ng88 ng88_isp: BAUD = $(OPTI_BAUD)
ng88 ng88_isp: LDSECTION  = 0x1e00
ng88 ng88_isp: EFUSE = FC # 04
else
ng88 ng88_isp: BAUD = 19200
ng88 ng88_isp: LDSECTION  = 0x1800
ng88 ng88_isp: EFUSE = F8 # 00
endif
ng88 ng88_isp: LFUSE = FF
ng88 ng88_isp: HFUSE = DD

# Arduino w/ ATmega168@8MHz w/o auto-reset 
lilypad lilypad_isp: TARGET = lilypad
lilypad lilypad_isp: MCU_TARGET = atmega168
lilypad lilypad_isp: AVR_FREQ = 8000000L
lilypad lilypad_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=3' 
lilypad lilypad_isp: CFLAGS += '-DLED_B=5' '-DLED_P=B'
ifeq ($(PROGRAM),optiboot)
lilypad lilypad_isp: BAUD = $(OPTI_BAUD)
lilypad lilypad_isp: LDSECTION  = 0x3e00
lilypad lilypad_isp: EFUSE = FC # 04
else
lilypad lilypad_isp: BAUD = 19200
lilypad lilypad_isp: LDSECTION  = 0x3800
lilypad lilypad_isp: EFUSE = 00
endif
lilypad lilypad_isp: LFUSE = E2
lilypad lilypad_isp: HFUSE = DD

# Arduino w/ ATmega328@16MHz auto-reset 
duemilanove duemilanove_isp: TARGET = duemilanove
duemilanove duemilanove_isp: MCU_TARGET = atmega328p
duemilanove duemilanove_isp: AVR_FREQ = 16000000L 
duemilanove duemilanove_isp: CFLAGS+= '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1' 
duemilanove duemilanove_isp: CFLAGS += '-DLED_B=5' '-DLED_P=B'
ifeq ($(PROGRAM),optiboot)
duemilanove duemilanove_isp: BAUD = $(OPTI_BAUD)
duemilanove duemilanove_isp: LDSECTION  = 0x7e00
duemilanove duemilanove_isp: HFUSE = DE
else
duemilanove duemilanove_isp: BAUD = 57600
duemilanove duemilanove_isp: LDSECTION  = 0x7800
duemilanove duemilanove_isp: HFUSE = DA
endif
duemilanove duemilanove_isp: LFUSE = FF
duemilanove duemilanove_isp: EFUSE = FD # 05

# Arduino w/ ATmega1280@16MHz auto-reset 
mega mega_isp: TARGET = mega
mega mega_isp: MCU_TARGET = atmega1280
mega mega_isp: AVR_FREQ = 16000000L 
mega mega_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=0' '-DBIGBOOT'
mega mega_isp: CFLAGS += '-DLED_B=7' '-DLED_P=B' '-DUART_P=E'
ifeq ($(PROGRAM),optiboot)
mega mega_isp: BAUD = $(OPTI_BAUD)
mega mega_isp: LDSECTION  = 0x1Fc00
mega mega_isp: HFUSE = DE
else
mega mega_isp: BAUD = 57600
mega mega_isp: LDSECTION  = 0x1F000
mega mega_isp: HFUSE = DA
endif
mega mega_isp: LFUSE = FF
mega mega_isp: EFUSE = F5

### AVR-Net-IO w/ ATmega32@16MHz auto-reset
anio32 anio32_isp: TARGET = anio32
anio32 anio32_isp: MCU_TARGET = atmega32
anio32 anio32_isp: AVR_FREQ = 16000000L 
anio32 anio32_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
# at D2 is LED1 on the AddOn board 
anio32 anio32_isp: CFLAGS += '-DLED_B=2' '-DLED_P=D'
ifeq ($(PROGRAM),optiboot)
anio32 anio32_isp: BAUD = $(OPTI_BAUD)
anio32 anio32_isp: LDSECTION  = 0x7e00
anio32 anio32_isp: HFUSE = DE
else
anio32 anio32_isp: BAUD = 57600 
anio32 anio32_isp: LDSECTION  = 0x7800
anio32 anio32_isp: HFUSE = DA
endif
anio32 anio32_isp: LFUSE = BF

### AVR-Net-IO w/ ATmega644@16MHz auto-reset
anio644 anio644_isp: TARGET = anio644
anio644 anio644_isp: MCU_TARGET = atmega644
anio644 anio644_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
# at D2 is LED1 on the AddOn board 
anio644 anio644_isp: CFLAGS +=  '-DLED_B=2' '-DLED_P=D' -DBIG_BOOT
anio644 anio644_isp: AVR_FREQ = 16000000L 
ifeq ($(PROGRAM),optiboot)
anio644 anio644_isp: BAUD = $(OPTI_BAUD)
anio644 anio644_isp: LDSECTION  = 0xfc00
anio644 anio644_isp: HFUSE = DE
else
anio644 anio644_isp: BAUD = 57600
anio644 anio644_isp: LDSECTION  = 0xF800
anio644 anio644_isp: HFUSE = DC
endif
anio644 anio644_isp: LFUSE = FF
anio644 anio644_isp: EFUSE = FD # 05

### AVR-Net-IO w/ ATmega644p@16MHz auto-reset
anio644p anio644p_isp: TARGET = anio644p
anio644p anio644p_isp: MCU_TARGET = atmega644p
anio644p anio644p_isp: AVR_FREQ = 16000000L 
anio644p anio644p_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
# at D2 is LED1 on the AddOn board 
#'-DLED=PIND2' '-DLED_DDR=DDRD' '-DLED_PORT=PORTD' '-DLED_PGM=PIND2'
anio644p anio644p_isp: CFLAGS +=  '-DLED_B=2' '-DLED_P=D' -DBIG_BOOT
ifeq ($(PROGRAM),optiboot)
anio644p anio644p_isp: BAUD = $(OPTI_BAUD)
anio644p anio644p_isp: LDSECTION  = 0xfc00
anio644p anio644p_isp: HFUSE = DE
else
anio644p anio644p_isp: BAUD = 57600
anio644p anio644p_isp: LDSECTION  = 0xF800
anio644p anio644p_isp: HFUSE = DC
endif
anio644p anio644p_isp: LFUSE = FF
anio644p anio644p_isp: EFUSE = FD # 05

$(ALL):%:$(PROGRAM)_%.hex $(PROGRAM)_%.boards.txt
$(ALL:%=%_isp):%_isp: % isp

isp: $(TARGET)
	$(ISPFUSES)
	$(ISPFLASH)

isp-stk500: $(PROGRAM)_$(TARGET).hex
	$(STK500-1)
	$(STK500-2)

%.elf: $(SRC)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)
	avr-size $@

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex *boards.txt *~

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

reverse = $(if $(1),$(call reverse,$(wordlist 2,$(words $(1)),$(1)))) $(firstword $(1))
ldir = $(subst /, ,$(PWD))
rdir = $(call reverse,$(ldir))
bldr = $(word 1,$(rdir))
core = $(word 3,$(rdir))

%.boards.txt: %.hex
	echo "##################################################" > $@
	echo "# $(TARGET): $(core) /w $(MCU_TARGET)" >> $@
	echo "##################################################" >> $@
	echo "$(TARGET).name=$(core) w/ $(MCU_TARGET)" >> $@
	echo "$(TARGET).build.mcu=$(MCU_TARGET)" >> $@
	echo "$(TARGET).build.f_cpu=$(AVR_FREQ)" >> $@
	echo "$(TARGET).build.core=$(core)" >> $@
	echo "$(TARGET).upload.protocol=stk500" >> $@
	echo -n "$(TARGET).upload.maximum_size=" >> $@
	$(AWK) 'BEGIN {print $(LDSECTION);}' >> $@
	echo "$(TARGET).upload.speed=$(BAUD)" >> $@
	echo "$(TARGET).bootloader.path=$(bldr)" >> $@
	echo "$(TARGET).bootloader.file=$<" >> $@
	avr-size $(<:%.hex=%.elf) | $(AWK) '{print "# " $$0;}' >> $@
	echo "$(TARGET).bootloader.low_fuses=0x$(LFUSE)" >> $@
	echo "$(TARGET).bootloader.high_fuses=0x$(HFUSE)" >> $@
	echo $(if $(EFUSE),,'#') "$(TARGET).bootloader.extended_fuses=0x$(EFUSE)" >> $@
	echo "$(TARGET).bootloader.unlock_bits=0x$(ULOCK)" >> $@
	echo "$(TARGET).bootloader.lock_bits=0x$(LOCK)" >> $@
	echo "##################################################" >> $@

BOARDS = $(wildcard *.boards.txt)
boards.txt: $(BOARDS)
ifeq (,$(BOARDS))
	echo run make '<TARGET>' first
else
	cat $^ > $@
endif