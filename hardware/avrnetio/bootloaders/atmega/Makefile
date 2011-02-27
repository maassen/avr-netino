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

# program name should not be changed...
#PROGRAM    = ATmegaBOOT_168
#PROGRAM	   = optiboot
PROGRAM    = $(basename $(wildcard *.c))

PROTOCOL   = stk500

# enter the parameters for the avrdude isp tool
ISPTOOL	   = stk500v2
ISPTOOL	   = avrispmkII
#ISPTOOL	   = usbasp
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
OBJ        = 
#$(PROGRAM).o

DEFS       = '-DBOOT_SEC=$(LDSECTION)'
LIBS       =

ifneq ($(windir),)
CC         = ..\..\..\tools\avr\bin\avr-gcc
OBJSIZE    = ..\..\..\tools\avr\bin\avr-size
OBJCOPY    = ..\..\..\tools\avr\bin\avr-objcopy
OBJDUMP    = ..\..\..\tools\avr\bin\avr-objdump
AWK        = ..\..\..\tools\avr\utils\bin\gawk
else
CC         = avr-gcc
OBJSIZE    = avr-size
OBJCOPY    = avr-objcopy
OBJDUMP    = avr-objdump
AWK        = awk
endif

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) -DBAUD_RATE=$(BAUD) $(DEFS)

ifeq ($(PROGRAM),optiboot)
OPTI_BAUD  = 115200
OPTIMIZE = -Os -fno-inline-small-functions -fno-split-wide-types -mshort-calls
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


ANIO = anio32 anio644 anio644p anio1284p anio16 anio164p anio324p
ARDUINOx8 = duemilanove diecimila ng ng8 ng88 lilypad
ATTINY = luminet
ALL = $(ANIO) $(ARDUINOx8) $(ATTINY) mega 
anio: $(ANIO)
all: $(ALL)

#all:
#	for i in $(ALL) ; do $(MAKE) $$i ; done

############################################################
# Arduino w/ ATmega*8
############################################################
# Arduino w/ ATmega168@16MHz auto-reset 
diecimila diecimila_isp: TARGET = diecimila
diecimila diecimila_isp: MCU_TARGET = atmega168
diecimila diecimila_isp: AVR_FREQ = 16000000L 
diecimila diecimila_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
diecimila diecimila_isp: CFLAGS +=  '-DLED_B=5' '-DLED_P=B'
diecimila diecimila_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),19200)
ifeq ($(PROGRAM),optiboot)
diecimila diecimila_isp: LDSECTION  = 0x3e00
diecimila diecimila_isp: EFUSE = FC # 04
else
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
ng8 ng8_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=3' 
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

# Arduino w/ ATmega8@16MHz w/o auto-reset 
ng88 ng88_isp: TARGET = ng88
ng88 ng88_isp: MCU_TARGET = atmega8
ng88 ng88_isp: AVR_FREQ = 16000000L
ng88 ng88_isp: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=3'
ng88 ng88_isp: CFLAGS += '-DLED_B=5' '-DLED_P=B'
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

# Arduino w/ ATmega168@8MHz w/o auto-reset 
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

# Arduino w/ ATmega328@16MHz auto-reset 
duemilanove duemilanove_isp: TARGET = duemilanove
duemilanove duemilanove_isp: MCU_TARGET = atmega328p
duemilanove duemilanove_isp: AVR_FREQ = 16000000L 
duemilanove duemilanove_isp: CFLAGS+= '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1' 
duemilanove duemilanove_isp: CFLAGS += '-DLED_B=5' '-DLED_P=B'
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

############################################################
# Arduino Megas
############################################################
# Arduino w/ ATmega1280@16MHz auto-reset 
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
CFLAGS_ANIO  = '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
#CFLAGS_ANIO += '-DLED_B=1' '-DLED_P=B' # led at BL jumper
CFLAGS_ANIO += '-DLED_B=6' '-DLED_P=D' # LED3
#CFLAGS_ANIO += '-DBL_B=1' '-DBL_P=B' # BL JMP
LDFLAGS_ANIO =
### AVR-Net-IO w/ ATmega32@16MHz auto-reset
anio32 anio32_isp: TARGET = anio32
anio32 anio32_isp: MCU_TARGET = atmega32
anio32 anio32_isp: AVR_FREQ = 16000000L 
anio32 anio32_isp: CFLAGS += $(CFLAGS_ANIO)
anio32 anio32_isp: LDFLAGS += $(LDFLAGS_ANIO)
anio32 anio32_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
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
anio16 anio16_isp: AVR_FREQ = 16000000L 
anio16 anio16_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
anio16 anio16_isp: CFLAGS += $(CFLAGS_ANIO)
anio16 anio16_isp: LDFLAGS += $(LDFLAGS_ANIO)
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
anio164p anio164p_isp: AVR_FREQ = 16000000L 
anio164p anio164p_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
anio164p anio164p_isp: CFLAGS += $(CFLAGS_ANIO)
anio164p anio164p_isp: LDFLAGS += $(LDFLAGS_ANIO)
ifeq ($(PROGRAM),optiboot)
anio164p anio164p_isp: LDSECTION  = 0x3e00
anio164p anio164p_isp: HFUSE = DC
else
anio164p anio164p_isp: LDSECTION  = 0x3800
anio164p anio164p_isp: HFUSE = D8
endif
anio164p anio164p_isp: LFUSE = FF
anio164p anio164p_isp: EFUSE = FD # 05

### AVR-Net-IO w/ ATmega324p@16MHz auto-reset
anio324p anio324p_isp: TARGET = anio324p
anio324p anio324p_isp: MCU_TARGET = atmega324p
anio324p anio324p_isp: AVR_FREQ = 16000000L 
anio324p anio324p_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
anio324p anio324p_isp: CFLAGS += $(CFLAGS_ANIO)
anio324p anio324p_isp: LDFLAGS += $(LDFLAGS_ANIO)
ifeq ($(PROGRAM),optiboot)
anio324p anio324p_isp: LDSECTION  = 0x7e00
anio324p anio324p_isp: HFUSE = DE
else
anio324p anio324p_isp: LDSECTION  = 0x7800
anio324p anio324p_isp: HFUSE = DA
endif
anio324p anio324p_isp: LFUSE = FF
anio324p anio324p_isp: EFUSE = FD # 05

### AVR-Net-IO w/ ATmega644@16MHz auto-reset
anio644 anio644_isp: TARGET = anio644
anio644 anio644_isp: MCU_TARGET = atmega644
anio644 anio644_isp: AVR_FREQ = 16000000L 
anio644 anio644_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
anio644 anio644_isp: CFLAGS += $(CFLAGS_ANIO)
anio644 anio644_isp: LDFLAGS += $(LDFLAGS_ANIO)
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
anio644p anio644p_isp: AVR_FREQ = 16000000L 
anio644p anio644p_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
anio644p anio644p_isp: CFLAGS += $(CFLAGS_ANIO)
anio644p anio644p_isp: LDFLAGS += $(LDFLAGS_ANIO)
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
anio1284p anio1284p_isp: AVR_FREQ = 16000000L 
anio1284p anio1284p_isp: BAUD = $(if $(OPTI_BAUD),$(OPTI_BAUD),57600)
anio1284p anio1284p_isp: CFLAGS += $(CFLAGS_ANIO)
anio1284p anio1284p_isp: LDFLAGS += $(LDFLAGS_ANIO)
ifeq ($(PROGRAM),optiboot)
anio1284p anio1284p_isp: CFLAGS += -DBIG_BOOT 
anio1284p anio1284p_isp: LDSECTION  = 0x1fc00
anio1284p anio1284p_isp: HFUSE = DE
else
anio1284p anio1284p_isp: CFLAGS += '-DHAVE_EEPROM_PAGED_ACCESS=0' -Os
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
# machine room
############################################################
$(ALL):%:$(PROGRAM)_%.hex $(PROGRAM)_%.boards.txt
$(ALL:%=%_isp):%_isp: % isp

isp: $(TARGET)
	$(ISPFUSES)
	$(ISPFLASH)

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
	@echo "##################################################" > $@
	@echo "# $(TARGET): $(core) /w $(MCU_TARGET) ($(SRC))" >> $@
	@echo "##################################################" >> $@
	@echo "$(TARGET).name=$(core) w/ $(MCU_TARGET) ($(SRC))" >> $@
	@echo "$(TARGET).build.mcu=$(MCU_TARGET)" >> $@
	@echo "$(TARGET).build.f_cpu=$(AVR_FREQ)" >> $@
	@echo "$(TARGET).build.core=$(core)" >> $@
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
	@echo "##################################################" >> $@

BOARDS = $(wildcard *.boards.txt)
boards.txt: $(BOARDS)
ifeq (,$(BOARDS))
	echo run make '<TARGET>' first
else
	cat $^ > $@
endif