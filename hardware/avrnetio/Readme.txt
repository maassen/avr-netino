AVR-Netino - Arduino port for AVR-Net-IO from Pollin
============================================================

Description
--------------------
This project summarizes some patches to the arduino core and boot-loader to get 
the AVR-Net-IO board programmable by the arduino IDE. Additional included are 
some patched libraries to support the io functions of the board (network,
LCD,RFM12,...) 

The philosophy behind my changes is to support many avr's in the core files and
to concentrate the board specific details in one file. So porting to other 
hardware should be very easy.

Getting started
--------------------
You need:
1) AVR-Net-IO board (Art.Nr. 810 058 or 810 073) from Pollin [2] with 
   Auto-Reset feature (see Hardware mods below)
2) arduino-0022 (or later) from [3] 
3) avr isp programmer to burn the boot-loader

Then install this software in the arduino installation directory or in
your sketchbook directory. You'll get hardware/avrnetio with the core and
boot-loaders. In libraries you'll get patched versions of some libs by 
overwriting (!) the original ones.

If you install from the repository (not the zip files), you must link/copy/move
board_avrnetio.def to board.def in hardware/avrnetio/cores/avrnetio. 
To compile the boot-loader in hardware/avrnetio/bootloaders/atmega 
make atmega32 atmega644p 

Now start your Arduino IDE. You should find two new entries
"AVR-NET-IO w/ ATmega32" and "AVR-NET-IO w/ ATmega32" in Tools->Board.
Select the one you have. Connect your isp programmer to the board and
select Tools->Burn Bootlader->[your ISP] to burn the boot-loader.

Open an example, connect the serial port to your
computer and download the sketch - Have Fun.

Pinning
--------------------
The arduino pin numbers are as follow
D0 ..D11 are on SubD25 (J3)
D12..D15 are on screw connectors (ADC 1 - ADC 4)
D16..D17 are RS232 
D18..D24,D26 are on EXT. connector

Hardware mods
--------------------
The only thing you have to modify is to add the auto reset functionality
to your board. Otherwise it will be very difficult to enter the boot-loader.

To do so, you just have to solder a capacitor of 100nF between the RS232 DTR
signal (J5 Pin4) and the AVR Reset signal (ATmega Pin9).
I've sometimes observed, that the ATmega hangs after reset. I could figure out
that if came from a to high voltage at the reset pin and the ATmega was entering
high voltage programming mode. As a solution I added a diode (ie. 1N4148) from
reset to Vcc. After that the auto reset was very reliable.
The circuit looks like:

DTR o-----||-----+-----|>|-----o Vcc 
        100nF    |    1N4148
                 |
                 o Reset

Licence
--------------------
This Project contains mostly (modified) files from other projects with 
different OSS-Licences. So please look at the files to find the licence.
File which are entirely written by me are published under the
Common Development and Distribution License (CDDL) version 1 [6]
and if you like the GNU General Public License (GPLv2) version 2 [7]

Author
--------------------
Most of the files are imported from other projects, and so this files
are from many different authors (look in the files for details).
The porting to the AVR-Net-IO board are done by me, 
Michael Maassen <mic.maassen@gmail.com>

Links
--------------------
[1]	http://code.google.com/p/avr-netino/
[2]	http://www.pollin.de
[3]	http://arduino.cc
[4]	http://jeelabs.net/projects/cafe/wiki/Libraries
[5]	http://www.xs4all.nl/~hmario/arduino/LiquidCrystal_I2C/
[6]	http://www.opensource.org/licenses/cddl1.php
[7]	http://www.opensource.org/licenses/gpl-2.0

