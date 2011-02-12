// Generalized decoder and relay for 868 MHz and 433 MHz OOK signals.
// 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: ookRelay2.pde 7083 2011-02-03 00:55:15Z jcw $

#include <Ports.h>
#include <RF12.h>
#include "decoders.h"

#define DEBUG 1     // set to 1 to also report results on the serial port
#define DEBUG_LED 0 // define as pin 4..7 to blink LED on each pin change

// RF12 communication settings
#define NODEID 19
#define NETGRP 5

// since we have two signal sources, we also need two lists of decoders

#define PIN_868 14  // AIO1
#define PIN_433 17  // AIO4

// 868 MHz
VisonicDecoder viso;
EMxDecoder emx;
KSxDecoder ksx;
FSxDecoder fsx;

DecoderInfo di_868[] = {
    { 1, "VISO", &viso },
    { 2, "EMX", &emx },
    { 3, "KSX", &ksx },
    { 4, "FSX", &fsx },
    { -1, 0, 0 }
};

// 433 MHz
OregonDecoder orsc;
CrestaDecoder cres;
KakuDecoder kaku;
XrfDecoder xrf;
HezDecoder hez;

DecoderInfo di_433[] = {
    { 5, "ORSC", &orsc },
    { 6, "CRES", &cres },
    { 7, "KAKU", &kaku },
    { 8, "XRF", &xrf },
    { 9, "HEZ", &hez },
    { -1, 0, 0 }
};

// Outgoing data buffer for RF12
byte packetBuffer [RF12_MAXDATA], packetFill;

// State to track pulse durations measured in the pin change interrupts
volatile word pulse_868, pulse_433;
word last_433, last_868; // never accessed outside ISR's

// Timer to only relay packets up to 10x per second, even if more come in.
MilliTimer sendTimer;

ISR(ANALOG_COMP_vect) {
    word now = micros();
    pulse_868 = now - last_868;
    last_868 = now;
}

ISR(PCINT1_vect) {
    word now = micros();
    pulse_433 = now - last_433;
    last_433 = now;
}

static void setupPinChangeInterrupt () {
    pinMode(PIN_433, INPUT);
    digitalWrite(PIN_433, 1);   // pull-up
    
    // interrupt on pin change
    bitSet(PCMSK1, PIN_433 - 14);
    bitSet(PCICR, PCIE1);

    pinMode(PIN_868, INPUT);
    digitalWrite(PIN_868, 1);   // pull-up
    
    // enable analog comparator with fixed voltage reference
    ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);
    ADCSRA &= ~ _BV(ADEN);
    ADCSRB |= _BV(ACME);
    ADMUX = PIN_868 - 14;
}

// Append a new data item to the outgoing packet buffer (if there is room
static void addToBuffer (DecoderInfo& di) {
    byte size;
    const byte* data = di.decoder->getData(size);

#if DEBUG
    Serial.print(di.name);
    for (byte i = 0; i < size; ++i) {
        Serial.print(' ');
        Serial.print((int) data[i]);
    }
    // Serial.print(' ');
    // Serial.print(millis() / 1000);
    Serial.println();
#endif

    if (packetFill + size < sizeof packetBuffer) {
        packetBuffer[packetFill++] = di.typecode + (size << 4);
        memcpy(packetBuffer + packetFill, data, size);
        packetFill += size;
    } else {
#if DEBUG
        Serial.print(" dropped: type ");
        Serial.print((int) di.typecode);
        Serial.print(", ");
        Serial.print((int) size);
        Serial.println(" bytes");
#endif        
    }
    
    di.decoder->resetDecoder();
}

// Check for a new pulse and run the corresponding decoders for it
static void runPulseDecoders (DecoderInfo* pdi, volatile word& pulse) {
    // get next pulse with and reset it - need to protect against interrupts
    cli();
    word p = pulse;
    pulse = 0;
    sei();

    // if we had a pulse, go through each of the decoders
    if (p != 0) { 
        bitSet(PORTD, DEBUG_LED);
        while (pdi->typecode >= 0) {
            if (pdi->decoder->nextPulse(p))
                addToBuffer(*pdi);
            ++pdi;
        }
        bitClear(PORTD, DEBUG_LED); 
    }
}

// see http://jeelabs.org/2011/01/27/ook-reception-with-rfm12b-2/
static void rf12_init_OOK () 
{
    rf12_control(0x8027); // 8027    868 Mhz;disabel tx register; disable RX
                          //         fifo buffer; xtal cap 12pf, same as xmitter
    rf12_control(0x82c0); // 82C0    enable receiver; enable basebandblock 
    rf12_control(0xA68a); // A68A    868.2500 MHz
    rf12_control(0xc691); // C691    c691 datarate 2395 kbps 0xc647 = 4.8kbps 
    rf12_control(0x9489); // 9489    VDI; FAST;200khz;GAIn -6db; DRSSI 97dbm 
    rf12_control(0xC220); // C220    datafiltercommand; ** not documented cmd 
    rf12_control(0xCA00); // CA00    FiFo and resetmode cmd; FIFO fill disabeld
    rf12_control(0xC473); // C473    AFC run only once; enable AFC; enable
                          //         frequency offset register; +3 -4
    rf12_control(0xCC67); // CC67    pll settings command
    rf12_control(0xB800); // TX register write command not used
    rf12_control(0xC800); // disable low dutycycle 
    rf12_control(0xC040); // 1.66MHz,2.2V not used see 82c0  
}

void setup () {
#if DEBUG_LED    
    bitSet(DDRD, DEBUG_LED);
    // brief LED flash on startup to make sure it works
    bitSet(PORTD, DEBUG_LED);
    delay(100);
    bitClear(PORTD, DEBUG_LED);
#endif

#if DEBUG
    Serial.begin(57600);
    Serial.println("\n[ookRelay2]");
#endif

    rf12_initialize(0, RF12_868MHZ);
    rf12_init_OOK();

    setupPinChangeInterrupt();
}

void loop () {
    runPulseDecoders(di_868, pulse_868);    
    runPulseDecoders(di_433, pulse_433);    
    
    if (sendTimer.poll(100) && packetFill > 0) {
        rf12_initialize(NODEID, RF12_868MHZ, NETGRP);
        while (!rf12_canSend())
            rf12_recvDone(); // ignores incoming
        rf12_sendStart(0, packetBuffer, packetFill, 1);
        rf12_init_OOK();
        packetFill = 0;
    }
}
