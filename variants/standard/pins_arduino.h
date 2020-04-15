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
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/interrupt.h>

#define NUM_DIGITAL_PINS            20
#define NUM_ANALOG_INPUTS           6
#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 14 : -1)
// analogInputToDigitalPin does not seem to be used





#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 2 : (((p) <= 13) ? 0 : 1))
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK2) : (((p) <= 13) ? (&PCMSK0) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
#define digitalPinToPCMSKbit(p) (((p) <= 7) ? (p) : (((p) <= 13) ? ((p) - 8) : ((p) - 14)))

#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))


// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+
//
// (PWM+ indicates the additional PWM pins on the ATmega168.)

// ATMEL ATMEGA1280 / ARDUINO
//
// 0-7 PE0-PE7   works
// 8-13 PB0-PB5  works
// 14-21 PA0-PA7 works 
// 22-29 PH0-PH7 works
// 30-35 PG5-PG0 works
// 36-43 PC7-PC0 works
// 44-51 PJ7-PJ0 works
// 52-59 PL7-PL0 works
// 60-67 PD7-PD0 works
// A0-A7 PF0-PF7
// A8-A15 PK0-PK7


// standard Arduino boards have 3 ports PB, PC, PD
enum class PortType: uint8_t {
	No_Port=0, PB=2, PC=3, PD=4
};
// legacy support macro
#define NO_PORT PortType::No_Port

constexpr inline
volatile uint8_t *port_to_mode_PS(PortType port) noexcept {
	switch(port){
	case PortType::PB: return  &DDRB;
	case PortType::PC: return  &DDRC;
	case PortType::PD: return  &DDRD;
	default: return NOT_A_PORT; // nullptr
	}
}
constexpr inline
volatile uint8_t *port_to_output_PS(PortType port) noexcept {
	switch(port){
	case PortType::PB: return  &PORTB;
	case PortType::PC: return  &PORTC;
	case PortType::PD: return  &PORTD;
	default: return NOT_A_PORT; // nullptr
	}
}
constexpr inline
volatile uint8_t *port_to_input_PS(PortType port) noexcept {
	switch(port){
	case PortType::PB: return  &PINB;
	case PortType::PC: return  &PINC;
	case PortType::PD: return  &PIND;
	default: return NOT_A_PORT; // nullptr
	}
}
enum PinType:uint8_t {
	D00, D01, D02, D03, D04, D05, D06, D07, D08, D09, D10, D11, D12, D13,
	A0, A1, A2, A3, A4, A5,
	// short names for single digit pins
	D0=D00, D1=D01, D2=D02, D3=D03, D4=D04, D5=D05, D6=D06, D7=D07, D8=D08, D9=D09,
	SS=D10, MOSI=D11, MISO=D12, SCK=D12, // I2C
	SDA=A4, SCL=A5, // TWI
	LED_BUILTIN=D13
};
template <PinType ...pins>
constexpr inline bool isOneOfPins(uint8_t pin)  {
	return (1UL << pin) & ((1UL << pins)|...); // faster and smaller than == || , but only if max 32 bits
}
constexpr inline bool digitalPinHasPWM(uint8_t p) {
#if defined(__AVR_ATmega8__)
//#define digitalPinHasPWM(p)         ((p) == 9 || (p) == 10 || (p) == 11)
	return isOneOfPins<D09,D10,D11>(p);
#else
	return isOneOfPins<D03,D05,D06,D09,D10,D11>(p);
//#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11)
#endif
}


constexpr inline PortType digital_pin_to_Port_PS(PinType const pin)  {
	if (static_cast<PinType>(pin) < PinType::D08) return PortType::PD;
	else if (static_cast<PinType>(pin) < PinType::A0) return PortType::PB;
	else if (static_cast<PinType>(pin) <= PinType::A5) return PortType::PC;
	else return NO_PORT;
}
struct bitmask {
enum  bitmask_in_byte:uint8_t {
	b0=1,b1=2,b2=4,b3=8,b4=16,b5=32,b6=64,b7=128
};
static constexpr inline uint8_t digital_pin_to_BitMask_PS(PinType pin) noexcept {
	// benefit from regular pin bit mapping, saves space in ram, because all bad, section .rodata is copied into ram for switch statement tables
    if (pin > D13) pin = PinType(pin-14);
    else if (pin > 7) pin = PinType(pin-8);
    return 1u << pin;
}
};
constexpr inline timer_values digital_pin_to_timer_PS(PinType const pin) noexcept {
#if defined(__AVR_ATmega8__)
	if (pin == PinType::D11)
		return TIMER2;//,		/* 11 */ -> 6
#else
	if (pin == PinType::D03) //        /* 3 */ -> 8
		return 	TIMER2B;
	if (pin ==  PinType::D05)
		return TIMER0B;//,		/* 5 */ -> 2
	if (pin ==  PinType::D06)
		return TIMER0A;//,		/* 6 */ -> 1
	if (pin ==  PinType::D11)
		return TIMER2A;//,		/* 11 */ -> 7
#endif
	if (pin ==  PinType::D09)
		return TIMER1A;//,		/* 9 */ -> 3
	if (pin ==  PinType::D10)
		return TIMER1B;//,		/* 10 */ -> 4
	return NOT_ON_TIMER;
}

#include "wiring_inline.h"

inline void analog_timer_turnoff(timer_values const theTimer)  {
	switch (theTimer) {
#if defined(__AVR_ATmega8__)
	case TIMER2: analog_timer_turnoff<TIMER2>(); break;
#else
	case TIMER2B: analog_timer_turnoff<TIMER2B>(); break;
	case TIMER0B: analog_timer_turnoff<TIMER0B>(); break;
	case TIMER0A: analog_timer_turnoff<TIMER0A>(); break;
	case TIMER2A: analog_timer_turnoff<TIMER2A>(); break;
#endif
	case TIMER1A: analog_timer_turnoff<TIMER1A>(); break;
	case TIMER1B: analog_timer_turnoff<TIMER1B>(); break;
	default:;
	}
}
inline void setPWMValue(timer_values const theTimer, int val)  {
	switch (theTimer) {
#if defined(__AVR_ATmega8__)
	case TIMER2: setPWMValue<TIMER2>(val); break;
#else
	case TIMER2B: setPWMValue<TIMER2B>(val); break;
	case TIMER0B: setPWMValue<TIMER0B>(val); break;
	case TIMER0A: setPWMValue<TIMER0A>(val); break;
	case TIMER2A: setPWMValue<TIMER2A>(val); break;
#endif
	case TIMER1A: setPWMValue<TIMER1A>(val); break;
	case TIMER1B: setPWMValue<TIMER1B>(val); break;
	default:;
	}
}

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif
