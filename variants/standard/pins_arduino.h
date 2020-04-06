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

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
//#include <avr/iom328p.h>  // just for better parsing in Cevelop

#define NUM_DIGITAL_PINS            20
#define NUM_ANALOG_INPUTS           6
#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 14 : -1)
// analogInputToDigitalPin does not seem to be used

#ifndef UsePetersCpp17
#define NO_PORT 0 // PS, to indicate there is no port for pin to port mapping
typedef uint8_t PortType;

#if defined(__AVR_ATmega8__)
#define digitalPinHasPWM(p)         ((p) == 9 || (p) == 10 || (p) == 11)
#else
#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11)
#endif


#define PIN_SPI_SS    (10)
#define PIN_SPI_MOSI  (11)
#define PIN_SPI_MISO  (12)
#define PIN_SPI_SCK   (13)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_WIRE_SDA        (18)
#define PIN_WIRE_SCL        (19)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define LED_BUILTIN 13

#define PIN_A0   (14)
#define PIN_A1   (15)
#define PIN_A2   (16)
#define PIN_A3   (17)
#define PIN_A4   (18)
#define PIN_A5   (19)
#define PIN_A6   (20)
#define PIN_A7   (21)

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 2 : (((p) <= 13) ? 0 : 1))
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK2) : (((p) <= 13) ? (&PCMSK0) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
#define digitalPinToPCMSKbit(p) (((p) <= 7) ? (p) : (((p) <= 13) ? ((p) - 8) : ((p) - 14)))

#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))

#ifdef ARDUINO_MAIN

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


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PD, /* 0 */
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PB, /* 8 */
	PB,
	PB,
	PB,
	PB,
	PB,
	PC, /* 14 */
	PC,
	PC,
	PC,
	PC,
	PC,
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(0), /* 0, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 8, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(0), /* 14, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER, /* 0 - port D */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	// on the ATmega168, digital pin 3 has hardware pwm
#if defined(__AVR_ATmega8__)
	NOT_ON_TIMER,
#else
	TIMER2B,
#endif
	NOT_ON_TIMER,
	// on the ATmega168, digital pins 5 and 6 have hardware pwm
#if defined(__AVR_ATmega8__)
	NOT_ON_TIMER,
	NOT_ON_TIMER,
#else
	TIMER0B,
	TIMER0A,
#endif
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 8 - port B */
	TIMER1A,
	TIMER1B,
#if defined(__AVR_ATmega8__)
	TIMER2,
#else
	TIMER2A,
#endif
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 14 - port C */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
};

#endif
#else


//#define PIN_SPI_SS    (10)
//#define PIN_SPI_MOSI  (11)
//#define PIN_SPI_MISO  (12)
//#define PIN_SPI_SCK   (13)

//constexpr inline uint8_t SS   = PIN_SPI_SS;
//constexpr inline uint8_t MOSI = PIN_SPI_MOSI;
//constexpr inline uint8_t MISO = PIN_SPI_MISO;
//constexpr inline uint8_t SCK  = PIN_SPI_SCK;
//
//#define PIN_WIRE_SDA        (18)
//#define PIN_WIRE_SCL        (19)
//
//constexpr inline uint8_t SDA = PIN_WIRE_SDA;
//constexpr inline uint8_t SCL = PIN_WIRE_SCL;

#ifndef UsePetersCpp17
#define LED_BUILTIN 13

#define PIN_A0   (14)
#define PIN_A1   (15)
#define PIN_A2   (16)
#define PIN_A3   (17)
#define PIN_A4   (18)
#define PIN_A5   (19)
#define PIN_A6   (20)
#define PIN_A7   (21)

constexpr inline uint8_t A0 = PIN_A0;
constexpr inline uint8_t A1 = PIN_A1;
constexpr inline uint8_t A2 = PIN_A2;
constexpr inline uint8_t A3 = PIN_A3;
constexpr inline uint8_t A4 = PIN_A4;
constexpr inline uint8_t A5 = PIN_A5;
constexpr inline uint8_t A6 = PIN_A6;
constexpr inline uint8_t A7 = PIN_A7;
#endif

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


// standard arduinos have 3 ports
// Arduino.h defines plenty of port macros if ARDUINO_MAIN is defined. shouldn't?
#ifdef PB
#undef PA
#undef PB
#undef PC
#undef PD
#undef PE
#undef PF
#undef PG
#undef PH
#undef PJ
#undef PK
#undef PL
#endif

enum class PortType: uint8_t {
	No_Port=0, PB=2, PC=3, PD=4
};
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


constexpr inline PortType digital_pin_to_Port_PS(uint8_t const pin)  {
	if (static_cast<PinType>(pin) < PinType::D08) return PortType::PD;
	else if (static_cast<PinType>(pin) < PinType::A0) return PortType::PB;
	else if (static_cast<PinType>(pin) <= PinType::A5) return PortType::PC;
	else return NO_PORT;
}
struct bitmask {
enum  bitmask_in_byte:uint8_t {
	b0=1,b1=2,b2=4,b3=8,b4=16,b5=32,b6=64,b7=128
};
static constexpr inline uint8_t digital_pin_to_BitMask_PS(uint8_t pin) noexcept {
	// benefit from regular pin bit mapping, saves space in ram, because all bad, section .rodata is copied into ram for switch statement tables
    if (pin > 13) pin -=14;
    else if (pin > 7) pin -= 8;
    return 1u << pin;

//	switch (static_cast<PinType>(pin)) {
//	case PinType::D00:	case PinType::D08:	case PinType::A0:
//		return bitmask_in_byte::b0;
//	case PinType::D01:	case PinType::D09:	case PinType::A1:
//		return bitmask_in_byte::b1;
//	case PinType::D02:	case PinType::D10:	case PinType::A2:
//		return bitmask_in_byte::b2;
//	case PinType::D03:	case PinType::D11:	case PinType::A3:
//		return bitmask_in_byte::b3;
//	case PinType::D04:	case PinType::D12:	case PinType::A4:
//		return bitmask_in_byte::b4;
//	case PinType::D05:	case PinType::D13:	case PinType::A5:
//		return bitmask_in_byte::b5;
//	case PinType::D06:
//		return bitmask_in_byte::b6;
//	case PinType::D07:
//		return bitmask_in_byte::b7;
//	}
//	return 0; // 0 might break code, may be, but is better indicator.
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




#endif

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
