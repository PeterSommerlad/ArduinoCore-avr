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
constexpr inline bool isOneOfPins(uint8_t pin) noexcept {
	return (1UL << pin) & ((1UL << pins)|...); // faster and smaller than == || , but only if max 32 bits
}
constexpr inline bool digitalPinHasPWM(uint8_t p)noexcept {
#if defined(__AVR_ATmega8__)
//#define digitalPinHasPWM(p)         ((p) == 9 || (p) == 10 || (p) == 11)
	return isOneOfPins<D09,D10,D11>(p);
#else
	return isOneOfPins<D03,D05,D06,D09,D10,D11>(p);
//#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11)
#endif
}


constexpr inline PortType digital_pin_to_Port_PS(uint8_t const pin) noexcept {
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


constexpr inline timer_values digital_pin_to_timer_PS(uint8_t const pin) noexcept {
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

//
//	switch (static_cast<PinType>(pin)) {
//#if defined(__AVR_ATmega8__)
//	case PinType::D11:
//		return TIMER2;//,		/* 11 */ -> 6
//#else
//	case PinType::D03://        /* 3 */ -> 8
//		return 	TIMER2B;
//	case PinType::D05:
//		return TIMER0B;//,		/* 5 */ -> 2
//	case PinType::D06:
//		return TIMER0A;//,		/* 6 */ -> 1
//	case PinType::D11:
//		return TIMER2A;//,		/* 11 */ -> 7
//#endif
//	case PinType::D09:
//		return TIMER1A;//,		/* 9 */ -> 3
//	case PinType::D10:
//		return TIMER1B;//,		/* 10 */ -> 4
//	default:
//		return NOT_ON_TIMER;
//	}
}

//*************** try inline digitalWrite/Read and see what code is generated....
struct port_bit {
	PortType port;
	uint8_t bit;
};
constexpr inline
port_bit digital_pin_to_port_bit(uint8_t const pin) noexcept {
	return {digital_pin_to_Port_PS(pin),bitmask::digital_pin_to_BitMask_PS(pin)};
}
//struct SafeStatusRegisterAndClearInterrupt{
//	uint8_t oldSREG;
//	SafeStatusRegisterAndClearInterrupt()
//	:oldSREG{SREG} {
//		cli();
//	}
//	~SafeStatusRegisterAndClearInterrupt(){
//		SREG = oldSREG;
//	}
//};
#include <avr/io.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define CBI_SET
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define SBI_SET
#endif

struct timer_info {
	volatile uint8_t * tccr;
	volatile void * ocr; // oh shit, 16 or 8 bit pointer.... need to put more smartness here
	uint8_t          combit;
	bool	is16bit;
	void setValue(int val) const {
		if (is16bit)
			*reinterpret_cast<volatile uint16_t *>(ocr) = val;
		else
			*reinterpret_cast<volatile uint8_t *>(ocr) = val;
	}
	constexpr static timer_info make(volatile uint8_t * tccr_, volatile uint8_t *ocr_, uint8_t combit_){
		return timer_info{tccr_,ocr_,combit_,false};
	}
	constexpr static timer_info make(volatile uint8_t * tccr_, volatile uint16_t *ocr_, uint8_t combit_){
			return timer_info{tccr_,ocr_,combit_,true};
	}
};
struct timer_OCR_info {
	volatile void * const ocr; // oh shit, 16 or 8 bit pointer.... need to put more smartness here
	bool	const is16bit;
	void setValue(int val) const {
		if (is16bit)
			*reinterpret_cast<volatile uint16_t *>(ocr) = val;
		else
			*reinterpret_cast<volatile uint8_t *>(ocr) = val;
	}
	constexpr static timer_OCR_info make(volatile uint8_t *ocr_){
		return timer_OCR_info{ocr_,false};
	}
	constexpr static timer_OCR_info make(volatile uint16_t *ocr_){
			return timer_OCR_info{ocr_,true};
	}
};
constexpr
inline volatile uint8_t * digital_pin_to_timer_tccr(uint8_t const pin) noexcept {
	// values taken from wiring_analog.c
#if defined(__AVR_ATmega8__)
	if (pin == PinType::D11)
		return  &TCCR2;//TIMER2;//,		/* 11 */ -> 6 // OCR2 always 8bit
#else
	if (pin == PinType::D03) //        /* 3 */ -> 8
		return &TCCR2A;//TIMER2B; // OCR2B 8 or 16
	if (pin ==  PinType::D05)
		return &TCCR0A;//TIMER0B;//,		/* 5 */ -> 2 // OCR0B 8bit except some tiny iotn[4,5,9,10].h
	if (pin ==  PinType::D06){
#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
		return &TCCR0;//TIMER0A;//,		/* 6 */ -> 1 // OCR0 8bit
#elif defined(TCCR0A) && defined(COM0A1)
		return &TCCR0A;//TIMER0A;//,		/* 6 */ -> 1 // OCR0A 8bit except some tiny like OCR0B
#endif
	}
	if (pin ==  PinType::D11)
		return &TCCR2A;//TIMER2A;//,		/* 11 */ -> 7 // OCR2A 8bit, except iotn441 iotn841
#endif
	if (pin ==  PinType::D09)
		return  &TCCR1A;//TIMER1A;//,		/* 9 */ -> 3 // mostly 16 bit, except some mega hv, some tiny
	if (pin ==  PinType::D10)
		return  &TCCR1A;//TIMER1B;//,		/* 10 */ -> 4 // mostly 16 bit, except same as OCR1B
	return nullptr; // COMxxx bits are never 0
}

constexpr inline uint8_t digital_pin_to_timer_combits(uint8_t const pin) noexcept {
	// values taken from wiring_analog.c
#if defined(__AVR_ATmega8__)
	if (pin == PinType::D11)
		return  COM21 ;//TIMER2;//,		/* 11 */ -> 6 // OCR2 always 8bit
#else
	if (pin == PinType::D03) //        /* 3 */ -> 8
		return COM2B1 ;//TIMER2B; // OCR2B 8 or 16
	if (pin ==  PinType::D05)
		return COM0B1 ;//TIMER0B;//,		/* 5 */ -> 2 // OCR0B 8bit except some tiny iotn[4,5,9,10].h
	if (pin ==  PinType::D06){
#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
		return COM00 ;//TIMER0A;//,		/* 6 */ -> 1 // OCR0 8bit
#elif defined(TCCR0A) && defined(COM0A1)
		return COM0A1 ;//TIMER0A;//,		/* 6 */ -> 1 // OCR0A 8bit except some tiny like OCR0B
#endif
	}
	if (pin ==  PinType::D11)
		return COM2A1 ;//TIMER2A;//,		/* 11 */ -> 7 // OCR2A 8bit, except iotn441 iotn841
#endif
	if (pin ==  PinType::D09)
		return  COM1A1 ;//TIMER1A;//,		/* 9 */ -> 3 // mostly 16 bit, except some mega hv, some tiny
	if (pin ==  PinType::D10)
		return  COM1B1 ;//TIMER1B;//,		/* 10 */ -> 4 // mostly 16 bit, except same as OCR1B
	return 0; // COMxxx bits are never 0
}

inline void analog_pin_to_timer_assign(uint8_t const pin, int const val) noexcept {
	// values taken from wiring_analog.c
#if defined(__AVR_ATmega8__)
	if (pin == PinType::D11)
		 OCR2 = val;//TIMER2;//,		/* 11 */ -> 6 // OCR2 always 8bit
	else
#else
	if (pin == PinType::D03) //        /* 3 */ -> 8
		OCR2B = val;//TIMER2B; // OCR2B 8 or 16
	else if (pin ==  PinType::D05)
		OCR0B = val;//TIMER0B;//,		/* 5 */ -> 2 // OCR0B 8bit except some tiny iotn[4,5,9,10].h
	else if (pin ==  PinType::D06){
#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
		OCR0 = val;//TIMER0A;//,		/* 6 */ -> 1 // OCR0 8bit
#elif defined(TCCR0A) && defined(COM0A1)
		OCR0A=val;//TIMER0A;//,		/* 6 */ -> 1 // OCR0A 8bit except some tiny like OCR0B
#endif
	}
	else if (pin ==  PinType::D11)
		OCR2A = val;//TIMER2A;//,		/* 11 */ -> 7 // OCR2A 8bit, except iotn441 iotn841
#endif
	else if (pin ==  PinType::D09)
		OCR1A = val;//TIMER1A;//,		/* 9 */ -> 3 // mostly 16 bit, except some mega hv, some tiny
	else if (pin ==  PinType::D10)
		OCR1B = val;//TIMER1B;//,		/* 10 */ -> 4 // mostly 16 bit, except same as OCR1B
}

constexpr inline timer_info digital_pin_to_timer_info(uint8_t const pin) noexcept {
	// values taken from wiring_analog.c
#if defined(__AVR_ATmega8__)
	if (pin == PinType::D11)
		return { &TCCR2,  &OCR2,  COM21 };//TIMER2;//,		/* 11 */ -> 6 // OCR2 always 8bit
#else
	if (pin == PinType::D03) //        /* 3 */ -> 8
		return timer_info::make( &TCCR2A, &OCR2B, COM2B1 );//TIMER2B; // OCR2B 8 or 16
	if (pin ==  PinType::D05)
		return timer_info::make( &TCCR0A, &OCR0B, COM0B1 );//TIMER0B;//,		/* 5 */ -> 2 // OCR0B 8bit except some tiny iotn[4,5,9,10].h
	if (pin ==  PinType::D06){
#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
		return timer_info::make( &TCCR0,  &OCR0,  COM00  );//TIMER0A;//,		/* 6 */ -> 1 // OCR0 8bit
#elif defined(TCCR0A) && defined(COM0A1)
		return timer_info::make( &TCCR0A,  &OCR0A,  COM0A1  );//TIMER0A;//,		/* 6 */ -> 1 // OCR0A 8bit except some tiny like OCR0B
#endif
	}
	if (pin ==  PinType::D11)
		return timer_info::make( &TCCR2A, &OCR2A, COM2A1 );//TIMER2A;//,		/* 11 */ -> 7 // OCR2A 8bit, except iotn441 iotn841
#endif
	if (pin ==  PinType::D09)
		return timer_info::make( &TCCR1A, &OCR1A, COM1A1 );//TIMER1A;//,		/* 9 */ -> 3 // mostly 16 bit, except some mega hv, some tiny
	if (pin ==  PinType::D10)
		return timer_info::make( &TCCR1A, &OCR1B, COM1B1 );//TIMER1B;//,		/* 10 */ -> 4 // mostly 16 bit, except same as OCR1B
	return timer_info{nullptr,nullptr,0,false}; // COMxxx bits are never 0
}
template<PinType pin>
inline
void turnOffPWMPS()
{
	auto const ti_tccr = digital_pin_to_timer_tccr(pin);
	auto const ti_combit = digital_pin_to_timer_combits(pin);
	cbi(*ti_tccr,ti_combit);
}
inline
void turnOffPWMPS(PinType pin)
{
	auto const ti_tccr = digital_pin_to_timer_tccr(pin);
	auto const ti_combit = digital_pin_to_timer_combits(pin);
	cbi(*ti_tccr,ti_combit);
}

// no need for further constant folding? try it anyway
template <uint8_t pin,uint8_t  mode>
inline
void pinMode()
{
	constexpr uint8_t const bit = digitalPinToBitMask(pin);
	constexpr PortType const port = digitalPinToPort(pin);


	if constexpr (port == NO_PORT) return;

	 volatile uint8_t * const reg = portModeRegister(port);
	 volatile uint8_t * const out = portOutputRegister(port);
	if constexpr (mode == INPUT) {
		SafeStatusRegisterAndClearInterrupt safe;
		*reg &= ~bit;
		*out &= ~bit;
	} else if constexpr (mode == INPUT_PULLUP) {
		SafeStatusRegisterAndClearInterrupt safe;
		*reg &= ~bit;
		*out |= bit;
	} else {
		SafeStatusRegisterAndClearInterrupt safe;
		*reg |= bit;
	}
}

template <uint8_t pin>
inline
void pinMode(uint8_t const mode)
{
	constexpr uint8_t const bit = digitalPinToBitMask(pin);
	constexpr PortType const port = digitalPinToPort(pin);


	if constexpr (port == NO_PORT) return;

	 volatile uint8_t *  reg = portModeRegister(port);
	 volatile uint8_t *  out = portOutputRegister(port);
	if (mode == INPUT) {
		SafeStatusRegisterAndClearInterrupt safe;
		*reg &= ~bit;
		*out &= ~bit;
	} else if (mode == INPUT_PULLUP) {
		SafeStatusRegisterAndClearInterrupt safe;
		*reg &= ~bit;
		*out |= bit;
	} else {
		SafeStatusRegisterAndClearInterrupt safe;
		*reg |= bit;
	}
}

inline
void pinMode(uint8_t const pin, uint8_t const mode)
{
	uint8_t const bit = digitalPinToBitMask(pin);
	PortType const port = digitalPinToPort(pin);


	if (port == NO_PORT) return;

	volatile uint8_t * const reg = portModeRegister(port);
	volatile uint8_t * const out = portOutputRegister(port);
	if (mode == INPUT) {
		SafeStatusRegisterAndClearInterrupt safe;
		*reg &= ~bit;
		*out &= ~bit;
	} else if (mode == INPUT_PULLUP) {
		SafeStatusRegisterAndClearInterrupt safe;
		*reg &= ~bit;
		*out |= bit;
	} else {
		SafeStatusRegisterAndClearInterrupt safe;
		*reg |= bit;
	}
}
inline
void digitalWrite(uint8_t const pin, uint8_t const val)
{
	auto const timer = digitalPinToTimer(pin);
	auto const port = digital_pin_to_Port_PS(pin);
	auto const bit = bitmask::digital_pin_to_BitMask_PS(pin);

	if (port == NO_PORT) return; // should use the enum...

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) turnOffPWMPS(PinType(pin));

	volatile uint8_t * const out = portOutputRegister(port);

	SafeStatusRegisterAndClearInterrupt safe;
	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}
}

template<uint8_t L_H>
inline
void digitalWrite_LH(uint8_t const pin)
{
	auto const timer = digitalPinToTimer(pin);
	auto const port = digital_pin_to_Port_PS(pin);
	auto const bit = bitmask::digital_pin_to_BitMask_PS(pin);

	if (port == NO_PORT) return; // should use the enum...

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) turnOffPWMPS(PinType(pin));

	volatile uint8_t * const out = portOutputRegister(port);

	SafeStatusRegisterAndClearInterrupt safe;
	if constexpr (L_H == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}
}



template <uint8_t pin>
inline
void digitalWrite(uint8_t const val)
{
	constexpr uint8_t const timer = digitalPinToTimer(pin);
	constexpr auto port = digital_pin_to_Port_PS(pin);
	constexpr auto bit = bitmask::digital_pin_to_BitMask_PS(pin);

	if constexpr (port == NO_PORT) return; // should use the enum...

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if constexpr (timer != NOT_ON_TIMER) turnOffPWMPS<PinType(pin)>();

	volatile uint8_t * const out = portOutputRegister(port);

	SafeStatusRegisterAndClearInterrupt safe;
	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}
}
template <uint8_t pin, uint8_t val>
void digitalWrite()
{
	constexpr auto const timer = digitalPinToTimer(pin);
	constexpr auto port = digital_pin_to_Port_PS(pin);
	constexpr auto bit = bitmask::digital_pin_to_BitMask_PS(pin);

	if constexpr (port == NO_PORT) return; // should use the enum...

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if constexpr (timer != NOT_ON_TIMER) turnOffPWMPS<PinType(pin)>();

	volatile uint8_t * const out = portOutputRegister(port);

	SafeStatusRegisterAndClearInterrupt safe;
	if constexpr (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}
}
template <uint8_t pin>
inline
int digitalRead()
{
	constexpr auto timer = digitalPinToTimer(pin);
	constexpr uint8_t bit = digitalPinToBitMask(pin);
	constexpr PortType port = digitalPinToPort(pin);

	if constexpr (port == NO_PORT) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if constexpr (timer != NOT_ON_TIMER) turnOffPWMPS<PinType(pin)>();

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}
inline
int digitalRead(uint8_t const pin)
{
	auto const timer = digitalPinToTimer(pin);
	uint8_t const bit = digitalPinToBitMask(pin);
	PortType const port = digitalPinToPort(pin);

	if (port == NO_PORT) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (timer != NOT_ON_TIMER) turnOffPWMPS(PinType(pin));

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}

namespace analog{
inline uint8_t analog_reference = DEFAULT;

inline
void analogReference(uint8_t mode)
{
	// can't actually set the register here because the default setting
	// will connect AVCC and the AREF pin, which would cause a short if
	// there's something connected to AREF.
	analog_reference = mode;
}
}
inline
int analogRead(uint8_t pin)
{
	using ::analog::analog_reference;

#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
	if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#endif
	pin = analogPinToChannel(pin);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
	if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
	if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#else
	if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif

#if defined(ADCSRB) && defined(MUX5)
	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif

	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
#if defined(ADMUX)
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = (analog_reference << 4) | (pin & 0x07);
#else
	ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif
#endif

	// without a delay, we seem to read from the wrong channel
	//delay(1);

#if defined(ADCSRA) && defined(ADCL)
	// start the conversion
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	uint8_t const low  = ADCL;
	uint8_t const high = ADCH;
	return (high << 8) | low;
#else
	// we dont have an ADC,
	return 0;
#endif
}



template <uint8_t const pin>
inline
void analogWrite( int const val)
{
	// We need to make sure the PWM output is enabled for those pins
	// that support it, as we turn it off when digitally reading or
	// writing with them.  Also, make sure the pin is in output mode
	// for consistenty with Wiring, which doesn't require a pinMode
	// call for the analog output pins.
	pinMode<pin, OUTPUT>();
	if (val == 0)
	{
		digitalWrite<pin,LOW>();
	}
	else if (val == 255)
	{
		digitalWrite<pin,HIGH>();
	}
	else
	{
		volatile uint8_t* const  ti_tccr = digital_pin_to_timer_tccr(pin);
		constexpr auto const ti_combit = digital_pin_to_timer_combits(pin);
		if constexpr (ti_combit){
			sbi(*ti_tccr,ti_combit);
			analog_pin_to_timer_assign(pin, val); // set pwm duty
		} else { // not on a timer
			if (val < 128) {
				digitalWrite<pin,LOW>();
			} else {
				digitalWrite<pin,HIGH>();
			}

		}
	}
}

inline
void analogWrite(uint8_t const pin, int const val)
{
	// We need to make sure the PWM output is enabled for those pins
	// that support it, as we turn it off when digitally reading or
	// writing with them.  Also, make sure the pin is in output mode
	// for consistenty with Wiring, which doesn't require a pinMode
	// call for the analog output pins.
	pinMode(pin, OUTPUT);
	if (val == 0)
	{
		digitalWrite_LH<LOW>(pin);
	}
	else if (val == 255)
	{
		digitalWrite_LH<HIGH>(pin);
	}
	else
	{
		auto const ti_tccr = digital_pin_to_timer_tccr(pin);
		auto const ti_combit = digital_pin_to_timer_combits(pin);
		if (ti_combit){
			sbi(*ti_tccr,ti_combit);
			analog_pin_to_timer_assign(pin, val); // set pwm duty
		} else { // not on a timer
			if (val < 128) {
				digitalWrite_LH<LOW>(pin);
			} else {
				digitalWrite_LH<HIGH>(pin);
			}

		}
	}
}
#ifdef CBI_SET
#undef CBI_SET
#undef cbi
#endif
#ifdef SBI_SET
#undef SBI_SET
#undef sbi
#endif


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
