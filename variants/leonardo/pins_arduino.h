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

// Workaround for wrong definitions in "iom32u4.h".
// This should be fixed in the AVR toolchain.
#undef UHCON
#undef UHINT
#undef UHIEN
#undef UHADDR
#undef UHFNUM
#undef UHFNUML
#undef UHFNUMH
#undef UHFLEN
#undef UPINRQX
#undef UPINTX
#undef UPNUM
#undef UPRST
#undef UPCONX
#undef UPCFG0X
#undef UPCFG1X
#undef UPSTAX
#undef UPCFG2X
#undef UPIENX
#undef UPDATX
#undef TCCR2A
#undef WGM20
#undef WGM21
#undef COM2B0
#undef COM2B1
#undef COM2A0
#undef COM2A1
#undef TCCR2B
#undef CS20
#undef CS21
#undef CS22
#undef WGM22
#undef FOC2B
#undef FOC2A
#undef TCNT2
#undef TCNT2_0
#undef TCNT2_1
#undef TCNT2_2
#undef TCNT2_3
#undef TCNT2_4
#undef TCNT2_5
#undef TCNT2_6
#undef TCNT2_7
#undef OCR2A
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7
#undef OCR2B
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7

#define NUM_DIGITAL_PINS  31
#define NUM_ANALOG_INPUTS 12

#define TX_RX_LED_INIT	DDRD |= (1<<5), DDRB |= (1<<0)
#define TXLED0			PORTD |= (1<<5)
#define TXLED1			PORTD &= ~(1<<5)
#define RXLED0			PORTB |= (1<<0)
#define RXLED1			PORTB &= ~(1<<0)



// Map SPI port to 'new' pins D14..D17

// Mapping of analog pins as digital I/O
// A6-A11 share with digital pins
#ifndef UsePetersCpp17
#define PIN_WIRE_SDA         (2)
#define PIN_WIRE_SCL         (3)

constexpr inline uint8_t SDA = PIN_WIRE_SDA;
constexpr inline uint8_t SCL = PIN_WIRE_SCL;
#define PIN_SPI_SS    (17)
#define PIN_SPI_MOSI  (16)
#define PIN_SPI_MISO  (14)
#define PIN_SPI_SCK   (15)

constexpr inline uint8_t SS   = PIN_SPI_SS;
constexpr inline uint8_t MOSI = PIN_SPI_MOSI;
constexpr inline uint8_t MISO = PIN_SPI_MISO;
constexpr inline uint8_t SCK  = PIN_SPI_SCK;
#define LED_BUILTIN 13
#define LED_BUILTIN_RX 17
#define LED_BUILTIN_TX 30
#define PIN_A0   (18)
#define PIN_A1   (19)
#define PIN_A2   (20)
#define PIN_A3   (21)
#define PIN_A4   (22)
#define PIN_A5   (23)
#define PIN_A6   (24)
#define PIN_A7   (25)
#define PIN_A8   (26)
#define PIN_A9   (27)
#define PIN_A10  (28)
#define PIN_A11  (29)

// could be an enum in C++17 or constexpr
constexpr inline uint8_t A0 = PIN_A0;
constexpr inline uint8_t A1 = PIN_A1;
constexpr inline uint8_t A2 = PIN_A2;
constexpr inline uint8_t A3 = PIN_A3;
constexpr inline uint8_t A4 = PIN_A4;
constexpr inline uint8_t A5 = PIN_A5;
constexpr inline uint8_t A6 = PIN_A6;	// D4
constexpr inline uint8_t A7 = PIN_A7;	// D6
constexpr inline uint8_t A8 = PIN_A8;	// D8
constexpr inline uint8_t A9 = PIN_A9;	// D9
constexpr inline uint8_t A10 = PIN_A10;	// D10
constexpr inline uint8_t A11 = PIN_A11;	// D12
#endif

#define digitalPinToPCICR(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCMSK0) : ((uint8_t *)0))
#define digitalPinToPCMSKbit(p) ( ((p) >= 8 && (p) <= 11) ? (p) - 4 : ((p) == 14 ? 3 : ((p) == 15 ? 1 : ((p) == 16 ? 2 : ((p) == 17 ? 0 : (p - A8 + 4))))))

//	__AVR_ATmega32U4__ has an unusual mapping of pins to channels
// PS: must remain a macro, because check is made in wiring_analog.c
#ifdef UsePetersCpp17
#define analogPinToChannel(P) analog_pin_to_channel_PS(P)
#else
extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )
#endif

#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11 || (p) == 13)

#define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))
#ifndef UsePetersCpp17
#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA32U4 / ARDUINO LEONARDO
//
// D0				PD2					RXD1/INT2
// D1				PD3					TXD1/INT3
// D2				PD1		SDA			SDA/INT1
// D3#				PD0		PWM8/SCL	OC0B/SCL/INT0
// D4		A6		PD4					ADC8
// D5#				PC6		???			OC3A/#OC4A
// D6#		A7		PD7		FastPWM		#OC4D/ADC10
// D7				PE6					INT6/AIN0
//
// D8		A8		PB4					ADC11/PCINT4
// D9#		A9		PB5		PWM16		OC1A/#OC4B/ADC12/PCINT5
// D10#		A10		PB6		PWM16		OC1B/0c4B/ADC13/PCINT6
// D11#				PB7		PWM8/16		0C0A/OC1C/#RTS/PCINT7
// D12		A11		PD6					T1/#OC4D/ADC9
// D13#				PC7		PWM10		CLK0/OC4A
//
// A0		D18		PF7					ADC7
// A1		D19		PF6					ADC6
// A2		D20 	PF5					ADC5
// A3		D21 	PF4					ADC4
// A4		D22		PF1					ADC1
// A5		D23 	PF0					ADC0
//
// New pins D14..D17 to map SPI port to digital pins
//
// MISO		D14		PB3					MISO,PCINT3
// SCK		D15		PB1					SCK,PCINT1
// MOSI		D16		PB2					MOSI,PCINT2
// SS		D17		PB0					RXLED,SS/PCINT0
//
// TXLED	D30		PD5					XCK1
// RXLED	D17	    PB0
// HWB				PE2					HWB

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
};
#endif
#define NO_PORT 0 // PS, to indicate there is no port for pin to port mapping
typedef uint8_t PortType;
#else
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

enum class PortType: uint8_t {
	No_Port=0, PA=1, PB=2, PC=3, PD=4, PE=5, PF=6//, PG=7, PH=8, PJ=10, PK=11, PL=12
};
#define NO_PORT PortType::No_Port
constexpr inline
volatile uint8_t *port_to_mode_PS(PortType port) noexcept {
	switch(port){
	case PortType::PB: return  &DDRB;
	case PortType::PC: return  &DDRC;
	case PortType::PD: return  &DDRD;
	case PortType::PE: return  &DDRE;
	case PortType::PF: return  &DDRF;
	default: return NOT_A_PORT; // nullptr
	}
}
constexpr inline
volatile uint8_t *port_to_output_PS(PortType port) noexcept {
	switch(port){
	case PortType::PB: return  &PORTB;
	case PortType::PC: return  &PORTC;
	case PortType::PD: return  &PORTD;
	case PortType::PE: return  &PORTE;
	case PortType::PF: return  &PORTF;
	default: return NOT_A_PORT; // nullptr
	}
}
constexpr inline
volatile uint8_t *port_to_input_PS(PortType port) noexcept {
	switch(port){
	case PortType::PB: return  &PINB;
	case PortType::PC: return  &PINC;
	case PortType::PD: return  &PIND;
	case PortType::PE: return  &PINE;
	case PortType::PF: return  &PINF;
	default: return NOT_A_PORT; // nullptr
	}
}
//#define portOutputRegister(P) port_to_output_PS(P)
//#define portInputRegister(P) port_to_input_PS(p)
//#define portModeRegister(P) port_to_mode_PS(p)
#endif

#ifndef UsePetersCpp17
#ifdef ARDUINO_MAIN
const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PD, // D0 - PD2
	PD,	// D1 - PD3
	PD, // D2 - PD1
	PD,	// D3 - PD0
	PD,	// D4 - PD4
	PC, // D5 - PC6
	PD, // D6 - PD7
	PE, // D7 - PE6

	PB, // D8 - PB4
	PB,	// D9 - PB5
	PB, // D10 - PB6
	PB,	// D11 - PB7
	PD, // D12 - PD6
	PC, // D13 - PC7

	PB,	// D14 - MISO - PB3
	PB,	// D15 - SCK - PB1
	PB,	// D16 - MOSI - PB2
	PB,	// D17 - SS - PB0

	PF,	// D18 - A0 - PF7
	PF, // D19 - A1 - PF6
	PF, // D20 - A2 - PF5
	PF, // D21 - A3 - PF4
	PF, // D22 - A4 - PF1
	PF, // D23 - A5 - PF0

	PD, // D24 / D4 - A6 - PD4
	PD, // D25 / D6 - A7 - PD7
	PB, // D26 / D8 - A8 - PB4
	PB, // D27 / D9 - A9 - PB5
	PB, // D28 / D10 - A10 - PB6
	PD, // D29 / D12 - A11 - PD6
	PD, // D30 / TX Led - PD5
};
#endif
#else
enum PinType:uint8_t {
	D00, D01, D02, D03, D04, D05, D06, D07, D08, D09, D10, D11, D12, D13, D14, D15,
	D16, D17, D18, D19, D20, D21, D22, D23, D24, D25, D26, D27, D28, D29, D30,
	A0=D18, A1=D19, A2=D20, A3=D21, A4=D22, A5=D23, A6=D04, A7=D06, A8=D08, A9=D09, A10=D10, A11=D12,
	MISO=D14, SCK=D15, MOSI=D16, SS=D17, TXLED=D30, RXLED=D17,
	LED_BUILTIN=D13, LED_BUILTIN_RX=D17, LED_BUILTIN_TX=D30,
	SDA=D02,SCL=D03

};
template <PinType ...pins>
constexpr inline bool isOneOfPins(uint8_t pin) noexcept {
	return (1UL << pin) & ((1UL << pins)|...);
}

constexpr inline PortType digital_pin_to_Port_PS(uint8_t pin) noexcept {
	// check if this also benefits from using if instead of switch
	switch(static_cast<PinType>(pin)){
	case D08: case D09: case D10: case D11: case D14: case D15: case D16: case D17: case D26: case D27: case D28:
		return PortType::PB;
	case D05: case D13:
		return PortType::PC;
	case D00: case D01: case D02: case D03: case D04: case D06: case D12: case D24: case D25: case D29: case D30:
		return PortType::PD;
	case D07:
		return PortType::PE;
	case D18: case D19: case D20: case D21: case D22: case D23:
		return PortType::PF;
	}
	return NO_PORT;
}
#endif
#ifndef UsePetersCpp17
#ifdef ARDUINO_MAIN
const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(2), // D0 - PD2
	_BV(3),	// D1 - PD3
	_BV(1), // D2 - PD1
	_BV(0),	// D3 - PD0
	_BV(4),	// D4 - PD4
	_BV(6), // D5 - PC6
	_BV(7), // D6 - PD7
	_BV(6), // D7 - PE6

	_BV(4), // D8 - PB4
	_BV(5),	// D9 - PB5
	_BV(6), // D10 - PB6
	_BV(7),	// D11 - PB7
	_BV(6), // D12 - PD6
	_BV(7), // D13 - PC7

	_BV(3),	// D14 - MISO - PB3
	_BV(1),	// D15 - SCK - PB1
	_BV(2),	// D16 - MOSI - PB2
	_BV(0),	// D17 - SS - PB0

	_BV(7),	// D18 - A0 - PF7
	_BV(6), // D19 - A1 - PF6
	_BV(5), // D20 - A2 - PF5
	_BV(4), // D21 - A3 - PF4
	_BV(1), // D22 - A4 - PF1
	_BV(0), // D23 - A5 - PF0

	_BV(4), // D24 / D4 - A6 - PD4
	_BV(7), // D25 / D6 - A7 - PD7
	_BV(4), // D26 / D8 - A8 - PB4
	_BV(5), // D27 / D9 - A9 - PB5
	_BV(6), // D28 / D10 - A10 - PB6
	_BV(6), // D29 / D12 - A11 - PD6
	_BV(5), // D30 / TX Led - PD5
};
#endif
#else
struct bitmask {
enum  bitmask_in_byte:uint8_t {
	b0=1,b1=2,b2=4,b3=8,b4=16,b5=32,b6=64,b7=128
};
static constexpr inline uint8_t digital_pin_to_BitMask_PS(uint8_t const pin) noexcept {
	if (isOneOfPins<D03,D17,D23>(pin))
		return bitmask_in_byte::b0;
	if (isOneOfPins<D02,D15,D22>(pin))
		return bitmask_in_byte::b1;
	if (isOneOfPins<D00,D16>(pin))
		return bitmask_in_byte::b2;
	if (isOneOfPins<D01,D14>(pin))
		return bitmask_in_byte::b3;
	if (isOneOfPins<D04,D08,D21,D24,D26>(pin))
		return bitmask_in_byte::b4;
	if (isOneOfPins<D09,D20,D27,D30>(pin))
		return bitmask_in_byte::b5;
	if (isOneOfPins<D05,D07,D10,D12,D19,D28,D29>(pin))
		return bitmask_in_byte::b6;
	if (isOneOfPins<D06,D11,D13,D18,D25>(pin))
		return bitmask_in_byte::b7;
// switch statement generates table in .data (section .rodata mapped to RAM, unfortunately)
//	switch (static_cast<PinType>(pin)) {
//	case D03:	case D17:	case D23:
//		return bitmask_in_byte::b0;
//	case D02:	case D15:	case D22:
//		return bitmask_in_byte::b1;
//	case D00:	case D16:
//		return bitmask_in_byte::b2;
//	case D01:	case D14:
//		return bitmask_in_byte::b3;
//	case D04:	case D08:	case D21:	case D24:	case D26:
//		return bitmask_in_byte::b4;
//	case D09:	case D20:	case D27:	case D30:
//		return bitmask_in_byte::b5;
//	case D05:	case D07:	case D10:	case D12:	case D19:	case D28:	case D29:
//		return bitmask_in_byte::b6;
//	case D06:	case D11:	case D13:	case D18:	case D25:
//		return bitmask_in_byte::b7;
//	}
	return 0; // 0 might break code, may be, but is better error indicator.
}
};
#endif
#ifndef UsePetersCpp17
#ifdef ARDUINO_MAIN
const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER0B,		/* 3 */
	NOT_ON_TIMER,
	TIMER3A,		/* 5 */
	TIMER4D,		/* 6 */
	NOT_ON_TIMER,

	NOT_ON_TIMER,
	TIMER1A,		/* 9 */
	TIMER1B,		/* 10 */
	TIMER0A,		/* 11 */

	NOT_ON_TIMER,
	TIMER4A,		/* 13 */

	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,

	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
};
#endif
#else
constexpr inline timer_values digital_pin_to_timer_PS(uint8_t const pin) noexcept {
	// might need different approach than switch, because of RAM usage of section .rodata
	switch (static_cast<PinType>(pin)) {
	case D03:
		return TIMER0B;//,		/* 3 */ -> 2
	case D05:
		return TIMER3A;//,		/* 5 */ -> 9
	case D06:
		return TIMER4D;//,		/* 6 */ ->15
	case D09:
		return TIMER1A;//,		/* 9 */ ->3
	case D10:
		return TIMER1B;//,		/* 10 */->4
	case D11:
		return TIMER0A;//,		/* 11 */->1
	case D13:
		return TIMER4A;//,		/* 13 */->12
	default:
		return NOT_ON_TIMER;
	}
}
constexpr
inline volatile uint8_t * digital_pin_to_timer_tccr(uint8_t const pin) noexcept {
	switch (static_cast<PinType>(pin)) {
	case D03:
		return &TCCR0B;//,		/* 3 */ -> 2
	case D05:
		return &TCCR3A;//,		/* 5 */ -> 9
	case D06:
		return &TCCR4C;//,TIMER4D !		/* 6 */ ->15
	case D09:
		return &TCCR1A;//,		/* 9 */ ->3
	case D10:
		return &TCCR1A;//,TIMER1B		/* 10 */->4
	case D11:
		return &TCCR0A;//,		/* 11 */->1
	case D13:
		return &TCCR0A;//,		/* 13 */->12
	default:
		return nullptr;
	}
}

#endif
#ifndef UsePetersCpp17
#ifdef ARDUINO_MAIN

const uint8_t PROGMEM analog_pin_to_channel_PGM[] = {
	7,	// A0				PF7					ADC7
	6,	// A1				PF6					ADC6
	5,	// A2				PF5					ADC5
	4,	// A3				PF4					ADC4
	1,	// A4				PF1					ADC1
	0,	// A5				PF0					ADC0
	8,	// A6		D4		PD4					ADC8
	10,	// A7		D6		PD7					ADC10
	11,	// A8		D8		PB4					ADC11
	12,	// A9		D9		PB5					ADC12
	13,	// A10		D10		PB6					ADC13
	9	// A11		D12		PD6					ADC9
};

#endif /* ARDUINO_MAIN */
#else
constexpr inline uint8_t analog_pin_to_channel_PS(uint8_t pin){
	switch(pin) {
	case A0: return 7;
	case A1: return 6;
	case A2: return 5;
	case A3: return 4;
	case A4: return 1;
	case A5: return 0;
	case A6: return 8;
	case A7: return 10;
	case A8: return 11;
	case A9: return 12;
	case A10: return 13;
	case A11: return 9;
	}
	return 0; // to silence warning;
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
#define SERIAL_PORT_MONITOR        Serial
#define SERIAL_PORT_USBVIRTUAL     Serial
#define SERIAL_PORT_HARDWARE       Serial1
#define SERIAL_PORT_HARDWARE_OPEN  Serial1

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

// end leonardo include
#endif /* Pins_Arduino_h */
