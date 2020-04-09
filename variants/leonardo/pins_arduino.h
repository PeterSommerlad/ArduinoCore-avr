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


#define digitalPinToPCICR(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCMSK0) : ((uint8_t *)0))
#define digitalPinToPCMSKbit(p) ( ((p) >= 8 && (p) <= 11) ? (p) - 4 : ((p) == 14 ? 3 : ((p) == 15 ? 1 : ((p) == 16 ? 2 : ((p) == 17 ? 0 : (p - A8 + 4))))))

//	__AVR_ATmega32U4__ has an unusual mapping of pins to channels
// PS: must remain a macro, because check is made in analogRead
#define analogPinToChannel(P) analog_pin_to_channel_PS(PinType(P))



#define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))

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

enum PinType:uint8_t {
	D00, D01, D02, D03, D04, D05, D06, D07, D08, D09, D10, D11, D12, D13, D14, D15,
	D16, D17, D18, D19, D20, D21, D22, D23, D24, D25, D26, D27, D28, D29, D30,
	A0=D18, A1=D19, A2=D20, A3=D21, A4=D22, A5=D23, A6=D04, A7=D06, A8=D08, A9=D09, A10=D10, A11=D12,
	D0=D00, D1=D01, D2=D02, D3=D03, D4=D04, D5=D05, D6=D06, D7=D07, D8=D08, D9=D09,
	MISO=D14, SCK=D15, MOSI=D16, SS=D17, TXLED=D30, RXLED=D17,
	LED_BUILTIN=D13, LED_BUILTIN_RX=D17, LED_BUILTIN_TX=D30,
	SDA=D02,SCL=D03
	// short names for single digit pins

};

// works up to 32 pins..., fortunately we have max 31. for alternative impl, see below...
template <PinType ...pins>
constexpr inline bool isOneOfPins(PinType pin) noexcept {
	return (1UL << pin) & ((1UL << pins)|...);
//	return ((pin == pins)||...);
}
// need to elaborate which is faster...

inline constexpr bool digitalPinHasPWM(PinType p)
{
	return isOneOfPins<D03,D05,D06,D09,D10,D11,D13>(p);
//    return (((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11 || (p) == 13));
}

constexpr inline PortType digital_pin_to_Port_PS(PinType const pin) noexcept {
	// check if this also benefits from using if instead of switch
	switch(pin){
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
struct bitmask {
enum  bitmask_in_byte:uint8_t {
	b0=1,b1=2,b2=4,b3=8,b4=16,b5=32,b6=64,b7=128
};
static constexpr inline uint8_t digital_pin_to_BitMask_PS(PinType const pin) noexcept {
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
constexpr inline timer_values digital_pin_to_timer_PS(PinType const pin) noexcept {
	// might need different approach than switch, because of RAM usage of section .rodata
	// not really, if it is used as constexpr function only.
	switch (pin) {
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

constexpr inline uint8_t analog_pin_to_channel_PS(PinType pin){
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
	default:;
	}
	return 0; // to silence warning;
}

// API for wiring_inline should go there.


#include "wiring_inline.h"

// template variations/non-argument overloads in pwm_timer_handling

inline void analog_timer_turnoff(timer_values const theTimer)  {
	switch (theTimer) {
	case TIMER0A: analog_timer_turnoff<TIMER0A>(); break;
	case TIMER0B: analog_timer_turnoff<TIMER0B>(); break;
	case TIMER1A: analog_timer_turnoff<TIMER1A>(); break;
	case TIMER1B: analog_timer_turnoff<TIMER1B>(); break;
	case TIMER3A: analog_timer_turnoff<TIMER3A>(); break;
	case TIMER4A: analog_timer_turnoff<TIMER4A>(); break;
	case TIMER4B: analog_timer_turnoff<TIMER4B>(); break;
	case TIMER4D: analog_timer_turnoff<TIMER4D>(); break;
	default:;
	}
}
inline void setPWMValue(timer_values const theTimer, int val)  {
	switch (theTimer) {
	case TIMER0A: setPWMValue<TIMER0A>(val); break;
	case TIMER0B: setPWMValue<TIMER0B>(val); break;
	case TIMER1A: setPWMValue<TIMER1A>(val); break;
	case TIMER1B: setPWMValue<TIMER1B>(val); break;
	case TIMER3A: setPWMValue<TIMER3A>(val); break;
	case TIMER3B: setPWMValue<TIMER3B>(val); break;
	case TIMER4A: setPWMValue<TIMER4A>(val); break;
	case TIMER4D: setPWMValue<TIMER4D>(val); break;
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
#define SERIAL_PORT_MONITOR        Serial
#define SERIAL_PORT_USBVIRTUAL     Serial
#define SERIAL_PORT_HARDWARE       Serial1
#define SERIAL_PORT_HARDWARE_OPEN  Serial1

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

// end leonardo include
#endif /* Pins_Arduino_h */
