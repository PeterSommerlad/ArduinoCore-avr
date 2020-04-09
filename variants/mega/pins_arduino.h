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


#define NUM_DIGITAL_PINS            70
#define NUM_ANALOG_INPUTS           16
#define analogInputToDigitalPin(p)  ((p < 16) ? (p) + 54 : -1)


// A majority of the pins are NOT PCINTs, SO BE WARNED (i.e. you cannot use them as receive pins)
// Only pins available for RECEIVE (TRANSMIT can be on any pin):
// (I've deliberately left out pin mapping to the Hardware USARTs - seems senseless to me)
// Pins: 10, 11, 12, 13,  50, 51, 52, 53,  62, 63, 64, 65, 66, 67, 68, 69

#define digitalPinToPCICR(p)    ( (((p) >= 10) && ((p) <= 13)) || \
                                  (((p) >= 50) && ((p) <= 53)) || \
                                  (((p) >= 62) && ((p) <= 69)) ? (&PCICR) : ((uint8_t *)0) )

#define digitalPinToPCICRbit(p) ( (((p) >= 10) && ((p) <= 13)) || (((p) >= 50) && ((p) <= 53)) ? 0 : \
                                ( (((p) >= 62) && ((p) <= 69)) ? 2 : \
                                0 ) )

#define digitalPinToPCMSK(p)    ( (((p) >= 10) && ((p) <= 13)) || (((p) >= 50) && ((p) <= 53)) ? (&PCMSK0) : \
                                ( (((p) >= 62) && ((p) <= 69)) ? (&PCMSK2) : \
                                ((uint8_t *)0) ) )

#define digitalPinToPCMSKbit(p) ( (((p) >= 10) && ((p) <= 13)) ? ((p) - 6) : \
                                ( ((p) == 50) ? 3 : \
                                ( ((p) == 51) ? 2 : \
                                ( ((p) == 52) ? 1 : \
                                ( ((p) == 53) ? 0 : \
                                ( (((p) >= 62) && ((p) <= 69)) ? ((p) - 62) : \
                                0 ) ) ) ) ) )

#define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) >= 18 && (p) <= 21 ? 23 - (p) : NOT_AN_INTERRUPT)))


enum  PortType: uint8_t {
	No_Port=0, PA=1, PB=2, PC=3, PD=4, PE=5, PF=6, PG=7, PH=8, PJ=9, PK=10, PL=11 // contiguous port numbers make code simpler
};
#define NO_PORT PortType::No_Port

// need to check if these are really computed at compile time. Otherwise need to return uint16_t and to the cast at usage site
constexpr inline
volatile uint8_t *port_to_mode_PS(PortType port) noexcept {

	if (port >= PortType::PA && port < PortType::PH) { // ports A to G DDR map to 0x21-0x33 +every 3 bits
		return reinterpret_cast<volatile uint8_t *>((port-PortType::PA)*3 + 1 + 0x20);
	} else if (port >= PortType::PH && port <= PortType::PL) {
		return reinterpret_cast<volatile uint8_t *>((port-PortType::PH)*3 + 1 + 0x100);//(*(volatile uint8_t *)(0x104));
	}
	return NOT_A_PORT; // nullptr
}
constexpr inline
volatile uint8_t *port_to_output_PS(PortType port) noexcept {

	if (port >= PortType::PA && port < PortType::PH) { // ports A to G DDR map to 0x21-0x33 +every 3 bits
		return reinterpret_cast<volatile uint8_t *>((port-PortType::PA)*3 + 2 + 0x20);
	} else if (port >= PortType::PH && port <= PortType::PL) {
		return reinterpret_cast<volatile uint8_t *>((port-PortType::PH)*3 + 2 + 0x100);//(*(volatile uint8_t *)(0x104));
	}
	return NOT_A_PORT; // nullptr
}
constexpr inline
volatile uint8_t *port_to_input_PS(PortType port) noexcept {
	if (port >= PortType::PA && port < PortType::PH) { // ports A to G DDR map to 0x21-0x33 +every 3 bits
		return reinterpret_cast<volatile uint8_t *>((port-PortType::PA)*3  + 0x20);
	} else if (port >= PortType::PH && port <= PortType::PL) {
		return reinterpret_cast<volatile uint8_t *>((port-PortType::PH)*3  + 0x100);//(*(volatile uint8_t *)(0x104));
	}
	return NOT_A_PORT; // nullptr
}
enum PinType:uint8_t {
	D00, D01, D02, D03, D04, D05, D06, D07,	D08, D09,
	D10, D11, D12, D13, D14, D15, D16, D17, D18, D19,
	D20, D21, D22, D23, D24, D25, D26, D27, D28, D29,
	D30, D31, D32, D33, D34, D35, D36, D37, D38, D39,
	D40, D41, D42, D43, D44, D45, D46, D47, D48, D49,
	D50, D51, D52, D53, D54, D55, D56, D57, D58, D59,
	D60, D61, D62, D63, D64, D65, D66, D67, D68, D69,
	// short names for single digit pins
	D0=D00, D1=D01, D2=D02, D3=D03, D4=D04, D5=D05, D6=D06, D7=D07, D8=D08, D9=D09,
	USART0_RX=D00,USART0_TX=D01,
	PWM2=D02, PWM3=D03, PWM4=D04, PWM5=D05, PWM6=D06, PWM7=D07,
	PWM8=D08, PWM9=D09, PWM10=D10, PWM11=D11, PWM12=D12, PWM13=D13,
	USART3_TX=D14, USART3_RX=D15, USART2_TX=D16, USART2_RX=D17,
	USART1_TX=D18, USART1_RX=D19,
	MISO=D50, MOSI=D51, SCK=D52, SS=D53,
	SDA=D20, SCL=D21,
	A0=D54, A1=D55, A2=D56, A3=D57, A4=D58, A5=D59,
	A6=D60, A7=D61, A8=D62, A9=D63, A10=D64, A11=D65,
	A12=D66, A13=D67, A14=D68, A15=D69,
	LED_BUILTIN=13
};

template <PinType ...pins>
constexpr inline bool isOneOfPins(PinType pin) noexcept {
	return ((pin == pins)||...);
}
inline constexpr bool digitalPinHasPWM(PinType p)
{
    return (p >= 2 && p <= 13) || (p >= 44 && p <= 46);
}

constexpr inline PortType digital_pin_to_Port_PS(PinType const pin) noexcept {
	if (pin >= D22 && pin <= D29)
		return PortType::PA;
	if ((pin >= D10 && pin <= D13) || (pin >= D50 && pin <= D53))
		return PortType::PB;
	if (pin >= D30 && pin <= D37)
		return PortType::PC;
	if (isOneOfPins<D18,D19,D20,D21,D38>(pin))
		return PortType::PD;
	if (isOneOfPins<D00,D01,D02,D03,D05>(pin))
		return PortType::PE;
	if (pin >= D54 && pin <= D61)
		return PortType::PF;
	if (isOneOfPins<D04,D39,D40,D41>(pin))
		return PortType::PG;
	if (isOneOfPins<D06,D07,D08,D09,D16,D17>(pin))
		return PortType::PH;
	if (isOneOfPins<D14,D15>(pin))
		return PortType::PJ;
	if (pin >= D62 && pin <= D69)
		return PortType::PK;
	if (pin >= D42 && pin <= D49)
		return PortType::PL;
	return NO_PORT;
}


struct bitmask {
enum  bitmask_in_byte:uint8_t {
	b0=1,b1=2,b2=4,b3=8,b4=16,b5=32,b6=64,b7=128
};
static constexpr inline uint8_t digital_pin_to_BitMask_PS(PinType const pin) noexcept {
	if( isOneOfPins<D00,D15,D17,D21,D22,D37,D41,D49,D53,D54,D62>(pin))
		return bitmask_in_byte::b0;
	if( isOneOfPins<D01, D14, D16, D20, D23, D36, D40, D48, D52, D55, D63>(pin))
		return bitmask_in_byte::b1;
	if( isOneOfPins<D19, D24,D35, D39, D47, D51, D56,D64>(pin))
		return bitmask_in_byte::b2;
	if( isOneOfPins<D05, D06, D18, D25, D34, D46, D50, D57, D65>(pin))
		return bitmask_in_byte::b3;
	if( isOneOfPins<D02, D07, D10, D26, D33, D45, D58, D66>(pin))
		return bitmask_in_byte::b4;
	if( isOneOfPins<D03, D04, D08, D11, D27, D32, D44, D59, D67>(pin))
		return bitmask_in_byte::b5;
	if( isOneOfPins<D09, D12, D28, D31, D43, D60, D68>(pin))
		return bitmask_in_byte::b6;
	if( isOneOfPins<D13, D29, D30, D38, D42, D61, D69>(pin))
		return bitmask_in_byte::b7;
	return 0; // 0 might break code, may be, but would be better indicator.

}


//static constexpr inline uint8_t digital_pin_to_BitMask_PS(uint8_t const pin) noexcept {
//	using PT=PinType;
//	switch (static_cast<PinType>(pin)) {
//		case PT::D00: case PT::D15: case PT::D17: case PT::D21: case PT::D22: case PT::D37: case PT::D41: case PT::D49: case PT::D53: case PT::D54: case PT::D62:
//			return bitmask_in_byte::b0;
//		case PT::D01: case PT::D14: case PT::D16: case PT::D20: case PT::D23: case PT::D36: case PT::D40: case PT::D48: case PT::D52: case PT::D55: case PT::D63:
//			return bitmask_in_byte::b1;
//
//		case PT::D19: case PT::D24: case PT::D35: case PT::D39: case PT::D47: case PT::D51: case PT::D56: case PT::D64:
//			return bitmask_in_byte::b2;
//
//		case PT::D05: case PT::D06: case PT::D18: case PT::D25: case PT::D34: case PT::D46: case PT::D50: case PT::D57: case PT::D65:
//			return bitmask_in_byte::b3;
//
//		case PT::D02: case PT::D07: case PT::D10: case PT::D26: case PT::D33: case PT::D45: case PT::D58: case PT::D66:
//			return bitmask_in_byte::b4;
//		case PT::D03: case PT::D04: case PT::D08: case PT::D11: case PT::D27: case PT::D32: case PT::D44: case PT::D59: case PT::D67:
//			return bitmask_in_byte::b5;
//		case PT::D09: case PT::D12: case PT::D28: case PT::D31: case PT::D43: case PT::D60: case PT::D68:
//			return bitmask_in_byte::b6;
//		case PT::D13: case PT::D29: case PT::D30: case PT::D38: case PT::D42: case PT::D61: case PT::D69:
//			return bitmask_in_byte::b7;
//	}
//	return 0; // 0 might break code, may be, but would be better indicator.
//}
};

constexpr inline timer_values digital_pin_to_timer_PS(PinType const pin) noexcept {
	using PT=PinType;
	constexpr timer_values timermap[] {
		/* PT::D02 */	 TIMER3B,
		/* PT::D03 */	 TIMER3C,
		/* PT::D04 */	 TIMER0B,
		/* PT::D05 */ TIMER3A,
		/* PT::D06 */ TIMER4A,
		/* PT::D07 */ TIMER4B,
		/* PT::D08 */ TIMER4C,
		/* PT::D09 */ TIMER2B,
		/* PT::D10 */ TIMER2A,
		/* PT::D11 */ TIMER1A,
		/* PT::D12 */ TIMER1B,
		/* PT::D13 */ TIMER0A,
		/* PT::D44 */ TIMER5C,
		/* PT::D45 */ TIMER5B,
		/* PT::D46 */ TIMER5A
	};
	if (pin>=PT::D02 && pin <= PT::D13){
		return timermap[pin-PT::D02];
	} else if (pin >=PT::D44 && pin <= PT::D46){
		return timermap[pin-PT::D44 + 12];
	}
	return NOT_ON_TIMER;

}

#include "wiring_inline.h"
inline void setPWMValue(timer_values const theTimer, int val)  {
	switch (theTimer) {
	case TIMER0A: setPWMValue<TIMER0A>(val); break;
	case TIMER0B: setPWMValue<TIMER0B>(val); break;
	case TIMER1A: setPWMValue<TIMER1A>(val); break;
	case TIMER1B: setPWMValue<TIMER1B>(val); break;
	case TIMER1C: setPWMValue<TIMER1C>(val); break;
	case TIMER2A: setPWMValue<TIMER2A>(val); break;
	case TIMER2B: setPWMValue<TIMER2B>(val); break;
	case TIMER3A: setPWMValue<TIMER3A>(val); break;
	case TIMER3B: setPWMValue<TIMER3B>(val); break;
	case TIMER3C: setPWMValue<TIMER3C>(val); break;
	case TIMER4A: setPWMValue<TIMER4A>(val); break;
	case TIMER4B: setPWMValue<TIMER4B>(val); break;
	case TIMER4C: setPWMValue<TIMER4C>(val); break;
	case TIMER5A: setPWMValue<TIMER5A>(val); break;
	case TIMER5B: setPWMValue<TIMER5B>(val); break;
	case TIMER5C: setPWMValue<TIMER5C>(val); break;
	default:;
	}
}

inline void analog_timer_turnoff(timer_values const theTimer)  {
	switch (theTimer) {
	case TIMER0A: analog_timer_turnoff<TIMER0A>(); break;
	case TIMER0B: analog_timer_turnoff<TIMER0B>(); break;
	case TIMER1A: analog_timer_turnoff<TIMER1A>(); break;
	case TIMER1B: analog_timer_turnoff<TIMER1B>(); break;
	case TIMER1C: analog_timer_turnoff<TIMER1C>(); break;
	case TIMER2A: analog_timer_turnoff<TIMER2A>(); break;
	case TIMER2B: analog_timer_turnoff<TIMER2B>(); break;
	case TIMER3A: analog_timer_turnoff<TIMER3A>(); break;
	case TIMER3B: analog_timer_turnoff<TIMER3B>(); break;
	case TIMER3C: analog_timer_turnoff<TIMER3C>(); break;
	case TIMER4A: analog_timer_turnoff<TIMER4A>(); break;
	case TIMER4B: analog_timer_turnoff<TIMER4B>(); break;
	case TIMER4C: analog_timer_turnoff<TIMER4C>(); break;
	case TIMER5A: analog_timer_turnoff<TIMER5A>(); break;
	case TIMER5B: analog_timer_turnoff<TIMER5B>(); break;
	case TIMER5C: analog_timer_turnoff<TIMER5C>(); break;
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
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2
#define SERIAL_PORT_HARDWARE3       Serial3
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2
#define SERIAL_PORT_HARDWARE_OPEN2  Serial3

#endif
