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

#define NUM_DIGITAL_PINS            70
#define NUM_ANALOG_INPUTS           16
#define analogInputToDigitalPin(p)  ((p < 16) ? (p) + 54 : -1)
#define digitalPinHasPWM(p)         (((p) >= 2 && (p) <= 13) || ((p) >= 44 && (p)<= 46))

#ifndef UsePetersCpp17
// the following stuff *except macros, are now in enum PinType
#define PIN_SPI_SS    (53)
#define PIN_SPI_MOSI  (51)
#define PIN_SPI_MISO  (50)
#define PIN_SPI_SCK   (52)

constexpr inline uint8_t SS   = PIN_SPI_SS;
constexpr inline uint8_t MOSI = PIN_SPI_MOSI;
constexpr inline uint8_t MISO = PIN_SPI_MISO;
constexpr inline uint8_t SCK  = PIN_SPI_SCK;

#define PIN_WIRE_SDA        (20)
#define PIN_WIRE_SCL        (21)

constexpr inline uint8_t SDA = PIN_WIRE_SDA;
constexpr inline uint8_t SCL = PIN_WIRE_SCL;

#define LED_BUILTIN 13
#define PIN_A0   (54)
#define PIN_A1   (55)
#define PIN_A2   (56)
#define PIN_A3   (57)
#define PIN_A4   (58)
#define PIN_A5   (59)
#define PIN_A6   (60)
#define PIN_A7   (61)
#define PIN_A8   (62)
#define PIN_A9   (63)
#define PIN_A10  (64)
#define PIN_A11  (65)
#define PIN_A12  (66)
#define PIN_A13  (67)
#define PIN_A14  (68)
#define PIN_A15  (69)

constexpr inline uint8_t A0 = PIN_A0;
constexpr inline uint8_t A1 = PIN_A1;
constexpr inline uint8_t A2 = PIN_A2;
constexpr inline uint8_t A3 = PIN_A3;
constexpr inline uint8_t A4 = PIN_A4;
constexpr inline uint8_t A5 = PIN_A5;
constexpr inline uint8_t A6 = PIN_A6;
constexpr inline uint8_t A7 = PIN_A7;
constexpr inline uint8_t A8 = PIN_A8;
constexpr inline uint8_t A9 = PIN_A9;
constexpr inline uint8_t A10 = PIN_A10;
constexpr inline uint8_t A11 = PIN_A11;
constexpr inline uint8_t A12 = PIN_A12;
constexpr inline uint8_t A13 = PIN_A13;
constexpr inline uint8_t A14 = PIN_A14;
constexpr inline uint8_t A15 = PIN_A15;
#endif
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

#ifndef UsePetersCpp17
#ifdef ARDUINO_MAIN

const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
	(uint16_t) &DDRG,
	(uint16_t) &DDRH,
	NOT_A_PORT,
	(uint16_t) &DDRJ,
	(uint16_t) &DDRK,
	(uint16_t) &DDRL,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
	(uint16_t) &PORTG,
	(uint16_t) &PORTH,
	NOT_A_PORT,
	(uint16_t) &PORTJ,
	(uint16_t) &PORTK,
	(uint16_t) &PORTL,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PIN,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
	(uint16_t) &PING,
	(uint16_t) &PINH,
	NOT_A_PIN,
	(uint16_t) &PINJ,
	(uint16_t) &PINK,
	(uint16_t) &PINL,
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
#endif
#ifndef UsePetersCpp17
#ifdef ARDUINO_MAIN
const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	// PORTLIST		
	// -------------------------------------------		
	PE	, // PE 0 ** 0 ** USART0_RX	
	PE	, // PE 1 ** 1 ** USART0_TX	
	PE	, // PE 4 ** 2 ** PWM2	
	PE	, // PE 5 ** 3 ** PWM3	
	PG	, // PG 5 ** 4 ** PWM4	
	PE	, // PE 3 ** 5 ** PWM5	
	PH	, // PH 3 ** 6 ** PWM6	
	PH	, // PH 4 ** 7 ** PWM7	
	PH	, // PH 5 ** 8 ** PWM8	
	PH	, // PH 6 ** 9 ** PWM9	
	PB	, // PB 4 ** 10 ** PWM10	
	PB	, // PB 5 ** 11 ** PWM11	
	PB	, // PB 6 ** 12 ** PWM12	
	PB	, // PB 7 ** 13 ** PWM13	
	PJ	, // PJ 1 ** 14 ** USART3_TX	
	PJ	, // PJ 0 ** 15 ** USART3_RX	
	PH	, // PH 1 ** 16 ** USART2_TX	
	PH	, // PH 0 ** 17 ** USART2_RX	
	PD	, // PD 3 ** 18 ** USART1_TX	
	PD	, // PD 2 ** 19 ** USART1_RX	
	PD	, // PD 1 ** 20 ** I2C_SDA	
	PD	, // PD 0 ** 21 ** I2C_SCL	
	PA	, // PA 0 ** 22 ** D22	
	PA	, // PA 1 ** 23 ** D23	
	PA	, // PA 2 ** 24 ** D24	
	PA	, // PA 3 ** 25 ** D25	
	PA	, // PA 4 ** 26 ** D26	
	PA	, // PA 5 ** 27 ** D27	
	PA	, // PA 6 ** 28 ** D28	
	PA	, // PA 7 ** 29 ** D29	
	PC	, // PC 7 ** 30 ** D30	
	PC	, // PC 6 ** 31 ** D31	
	PC	, // PC 5 ** 32 ** D32	
	PC	, // PC 4 ** 33 ** D33	
	PC	, // PC 3 ** 34 ** D34	
	PC	, // PC 2 ** 35 ** D35	
	PC	, // PC 1 ** 36 ** D36	
	PC	, // PC 0 ** 37 ** D37	
	PD	, // PD 7 ** 38 ** D38	
	PG	, // PG 2 ** 39 ** D39	
	PG	, // PG 1 ** 40 ** D40	
	PG	, // PG 0 ** 41 ** D41	
	PL	, // PL 7 ** 42 ** D42	
	PL	, // PL 6 ** 43 ** D43	
	PL	, // PL 5 ** 44 ** D44	
	PL	, // PL 4 ** 45 ** D45	
	PL	, // PL 3 ** 46 ** D46	
	PL	, // PL 2 ** 47 ** D47	
	PL	, // PL 1 ** 48 ** D48	
	PL	, // PL 0 ** 49 ** D49	
	PB	, // PB 3 ** 50 ** SPI_MISO	
	PB	, // PB 2 ** 51 ** SPI_MOSI	
	PB	, // PB 1 ** 52 ** SPI_SCK	
	PB	, // PB 0 ** 53 ** SPI_SS	
	PF	, // PF 0 ** 54 ** A0	
	PF	, // PF 1 ** 55 ** A1	
	PF	, // PF 2 ** 56 ** A2	
	PF	, // PF 3 ** 57 ** A3	
	PF	, // PF 4 ** 58 ** A4	
	PF	, // PF 5 ** 59 ** A5	
	PF	, // PF 6 ** 60 ** A6	
	PF	, // PF 7 ** 61 ** A7	
	PK	, // PK 0 ** 62 ** A8	
	PK	, // PK 1 ** 63 ** A9	
	PK	, // PK 2 ** 64 ** A10	
	PK	, // PK 3 ** 65 ** A11	
	PK	, // PK 4 ** 66 ** A12	
	PK	, // PK 5 ** 67 ** A13	
	PK	, // PK 6 ** 68 ** A14	
	PK	, // PK 7 ** 69 ** A15	
};
#endif
#else
enum PinType:uint8_t {
	D00, D01, D02, D03, D04, D05, D06, D07,	D08, D09,
	D10, D11, D12, D13, D14, D15, D16, D17, D18, D19,
	D20, D21, D22, D23, D24, D25, D26, D27, D28, D29,
	D30, D31, D32, D33, D34, D35, D36, D37, D38, D39,
	D40, D41, D42, D43, D44, D45, D46, D47, D48, D49,
	D50, D51, D52, D53, D54, D55, D56, D57, D58, D59,
	D60, D61, D62, D63, D64, D65, D66, D67, D68, D69,
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
namespace detail{
template<uint8_t thebyte>
constexpr inline bool isInByte(uint8_t pin){
	constexpr uint8_t bitstart = thebyte * 8;

	return (pin >=bitstart) && (pin < (bitstart +8));
}

template <uint8_t thebyte,PinType ...pins>
constexpr inline uint8_t bytebitmask(){
	constexpr uint8_t bitstart = thebyte * 8;
	return ( (static_cast<unsigned int>(isInByte<thebyte>(pins) << ((pins-bitstart) * isInByte<thebyte>(pins))))|...);
}

template <uint8_t thebyte,PinType ...pins>
constexpr inline bool isOneOfPinsInByte(uint8_t pin) noexcept {
	constexpr uint8_t bitstart = thebyte * 8;

	return isInByte<thebyte>(pin)&& ((1ul << pin-bitstart ) & bytebitmask<thebyte,pins...>());
}

template<uint8_t...allbytes>
struct isPinInBytesHelper {
	template <PinType ...pins>
	static constexpr inline bool isOneOfPins(uint8_t pin) noexcept {
		return ((isOneOfPinsInByte<allbytes,pins...>(pin)||...));
	}

};
}

template <PinType ...pins>
constexpr inline bool isOneOfPins(uint8_t pin) noexcept {
	return ((pin == pins)||...);

//	return detail::isPinInBytesHelper<0,1,2,3,4,5,6,7,8,9>::isOneOfPins<pins...>(pin); // maximum of 72 pins on Mega
}


constexpr inline PortType digital_pin_to_Port_PS(uint8_t const pin) noexcept {
	if (pin >= D22 && pin <= D29)
		return PortType::PA;
	if ((pin >= D10 && pin <= D13) || (pin >= D50 && pin <= D53))
		return PortType::PB;
	if (pin >= D30 && pin <= D37)
		return PortType::PC;
	if ((pin >= D18 && pin <= D21) || pin == D38)
		return PortType::PD;
	if (isOneOfPins<D00,D01,D02,D03,D05>(pin))
		return PortType::PE;
	if (pin >= D54 && pin <= D61)
		return PortType::PF;
	if (pin == D04 || (pin >= D39 && pin <= D41))
		return PortType::PG;
	if (isOneOfPins<D06,D07,D08,D09,D16,D17>(pin))
		return PortType::PH;
	if (pin == D14 || pin == D15)
		return PortType::PJ;
	if (pin >= D62 && pin <= D69)
		return PortType::PK;
	if (pin >= D42 && pin <= D49)
		return PortType::PL;
	return NO_PORT;
}
//constexpr inline PortType digital_pin_to_Port_PS(uint8_t pin) noexcept {
//	using PT=PinType;
//	switch(static_cast<PinType>(pin)){
//	case PT::D22: case PT::D23: case PT::D24: case PT::D25: case PT::D26: case PT::D27: case PT::D28: case PT::D29:
//		return PortType::PA;
//	case PT::D10: case PT::D11: case PT::D12: case PT::D13: case PT::D50: case PT::D51: case PT::D52: case PT::D53:
//		return PortType::PB;
//	case PT::D30: case PT::D31: case PT::D32: case PT::D33: case PT::D34: case PT::D35: case PT::D36: case PT::D37:
//		return PortType::PC;
//	case PT::D18: case PT::D19: case PT::D20: case PT::D21: case PT::D38:
//		return PortType::PD;
//	case PT::D00: case PT::D01: case PT::D02: case PT::D03:	case PT::D05:
//		return PortType::PE;
//	case PT::D54: case PT::D55: case PT::D56: case PT::D57: case PT::D58: case PT::D59: case PT::D60: case PT::D61:
//		return PortType::PF;
//	case PT::D04: case PT::D39: case PT::D40: case PT::D41:
//		return PortType::PG;
//	case PT::D06: case PT::D07: case PT::D08: case PT::D09: case PT::D16: case PT::D17:
//		return PortType::PH;
//	case PT::D14: case PT::D15:
//		return PortType::PJ;
//	case PT::D62: case PT::D63: case PT::D64: case PT::D65: case PT::D66: case PT::D67: case PT::D68: case PT::D69:
//		return PortType::PK;
//	case PT::D42: case PT::D43: case PT::D44: case PT::D45: case PT::D46: case PT::D47: case PT::D48: case PT::D49:
//		return PortType::PL;
//	}
//	return NO_PORT;
//}

#endif

#ifndef UsePetersCpp17
#ifdef ARDUINO_MAIN
const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PIN IN PORT		
	// -------------------------------------------		
	_BV( 0 )	, // PE 0 ** 0 ** USART0_RX	
	_BV( 1 )	, // PE 1 ** 1 ** USART0_TX	
	_BV( 4 )	, // PE 4 ** 2 ** PWM2	
	_BV( 5 )	, // PE 5 ** 3 ** PWM3	
	_BV( 5 )	, // PG 5 ** 4 ** PWM4	
	_BV( 3 )	, // PE 3 ** 5 ** PWM5	
	_BV( 3 )	, // PH 3 ** 6 ** PWM6	
	_BV( 4 )	, // PH 4 ** 7 ** PWM7	
	_BV( 5 )	, // PH 5 ** 8 ** PWM8	
	_BV( 6 )	, // PH 6 ** 9 ** PWM9	
	_BV( 4 )	, // PB 4 ** 10 ** PWM10	
	_BV( 5 )	, // PB 5 ** 11 ** PWM11	
	_BV( 6 )	, // PB 6 ** 12 ** PWM12	
	_BV( 7 )	, // PB 7 ** 13 ** PWM13	
	_BV( 1 )	, // PJ 1 ** 14 ** USART3_TX	
	_BV( 0 )	, // PJ 0 ** 15 ** USART3_RX	
	_BV( 1 )	, // PH 1 ** 16 ** USART2_TX	
	_BV( 0 )	, // PH 0 ** 17 ** USART2_RX	
	_BV( 3 )	, // PD 3 ** 18 ** USART1_TX	
	_BV( 2 )	, // PD 2 ** 19 ** USART1_RX	
	_BV( 1 )	, // PD 1 ** 20 ** I2C_SDA	
	_BV( 0 )	, // PD 0 ** 21 ** I2C_SCL	
	_BV( 0 )	, // PA 0 ** 22 ** D22	
	_BV( 1 )	, // PA 1 ** 23 ** D23	
	_BV( 2 )	, // PA 2 ** 24 ** D24	
	_BV( 3 )	, // PA 3 ** 25 ** D25	
	_BV( 4 )	, // PA 4 ** 26 ** D26	
	_BV( 5 )	, // PA 5 ** 27 ** D27	
	_BV( 6 )	, // PA 6 ** 28 ** D28	
	_BV( 7 )	, // PA 7 ** 29 ** D29	
	_BV( 7 )	, // PC 7 ** 30 ** D30	
	_BV( 6 )	, // PC 6 ** 31 ** D31	
	_BV( 5 )	, // PC 5 ** 32 ** D32	
	_BV( 4 )	, // PC 4 ** 33 ** D33	
	_BV( 3 )	, // PC 3 ** 34 ** D34	
	_BV( 2 )	, // PC 2 ** 35 ** D35	
	_BV( 1 )	, // PC 1 ** 36 ** D36	
	_BV( 0 )	, // PC 0 ** 37 ** D37	
	_BV( 7 )	, // PD 7 ** 38 ** D38	
	_BV( 2 )	, // PG 2 ** 39 ** D39	
	_BV( 1 )	, // PG 1 ** 40 ** D40	
	_BV( 0 )	, // PG 0 ** 41 ** D41	
	_BV( 7 )	, // PL 7 ** 42 ** D42	
	_BV( 6 )	, // PL 6 ** 43 ** D43	
	_BV( 5 )	, // PL 5 ** 44 ** D44	
	_BV( 4 )	, // PL 4 ** 45 ** D45	
	_BV( 3 )	, // PL 3 ** 46 ** D46	
	_BV( 2 )	, // PL 2 ** 47 ** D47	
	_BV( 1 )	, // PL 1 ** 48 ** D48	
	_BV( 0 )	, // PL 0 ** 49 ** D49	
	_BV( 3 )	, // PB 3 ** 50 ** SPI_MISO	
	_BV( 2 )	, // PB 2 ** 51 ** SPI_MOSI	
	_BV( 1 )	, // PB 1 ** 52 ** SPI_SCK	
	_BV( 0 )	, // PB 0 ** 53 ** SPI_SS	
	_BV( 0 )	, // PF 0 ** 54 ** A0	
	_BV( 1 )	, // PF 1 ** 55 ** A1	
	_BV( 2 )	, // PF 2 ** 56 ** A2	
	_BV( 3 )	, // PF 3 ** 57 ** A3	
	_BV( 4 )	, // PF 4 ** 58 ** A4	
	_BV( 5 )	, // PF 5 ** 59 ** A5	
	_BV( 6 )	, // PF 6 ** 60 ** A6	
	_BV( 7 )	, // PF 7 ** 61 ** A7	
	_BV( 0 )	, // PK 0 ** 62 ** A8	
	_BV( 1 )	, // PK 1 ** 63 ** A9	
	_BV( 2 )	, // PK 2 ** 64 ** A10	
	_BV( 3 )	, // PK 3 ** 65 ** A11	
	_BV( 4 )	, // PK 4 ** 66 ** A12	
	_BV( 5 )	, // PK 5 ** 67 ** A13	
	_BV( 6 )	, // PK 6 ** 68 ** A14	
	_BV( 7 )	, // PK 7 ** 69 ** A15	
};
#endif
#else


struct bitmask {
enum  bitmask_in_byte:uint8_t {
	b0=1,b1=2,b2=4,b3=8,b4=16,b5=32,b6=64,b7=128
};
static constexpr inline uint8_t digital_pin_to_BitMask_PS(uint8_t const pin) noexcept {
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
#endif
#ifndef UsePetersCpp17
#ifdef ARDUINO_MAIN

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	// TIMERS		
	// -------------------------------------------		
	NOT_ON_TIMER	, // PE 0 ** 0 ** USART0_RX	
	NOT_ON_TIMER	, // PE 1 ** 1 ** USART0_TX	
	TIMER3B	, // PE 4 ** 2 ** PWM2	
	TIMER3C	, // PE 5 ** 3 ** PWM3	
	TIMER0B	, // PG 5 ** 4 ** PWM4	
	TIMER3A	, // PE 3 ** 5 ** PWM5	
	TIMER4A	, // PH 3 ** 6 ** PWM6	
	TIMER4B	, // PH 4 ** 7 ** PWM7	
	TIMER4C	, // PH 5 ** 8 ** PWM8	
	TIMER2B	, // PH 6 ** 9 ** PWM9	
	TIMER2A	, // PB 4 ** 10 ** PWM10	
	TIMER1A	, // PB 5 ** 11 ** PWM11	
	TIMER1B	, // PB 6 ** 12 ** PWM12	
	TIMER0A	, // PB 7 ** 13 ** PWM13	
	NOT_ON_TIMER	, // PJ 1 ** 14 ** USART3_TX	
	NOT_ON_TIMER	, // PJ 0 ** 15 ** USART3_RX	
	NOT_ON_TIMER	, // PH 1 ** 16 ** USART2_TX	
	NOT_ON_TIMER	, // PH 0 ** 17 ** USART2_RX	
	NOT_ON_TIMER	, // PD 3 ** 18 ** USART1_TX	
	NOT_ON_TIMER	, // PD 2 ** 19 ** USART1_RX	
	NOT_ON_TIMER	, // PD 1 ** 20 ** I2C_SDA	
	NOT_ON_TIMER	, // PD 0 ** 21 ** I2C_SCL	
	NOT_ON_TIMER	, // PA 0 ** 22 ** D22	
	NOT_ON_TIMER	, // PA 1 ** 23 ** D23	
	NOT_ON_TIMER	, // PA 2 ** 24 ** D24	
	NOT_ON_TIMER	, // PA 3 ** 25 ** D25	
	NOT_ON_TIMER	, // PA 4 ** 26 ** D26	
	NOT_ON_TIMER	, // PA 5 ** 27 ** D27	
	NOT_ON_TIMER	, // PA 6 ** 28 ** D28	
	NOT_ON_TIMER	, // PA 7 ** 29 ** D29	
	NOT_ON_TIMER	, // PC 7 ** 30 ** D30	
	NOT_ON_TIMER	, // PC 6 ** 31 ** D31	
	NOT_ON_TIMER	, // PC 5 ** 32 ** D32	
	NOT_ON_TIMER	, // PC 4 ** 33 ** D33	
	NOT_ON_TIMER	, // PC 3 ** 34 ** D34	
	NOT_ON_TIMER	, // PC 2 ** 35 ** D35	
	NOT_ON_TIMER	, // PC 1 ** 36 ** D36	
	NOT_ON_TIMER	, // PC 0 ** 37 ** D37	
	NOT_ON_TIMER	, // PD 7 ** 38 ** D38	
	NOT_ON_TIMER	, // PG 2 ** 39 ** D39	
	NOT_ON_TIMER	, // PG 1 ** 40 ** D40	
	NOT_ON_TIMER	, // PG 0 ** 41 ** D41	
	NOT_ON_TIMER	, // PL 7 ** 42 ** D42	
	NOT_ON_TIMER	, // PL 6 ** 43 ** D43	
	TIMER5C	, // PL 5 ** 44 ** D44	
	TIMER5B	, // PL 4 ** 45 ** D45	
	TIMER5A	, // PL 3 ** 46 ** D46	
	NOT_ON_TIMER	, // PL 2 ** 47 ** D47	
	NOT_ON_TIMER	, // PL 1 ** 48 ** D48	
	NOT_ON_TIMER	, // PL 0 ** 49 ** D49	
	NOT_ON_TIMER	, // PB 3 ** 50 ** SPI_MISO	
	NOT_ON_TIMER	, // PB 2 ** 51 ** SPI_MOSI	
	NOT_ON_TIMER	, // PB 1 ** 52 ** SPI_SCK	
	NOT_ON_TIMER	, // PB 0 ** 53 ** SPI_SS	
	NOT_ON_TIMER	, // PF 0 ** 54 ** A0	
	NOT_ON_TIMER	, // PF 1 ** 55 ** A1	
	NOT_ON_TIMER	, // PF 2 ** 56 ** A2	
	NOT_ON_TIMER	, // PF 3 ** 57 ** A3	
	NOT_ON_TIMER	, // PF 4 ** 58 ** A4	
	NOT_ON_TIMER	, // PF 5 ** 59 ** A5	
	NOT_ON_TIMER	, // PF 6 ** 60 ** A6	
	NOT_ON_TIMER	, // PF 7 ** 61 ** A7	
	NOT_ON_TIMER	, // PK 0 ** 62 ** A8	
	NOT_ON_TIMER	, // PK 1 ** 63 ** A9	
	NOT_ON_TIMER	, // PK 2 ** 64 ** A10	
	NOT_ON_TIMER	, // PK 3 ** 65 ** A11	
	NOT_ON_TIMER	, // PK 4 ** 66 ** A12	
	NOT_ON_TIMER	, // PK 5 ** 67 ** A13	
	NOT_ON_TIMER	, // PK 6 ** 68 ** A14	
	NOT_ON_TIMER	, // PK 7 ** 69 ** A15	
};

#endif
#else
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
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2
#define SERIAL_PORT_HARDWARE3       Serial3
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2
#define SERIAL_PORT_HARDWARE_OPEN2  Serial3

#endif
