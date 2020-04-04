/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Arduino_h
#define Arduino_h

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//#include "binary.h" // needless in C++, because we have binary literals

#ifdef __cplusplus
// for checking that this core is actually used
constexpr auto AN{0x1};
constexpr auto AUS{0x0};

extern "C"{
void yield(void);
}
constexpr auto HIGH = 0x1;
constexpr auto LOW = 0x0;

constexpr auto INPUT = 0x0;
constexpr auto OUTPUT = 0x1;
constexpr auto INPUT_PULLUP = 0x2;

constexpr auto PI = 3.1415926535897932384626433832795;
constexpr auto HALF_PI = 1.5707963267948966192313216916398;
constexpr auto TWO_PI = 6.283185307179586476925286766559;
constexpr auto DEG_TO_RAD = 0.017453292519943295769236907684886;
constexpr auto RAD_TO_DEG = 57.295779513082320876798154814105;
constexpr auto EULER = 2.718281828459045235360287471352;

constexpr auto SERIAL = 0x0;
constexpr auto DISPLAY = 0x1;

#define LSBFIRST 0
#define MSBFIRST 1

constexpr auto CHANGE = 1;
constexpr auto FALLING = 2;
constexpr auto RISING = 3;

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  constexpr auto DEFAULT = 0;
  constexpr auto EXTERNAL = 1;
  constexpr auto INTERNAL1V1 = 2;
  constexpr auto INTERNAL = INTERNAL1V1;
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  constexpr auto DEFAULT = 0;
  constexpr auto EXTERNAL = 4;
  constexpr auto INTERNAL1V1 = 8;
  constexpr auto INTERNAL = INTERNAL1V1;
  constexpr auto INTERNAL2V56 = 9;
  constexpr auto INTERNAL2V56_EXTCAP = 13;
#else
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
constexpr auto INTERNAL1V1 = 2;
constexpr auto INTERNAL2V56 = 3;
#else
constexpr auto INTERNAL = 3;
#endif
constexpr auto DEFAULT = 1;
constexpr auto EXTERNAL = 0;
#endif

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif

// might be replaced by standard library features later
template<typename T1, typename T2> 
inline constexpr auto min(T1&& a, T2&& b) //-> decltype(((a) < (b) ? (a) : (b)))
{
    return (((a) < (b) ? (a) : (b)));
}
template<typename T1, typename T2> 
inline constexpr auto max(T1&& a, T2&& b) //-> decltype(((a) > (b) ? (a) : (b)))
{
    return (((a) > (b) ? (a) : (b)));
}
#define abs(x) ((x)>0?(x):-(x))
template<typename T1, typename T2, typename T3> 
inline constexpr auto constrain(T1&& amt, T2&& low, T3&& high) //-> decltype(((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))))
{
    return (((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))));
}
template<typename T1> 
inline constexpr auto round(T1&& x) //-> decltype(((x) >= 0 ? (long )(((x) + 0.5)) : (long )(((x) - 0.5))))
{
    return (((x) >= 0 ? (long )(((x) + 0.5)) : (long )(((x) - 0.5))));
}
template<typename T1> 
inline constexpr auto radians(T1&& deg) //-> decltype(((deg) * DEG_TO_RAD))
{
    return (((deg) * DEG_TO_RAD));
}
template<typename T1> 
inline constexpr auto degrees(T1&& rad) //-> decltype(((rad) * RAD_TO_DEG))
{
    return (((rad) * RAD_TO_DEG));
}
template<typename T1> 
inline constexpr auto sq(T1&& x) //-> decltype(((x) * (x)))
{
    return (((x) * (x)));
}

#define interrupts() sei()
#define noInterrupts() cli()

inline constexpr auto clockCyclesPerMicrosecond() //-> decltype((F_CPU / 1000000L))
{
    return ((F_CPU / 1000000L));
}
template<typename T1> 
inline constexpr auto clockCyclesToMicroseconds(T1&& a) //-> decltype(((a) / clockCyclesPerMicrosecond()))
{
    return (((a) / clockCyclesPerMicrosecond()));
}
template<typename T1> 
inline constexpr auto microsecondsToClockCycles(T1&& a) //-> decltype(((a) * clockCyclesPerMicrosecond()))
{
    return (((a) * clockCyclesPerMicrosecond()));
}

template<typename T1> 
inline constexpr uint8_t lowByte(T1&& w)// -> decltype(((uint8_t)((w) & 0xff)))
{
    return w & 0xff;
}
template<typename T1> 
inline constexpr uint8_t highByte(T1&& w) //-> decltype(((uint8_t)((w) >> 8)))
{
    return w >> 8;
}

template<typename T1, typename T2> 
inline constexpr auto bitRead(T1&& value, T2&& bit) //-> decltype((((value) >> (bit)) & 0x01))
{
    return ((((value) >> (bit)) & 0x01));
}
template<typename T1, typename T2> 
inline constexpr auto bitSet(T1& value, T2&& bit)
{
    return (((value) |= (1UL << (bit))));
}
template<typename T1, typename T2> 
inline constexpr auto bitClear(T1& value, T2&& bit)
{
    return (((value) &= ~(1UL << (bit))));
}
template<typename T1, typename T2> 
inline constexpr auto bitToggle(T1& value, T2&& bit)
{
    return (((value) ^= (1UL << (bit))));
}
template<typename T1, typename T2, typename T3> 
inline constexpr auto bitWrite(T1& value, T2&& bit, T3&& bitvalue)
{
    return (((bitvalue) ? bitSet(value, bit) : bitClear(value, bit)));
}

using word = unsigned int;

template<typename T1> 
inline constexpr auto bit(T1&& b)
{
    return ((1UL << (b)));
}

using boolean = bool;
using byte = uint8_t;

#else
void yield(void);

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  #define DEFAULT 0
  #define EXTERNAL 1
  #define INTERNAL1V1 2
  #define INTERNAL INTERNAL1V1
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  #define DEFAULT 0
  #define EXTERNAL 4
  #define INTERNAL1V1 8
  #define INTERNAL INTERNAL1V1
  #define INTERNAL2V56 9
  #define INTERNAL2V56_EXTCAP 13
#else  
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
#define INTERNAL1V1 2
#define INTERNAL2V56 3
#else
#define INTERNAL 3
#endif
#define DEFAULT 1
#define EXTERNAL 0
#endif

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define interrupts() sei()
#define noInterrupts() cli()

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
typedef unsigned int word;

#define bit(b) (1UL << (b))

typedef bool boolean;
typedef uint8_t byte;

// triage C/C++ part
#endif


// avr-libc defines _NOP() since 1.6.2
#ifndef _NOP
#define _NOP() do { __asm__ volatile ("nop"); } while (0)
#endif


void init(void);
void initVariant(void);
#ifdef __cplusplus
extern "C"{
#endif
int atexit(void (*func)()) __attribute__((weak));
#ifdef __cplusplus
}
#endif
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogReference(uint8_t mode);
void analogWrite(uint8_t pin, int val);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode);
void detachInterrupt(uint8_t interruptNum);

void setup(void);
void loop(void);
// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.

#define analogInPinToBit(P) (P)


// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
// 
// These perform slightly better as macros compared to inline functions
//
#ifdef UsePetersCpp17
#define digitalPinToPort(P) digital_pin_to_Port_PS(P)
#define digitalPinToBitMask(P) bitmask::digital_pin_to_BitMask_PS(P)
#define digitalPinToTimer(P) (digital_pin_to_timer_PS(P))
#define portOutputRegister(P) port_to_output_PS(P)
#define portInputRegister(P) port_to_input_PS(P)
#define portModeRegister(P) port_to_mode_PS(P)
#define NOT_A_PORT nullptr

// for use in wiring etc., already similar in USBCore, but with extras
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
struct SafeStatusRegisterAndClearInterrupt{
	uint8_t oldSREG;
	[[nodiscard]] SafeStatusRegisterAndClearInterrupt() // attribute only officially in C++20
	:oldSREG{SREG} {
		cli();
	}
	~SafeStatusRegisterAndClearInterrupt(){
		SREG = oldSREG;
	}
};
#pragma GCC diagnostic pop


#else
#ifndef NO_PORT
#define NO_PORT 0 // PS, to indicate there is no port for pin to port mapping
typedef uint8_t PortType;
#endif
#ifdef __cplusplus
extern "C"{
#endif
// On the ATmega1280, the addresses of some of the port registers are
// greater than 255, so we can't store them in uint8_t's.
extern const uint16_t PROGMEM port_to_mode_PGM[];
extern const uint16_t PROGMEM port_to_input_PGM[];
extern const uint16_t PROGMEM port_to_output_PGM[];
extern const uint8_t PROGMEM digital_pin_to_port_PGM[];
// extern const uint8_t PROGMEM digital_pin_to_bit_PGM[];
extern const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[];
extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )
#define NOT_A_PORT 0
#ifdef __cplusplus
} // extern "C"
#endif
// PS: should not be done here...
#ifdef ARDUINO_MAIN
#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PK 11
#define PL 12
#endif
#endif
#define analogInPinToBit(P) (P)

#define NOT_A_PIN 0

#define NOT_AN_INTERRUPT -1


#ifdef UsePetersCpp17
enum timer_values:uint8_t {
NOT_ON_TIMER=0,
TIMER0A = 1,
TIMER0B = 2,
TIMER1A = 3,
TIMER1B = 4,
TIMER1C = 5,
TIMER2  = 6,
TIMER2A = 7,
TIMER2B = 8,

TIMER3A = 9,
TIMER3B = 10,
TIMER3C = 11,
TIMER4A = 12,
TIMER4B = 13,
TIMER4C = 14,
TIMER4D = 15,
TIMER5A = 16,
TIMER5B = 17,
TIMER5C = 18

};
#else
#define NOT_ON_TIMER 0
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER1C 5
#define TIMER2  6
#define TIMER2A 7
#define TIMER2B 8

#define TIMER3A 9
#define TIMER3B 10
#define TIMER3C 11
#define TIMER4A 12
#define TIMER4B 13
#define TIMER4C 14
#define TIMER4D 15
#define TIMER5A 16
#define TIMER5B 17
#define TIMER5C 18
#endif


#ifdef __cplusplus
#include "WCharacter.h"
#include "WString.h"
#include "HardwareSerial.h"
#include "USBAPI.h"
#if defined(HAVE_HWSERIAL0) && defined(HAVE_CDCSERIAL)
#error "Targets with both UART0 and CDC serial not supported"
#endif
#ifdef UsePetersCpp17
constexpr inline
uint16_t makeWord(uint16_t w) noexcept { return w; }
constexpr inline
uint16_t makeWord(uint8_t h, uint8_t l) noexcept {
	return (static_cast<uint16_t>(h) << 8u) | l; // PS: without cast or u, int is used for operations
}
#else
uint16_t makeWord(uint16_t w);
uint16_t makeWord(uint8_t h, uint8_t l);
#endif
// PS: I think the following macro is ridiculous.... but it is part of the API
#define word(...) makeWord(__VA_ARGS__)

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
void noTone(uint8_t _pin);

// WMath prototypes
long random(long);
long random(long, long);
void randomSeed(unsigned long);
#ifdef UsePetersCpp17
constexpr inline
long map(long x, long in_min, long in_max, long out_min, long out_max) noexcept
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // PS could result in overflow and div 0
}
#else
long map(long, long, long, long, long);
#endif

#endif

#include "pins_arduino.h"

#endif
