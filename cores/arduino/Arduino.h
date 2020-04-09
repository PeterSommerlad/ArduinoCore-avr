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


  20200410 Peter Sommerlad: changed to fully C++ only. no more C....
  except for missing libstdc++ and libcxxabi functionality
*/

#ifndef Arduino_h
#define Arduino_h

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>


// for checking that this core is actually used
extern "C"{
void yield(void);
}

enum PinState :uint8_t{ // arduinoAPI has PinStatus for Interrupt attachement AFAIK
LOW,
HIGH
};
enum PinMode: uint8_t{
 INPUT = 0x0,
 OUTPUT = 0x1,
 INPUT_PULLUP = 0x2 // arduinoAPI also defines INPUT_PULLDOWN
};

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

// for attachInterrupt, should be an enum (including LOW and HIGH - different meaning, therefore values change in ArduinoAPI)
constexpr auto CHANGE = 1;
constexpr auto FALLING = 2;
constexpr auto RISING = 3;

// should be enums in wiring_inline.h
enum AnalogInReferenceVoltageMode : uint8_t {

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
   DEFAULT = 0,
   EXTERNAL = 1,
   INTERNAL1V1 = 2,
   INTERNAL = INTERNAL1V1
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
   DEFAULT = 0,
   EXTERNAL = 4,
   INTERNAL1V1 = 8,
   INTERNAL = INTERNAL1V1,
   INTERNAL2V56 = 9,
   INTERNAL2V56_EXTCAP = 13
#else
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
   INTERNAL1V1 = 2,
   INTERNAL2V56 = 3,
#else
   INTERNAL = 3,
#endif
   DEFAULT = 1,
   EXTERNAL = 0
#endif
};



// might be replaced by standard library features later

template<typename T1> 
inline constexpr long round(T1&& x)
{
    return x >= 0 ? long(x + 0.5) : long(x - 0.5);
}
template<typename T1> 
inline constexpr auto radians(T1&& deg)
{
    return deg * DEG_TO_RAD; // *PI/180
}
template<typename T1> 
inline constexpr auto degrees(T1&& rad)
{
    return rad * RAD_TO_DEG; // *180/PI
}

// does not seem to be used
inline
__attribute__((always_inline))
void interrupts()
{
    sei();
}
inline
__attribute__((always_inline))
void  noInterrupts()
{
    cli();
}

inline constexpr long clockCyclesPerMicrosecond()
{
    return F_CPU / 1000000L; // this is a constant!
}
template<typename T1> 
inline constexpr auto clockCyclesToMicroseconds(T1 const & a)
{
    return a / clockCyclesPerMicrosecond();
}
template<typename T1> 
inline constexpr auto microsecondsToClockCycles(T1 const & a)
{
    return a * clockCyclesPerMicrosecond();
}
// Arduino Functions: Math
// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif
// // also fixed to not introduce -0.0
template<typename T>
inline constexpr auto abs(T&& x) { return x>=0 ? x : -x ; }

// available as std::clamp in C++17
template<typename T1>
inline constexpr T1 constrain(T1 const & amt, T1 const & low, T1 const & high)
{
    return (amt < low) ? low : (amt > high) ? high : amt;
}

inline constexpr
long map(long const x, long const in_min, long const in_max, long const out_min, long const out_max)
{
  assert(in_max != in_min); // sanity check div 0
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // PS could result in overflow and div 0
}

template<typename T1, typename T2>
inline constexpr auto max(T1&& a, T2&& b)
{
    return (a > b) ? a : b;
}
template<typename T1, typename T2>
inline constexpr auto min(T1&& a, T2&& b)
{
    return (a < b) ? a : b;
}

template<typename T1>
inline constexpr double sq(T1&& x) // according to arduino reference returns double
{
    return double(x) * x; // prevents overflow, macro could overflow
}

// Arduino Functions: Random Numbers
long random(long);
long random(long, long);
void randomSeed(unsigned long);


// Aduino Functions: Bits and Bytes
inline constexpr unsigned long bit(uint8_t bit)
{
    return 1UL << bit;
}

template<typename T1> // should be unsigned...
inline
__attribute__((always_inline))
constexpr auto bitRead(T1&& value, uint8_t theBit)
{
    return (value >> theBit) & 0x01;
}
template<typename T1> 
inline
__attribute__((always_inline))
constexpr void bitSet(T1& value, uint8_t theBit)
{
    value |= bit(theBit);
}
template<typename T1>
inline
__attribute__((always_inline))
constexpr void bitClear(T1& value, uint8_t theBit)
{
    value &= ~bit(theBit);
}
template<typename T1>
inline
__attribute__((always_inline))
constexpr auto bitToggle(T1& value, uint8_t theBit)
{
    return value ^= bit(theBit);
}
template<typename T1>
inline
__attribute__((always_inline))
constexpr void bitWrite(T1& value, uint8_t theBit, bool bitvalue)
{
    bitvalue ? bitSet(value, theBit) : bitClear(value, theBit);
}

template<typename T1>
inline
__attribute__((always_inline))
constexpr uint8_t highByte(T1&& w)
{
    return w >> 8;
}

template<typename T1>
inline
__attribute__((always_inline))
constexpr uint8_t lowByte(T1&& w)
{
    return w & 0xff;
}

/* TODO: request for removal  - from ArduinoAPI Common.h...*/

using boolean = bool;
using byte = uint8_t;
using word = uint16_t;

// avr-libc defines _NOP() since 1.6.2
#ifndef _NOP
#define _NOP() do { __asm__ volatile ("nop"); } while (0)
#endif


void init();
void initVariant();
extern "C"{
int atexit(void (*func)()) __attribute__((weak));
}


unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

// PS: also candidates for wiring_inline.h with better types
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode);
void detachInterrupt(uint8_t interruptNum);
// end candidates for wiring_inline

void setup(void);
void loop(void);
// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.

#define analogInPinToBit(P) (P)


// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
// 
// These perform slightly better as macros compared to inline functions? really
//
#define digitalPinToPort(P) digital_pin_to_Port_PS(PinType(P))
#define digitalPinToBitMask(P) bitmask::digital_pin_to_BitMask_PS(PinType(P))
#define digitalPinToTimer(P) (digital_pin_to_timer_PS(PinType(P)))
#define portOutputRegister(P) port_to_output_PS(PortType(P))
#define portInputRegister(P) port_to_input_PS(PortType(P))
#define portModeRegister(P) port_to_mode_PS(PortType(P))
#define NOT_A_PORT nullptr

// for use in wiring etc., already similar in USBCore, but with extras
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
struct SafeStatusRegisterAndClearInterrupt{
	uint8_t oldSREG;
	[[nodiscard]] // nodiscard attribute only officially in C++20, but works nevertheless
	 inline
	 SafeStatusRegisterAndClearInterrupt() __attribute__((always_inline))
	:oldSREG{SREG} {
		cli();
	}
	inline
	~SafeStatusRegisterAndClearInterrupt() __attribute__((always_inline))
	{
		SREG = oldSREG;
	}
};
#pragma GCC diagnostic pop


#define analogInPinToBit(P) (P)

#define NOT_A_PIN PinType(255)

#define NOT_AN_INTERRUPT -1


// should be specific for Variant, because not all timers are available on all AVR chips
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


#include "WCharacter.h"
#include "WString.h"
#include "HardwareSerial.h"
#include "USBAPI.h"
#if defined(HAVE_HWSERIAL0) && defined(HAVE_CDCSERIAL)
#error "Targets with both UART0 and CDC serial not supported"
#endif
constexpr inline
uint16_t makeWord(uint16_t w) noexcept { return w; }
constexpr inline
uint16_t makeWord(uint8_t h, uint8_t l) noexcept {
	return (static_cast<uint16_t>(h) << 8u) | l; // PS: without cast or u, int is used for operations
}
// PS: I think the following macro is ridiculous.... but it is part of the API
// cannot use function, because word as such is also a type alias....
//template<typename ... T>
//constexpr inline
//uint16_t word(T&&...x) { return makeWord(x...);}
#define word(...) makeWord(__VA_ARGS__)

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
void noTone(uint8_t _pin);



#include "pins_arduino.h"
// type safe versions defined in wiring_inline.h

// Arduino API: Digital I/O
inline PinState digitalRead(uint8_t pin) { return digitalRead(PinType(pin));}
inline void digitalWrite(uint8_t pin, uint8_t val){ digitalWrite(PinType(pin),PinState(val));}
inline void pinMode(uint8_t pin, uint8_t mode) { pinMode(PinType(pin),PinMode(mode));}

// Arduino API: Analog I/O

inline int analogRead(uint8_t pin) { return analogRead(PinType(pin)); }
inline void analogReference(uint8_t mode){ analogReference(static_cast<AnalogInReferenceVoltageMode>(mode)); }
inline void analogWrite(uint8_t const pin, int const val) { analogWrite(PinType(pin),val); }


// analogRead_PS...


#endif
