// wiring_inline - allow for C++ templates and inline functions for compact efficient and type safe binary code
// Copyright (c) 2020 Peter Sommerlad
/*
  wiring_inline.h - input and output functions
  To become and derived from
  Part of Arduino - http://www.arduino.cc/

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







#include "pwm_timer_handling.h"

inline void setPWMValue(timer_values const theTimer, int val);

// ************ pinMode - set pin to input, input_pullup, or output

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
	SafeStatusRegisterAndClearInterrupt safe{};
	if constexpr (mode == INPUT) {
		*reg &= ~bit;
		*out &= ~bit;
	} else if constexpr (mode == INPUT_PULLUP) {
		*reg &= ~bit;
		*out |= bit;
	} else {
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

	 volatile uint8_t * const reg = portModeRegister(port);
	 volatile uint8_t * const out = portOutputRegister(port);
	if (mode == INPUT) {
		SafeStatusRegisterAndClearInterrupt safe{}; // spread to block interrupts as short as possible
		*reg &= ~bit;
		*out &= ~bit;
	} else if (mode == INPUT_PULLUP) {
		SafeStatusRegisterAndClearInterrupt safe{};
		*reg &= ~bit;
		*out |= bit;
	} else {
		SafeStatusRegisterAndClearInterrupt safe{};
		*reg |= bit;
	}
}
template<uint8_t mode>
inline
void pinModeFixMode(uint8_t const pin)
{
	uint8_t const bit = digitalPinToBitMask(pin);
	PortType const port = digitalPinToPort(pin);


	if (port == NO_PORT) return;

	volatile uint8_t * const reg = portModeRegister(port);
	volatile uint8_t * const out = portOutputRegister(port);
	SafeStatusRegisterAndClearInterrupt safe{};
	if constexpr (mode == INPUT) {
		*reg &= ~bit;
		*out &= ~bit;
	} else if constexpr (mode == INPUT_PULLUP) {
		*reg &= ~bit;
		*out |= bit;
	} else {
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
		SafeStatusRegisterAndClearInterrupt safe{};
		*reg &= ~bit;
		*out &= ~bit;
	} else if (mode == INPUT_PULLUP) {
		SafeStatusRegisterAndClearInterrupt safe{};
		*reg &= ~bit;
		*out |= bit;
	} else {
		SafeStatusRegisterAndClearInterrupt safe{};
		*reg |= bit;
	}
}


inline void analog_pin_to_timer_turnoff(PinType const pin)  {
	auto const theTimer = digital_pin_to_timer_PS(pin);
	analog_timer_turnoff(theTimer); // this way hope for better constant folding and inlining
}


// *************** digitalWrite
inline
void digitalWrite(uint8_t const pin, uint8_t const val)
{
	auto const timer = digitalPinToTimer(static_cast<PinType>(pin));
	auto const port = digital_pin_to_Port_PS(static_cast<PinType>(pin));
	auto const bit = bitmask::digital_pin_to_BitMask_PS(static_cast<PinType>(pin));

	if (port == NO_PORT) return; // should use the enum...

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) analog_timer_turnoff(timer);

	volatile uint8_t * const out = portOutputRegister(port);

	SafeStatusRegisterAndClearInterrupt safe { };
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
	auto const timer = digitalPinToTimer(PinType(pin));
	auto const port = digital_pin_to_Port_PS(pin);
	auto const bit = bitmask::digital_pin_to_BitMask_PS(pin);

	if (port == NO_PORT) return; // should use the enum...

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) analog_pin_to_timer_turnoff(PinType(pin));

	volatile uint8_t * const out = portOutputRegister(port);

	SafeStatusRegisterAndClearInterrupt safe { };
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
	constexpr uint8_t const timer = digitalPinToTimer(PinType(pin));
	constexpr auto port = digital_pin_to_Port_PS(pin);
	constexpr auto bit = bitmask::digital_pin_to_BitMask_PS(pin);

	if constexpr (port == NO_PORT) return; // should use the enum...

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if constexpr (timer != NOT_ON_TIMER) analog_pin_to_timer_turnoff<PinType(pin)>();

	volatile uint8_t * const out = portOutputRegister(port);

	SafeStatusRegisterAndClearInterrupt safe { };
	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}
}
template <uint8_t pin, uint8_t val>
inline
__attribute__((always_inline))
void digitalWrite()
{
	constexpr auto const timer = digitalPinToTimer(PinType(pin));
	constexpr auto port = digital_pin_to_Port_PS(pin);
	constexpr auto bit = bitmask::digital_pin_to_BitMask_PS(pin);

	if constexpr (port == NO_PORT) return; // should use the enum...

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if constexpr (timer != NOT_ON_TIMER) analog_pin_to_timer_turnoff<PinType(pin)>();

	volatile uint8_t * const out = portOutputRegister(port);

	SafeStatusRegisterAndClearInterrupt safe { };
	if constexpr (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}
}

// ***************** digitalRead

template <uint8_t pin>
inline
int digitalRead()
{
	constexpr auto timer = digitalPinToTimer(PinType(pin));
	constexpr uint8_t bit = digitalPinToBitMask(pin);
	constexpr PortType port = digitalPinToPort(pin);

	if constexpr (port == NO_PORT) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if constexpr (timer != NOT_ON_TIMER) analog_pin_to_timer_turnoff<PinType(pin)>();

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}
inline
int digitalRead(uint8_t const pin)
{
	auto const timer = digitalPinToTimer(PinType(pin));
	uint8_t const bit = digitalPinToBitMask(pin);
	PortType const port = digitalPinToPort(pin);

	if (port == NO_PORT) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (timer != NOT_ON_TIMER) analog_pin_to_timer_turnoff(PinType(pin));

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}
// ************ analogRead() --- needs more work!
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
__attribute__((always_inline))
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

//******** analogWrite

template<uint8_t pin>
inline
void analogWrite(int const val)
{
	// We need to make sure the PWM output is enabled for those pins
	// that support it, as we turn it off when digitally reading or
	// writing with them.  Also, make sure the pin is in output mode
	// for consistenty with Wiring, which doesn't require a pinMode
	// call for the analog output pins.
	pinMode<pin,OUTPUT>();
	constexpr auto const timer = digital_pin_to_timer_PS(PinType(pin)); // compile time
	// assume compiler optimizes better
	if (val == 0 || (timer == NOT_ON_TIMER && val < 128))
	{
		digitalWrite<pin,LOW>();
	}
	else if (val == 255 || (timer == NOT_ON_TIMER))
	{
		digitalWrite<pin,HIGH>();
	}
	else
	{
		setPWMValue<timer>(val);
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
	pinModeFixMode<OUTPUT>(pin);
	if (val == 0)
	{
		digitalWrite(pin,LOW);
	}
	else if (val == 255)
	{
		digitalWrite(pin,HIGH);
	}
	else
	{
		auto const timer = digital_pin_to_timer_PS(PinType(pin));
		if (timer != NOT_ON_TIMER){
			setPWMValue(timer,val);
		} else { // not on a timer
			if (val < 128) {
				digitalWrite(pin,LOW);
			} else {
				digitalWrite(pin,HIGH);
			}

		}
	}
}

