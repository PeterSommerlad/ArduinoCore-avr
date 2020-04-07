/* Tone.cpp

  A Tone Generator Library

  Written by Brett Hagman

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

Version Modified By Date     Comments
------- ----------- -------- --------
0001    B Hagman    09/08/02 Initial coding
0002    B Hagman    09/08/18 Multiple pins
0003    B Hagman    09/08/18 Moved initialization from constructor to begin()
0004    B Hagman    09/09/26 Fixed problems with ATmega8
0005    B Hagman    09/11/23 Scanned prescalars for best fit on 8 bit timers
                    09/11/25 Changed pin toggle method to XOR
                    09/11/25 Fixed timer0 from being excluded
0006    D Mellis    09/12/29 Replaced objects with functions
0007    M Sproul    10/08/29 Changed #ifdefs from cpu to register
0008    S Kanemoto  12/06/22 Fixed for Leonardo by @maris_HY
0009    J Reucker   15/04/10 Issue #292 Fixed problems with ATmega8 (thanks to Pete62)
0010    jipp        15/04/13 added additional define check #2923
*************************************************/

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "pins_arduino.h" // PS superfluous in Arduino.h

// PS, hacky but change later...

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega128__)
#define TCCR2A TCCR2
#define TCCR2B TCCR2
#define COM2A1 COM21
#define COM2A0 COM20
#define OCR2A OCR2
#define TIMSK2 TIMSK
#define OCIE2A OCIE2
#define TIMER2_COMPA_vect TIMER2_COMP_vect
#define TIMSK1 TIMSK
#endif

// tone_toggle_count:
//  > 0 - duration specified
//  = 0 - stopped
//  < 0 - infinitely (until stop() method called, or new play() called)

// support only one pin... if more use other library.

namespace {
volatile long tone_toggle_count{};
volatile uint8_t * tone_pin_port{};
volatile uint8_t   tone_pin_mask{};
PinType            tone_pin{NOT_A_PIN};


// since only one timer is supported, simplify the code.

#if defined(__AVR_ATmega32U4__)
 
#define USE_TIMER3
constexpr uint8_t tone_timer = 3;
 
#else

#define USE_TIMER2
constexpr uint8_t tone_timer = 2;

#endif
}


static int8_t toneBegin(PinType _pin)
{
  if (tone_pin == _pin) {
	  return tone_timer;
  } else if (tone_pin != NOT_A_PIN) { // already in use
	  return -1;
  }
  if (tone_timer == 2){
      #if defined(TCCR2A) && defined(TCCR2B)
        // 8 bit timer
        TCCR2A = 0;
        TCCR2B = 0;
        bitWrite(TCCR2A, WGM21, 1);
        bitWrite(TCCR2B, CS20, 1);
        tone_pin_port = portOutputRegister(digitalPinToPort(_pin));
        tone_pin_mask = digitalPinToBitMask(_pin);
        return tone_timer;
      #endif
  } else { // tone_timer ==3
      #if defined(TCCR3A) && defined(TCCR3B) &&  defined(TIMSK3)
        // 16 bit timer
        TCCR3A = 0;
        TCCR3B = 0;
        bitSet(TCCR3B, WGM32);
        bitSet(TCCR3B, CS30);
        tone_pin_port = portOutputRegister(digitalPinToPort(_pin));
        tone_pin_mask = digitalPinToBitMask(_pin);
        return tone_timer;
      #endif
  }
  return -1; // something really wrong here...
}

uint8_t compute_ocr_and_prescalerbits_for_8bit_timer2(uint32_t &ocr, unsigned int frequency, uint8_t prescalerbits) {
	unsigned long baseticks = F_CPU/(2L*frequency);
	prescalerbits=1;
	while(baseticks>256 && prescalerbits < 7){
		uint8_t const shift = [&](){
			if (prescalerbits==1) return 3;
			if (prescalerbits>2 && prescalerbits<=5) return 1;
			return 2;
		}();
		baseticks >>= shift; // timer2 = 8(3),32(2),64(1),128(1),256(1),1024(2)
		prescalerbits++;
	}
	ocr=baseticks-1;
	return prescalerbits;
}

uint8_t compute_ocr_and_prescalerbits_for_16bit_timer(uint32_t &ocr, unsigned int frequency, uint8_t prescalerbits) {
	// two choices for the 16 bit timers: ck/1 or ck/64
	ocr = F_CPU / frequency / 2;
	prescalerbits = 0b001;
	if (ocr > 0x10000L ) {
		ocr /= 64 ;
		prescalerbits = 0b011;
	}
	ocr -= 1; // adjust accordingly
	return prescalerbits;
}

long compute_toggle_count(unsigned long duration, unsigned int frequency) {
	// Calculate the toggle count
	if (duration > 0) {
		return 2 * frequency * duration / 1000;
	} else {
		return -1;
	}
}

//uint8_t compute_ocr_and_prescalerbits_for_8bit_timer(uint32_t &ocr, unsigned int frequency, uint8_t prescalerbits, int8_t _timer) {
//	ocr = F_CPU / frequency / 2 - 1;
//	prescalerbits = 0b001; // ck/1: same for both timers
//	if (ocr > 255) {
//		ocr = F_CPU / frequency / 2 / 8 - 1;
//		prescalerbits = 0b010; // ck/8: same for both timers
//		if (_timer == 2 && ocr > 255) {
//			ocr = F_CPU / frequency / 2 / 32 - 1;
//			prescalerbits = 0b011;
//		}
//		if (ocr > 255) {
//			ocr = F_CPU / frequency / 2 / 64 - 1;
//			prescalerbits = _timer == 0 ? 0b011 : 0b100;
//			if (_timer == 2 && ocr > 255) {
//				ocr = F_CPU / frequency / 2 / 128 - 1;
//				prescalerbits = 0b101;
//			}
//			if (ocr > 255) {
//				ocr = F_CPU / frequency / 2 / 256 - 1;
//				prescalerbits = _timer == 0 ? 0b100 : 0b110;
//				if (ocr > 255) {
//					// can't do any better than /1024
//					ocr = F_CPU / frequency / 2 / 1024 - 1;
//					prescalerbits = _timer == 0 ? 0b101 : 0b111;
//				}
//			}
//		}
//	}
//	return prescalerbits;
//}

// frequency (in hertz) and duration (in milliseconds).

void tone(uint8_t pin, unsigned int frequency, unsigned long duration)
{
  uint8_t prescalerbits = 0b001;
  uint32_t ocr = 0;
  int8_t const theTimer = toneBegin(PinType(pin));

  if (theTimer >= 0)
  {
    // Set the pinMode as OUTPUT
    pinMode(pin, OUTPUT);
    
    // if we are using an 8 bit timer, scan through prescalars to find the best fit
#if defined(TCCR2B)
    if (theTimer == 2){
    	prescalerbits = compute_ocr_and_prescalerbits_for_8bit_timer2(ocr, frequency, prescalerbits);
        TCCR2B = (TCCR2B & 0b11111000) | prescalerbits;
      }
#endif

#if defined(TCCR3B)
    if (theTimer == 3) // no other timer is supported here
    {
      // two choices for the 16 bit timers: ck/1 or ck/64
	  prescalerbits = compute_ocr_and_prescalerbits_for_16bit_timer(ocr, frequency, prescalerbits);
      TCCR3B = (TCCR3B & 0b11111000) | prescalerbits;
    }
#endif

    // Calculate the toggle count
	long const toggle_count = compute_toggle_count(duration, frequency);

    // Set the OCR for the given timer,
    // set the toggle count,
    // then turn on the interrupts

#if defined(OCR2A) && defined(TIMSK2) && defined(OCIE2A)

	if (theTimer == 2){
        OCR2A = ocr;
        tone_toggle_count = toggle_count;
        sbi(TIMSK2, OCIE2A);
	}
#endif

#if defined(OCR3A) && defined(TIMSK3) && defined(OCIE3A)
     if (theTimer == 3){
        OCR3A = ocr;
        tone_toggle_count = toggle_count;
        bitSet(TIMSK3, OCIE3A);
     }
#endif
  }
}


// XXX: this function only works properly for timer 2 (the only one we use
// currently).  for the others, it should end the tone, but won't restore
// proper PWM functionality for the timer.
void disableTimer()
{
  switch (tone_timer)
  {

    case 2:
      #if defined(TIMSK2) && defined(OCIE2A)
        bitClear(TIMSK2, OCIE2A); // disable interrupt
      #endif
      #if defined(TCCR2A) && defined(WGM20)
        TCCR2A = (1 << WGM20);
      #endif
      #if defined(TCCR2B) && defined(CS22)
        TCCR2B = (TCCR2B & 0b11111000) | (1 << CS22);
      #endif
      #if defined(OCR2A)
        OCR2A = 0;
      #endif
      break;

#if defined(TIMSK3) && defined(OCIE3A)
    case 3:
      bitClear(TIMSK3, OCIE3A);
      break;
#endif

  }
}


void noTone(uint8_t pin)
{
	if (tone_pin == pin){
		tone_pin = NOT_A_PIN; // not a pin
		disableTimer();
	    *tone_pin_port &= ~tone_pin_mask;  // keep pin low after stop
	}
//  int8_t _timer = -1;
//
//  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
//    if (tone_pins[i] == _pin) {
//      _timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
//      tone_pins[i] = 255;
//      break;
//    }
//  }
//
//  disableTimer(_timer);
//
//  digitalWrite(_pin, 0);
}



#ifdef USE_TIMER2
ISR(TIMER2_COMPA_vect)
{

  if (tone_toggle_count != 0)
  {
    // toggle the pin
    *tone_pin_port ^= tone_pin_mask;

    if (tone_toggle_count > 0)
      tone_toggle_count--;
  }
  else
  {
	    disableTimer();
	    *tone_pin_port &= ~tone_pin_mask;  // keep pin low after stop
	    tone_pin = NOT_A_PIN;
  }
}
#endif


#ifdef USE_TIMER3
ISR(TIMER3_COMPA_vect)
{
  if (tone_toggle_count != 0)
  {
    // toggle the pin
    *tone_pin_port ^= tone_pin_mask;

    if (tone_toggle_count > 0)
      tone_toggle_count--;
  }
  else
  {
    disableTimer();
    *tone_pin_port &= ~tone_pin_mask;  // keep pin low after stop
    tone_pin = NOT_A_PIN;
  }
}
#endif


