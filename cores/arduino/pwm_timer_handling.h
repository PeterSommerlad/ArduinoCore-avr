// generated from createAnalogWriteTemplateSpecializations.sh
template<timer_values t>
void setPWMValue(int val);

// special cases taken from avr
#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
template <> inline void 
setPWMValue<TIMER0A>(int val){
	sbi(TCCR0, COM00);
	OCR0 = val;
}
#endif
#if defined(TCCR2) && defined(COM21)
template <> inline void 
setPWMValue<TIMER2>(int val){
	sbi(TCCR2, COM21);
	OCR2 = val;
}
#endif
#if defined(TCCR4A) 
template <> inline void 
setPWMValue<TIMER4A>(int val){
	sbi(TCCR4A, COM4A1);
	#if defined(COM4A0)
	cbi(TCCR4A, COM4A0);
	#endif
	OCR4A = val;
}
#endif
#if defined(TCCR4C) && defined(COM4D1) 
template <> inline void 
setPWMValue<TIMER4D>(int val){
	sbi(TCCR4C, COM4D1);
	#if defined(COM4D0)
	cbi(TCCR4C, COM4D0);
	#endif
	OCR4D = val;
}
#endif

#if defined(TCCR0A) && defined(COM0A1)
template <> inline void 
setPWMValue<TIMER0A>(int val){
	sbi(TCCR0A, COM0A1);
	OCR0A = val;
}
#endif
#if defined(TCCR0A) && defined(COM0B1)
template <> inline void 
setPWMValue<TIMER0B>(int val){
	sbi(TCCR0A, COM0B1);
	OCR0B = val;
}
#endif
#if defined(TCCR1A) && defined(COM1A1)
template <> inline void 
setPWMValue<TIMER1A>(int val){
	sbi(TCCR1A, COM1A1);
	OCR1A = val;
}
#endif
#if defined(TCCR1A) && defined(COM1B1)
template <> inline void 
setPWMValue<TIMER1B>(int val){
	sbi(TCCR1A, COM1B1);
	OCR1B = val;
}
#endif
#if defined(TCCR1A) && defined(COM1C1)
template <> inline void 
setPWMValue<TIMER1C>(int val){
	sbi(TCCR1A, COM1C1);
	OCR1C = val;
}
#endif
#if defined(TCCR2A) && defined(COM2A1)
template <> inline void 
setPWMValue<TIMER2A>(int val){
	sbi(TCCR2A, COM2A1);
	OCR2A = val;
}
#endif
#if defined(TCCR2A) && defined(COM2B1)
template <> inline void 
setPWMValue<TIMER2B>(int val){
	sbi(TCCR2A, COM2B1);
	OCR2B = val;
}
#endif
#if defined(TCCR3A) && defined(COM3A1)
template <> inline void 
setPWMValue<TIMER3A>(int val){
	sbi(TCCR3A, COM3A1);
	OCR3A = val;
}
#endif
#if defined(TCCR3A) && defined(COM3B1)
template <> inline void 
setPWMValue<TIMER3B>(int val){
	sbi(TCCR3A, COM3B1);
	OCR3B = val;
}
#endif
#if defined(TCCR3A) && defined(COM3C1)
template <> inline void 
setPWMValue<TIMER3C>(int val){
	sbi(TCCR3A, COM3C1);
	OCR3C = val;
}
#endif
#if defined(TCCR4A) && defined(COM4B1)
template <> inline void 
setPWMValue<TIMER4B>(int val){
	sbi(TCCR4A, COM4B1);
	OCR4B = val;
}
#endif
#if defined(TCCR4A) && defined(COM4C1)
template <> inline void 
setPWMValue<TIMER4C>(int val){
	sbi(TCCR4A, COM4C1);
	OCR4C = val;
}
#endif
#if defined(TCCR5A) && defined(COM5A1)
template <> inline void 
setPWMValue<TIMER5A>(int val){
	sbi(TCCR5A, COM5A1);
	OCR5A = val;
}
#endif
#if defined(TCCR5A) && defined(COM5B1)
template <> inline void 
setPWMValue<TIMER5B>(int val){
	sbi(TCCR5A, COM5B1);
	OCR5B = val;
}
#endif
#if defined(TCCR5A) && defined(COM5C1)
template <> inline void 
setPWMValue<TIMER5C>(int val){
	sbi(TCCR5A, COM5C1);
	OCR5C = val;
}
#endif
template<PinType pin>
void setPWMValuePin(int val){
	if constexpr (digitalPinToTimer(pin) != NOT_ON_TIMER)
		setPWMValue<digitalPinToTimer(pin)>(val);
	else {
		if (val < 128) {
			digitalWrite<pin, LOW>();
		} else {
			digitalWrite<pin, HIGH>();
		}
	}
}



// make it a compile error if called with NOT_ON_TIMER
template<timer_values t>
inline void analog_timer_turnoff() noexcept;

template<PinType pin>
inline void analog_pin_to_timer_turnoff() noexcept
{
	if constexpr(digital_pin_to_timer_PS(pin) != NOT_ON_TIMER)
		analog_timer_turnoff<digital_pin_to_timer_PS(pin)>();
}


#if defined(TCCR2) && defined(COM21)
template <> inline void 
analog_timer_turnoff<TIMER2>() noexcept {
	cbi(TCCR2, COM21);
}
#endif

#if defined(TCCR4C) && defined(COM4D1)
template <> inline void 
analog_timer_turnoff<TIMER4D>() noexcept {
	cbi(TCCR4C, COM4D1);
}
#endif

#if defined(TCCR0A) && defined(COM0A1)
template <> inline void 
analog_timer_turnoff<TIMER0A>() noexcept {
	cbi(TCCR0A, COM0A1);
}
#endif
#if defined(TCCR0A) && defined(COM0B1)
template <> inline void 
analog_timer_turnoff<TIMER0B>() noexcept {
	cbi(TCCR0A, COM0B1);
}
#endif
#if defined(TCCR1A) && defined(COM1A1)
template <> inline void 
analog_timer_turnoff<TIMER1A>() noexcept {
	cbi(TCCR1A, COM1A1);
}
#endif
#if defined(TCCR1A) && defined(COM1B1)
template <> inline void 
analog_timer_turnoff<TIMER1B>() noexcept {
	cbi(TCCR1A, COM1B1);
}
#endif
#if defined(TCCR1A) && defined(COM1C1)
template <> inline void 
analog_timer_turnoff<TIMER1C>() noexcept {
	cbi(TCCR1A, COM1C1);
}
#endif
#if defined(TCCR2A) && defined(COM2A1)
template <> inline void 
analog_timer_turnoff<TIMER2A>() noexcept {
	cbi(TCCR2A, COM2A1);
}
#endif
#if defined(TCCR2A) && defined(COM2B1)
template <> inline void 
analog_timer_turnoff<TIMER2B>() noexcept {
	cbi(TCCR2A, COM2B1);
}
#endif
#if defined(TCCR3A) && defined(COM3A1)
template <> inline void 
analog_timer_turnoff<TIMER3A>() noexcept {
	cbi(TCCR3A, COM3A1);
}
#endif
#if defined(TCCR3A) && defined(COM3B1)
template <> inline void 
analog_timer_turnoff<TIMER3B>() noexcept {
	cbi(TCCR3A, COM3B1);
}
#endif
#if defined(TCCR3A) && defined(COM3C1)
template <> inline void 
analog_timer_turnoff<TIMER3C>() noexcept {
	cbi(TCCR3A, COM3C1);
}
#endif
#if defined(TCCR4A) && defined(COM4A1)
template <> inline void 
analog_timer_turnoff<TIMER4A>() noexcept {
	cbi(TCCR4A, COM4A1);
}
#endif
#if defined(TCCR4A) && defined(COM4B1)
template <> inline void 
analog_timer_turnoff<TIMER4B>() noexcept {
	cbi(TCCR4A, COM4B1);
}
#endif
#if defined(TCCR4A) && defined(COM4C1)
template <> inline void 
analog_timer_turnoff<TIMER4C>() noexcept {
	cbi(TCCR4A, COM4C1);
}
#endif
#if defined(TCCR5A) && defined(COM5A1)
template <> inline void 
analog_timer_turnoff<TIMER5A>() noexcept {
	cbi(TCCR5A, COM5A1);
}
#endif
#if defined(TCCR5A) && defined(COM5B1)
template <> inline void 
analog_timer_turnoff<TIMER5B>() noexcept {
	cbi(TCCR5A, COM5B1);
}
#endif
#if defined(TCCR5A) && defined(COM5C1)
template <> inline void 
analog_timer_turnoff<TIMER5C>() noexcept {
	cbi(TCCR5A, COM5C1);
}
#endif
