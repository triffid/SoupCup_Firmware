#include	"timer.h"

/** \file
	\brief Timer management - step pulse clock and system clock

	Teacup uses timer1 to generate both step pulse clock and system clock.

	We achieve this by using the output compare registers to generate the two clocks while the timer free-runs.

	Teacup has tried numerous timer management methods, and this is the best so far.
*/

#include	<avr/interrupt.h>
#include	<stddef.h>

#include	"arduino.h"
#include	"config.h"

/// how often we overflow and update our clock; with F_CPU=16MHz, max is < 4.096ms (TICK_TIME = 65535)
#define		TICK_TIME			2 MS
/// convert back to ms from cpu ticks so our system clock runs properly if you change TICK_TIME
#define		TICK_TIME_MS	(TICK_TIME / (F_CPU / 1000))

/// time until next step, as output compare register is too small for long step times
volatile uint32_t	next_step_time;

/// tick callback
void_func_ptr tick_callback = NULL;

/// every time our clock fires, we increment this so we know when 10ms has elapsed
uint8_t						clock_counter_10ms = 0;
/// keep track of when 250ms has elapsed
uint8_t						clock_counter_250ms = 0;
/// keep track of when 1s has elapsed
uint8_t						clock_counter_1s = 0;
/// flags to tell main loop when above have elapsed
volatile uint8_t	clock_flag = 0;

/// comparator B is the system clock, happens every TICK_TIME
ISR(TIMER1_COMPB_vect) {
	// set output compare register to the next clock tick
	OCR1B = (OCR1B + TICK_TIME) & 0xFFFF;

	/*
	clock stuff
	*/
	clock_counter_10ms += TICK_TIME_MS;
	if (clock_counter_10ms >= 10) {
		clock_counter_10ms -= 10;
		clock_flag |= CLOCK_FLAG_10MS;

		clock_counter_250ms += 1;
		if (clock_counter_250ms >= 25) {
			clock_counter_250ms -= 25;
			clock_flag |= CLOCK_FLAG_250MS;

			clock_counter_1s += 1;
			if (clock_counter_1s >= 4) {
				clock_counter_1s -= 4;
				clock_flag |= CLOCK_FLAG_1S;
			}
		}
	}
}

/// comparator A is the step timer. It has higher priority then B.
ISR(TIMER1_COMPA_vect) {
	// Check if this is a real step, or just a next_step_time "overflow"
	if (next_step_time == 0) {
		// placeholder so we can tell if we set a new time- at 16MHz this is about 4 1/2 minutes
		next_step_time = 0xFFFFFFFF;
		// step!
		if (tick_callback)
			tick_callback();
		// if we didn't set a new time, disable interrupt
		if (next_step_time == 0xFFFFFFFF)
			TIMSK1 &= ~MASK(OCIE1A);

		return;
	}

	// similar algorithm as described in setTimer below.
	if (next_step_time < 65536) {
		OCR1A = (OCR1A + next_step_time) & 0xFFFF;
		next_step_time = 0;
	}
	else if (next_step_time < 131072) {
		uint16_t nd = (next_step_time >> 1L);
		OCR1A = (OCR1A + nd) & 0xFFFF;
		next_step_time -= nd;
	}
	else
		next_step_time -= 65536;
}

void timer_register_callback(void_func_ptr p) {
	tick_callback = p;
}

/// initialise timer and enable system clock interrupt.
/// step interrupt is enabled later when we start using it
void timer_init()
{
	// no outputs
	TCCR1A = 0;
	// Normal Mode
	TCCR1B = MASK(CS10);
	// set up "clock" comparator for first tick
	OCR1B = TICK_TIME & 0xFFFF;
	// enable interrupt
	TIMSK1 = MASK(OCIE1B);
}

/// specify how long until the step timer should fire
void timer_set(uint32_t delay)
{
	// re-enable clock interrupt in case we're recovering from emergency stop
	TIMSK1 |= MASK(OCIE1B);

	if (tick_callback == NULL)
		return;

	if (delay > 0) {
		uint16_t step_start;

		// save interrupt flag
		uint8_t sreg = SREG;
		// disable interrupts
		cli();

		if (TIMSK1 & MASK(OCIE1A))
			step_start = OCR1A;
		else
			step_start = TCNT1;

		// if we're firing in less than one overflow
		if (delay < 65536) {
			// just write the register
			OCR1A = step_start + delay;
			// don't wait any longer when we fire
			next_step_time = 0;
		}
		// if we're firing after the next overflow
		else if (delay < 131072) {
			// fire early to make the timing easier for the final countdown.
			// in the case where (delay - 65536) == small number, this will cause us to fire at
			// around 32800, leaving a similar amount of ticks for the next firing
			// instead of waiting until it's almost time then having to scramble or miss.
			OCR1A = (step_start + (delay >> 1L)) & 0xFFFF;
			next_step_time = delay - (delay >> 1L);
		}
		// we're not firing for a while, simply settle in for the long haul
		else {
			OCR1A = step_start;
			next_step_time = delay - 65536;
		}

		// enable interrupt
		TIMSK1 |= MASK(OCIE1A);

		// restore interrupt flag
		SREG = sreg;
	} else {
		// flag: move has ended
		TIMSK1 &= ~MASK(OCIE1A);
	}
}

/// stop timers - emergency stop
void timer_stop() {
	// disable all interrupts
	TIMSK1 = 0;
}
