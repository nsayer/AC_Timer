/*

    AC Timer
    Copyright (C) 2018 Nicholas W. Sayer

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
    
  */

#define F_CPU (8000000UL)

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>

// 8 MHz / 1024 is 7182.5. To get a millisecond timer out of that,
// divide that by 1000, which is 7.1825, or 7 + 73/400.

#define BASE (7)
#define CYCLE_COUNT (400)
#define LONG_CYCLES (73)

// debounce the button for 50 ms
#define DEBOUNCE_TIME (50)

// Turn off after 30 minutes
#define POWER_OFF_TIME (30 * 60 * 1000UL)

volatile unsigned long millis_cnt;
volatile unsigned int cycle_pos;

unsigned long power_off_time;
unsigned long debounce_start;
unsigned char button_state;

static inline unsigned long millis() {
	unsigned long out;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		out = millis_cnt;
	}
	return out;
}

ISR(TIM0_COMPA_vect) {
	if (++millis_cnt == 0) millis_cnt++; // it's not allowed to be zero.
	if (cycle_pos++ >= LONG_CYCLES) {
		OCR0A = BASE + 1;
	} else {
		OCR0A = BASE + 2;
	}
	if (cycle_pos == CYCLE_COUNT) cycle_pos = 0;
}

unsigned char check_button() {
	unsigned long now = millis();
	if (debounce_start != 0 && now - debounce_start < DEBOUNCE_TIME) {
		return 0;
	}
	debounce_start = 0;
	unsigned char button = !(PINB & _BV(1)); // 0 means down
	if (button == button_state) {
		return 0; // nothing has changed
	} else {
		button_state = button;
		return button != 0;
	}
}

void __ATTR_NORETURN__ main(void) {
	wdt_enable(WDTO_500MS);
	ADCSRA = 0; // DIE, ADC!!! DIE!!!
	ACSR = _BV(ACD); // Turn off analog comparator - but was it ever on anyway?
	power_adc_disable();
	power_usi_disable();
	power_timer1_disable();

	// set up timer 0
	TCCR0A = _BV(WGM01); //
	TCCR0B = _BV(CS02) | _BV(CS00); // prescale by 1024
	TIMSK = _BV(OCIE0A); // OCR0A interrupt only.
	OCR0A = BASE + 1 + 1; // because it's a long cycle, and it's zero-based and inclusive counting
	millis_cnt = 1;
	cycle_pos = 0;

	// pin 0 is the power output
	// pin 1 is the button
	PORTB = _BV(1); // pull-up the button, turn off the output
	DDRB = _BV(0);

	debounce_start = 0;
	button_state = 0;
	power_off_time = 0;

	sei(); // turn on interrupts

	while(1) {
		wdt_reset(); // pet the watchdog

		unsigned long now = millis();

		if (check_button()) {
			if (power_off_time) {
				// power is on
				power_off_time = 0;
				PORTB &= ~_BV(0); // turn it off
				continue;
			} else {
				// schedule the turn-off
				power_off_time = now + POWER_OFF_TIME;
				PORTB |= _BV(0); // turn it on
				continue;
			}
		}

		// Are we there yet?
		if (now > power_off_time) {
			power_off_time = 0;
			PORTB &= ~_BV(0); // turn it off
			continue;
		}
	}
	__builtin_unreachable();
}
