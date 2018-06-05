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

// Turn on the "warning" light 5 minutes before the end
#define WARN_TIME (25 * 60 * 1000UL)

// the two output bits
// POWER is the optoisolator for the AC power
#define BIT_POWER (_BV(0))
// WARN is the LED that indicates time is low. Pushing the button once during warning
// time will just reset the timer without turning off the power.
#define BIT_WARN (_BV(2))

volatile unsigned long millis_cnt;
volatile unsigned int cycle_pos;

unsigned long power_on_time;
unsigned long debounce_start;
unsigned char in_debounce;
unsigned char button_state;

// This is a millisecond counter.
static inline unsigned long millis() {
	unsigned long out;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		out = millis_cnt;
	}
	return out;
}

ISR(TIM0_COMPA_vect) {
	millis_cnt++;
	if (cycle_pos++ >= LONG_CYCLES) {
		OCR0A = BASE + 1;
	} else {
		OCR0A = BASE + 2;
	}
	if (cycle_pos == CYCLE_COUNT) cycle_pos = 0;
}

unsigned char check_button() {
	unsigned long now = millis();
	if (in_debounce && (now - debounce_start) < DEBOUNCE_TIME) {
		return 0;
	}
	in_debounce = 0;
	unsigned char button = !(PINB & _BV(1)); // 0 means down
	if (button == button_state) {
		return 0; // nothing has changed
	} else {
		debounce_start = now; // start debouncing
		in_debounce = 1;
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
	PORTB = _BV(1); // pull-up the button, turn off the outputs
	DDRB = _BV(0) | _BV(2); // opto and warn LEDs output

	in_debounce = 0;
	button_state = 0;

	sei(); // turn on interrupts

	while(1) {
		wdt_reset(); // pet the watchdog

		unsigned long now = millis();

		if (check_button()) {
			if (PORTB & BIT_POWER) {
				// power is on
				if (PORTB & BIT_WARN) { // warn is turned on
					power_on_time = now; // reset the timer only
					PORTB &= ~BIT_WARN; // turn off warn
				} else {
					PORTB &= ~(BIT_POWER | BIT_WARN); // turn it off (and warn too)
				}
				continue;
			} else {
				// schedule the turn-off
				power_on_time = now;
				PORTB |= BIT_POWER; // turn it on
				continue;
			}
		}

		// Are we there yet?
		if ((PORTB & BIT_POWER) && ((now - power_on_time) > POWER_OFF_TIME)) {
			PORTB &= ~(BIT_POWER | BIT_WARN); // turn it off (and warn too)
			continue;
		}

		// If the power is on, and it's past the warning time and the WARN light is off...
		if (((PORTB & (BIT_POWER | BIT_WARN)) == BIT_POWER) && ((now - power_on_time) > WARN_TIME)) {
			PORTB |= BIT_WARN; // turn on the warning light
			continue;
		}
	}
	__builtin_unreachable();
}
