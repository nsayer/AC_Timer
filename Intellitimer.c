/*

    Intelliflow Timer
    Copyright (C) 2020 Nicholas W. Sayer

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

#define F_CPU (1000000UL)

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>

// 1 MHz / 8 is 125,000. Divide that by 1000 to get a millisecond counter.

// BASE is one less than what we actually want, because it's 0 based inclusive counting
#define BASE (125 - 1)
// If it's ever required, we can perform fractional counting this way.
#define CYCLE_COUNT (0)
#define LONG_CYCLES (0)

// debounce the button for 50 ms
#define DEBOUNCE_MILLIS (50)

// Turn off after 6 hours
#define POWER_OFF_TIME (6 * 3600UL)

// BUTTON is the button. low = pushed
#define BIT_BUTTON (_BV(0))
// POWER is the MOSFET for the load
#define BIT_POWER (_BV(2))
// LED is the indicator light
#define BIT_LED (_BV(1))

volatile uint16_t millis_count, seconds_count;

uint16_t power_on_time;

static inline uint16_t __attribute__ ((always_inline)) millis() {
	uint16_t out;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		out = millis_count;
	}
	return out;
}

static inline uint16_t __attribute__ ((always_inline)) seconds() {
	uint16_t out;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		out = seconds_count;
	}
	return out;
}

ISR(TIM0_COMPA_vect) {
	if (++millis_count == 1000) {
		seconds_count++;
		millis_count = 0;
	}
#if (CYCLE_COUNT > 0)
	static uint16_t cycle_pos;
	if (cycle_pos++ >= LONG_CYCLES) {
		OCR0A = BASE; // short cycle
	} else {
		OCR0A = BASE + 1; // long cycle
	}
	if (cycle_pos == CYCLE_COUNT) cycle_pos = 0;
#endif
}

static inline uint8_t __attribute__ ((always_inline)) check_button() {
	static uint16_t debounce_time;
	static uint8_t button_state;
	static uint8_t in_debounce;
        uint16_t now = millis();
        uint8_t status = !(PINB & BIT_BUTTON);
        if ((button_state == 0) ^ (status == 0)) {
                // It changed. It must stay stable for a debounce period before we report.
                button_state = status;
                debounce_time = now;
                in_debounce = 1;
                return 0;
        }
        if (in_debounce == 0) return 0; // we're not waiting to report
	int16_t delta = now - debounce_time;
	if (delta < 0) delta += 1000;
        if (delta >= DEBOUNCE_MILLIS) {
                in_debounce = 0; // debounce ended without change.
                return status;
        }
        return 0;
}

void __ATTR_NORETURN__ main(void) {
	wdt_enable(WDTO_500MS);
	ACSR = _BV(ACD); // Turn off analog comparator
	power_adc_disable();

	// set up timer 0
	TCCR0A = 0;
	TCCR0B = _BV(WGM02) | _BV(CS01); // prescale by 8, CTC mode
	TIMSK0 = _BV(OCIE0A); // OCR0A interrupt only.

#if (CYCLE_COUNT > 0)
	OCR0A = BASE + 1; // we start with the long cycle(s)
#else
	OCR0A = BASE;
#endif

	millis_count = seconds_count = 0;

	PORTB = 0; // turn off the outputs
	PUEB = BIT_BUTTON | _BV(1); // pull-up the button and the unused pin

	DDRB = BIT_POWER; // power is output

	sei(); // turn on interrupts

	while(1) {
		wdt_reset(); // pet the watchdog

		uint16_t now = seconds();

		if (check_button()) {
			power_on_time = now; // reset the timer
			PORTB |= BIT_LED | BIT_POWER; // Power up
			continue;
		}

		// Are we there yet?
		if ((PORTB & BIT_LED) && (now - power_on_time >= POWER_OFF_TIME)) {
			PORTB &= ~(BIT_POWER | BIT_LED); // turn it all off
			continue;
		}
		if (PORTB & BIT_LED) {
			// We want to turn the load on for 60 seconds
			// every 3-ish minutes (but as a prime number to
			// try and horse the interval around a bit)
			// just to exercise the sensor.
			// This apparently makes the load look "busy"
			// and the intelliflow keeps the water on.
			uint8_t is_on = (PORTB & BIT_POWER) != 0;
			uint8_t should_be = ((now - power_on_time) % 223) < 60;
			if (is_on ^ should_be) { // if we're changing...
				// either turn the power on or off.
				if (should_be) PORTB |= BIT_POWER;
				else PORTB &= ~BIT_POWER;
			}
		}
	}
	__builtin_unreachable();
}
