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

#define F_CPU (1000000UL)

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>

// 1 MHz / 8 is 125,000. Divide that by 10 to get a 10th of a second counter.

#define BASE (12500)
#define CYCLE_COUNT (0)
#define LONG_CYCLES (0)

// debounce the button for 100 ms
#define DEBOUNCE_TIME (1)

// Turn off after 30 minutes
#define POWER_OFF_TIME (30)

// Turn on the "warning" light 5 minutes before the end
#define WARN_TIME (25)

// the two output bits
// POWER is the optoisolator for the AC power
#define BIT_POWER (_BV(0))
// WARN is the LED that indicates time is low. Pushing the button once during warning
// time will just reset the timer without turning off the power.
#define BIT_WARN (_BV(2))
// the input bit
// BUTTON is the button. low = pushed
#define BIT_BUTTON (_BV(1))

volatile uint16_t ticks_cnt;

uint16_t power_on_time;

// This is a second counter, but it skips over 0
static inline uint16_t __attribute__ ((always_inline)) ticks() {
	uint16_t out;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		out = ticks_cnt;
	}
	return out;
}

ISR(TIM0_COMPA_vect) {
	if (++ticks_cnt == 0) ticks_cnt++; // it can't be zero
#if (CYCLE_COUNT > 0)
	static uint16_t cycle_pos;
	if (cycle_pos++ >= LONG_CYCLES) {
		OCR0A = BASE + 1;
	} else {
		OCR0A = BASE + 2;
	}
	if (cycle_pos == CYCLE_COUNT) cycle_pos = 0;
#endif
}

static inline uint8_t __attribute__ ((always_inline)) check_button() {
	static uint16_t debounce_time;
	static uint8_t button_state;
        uint16_t now = ticks();
        uint8_t status = !(PINB & BIT_BUTTON);
        if ((button_state == 0) ^ (status == 0)) {
                // It changed. It must stay stable for a debounce period before we report.
                button_state = status;
                debounce_time = now;
                return 0;
        }
        if (debounce_time == 0) return 0; // we're not waiting to report
        if (now - debounce_time > DEBOUNCE_TIME) {
                debounce_time = 0; // debounce ended without change.
                return status;
        }
        return 0;
}

void __ATTR_NORETURN__ main(void) {
#if defined (__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined (__AVR_ATtiny85__)
	wdt_enable(WDTO_500MS);
#else
	// This doesn't take interrupts into account, but they're not on right now anyway.
	CCP = 0xd8;
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP0); // enable watchdog, half a second.
#endif
	ACSR = _BV(ACD); // Turn off analog comparator
	power_adc_disable();
#if defined (__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined (__AVR_ATtiny85__)
	power_usi_disable();
	power_timer1_disable();
#endif

	// set up timer 0
#if defined (__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined (__AVR_ATtiny85__)
	TCCR0A = _BV(WGM01); // CTC mode
	TCCR0B = _BV(CS01); // prescale by 8
	TIMSK = _BV(OCIE0A); // OCR0A interrupt only.
#else
	TCCR0A = 0;
	TCCR0B = _BV(WGM02) | _BV(CS01); // prescale by 8, CTC mode
	TIMSK0 = _BV(OCIE0A); // OCR0A interrupt only.
#endif
#if (CYCLE_COUNT > 0)
	OCR0A = BASE + 2; // because it's a long cycle, and it's zero-based and inclusive counting
#else
	OCR0A = BASE + 1; // it's zero-based and inclusive counting
#endif
	ticks_cnt = 1;

#if defined (__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined (__AVR_ATtiny85__)
	PORTB = BIT_BUTTON; // pull-up the button, turn off the outputs
#else
	PUEB = BIT_BUTTON; // pull-up the button
#endif

	DDRB = BIT_POWER | BIT_WARN; // opto and warn LEDs output

	sei(); // turn on interrupts

	while(1) {
		wdt_reset(); // pet the watchdog

		uint16_t now = ticks();

		if (check_button()) {
			if (PORTB & BIT_POWER) {
				// power is on, so either turn it off or cancel the WARN
				if (PORTB & BIT_WARN) { // warn is turned on
					power_on_time = now; // reset the timer only
					PORTB &= ~BIT_WARN; // turn off warn
				} else {
					PORTB &= ~(BIT_POWER | BIT_WARN); // turn it off (and warn too)
				}
			} else {
				// power is off, so turn it on and schedule the turn-off
				power_on_time = now;
				PORTB |= BIT_POWER; // turn it on
			}
			continue;
		}

		uint16_t elapsed_minutes =  ((now - power_on_time) / (10UL * 60));

		// Are we there yet?
		if ((PORTB & BIT_POWER) && (elapsed_minutes > POWER_OFF_TIME)) {
			PORTB &= ~(BIT_POWER | BIT_WARN); // turn it off (and warn too)
			continue;
		}

		// If the power is on, and it's past the warning time and the WARN light is off...
		if (((PORTB & (BIT_POWER | BIT_WARN)) == BIT_POWER) && (elapsed_minutes > WARN_TIME)) {
			PORTB |= BIT_WARN; // turn on the warning light
			continue;
		}

	}
	__builtin_unreachable();
}
