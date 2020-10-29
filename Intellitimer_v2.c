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

// Version 2 hardware controls the actual system power
// with PIN_LED. So imediately upon power-up, we start a cycle
// and when it's time to end, we turn off the LED pin and
// wait patiently for death.

#define F_CPU (1000000UL)

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>

// This is the magic value you have to write to CCP to modify protected registers
#define CCP_MAGIC (0xd8)

// 125 kHz / 8 is 15,625. We want one interrupt per second

// BASE is one less than what we actually want, because it's 0 based inclusive counting
#define BASE (15625 - 1)
// If it's ever required, we can perform fractional counting this way.
#define CYCLE_COUNT (0)
#define LONG_CYCLES (0)

// Turn off after 6 hours
#define POWER_OFF_TIME (6 * 3600UL)

// POWER is the system power
#define BIT_POWER (_BV(1))
// LOAD is the MOSFET for the load
#define BIT_LOAD (_BV(2))

volatile uint16_t seconds_count;

static inline uint16_t __attribute__ ((always_inline)) seconds() {
	uint16_t out;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		out = seconds_count;
	}
	return out;
}

ISR(TIM0_COMPA_vect) {
	seconds_count++;
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

void __ATTR_NORETURN__ main(void) {
	// Set up I/O first - we must hold the power on immediately.
	PORTB = BIT_LOAD | BIT_POWER; // turn on the outputs
	DDRB = BIT_LOAD | BIT_POWER; // power & load are output

	wdt_enable(WDTO_500MS);
	ACSR = _BV(ACD); // Turn off analog comparator
	power_adc_disable();

        VLMCSR = _BV(VLM1); // Set VLM up for VLM1H level.

	// Set clock to 125 kHz
	CCP = CCP_MAGIC;
	CLKMSR = 0; // 8 MHz oscillator source
	CCP = CCP_MAGIC;
	CLKPSR = _BV(CLKPS2) | _BV(CLKPS1); // divide by 64

	// set up timer 0
	TCCR0A = 0;
	TCCR0B = _BV(WGM02) | _BV(CS01); // prescale by 8, CTC mode
	TIMSK0 = _BV(OCIE0A); // OCR0A interrupt only.

#if (CYCLE_COUNT > 0)
	OCR0A = BASE + 1; // we start with the long cycle(s)
#else
	OCR0A = BASE;
#endif

	seconds_count = 0;

	sei(); // turn on interrupts

	while(1) {
		wdt_reset(); // pet the watchdog

		uint16_t now = seconds();

		// Are we there yet?
		if (now >= POWER_OFF_TIME) {
			PORTB &= ~(BIT_LOAD | BIT_POWER); // turn everything off
			while(1) wdt_reset(); // wait to die
		}

		// We want to turn the load on for 120 seconds
		// every 3-ish minutes (but as a prime number to
		// try and horse the interval around a bit)
		// just to exercise the sensor.
		// This apparently makes the load look "busy"
		// and the intelliflow keeps the water on.
		uint8_t is_on = (PORTB & BIT_LOAD) != 0;
		uint8_t should_be = (now % 223) < 120;
		if (is_on ^ should_be) { // if we're changing...
			// either turn the power on or off.
			if (should_be) PORTB |= BIT_LOAD;
			else PORTB &= ~BIT_LOAD;
		}
	}
	__builtin_unreachable();
}
