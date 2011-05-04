#include	<avr/interrupt.h>

#include	"serial.h"
#include	"analog.h"
#include	"arduino.h"
#include	"config.h"
#include	"clock.h"
#include	"timer.h"
#include	"gcode_parse.h"

uint8_t c;

void loopstuff(void);
void loopstuff() {
	ifclock(CLOCK_FLAG_10MS)
		clock_10ms();
}

void serial_getline(void);
void serial_getline() {
	do {
		if (serial_rxchars()) {
			c = serial_popchar();
			gcode_parse_char(c);
		}

		loopstuff();
	} while (c >= 32);
}

void main(void) __attribute__ ((noreturn));
void main() {
	analog_init();

	serial_init();

	timer_init();

	sei();

	timer_set(87654);

	for (;;) {
		if (serial_rxchars()) {
			serial_getline();
		}
	};
}
