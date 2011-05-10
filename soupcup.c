#include	<stdio.h>

#include	<avr/interrupt.h>

#include	"config.h"

#include	"serial.h"
#include	"analog.h"
#include	"arduino.h"
#include	"clock.h"
#include	"timer.h"
#include	"gcode_parse.h"
#include	"move.h"
#include	"sd.h"
#include	"machine.h"

uint8_t c;

void serial_getline(void);
void serial_getline() {
	do {
		if (serial_rxchars())
			gcode_parse_char(c = serial_popchar());

		loopstuff();
	} while (c >= 32);
}

void main(void) __attribute__ ((noreturn));
void main() {
	stdout = &mystdio;
	stderr = &mystdio;
	stdin = &mystdio;

	analog_init();

	serial_init();

	timer_init();

	sei();

	printf_P(PSTR("start\n"));

	for (;;) {
		// read serial
		if (serial_rxchars())
			serial_getline();

		// read SDCARD
		if (state_flags & STATE_READ_SD) {
			if (file.fs == &fatfs) {
				UINT i;
				do {
					if (f_read(&file, &c, 1, &i) == FR_OK) {
						if (i)
							gcode_parse_char(c);
					}
					else
						i = 0;
				} while ((i != 0) && (c >= 32));
			}
		}

		loopstuff();
	};
}
