#include	"gcode_parse.h"

#include	<stdlib.h>
#include	<stdio.h>
#include	<avr/pgmspace.h>
#include	<math.h>

#include	"machine.h"
#include	"move.h"
#include	"serial.h"

uint8_t linebuf_p = 0;
uint8_t linelen = 0;
uint8_t linebuf[GCODE_BUFFER_SIZE] __attribute__ ((section (".bss")));

uint32_t	linenumber_expect = 1;

uint32_t words_mask;
word words[32];

// uint16_t r = 0;
union {
	float f;
	uint32_t u;
	int32_t i;
} val;
uint8_t *p = NULL;
uint8_t checksum;
uint8_t i;

#define amask(c)	((axis_mask & AXIS_ ## c)?255:0)

void movemath(void) {
	float xs = 0.0, ys = 0.0, zs = 0.0, es = 0.0;
	float da, dd;

	uint8_t axis_mask = 0;

	int32_t x_steps = 0, y_steps = 0, z_steps = 0, e_steps = 0;
	float xd = 0.0, yd = 0.0, zd = 0.0, ed = 0.0;

// 	printf_P(PSTR("doing math- distance..."));

	if (seen('X')) {
		x_steps = labs(lround(words[idx('X')].f * x_steps_per_mm) - s_endpoint.X);
// 		s_endpoint.X += x_steps;
		xd = ((float) x_steps) / x_steps_per_mm;
		if (x_steps != 0)
			axis_mask |= AXIS_X;
	}
	if (seen('Y')) {
		y_steps = labs(lround(words[idx('Y')].f * y_steps_per_mm) - s_endpoint.Y);
// 		s_endpoint.Y += y_steps;
		yd = ((float) y_steps) / y_steps_per_mm;
		if (y_steps != 0)
			axis_mask |= AXIS_Y;
	}
	if (seen('Z')) {
		z_steps = labs(lround(words[idx('Z')].f * z_steps_per_mm) - s_endpoint.Z);
// 		s_endpoint.Z += z_steps;
		zd = ((float) z_steps) / z_steps_per_mm;
		if (z_steps != 0)
			axis_mask |= AXIS_Z;
	}
	if (seen('E')) {
		e_steps = labs(lround(words[idx('E')].f * e_steps_per_mm) - s_endpoint.E);
// 		s_endpoint.E += e_steps;
		ed = ((float) e_steps) / e_steps_per_mm;
		if (e_steps != 0)
			axis_mask |= AXIS_E;
	}

	if (axis_mask == 0)
		return;

	// work out distance
	if (seen(DISTANCE) == 0) {
		float d;
		if (amask(X) && amask(Y) && amask(Z) == 0)
			d = hypot(xd, yd);
		else if (amask(X) == 0 && amask(Y) == 0 && amask(Z))
			d = zd;
		else if (amask(X) == 0 && amask(Y) == 0 && amask(Z) == 0 && amask(E))
			d = ed;
		else {
			d = hypot(xd, yd);
			d = hypot(d, zd);
		}
		set_f(DISTANCE, d);
	}

// 	printf_P(PSTR("(%f) OK, time..."), words[idx(DISTANCE)].f);

	// work out time
	// time = distance / speed
	if (seen(TIME) == 0) {
		float f = words[idx('F')].f / 60.0;
		if (f < 1.0)
			f = 1.0;
		set_f(TIME, words[idx(DISTANCE)].f / f);
	}

// 	printf_P(PSTR("(%f) OK, accel..."), words[idx(TIME)].f);

	if (amask(X)) {
		if (seen(XS) == 0)
			set_f(XS, xs = (xd / words[idx(TIME)].f));
		else
			xs = words[idx(XS)].f;
	}

	if (amask(Y)) {
		if (seen(YS) == 0)
			set_f(YS, ys = (yd / words[idx(TIME)].f));
		else
			ys = words[idx(YS)].f;
	}

	if (amask(Z)) {
		if (seen(ZS) == 0)
			set_f(ZS, zs = (zd / words[idx(TIME)].f));
		else
			zs = words[idx(ZS)].f;
	}

	if (amask(E)) {
		if (seen(ES) == 0)
			set_f(ES, es = (ed / words[idx(TIME)].f));
		else
			es = words[idx(ES)].f;
	}

// 	printf_P(PSTR("[speeds: %g,%g,%g,%g]"), xs, ys, zs, es);

	// work out accel profiles
	if (seen(ACCEL_DISTANCE) == 0) {
		// distance to accel to speed w at acceleration w' = w**2 / 2w'
		da = 0.0;

		if (amask(X))
			da = fmax(da, square(xs) / 2.0 / x_accel);
		if (amask(Y))
			da = fmax(da, square(ys) / 2.0 / y_accel);
		if (amask(Z))
			da = fmax(da, square(zs) / 2.0 / z_accel);
		if (amask(E))
			da = fmax(da, square(es) / 2.0 / e_accel);

		set_f(ACCEL_DISTANCE, da);
	}
	else
		da = words[idx(ACCEL_DISTANCE)].f;

	// now apply this accel ramp to other axes
	// d = s**2/2a, therefore a = s**2/2d

	// 				printf_P(PSTR("[Accel: %g,%g,%g,%g]"), xa, ya, za, ea);

	if (seen(DECEL_DISTANCE) == 0) {
		// distance to decel to speed w at deceleration w' = w**2 / 2 / w'
		dd = 0.0;
		if (amask(X))
			dd = fmax(dd, square(xs) / 2.0 / x_decel);
		if (amask(Y))
			dd = fmax(dd, square(ys) / 2.0 / y_decel);
		if (amask(Z))
			dd = fmax(dd, square(zs) / 2.0 / z_decel);
		if (amask(E))
			dd = fmax(dd, square(es) / 2.0 / e_decel);
		set_f(DECEL_DISTANCE, dd);
	}
	else
		dd = words[idx(DECEL_DISTANCE)].f;

	if (amask(X)) {
		if (seen(XMC) == 0)
			set_i(XMC, lround(((float) F_CPU) / (xs * x_steps_per_mm)) * 256.0);
		if (seen(XC0) == 0)
			set_i(XC0, lround(((float) F_CPU) * sqrt(4.0 * da / square(xs) / x_steps_per_mm) * 256.0));
		if (seen(XDS) == 0)
			set_i(XDS, lround(square(xs) * x_steps_per_mm / 2.0 / dd));
	}

	if (amask(Y)) {
		if (seen(YMC) == 0)
			set_i(YMC, lround(((float) F_CPU) / (ys * y_steps_per_mm)) * 256.0);
		if (seen(YC0) == 0)
			set_i(YC0, lround(((float) F_CPU) * sqrt(4.0 * da / square(ys) / y_steps_per_mm) * 256.0));
		if (seen(YDS) == 0)
			set_i(YDS, lround(square(ys) * y_steps_per_mm / 2.0 / dd));
	}

	if (amask(Z)) {
		if (seen(ZMC) == 0)
			set_i(ZMC, lround(((float) F_CPU) / (zs * z_steps_per_mm)) * 256.0);
		if (seen(ZC0) == 0)
			set_i(ZC0, lround(((float) F_CPU) * sqrt(4.0 * da / square(zs) / z_steps_per_mm) * 256.0));
		if (seen(ZDS) == 0)
			set_i(ZDS, lround(square(zs) * z_steps_per_mm / 2.0 / dd));
	}

	if (amask(E)) {
		if (seen(EMC) == 0)
			set_i(EMC, lround(((float) F_CPU) / (es * e_steps_per_mm)) * 256.0);
		if (seen(EC0) == 0)
			set_i(EC0, lround(((float) F_CPU) * sqrt(4.0 * da / square(es) / e_steps_per_mm) * 256.0));
		if (seen(EDS) == 0)
			set_i(EDS, lround(square(es) * e_steps_per_mm / 2.0 / dd));
	}

	// 	printf_P(PSTR("(%lu,%lu,%lu,%lu/%lu,%lu,%lu,%lu) OK, decel..."), words[idx(XC0)].u, words[idx(YC0)].u, words[idx(ZC0)].u, words[idx(EC0)].u, words[idx(XMC)].u, words[idx(YMC)].u, words[idx(ZMC)].u, words[idx(EMC)].u);

// 	printf_P(PSTR(" OK\n"));
}

void gcode_process(void) {
	if (seen('M')) {
		uint16_t m = words[idx('M')].u & 0xFFFF;
		switch(m) {
			case 114:
				update_position();
				printf_P(PSTR(" Global X:%gmm Y:%gmm Z:%gmm E:%gmm"), f_global.X, f_global.Y, f_global.Z, f_global.E);
				printf_P(PSTR(" Steps X:%ld Y:%ld Z:%ld E:%ld"), s_global.X, s_global.Y, s_global.Z, s_global.E);
				printf_P(PSTR(" Endpoint X:%gmm Y:%gmm Z:%gmm E:%gmm"), ((float) s_endpoint.X) / x_steps_per_mm, ((float) s_endpoint.Y) / y_steps_per_mm, ((float) s_endpoint.Z) / z_steps_per_mm, ((float) s_endpoint.E) / e_steps_per_mm);
				break;
			case 115:
				printf_P(PSTR(" FIRMWARE_NAME:SoupCup FEATURES:0/dual-band,1/linenumbers-forceconsecutive"));
				break;
			case 118:
				do {
					uint8_t features = lround(words[idx('P')].f) & 0xFF;
					features <<= 4;
					state_flags = (state_flags & 0xF) | features;
				} while (0);
				break;
			default:
				printf_P(PSTR(" Unsupported M-code: %u"), m);
				break;
		}
	}
	else if (seen('G')) {
		uint8_t g = words[idx('G')].u & 0xFF;
		switch(g) {
			case 1:
				// G1

				movemath();

				enqueue();

				f_current.X = words[idx('X')].f;
				f_current.Y = words[idx('Y')].f;
				f_current.Z = words[idx('Z')].f;
				f_current.E = words[idx('E')].f;

				f_global.X = f_current.X - f_offset.X;
				f_global.Y = f_current.Y - f_offset.Y;
				f_global.Z = f_current.Z - f_offset.Z;
				f_global.E = f_current.E - f_offset.E;
				break;
			default:
				printf_P(PSTR(" Unsupported G-code: %u"), g);
				break;
		}
	}
}

void gcode_parse_char(uint8_t c) {
	linebuf[linebuf_p++] = c;

	if (c == '*')
		words_mask |= (1L << idx(CHECKSUM));
	else if (seen(CHECKSUM) == 0)
		checksum ^= c;

	if ((c < 32) || (linebuf_p >= (GCODE_BUFFER_SIZE - 1))) {
		linebuf[linebuf_p] = 0;

		linelen = linebuf_p;
		linebuf_p = 0;

		do {
			p = NULL;
// 			printf_P(PSTR("Parsing... "));
			for (; (linebuf[linebuf_p] < 'A' || linebuf[linebuf_p] > ('A' + 31)) && linebuf_p < GCODE_BUFFER_SIZE && linebuf[linebuf_p] != '*'; linebuf_p++)
				if (linebuf[linebuf_p] < 32)
					break;

			c = linebuf[linebuf_p++];

			if (c >= 'a')
				c -= ('a' - 'A');

			if (c >= 'A' && c <= ('A' + 31)) {
// 				printf_P(PSTR("Found '%c'(0x%2x) at %u... "), c, c, linebuf_p - 1);
				switch (c) {
					case 'G':
					case 'M':
					case 'N':
					case 'T':
					case idx(XC0):
					case idx(YC0):
					case idx(ZC0):
					case idx(EC0):
					case idx(XMC):
					case idx(YMC):
					case idx(ZMC):
					case idx(EMC):
					case idx(XDS):
					case idx(YDS):
					case idx(ZDS):
					case idx(EDS):
// 						printf_P(PSTR("Looking for an integer at %u... "), linebuf_p);
						val.u = strtoul((const char *) &linebuf[linebuf_p], (char **) &p, 0);
						if (p > &linebuf[linebuf_p]) {
// 							printf_P(PSTR("Found %lu consuming %d characters\n"), val.i, (p - linebuf) - linebuf_p);
							linebuf_p = p - linebuf;
							words[idx(c)].u = val.u;
						}
						break;
					default:
// 						printf_P(PSTR("Looking for a float at %u... "), linebuf_p);
						val.f = strtod((const char *) &linebuf[linebuf_p], (char **) &p);
						if (p > &linebuf[linebuf_p]) {
// 							printf_P(PSTR("Found %g consuming %d characters\n"), val.f, (p - linebuf) - linebuf_p);
							linebuf_p = p - linebuf;
							words[idx(c)].f = val.f;
						}
						break;
				}
				words_mask |= (1L << (idx(c)));
			}
			else if (c == '*') {
// 				printf_P(PSTR("Looking for a checksum at %u... "), linebuf_p);
				val.u = strtoul((const char *) &linebuf[linebuf_p], (char **) &p, 0);
				if (p > &linebuf[linebuf_p]) {
// 					printf_P(PSTR("Found %lu consuming %d characters\n"), val.i, (p - linebuf) - linebuf_p);
					linebuf_p = p - linebuf;
					words[idx(CHECKSUM)].u = val.u;
				}
				words_mask |= (1L << (idx(CHECKSUM)));
			}
			linebuf_p++;
		} while (p != NULL && linebuf_p < linelen);

// 		printf_P(PSTR(" OK: seen bitmask: 0x%08lx\n"), words_mask);

		// we store speed internally as mm/s
		if (seen(CHECKSUM)) {
			if (checksum != words[idx(CHECKSUM)].i) {
				if (seen('N'))
					printf_P(PSTR("rs %lu Checksum error! Calculated %u, received %u\n"), words[idx('N')].u, checksum, (uint8_t) (words[idx(CHECKSUM)].i & 0xFF));
				else
					printf_P(PSTR("!! Checksum error! Calculated %u, received %u\n"), checksum, (uint8_t) (words[idx(CHECKSUM)].i & 0xFF));

				linebuf_p = 0;
				checksum = 0;
				words_mask = 0;

				return;
			}
		}

// 		putchar('Z');

		printf_P(PSTR("ok"));

// 		putchar('0');

		if (seen('T'))
			next_tool = words[idx('T')].u & 0xFF;

// 		putchar('1');

		if (seen('G') || seen('M')) {
// 			putchar('[');
			gcode_process();
// 			putchar(']');
		}

// 		putchar('2');

		if (state_flags & STATE_DUALBAND) {
			if (queue_full() == 0)
				printf_P(PSTR(" buffer"));
			printf_P(PSTR(" Q%u/%u%c"), mb_tail, mb_head, queue_full()?'F':' ');
		}

// 		putchar('3');

		if (state_flags & STATE_WRITE_SD) {
			for (i = 0, checksum = 0; i < 32; i++) {
				if (words_mask & (1L << i)) {
					switch (i) {
						case idx('G'):
						case idx('M'):
						case idx('N'):
						case idx('T'):
						case idx(XC0):
						case idx(YC0):
						case idx(ZC0):
						case idx(EC0):
						case idx(XMC):
						case idx(YMC):
						case idx(ZMC):
						case idx(EMC):
						case idx(XDS):
						case idx(YDS):
						case idx(ZDS):
						case idx(EDS):
	// 						printf_P(PSTR(" %c%lu"), 'A' + i, words[i].i);
// 							if (state_flags & STATE_WRITE_SD)
								fprintf_P(&mysdcard, PSTR(" %c%lu"), 'A' + i, words[i].i);
							break;
						case idx(CHECKSUM):
	// 						printf_P(PSTR(" *%lu"), words[i].i);
							break;
						default:
	// 						printf_P(PSTR(" %c%f"), 'A' + i, words[i].f);
// 							if (state_flags & STATE_WRITE_SD)
								fprintf_P(&mysdcard, PSTR(" %c%f"), 'A' + i, words[i].f);
							break;
					}
				}
			}
		}

// 		putchar('4');

		putchar('\n');

		linebuf_p = 0;
		checksum = 0;
		words_mask = 0;
	}
}
