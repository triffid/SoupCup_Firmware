#include	"gcode_parse.h"

#include	<stdlib.h>
#include	<avr/pgmspace.h>
#include	<math.h>

// reserved words: E, F, G, M, N, P, S, T, X, Y, Z: 11 leaving 15 for our internal use
#define DISTANCE 'D'
#define	TIME	'U'
#define	ACCEL_DISTANCE 'V'
#define	DECEL_DISTANCE 'W'
#define	XC0	'A'
#define	YC0	'B'
#define	ZC0	'C'
#define	EC0	'H'
#define	XMC	'I'
#define	YMC 'J'
#define	ZMC	'K'
#define	EMC	'L'

#define	X_STEPS_PER_MM	(3200.0 / (10.0 * 5.0))
#define	Y_STEPS_PER_MM	(3200.0 / (8.0 * M_PI))
#define	Z_STEPS_PER_MM	(400.0 / 1.25)
#define	E_STEPS_PER_MM	(800.0 * 11.0 / 39.0 / (6.8 * M_PI))

float x_accel = 4.0;
float y_accel = 4.0;
float z_accel = 4.0;
float e_accel = 10.0;

typedef struct {
	float X;
	float Y;
	float Z;
	float E;
	float F;
} coord;

// position vs endstops
coord f_global;
// local vs global offset
coord f_offset;
// local position. should always be == (global + offset)
coord f_current;

typedef struct {
	int32_t	X;
	int32_t	Y;
	int32_t	Z;
	int32_t	E;
} location;

location s_global;
location s_offset;
location s_current;

uint8_t linebuf_p = 0;
uint8_t linebuf[GCODE_BUFFER_SIZE] __attribute__ ((section (".bss")));

uint32_t words_mask;
union {
	float f;
	uint32_t u;
	int32_t i;
} words[27];

#define	idx(c) (c - 'A')
#define seen(c) (words_mask & (1L << idx(c)))

void gcode_parse_char(uint8_t c) {

	linebuf[linebuf_p++] = c;

	if ((c < 32) || (linebuf_p >= (GCODE_BUFFER_SIZE - 1))) {
		linebuf[linebuf_p] = 0;
		linebuf_p = 0;
		uint16_t r = 0;
		uint32_t val_i;
		float val_f;
		uint8_t *p = 0;
		do {
			for (;(linebuf[linebuf_p] < 'A' || linebuf[linebuf_p] > 'Z') && linebuf_p < GCODE_BUFFER_SIZE;linebuf_p++)
				if (linebuf[linebuf_p] < 32)
					break;

			c = linebuf[linebuf_p];

			if (c >= 'a')
				c -= ('a' - 'A');

			if (c >= 'A' && c <= 'Z') {
				switch (c) {
					case 'G':
					case 'M':
					case 'T':
						val_i = strtoul((const char *) &linebuf[linebuf_p], (char **) &p, 0);
						if (p > &linebuf[linebuf_p]) {
							linebuf_p = p - linebuf;
							words[idx(c)].u = val_i;
						}
						break;
					default:
						val_f = strtod((const char *) &linebuf[linebuf_p], (char **) &p);
						if (p > &linebuf[linebuf_p]) {
							linebuf_p = p - linebuf;
							words[idx(c)].f = val_f;
						}
						break;
				}
				linebuf_p += r;
				words_mask |= (1 << (idx(c)));
			}
		} while (p != 0);

		// we store speed internally as mm/s
		if (seen('F'))
			words[idx('F')].f /= 60.0;

		if (seen('T')) {

		}

		if (seen('M')) {
		}
		else if (seen('G')) {
			if (words[idx('G')].u == 1L) {
				// G1
				float x = words[idx('X')].f - f_current.X;
				float y = words[idx('Y')].f - f_current.Y;
				float z = words[idx('Z')].f - f_current.Z;
				float e = words[idx('E')].f - f_current.E;
				float f = words[idx('F')].f;
				float d;
				float t;

				// work out distance
				if (seen(DISTANCE) == 0) {
					if (seen('X') && seen('Y') && seen('Z') == 0)
						d = hypot(x, y);
					if (seen('X') == 0 && seen('Y') == 0 && seen('Z'))
						d = z;
					else if (seen('X') == 0 && seen('Y') == 0 && seen('Z') == 0 && seen('E'))
						d = e;
					else {
						d = hypot(x, y);
						d = hypot(d, z);
					}
					words[idx(DISTANCE)].f = d;
				}
				else
					d = words[idx(DISTANCE)].f;

				// work out time
				// time = distance / speed
				if (seen(TIME) == 0) {
					t = words[idx(DISTANCE)].f / f;
					words[idx(TIME)].f = t;
				}
				else
					t = words[idx(TIME)].f;

				// work out accel profiles
				if (seen(ACCEL_DISTANCE) == 0) {
					// distance to accel to w at w' = w**2 / 2 / w'
					float xs = x / t;
					float xda = lround(square(xs) / 2.0 / x_accel);
					float ys = y / t;
					float yda = lround(square(ys) / 2.0 / y_accel);
					float zs = z / t;
					float zda = lround(square(zs) / 2.0 / z_accel);
					float es = e / t;
					float eda = lround(square(es) / 2.0 / e_accel);

					// now find slowest axis
					float da = fmax(fmax(xda, yda), fmax(zda, eda));

					// now apply this accel ramp to other axes
					// d = s**2/2a, therefore a = s**2/2d
					x_accel = square(xs) / 2.0 / da;
					y_accel = square(ys) / 2.0 / da;
					z_accel = square(zs) / 2.0 / da;
					e_accel = square(es) / 2.0 / da;

					words[idx(XC0)].u = lround(((float) F_CPU) * sqrt(2.0 / x_accel / X_STEPS_PER_MM) * 256.0);
					words[idx(YC0)].u = lround(((float) F_CPU) * sqrt(2.0 / y_accel / Y_STEPS_PER_MM) * 256.0);
					words[idx(ZC0)].u = lround(((float) F_CPU) * sqrt(2.0 / z_accel / Z_STEPS_PER_MM) * 256.0);
					words[idx(EC0)].u = lround(((float) F_CPU) * sqrt(2.0 / e_accel / E_STEPS_PER_MM) * 256.0);

					words[idx(XMC)].u = lround((float) F_CPU) * 256.0 / (x_speed * X_STEPS_PER_MM));
					words[idx(YMC)].u = lround((float) F_CPU) * 256.0 / (y_speed * Y_STEPS_PER_MM));
					words[idx(ZMC)].u = lround((float) F_CPU) * 256.0 / (z_speed * Z_STEPS_PER_MM));
					words[idx(EMC)].u = lround((float) F_CPU) * 256.0 / (e_speed * E_STEPS_PER_MM));
				}
			}
		}
	}
}
