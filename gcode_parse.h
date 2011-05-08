#ifndef	_GCODE_PARSE_H
#define	_GCODE_PARSE_H

#include	<stdint.h>

#define	GCODE_BUFFER_SIZE	256

// reserved words: E, F, G, M, N, P, S, T, X, Y, Z: 11 leaving 15 for our internal use
#define DISTANCE				'D'
#define	TIME						'U'
#define	ACCEL_DISTANCE	'V'
#define	DECEL_DISTANCE	'W'
#define	XC0							'A'
#define	YC0							'B'
#define	ZC0							'C'
#define	EC0							'H'
#define	XMC							'I'
#define	YMC 						'J'
#define	ZMC							'K'
#define	EMC							'L'
#define	XDS							'O'
#define	YDS							'Q'
#define	ZDS							'R'
#define	EDS							('Z' + 1)
#define	XS							('Z' + 2)
#define	YS							('Z' + 3)
#define	ZS							('Z' + 4)
#define	ES							('Z' + 5)
#define	CHECKSUM				('Z' + 6)

extern uint32_t	linenumber_expect;

typedef union {
	float f;
	uint32_t u;
	int32_t i;
} word;

extern uint32_t words_mask;
extern word words[32];

#define	idx(c) (c - 'A')
#define seen(c) ((words_mask & (1L << idx(c)))?255:0)

#define	set_f(w, v)	do { words[idx(w)].f = (v); words_mask |= (1L << idx(w)); } while (0)
#define	set_i(w, v)	do { words[idx(w)].i = (v); words_mask |= (1L << idx(w)); } while (0)

void movemath(void);
void gcode_process(void);
void gcode_parse_char(uint8_t c);

#endif	/* _GCODE_PARSE_H */
