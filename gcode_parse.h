#ifndef	_GCODE_PARSE_H
#define	_GCODE_PARSE_H

#include	<stdint.h>

#define	GCODE_BUFFER_SIZE	64

void gcode_parse_char(uint8_t c);

#endif	/* _GCODE_PARSE_H */
