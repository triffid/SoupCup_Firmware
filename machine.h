#ifndef	_MACHINE_H
#define	_MACHINE_H

#include	<stdint.h>
#include	<stdio.h>

#include	"sd.h"

extern volatile uint8_t sdflags;

extern FILE mystdio;
extern FILE mysdcard;

typedef struct {
	float X;
	float Y;
	float Z;
	float E;
	float F;
} coord;

typedef struct {
	int32_t	X;
	int32_t	Y;
	int32_t	Z;
	int32_t	E;
} location;

#define	STATE_READ_SD		1
#define	STATE_WRITE_SD	2
#define	STATE_RELATIVE	4

#define	STATE_DUALBAND	16
#define	STATE_LNC				32
extern volatile uint8_t state_flags;

extern float x_steps_per_mm;
extern float y_steps_per_mm;
extern float z_steps_per_mm;
extern float e_steps_per_mm;

extern float x_accel;
extern float y_accel;
extern float z_accel;
extern float e_accel;

extern float x_decel;
extern float y_decel;
extern float z_decel;
extern float e_decel;

// position in steps vs endstops. This is the master position record, f_global is calculated from this
extern location s_global;
// offset in steps
extern location s_offset;

// position in mm vs endstops. Calculated from s_global
extern coord f_global;
// offset in mm
extern coord f_offset;


// this is where the last move will finish
extern location s_endpoint;

void update_position(void);

extern uint8_t next_tool;

#define	x_step() do { putchar('x'); } while (0)
#define	y_step() do { putchar('y'); } while (0)
#define	z_step() do { putchar('z'); } while (0)
#define	e_step() do { putchar('e'); } while (0)
#define	unstep() do { putchar('.'); } while (0)

#define	x_dir(d)	do {} while (0)
#define	y_dir(d)	do {} while (0)
#define	z_dir(d)	do {} while (0)
#define	e_dir(d)	do {} while (0)

void loopstuff(void);

#define	AXIS_X	1
#define	AXIS_Y	2
#define	AXIS_Z	4
#define	AXIS_E	8

#endif	/* _MACHINE_H */
