#ifndef	_MOVE_H
#define	_MOVE_H

#include	<stdint.h>

#define	MOVEBUFFER_SIZE	2

#include	"config.h"

typedef struct {
	union {
		struct {
			uint8_t live:1;

			uint8_t x_direction:1;
			uint8_t y_direction:1;
			uint8_t z_direction:1;
			uint8_t e_direction:1;
		};
		uint8_t flags;
	};

	uint32_t	x_steps;
	uint32_t	y_steps;
	uint32_t	z_steps;
	uint32_t	e_steps;

	#if defined(BRESENHAM)
		uint32_t	c;
		uint32_t	ds;
		uint32_t	minc;
	#elif defined(PERAXIS)
		uint32_t	x_c;
		uint32_t	y_c;
		uint32_t	z_c;
		uint32_t	e_c;

		uint32_t	x_min_c;
		uint32_t	y_min_c;
		uint32_t	z_min_c;
		uint32_t	e_min_c;

		uint32_t	x_decel_steps;
		uint32_t	y_decel_steps;
		uint32_t	z_decel_steps;
		uint32_t	e_decel_steps;
	#endif
} move;

extern volatile uint8_t mb_head;
extern volatile uint8_t mb_tail;

extern volatile move movebuffer[MOVEBUFFER_SIZE];

#define	 fracmult(m, f) _fracmult(m, ((uint32_t) (f * 0x80000000)))
uint32_t _fracmult(uint32_t multiplicand, uint32_t fraction);

uint8_t	queue_full(void);
uint8_t queue_empty(void);
#define	queue_wait()	do { loopstuff(); } until (queue_empty() && movebuffer[mb_tail].live == 0)
#define	queue_wait_1()	do { loopstuff(); } while (queue_full())

void enqueue(void);

void move_start(void);
void move_step(void);

#endif	/* _MOVE_H */
