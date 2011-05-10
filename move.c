#include	"move.h"

#include	<math.h>
#include	<avr/interrupt.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<avr/pgmspace.h>

#include	"timer.h"
#include	"gcode_parse.h"
#include	"machine.h"
#include	"config.h"

#define	max(a, b) (((a) >= (b))?(a):(b))

volatile uint8_t mb_head = 0;
volatile uint8_t mb_tail = 0;

volatile move movebuffer[MOVEBUFFER_SIZE] __attribute__ ((section (".bss")));

// microseconds
#define JITTER_US			2L
#define	JITTER_CYCLES	((F_CPU / 1000000L) * JITTER_US)

// move runtimes - variables that don't need to be precalculated
uint32_t dt;
uint32_t x_delta, y_delta, z_delta, e_delta;
uint8_t axis_mask;
uint32_t ts, sr;
int32_t c, n;
int32_t x_bc, y_bc, z_bc, e_bc;

uint32_t _fracmult(uint32_t multiplicand, uint32_t fraction) {
	uint32_t r = 0;
	while (fraction && multiplicand) {
		if (fraction & 0x80000000)
			r += multiplicand;
		fraction <<= 1L;
		multiplicand >>= 1L;
	}
	return r;
}

uint8_t queue_full() {
	return (((mb_tail - mb_head - 1) & (MOVEBUFFER_SIZE - 1)) == 0)?255:0;
}

uint8_t queue_empty() {
	return (mb_head == mb_tail)?255:0;
}

void enqueue() {
	int32_t x_steps = 0, y_steps = 0, z_steps = 0, e_steps = 0;

	if (seen('X')) {
		if (state_flags & STATE_RELATIVE)
			x_steps = labs(lround(words[idx('X')].f * x_steps_per_mm));
		else
			x_steps = labs(lround(words[idx('X')].f * x_steps_per_mm) - s_endpoint.X);
		s_endpoint.X += x_steps;
	}
	if (seen('Y')) {
		if (state_flags & STATE_RELATIVE)
			y_steps = labs(lround(words[idx('Y')].f * y_steps_per_mm));
		else
			y_steps = labs(lround(words[idx('Y')].f * y_steps_per_mm) - s_endpoint.Y);
		s_endpoint.Y += y_steps;
	}
	if (seen('Z')) {
		if (state_flags & STATE_RELATIVE)
			z_steps = labs(lround(words[idx('Z')].f * z_steps_per_mm));
		else
			z_steps = labs(lround(words[idx('Z')].f * z_steps_per_mm) - s_endpoint.Z);
		s_endpoint.Z += z_steps;
	}
	if (seen('E')) {
		if (state_flags & STATE_RELATIVE)
			e_steps = labs(lround(words[idx('E')].f * e_steps_per_mm));
		else
			e_steps = labs(lround(words[idx('E')].f * e_steps_per_mm) - s_endpoint.E);
		s_endpoint.E += e_steps;
	}

	do {
		loopstuff();
	} while (queue_full());

	timer_register_callback(&move_step);

	move *m = (move *) &movebuffer[mb_head];

	m->flags = 0;

	if (x_steps >= 0)
		m->x_direction = 1;
	if (y_steps >= 0)
		m->y_direction = 1;
	if (z_steps >= 0)
		m->z_direction = 1;
	if (e_steps >= 0)
		m->e_direction = 1;

	x_steps = labs(x_steps);
	y_steps = labs(y_steps);
	z_steps = labs(z_steps);
	e_steps = labs(e_steps);

	m->x_steps = x_steps;
	m->y_steps = y_steps;
	m->z_steps = z_steps;
	m->e_steps = e_steps;

	ts = max(max(x_steps, y_steps), max(z_steps, e_steps));

	if (ts == 0)
		return;

	sr = ts;

	if (ts == x_steps) {
		m->ds = words[idx(XDS)].u;
		m->c = words[idx(XC0)].u;
		m->minc = words[idx(XMC)].u;
	}
	else if (ts == y_steps) {
		m->ds = words[idx(YDS)].u;
		m->c = words[idx(YC0)].u;
		m->minc = words[idx(YMC)].u;
	}
	else if (ts == z_steps) {
		m->ds = words[idx(ZDS)].u;
		m->c = words[idx(ZC0)].u;
		m->minc = words[idx(ZMC)].u;
	}
	else if (ts == e_steps) {
		m->ds = words[idx(EDS)].u;
		m->c = words[idx(EC0)].u;
		m->minc = words[idx(EMC)].u;
	}

	uint8_t sreg = SREG;
	cli();

	mb_head = (mb_head + 1) & (MOVEBUFFER_SIZE - 1);

	if (movebuffer[mb_tail].live == 0) {
		SREG = sreg;

		move_start();
	}
	else
		SREG = sreg;
}

void move_start() {
	// we don't need this to be volatile in interrupt context, and it will just slow us down
	move *m = (move *) &movebuffer[mb_tail];

	x_dir(m->x_direction);
	y_dir(m->y_direction);
	z_dir(m->z_direction);
	e_dir(m->e_direction);

	dt = 0xFFFFFFFF;

	x_delta = m->x_steps;
	y_delta = m->y_steps;
	z_delta = m->z_steps;
	e_delta = m->e_steps;

	ts = max(max(x_delta, y_delta), max(z_delta, e_delta));

	if (ts == 0)
		return;

	if (x_delta != 0) {
		x_bc = x_delta >> 1;
		axis_mask |= AXIS_X;
	}
	if (y_delta != 0) {
		y_bc = y_delta >> 1;
		axis_mask |= AXIS_Y;
	}
	if (z_delta != 0) {
		z_bc = z_delta >> 1;
		axis_mask |= AXIS_Z;
	}
	if (e_delta != 0) {
		e_bc = e_delta >> 1;
		axis_mask |= AXIS_E;
	}

	n = 5L;
	c = m->c;
	dt = c >> 8L;

	timer_set(dt);

	m->live = 1;
}

void move_step() {
	// we don't need this to be volatile in interrupt context, and it will just slow us down
	move *m = (move *) &movebuffer[mb_tail];

	// 	dt = 0xFFFFFFFF;
	((uint8_t *) (&dt))[0] = 0xFF;
	((uint8_t *) (&dt))[1] = 0xFF;
	((uint8_t *) (&dt))[2] = 0xFF;
	((uint8_t *) (&dt))[3] = 0xFF;

	if (axis_mask & AXIS_X) {
		x_bc -= x_delta;
		if (x_bc <= 0) {
			x_step();
			m->x_steps--;
			if (m->x_steps == 0)
				axis_mask &= ~AXIS_X;
			x_bc += ts;
		}
	}
	if (axis_mask & AXIS_Y) {
		y_bc -= y_delta;
		if (y_bc <= 0) {
			y_step();
			m->y_steps--;
			if (m->y_steps == 0)
				axis_mask &= ~AXIS_Y;
			y_bc += ts;
		}
	}
	if (axis_mask & AXIS_Z) {
		z_bc -= z_delta;
		if (z_bc <= 0) {
			z_step();
			m->z_steps--;
			if (m->z_steps == 0)
				axis_mask &= ~AXIS_Z;
			z_bc += ts;
		}
	}
	if (axis_mask & AXIS_E) {
		e_bc -= e_delta;
		if (e_bc <= 0) {
			e_step();
			m->e_steps--;
			if (m->e_steps == 0)
				axis_mask &= ~AXIS_E;
			e_bc += ts;
		}
	}

	if (sr == m->ds)
		n = (sr * -4L) + 1L;

	if (n != 0) {
		if (n == 5L)
			c = fracmult(c, 0.4056);
		else
			c = c - ((c * 2L) / n);

		n += 4L;

		if (c < m->minc) {
			c = m->minc;
			n = 0;
		}
	}

	sr--;
	dt = c >> 8;

	if (sr == 0) {
		// end of move
		m->live = 0;

		mb_tail++;
		mb_tail &= MOVEBUFFER_SIZE - 1;

		if (m->x_direction)
			s_global.X += x_delta;
		else
			s_global.X -= x_delta;
		if (m->y_direction)
			s_global.Y += y_delta;
		else
			s_global.Y -= y_delta;
		if (m->z_direction)
			s_global.Z += z_delta;
		else
			s_global.Z -= z_delta;
		if (m->e_direction)
			s_global.E += e_delta;
		else
			s_global.E -= e_delta;

		if (queue_empty() == 0) {
			// start next move
			move_start();
		}
		else {
			// end of buffer
			timer_set(0);
		}
	}
	else {
		timer_set(dt);
	}

	unstep();
}
