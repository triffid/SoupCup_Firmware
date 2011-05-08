#include	"move.h"

#include	<math.h>
#include	<avr/interrupt.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<avr/pgmspace.h>

#include	"timer.h"
#include	"gcode_parse.h"
#include	"machine.h"

volatile uint8_t mb_head = 0;
volatile uint8_t mb_tail = 0;

volatile move movebuffer[MOVEBUFFER_SIZE] __attribute__ ((section (".bss")));

// microseconds
#define JITTER	2L
#define	JITTER_CYCLES	((F_CPU / 1000000L) * JITTER)

// move runtimes - variables that don't need to be precalculated
uint32_t dt;
int32_t x_n, y_n, z_n, e_n;
uint32_t x_cr, y_cr, z_cr, e_cr;
uint32_t x_delta, y_delta, z_delta, e_delta;
uint8_t axis_mask;

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
		x_steps = lround(words[idx('X')].f * x_steps_per_mm) - s_endpoint.X;
		s_endpoint.X += x_steps;
	}
	if (seen('Y')) {
		y_steps = lround(words[idx('Y')].f * y_steps_per_mm) - s_endpoint.Y;
		s_endpoint.Y += y_steps;
	}
	if (seen('Z')) {
		z_steps = lround(words[idx('Z')].f * z_steps_per_mm) - s_endpoint.Z;
		s_endpoint.Z += z_steps;
	}
	if (seen('E')) {
		e_steps = lround(words[idx('E')].f * e_steps_per_mm) - s_endpoint.E;
		s_endpoint.E += e_steps;
	}

	do {
		loopstuff();
	} while (queue_full());

	timer_register_callback(&move_step);

	move *m = (move *) &movebuffer[mb_head];

	m->flags = 0;

	if (x_steps != 0) {
		if (x_steps >= 0)
			m->x_direction = 1;

		m->x_steps = labs(x_steps);
		m->x_c = words[idx(XC0)].u;
		m->x_decel_steps = words[idx(XDS)].u;
		m->x_min_c = words[idx(XMC)].u;
		x_n = 5;
	}
	else {
		m->x_steps = 0;
		m->x_c = 0xFFFFFFFF;
		m->x_decel_steps = 0;
		m->x_min_c = 0xFFFFFFFF;
		x_n = 0;
	}

	if (y_steps != 0) {
		if (y_steps >= 0)
			m->y_direction = 1;

		m->y_steps = labs(y_steps);
		m->y_c = words[idx(YC0)].u;
		m->y_decel_steps = words[idx(YDS)].u;
		m->y_min_c = words[idx(YMC)].u;
		y_n = 5;
	}
	else {
		m->y_steps = 0;
		m->y_c = 0xFFFFFFFF;
		m->y_decel_steps = 0;
		m->y_min_c = 0xFFFFFFFF;
		y_n = 0;
	}

	if (z_steps != 0) {
		if (z_steps >= 0)
			m->z_direction = 1;

		m->z_steps = labs(z_steps);
		m->z_c = words[idx(ZC0)].u;
		m->z_decel_steps = words[idx(ZDS)].u;
		m->z_min_c = words[idx(ZMC)].u;
		z_n = 5;
	}
	else {
		m->z_steps = 0;
		m->z_c = 0xFFFFFFFF;
		m->z_decel_steps = 0;
		m->z_min_c = 0xFFFFFFFF;
		z_n = 0;
	}

	if (e_steps != 0) {
		if (e_steps >= 0)
			m->e_direction = 1;

		m->e_steps = labs(e_steps);
		m->e_c = words[idx(EC0)].u;
		m->e_decel_steps = words[idx(EDS)].u;
		m->e_min_c = words[idx(EMC)].u;
		e_n = 5;
	}
	else {
		m->e_steps = 0;
		m->e_c = 0xFFFFFFFF;
		m->e_decel_steps = 0;
		m->e_min_c = 0xFFFFFFFF;
		e_n = 0;
	}

	if (x_n == 0 && y_n == 0 && z_n == 0 && e_n == 0)
		return;

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

	axis_mask = 0;

	x_delta = m->x_steps;
	y_delta = m->y_steps;
	z_delta = m->z_steps;
	e_delta = m->e_steps;

	if (m->x_steps != 0) {
		x_cr = m->x_c >> 8;
		if (x_cr < dt)
			dt = x_cr;
		printf_P(PSTR("X stepping, cr is %lu"), x_cr);
		x_n = 5;
		axis_mask |= AXIS_X;
	}
	else
		x_n = 0;

	if (m->y_steps != 0) {
		y_cr = m->y_c >> 8;
		if (y_cr < dt)
			dt = y_cr;
		y_n = 5;
		axis_mask |= AXIS_Y;
	}
	else
		y_n = 0;

	if (m->z_steps != 0) {
		z_cr = m->z_c >> 8;
		if (z_cr < dt)
			dt = z_cr;
		z_n = 5;
		axis_mask |= AXIS_Z;
	}
	else
		z_n = 0;

	if (m->e_steps != 0) {
		e_cr = m->e_c >> 8;
		if (e_cr < dt)
			dt = e_cr;
		e_n = 5;
		axis_mask |= AXIS_E;
	}
	else
		e_n = 0;

	printf_P(PSTR("[timer:%lu ticks/%gus]"), dt, ((float) dt) / ((float) (F_CPU / 1000000)) );

	timer_set(dt);

	m->live = 1;
}

void move_step() {
	// we don't need this to be volatile in interrupt context, and it will just slow us down
	move *m = (move *) &movebuffer[mb_tail];

	if (axis_mask & AXIS_X)
		x_cr -= dt;
	if (axis_mask & AXIS_Y)
		y_cr -= dt;
	if (axis_mask & AXIS_Z)
		z_cr -= dt;
	if (axis_mask & AXIS_E)
		e_cr -= dt;

	// 	dt = 0xFFFFFFFF;
	((uint8_t *) (&dt))[0] = 0xFF;
	((uint8_t *) (&dt))[1] = 0xFF;
	((uint8_t *) (&dt))[2] = 0xFF;
	((uint8_t *) (&dt))[3] = 0xFF;

	if (axis_mask & AXIS_X) {
		if (x_cr < JITTER_CYCLES) {
			// x step
			m->x_steps--;

			if (m->x_steps == 0)
				axis_mask &= ~AXIS_X;

			// step here
			x_step();

			// accel ramping
			if (x_n != 0) {
				if (x_n == 5L)
					m->x_c = fracmult(m->x_c, 0.4056);
				else
					m->x_c = m->x_c - (2L * m->x_c) / x_n;

				x_n += 4L;

				if (m->x_c < m->x_min_c) {
					m->x_c = m->x_min_c;
					x_n = 0;
				}
			}

			// decel time?
			if (m->x_steps == m->x_decel_steps)
				x_n = (m->x_steps * -4L) + 1L;

			x_cr += (m->x_c >> 8L);
		}
		if (x_cr < dt)
			dt = x_cr;
	}

	if (axis_mask & AXIS_Y) {
		if (y_cr < JITTER_CYCLES) {
			// y step
			m->y_steps--;

			if (m->y_steps == 0)
				axis_mask &= ~AXIS_Y;

			// step here
			y_step();

			// accel ramping
			if (y_n != 0) {
				if (y_n == 5L)
					m->y_c = fracmult(m->y_c, 0.4056);
				else
					m->y_c = m->y_c - (2L * m->y_c) / y_n;

				y_n += 4;

				// speed cap
				if (m->y_c < m->y_min_c) {
					m->y_c = m->y_min_c;
					y_n = 0;
				}
			}

			// decel time?
			if (m->y_steps == m->y_decel_steps)
				y_n = (m->y_steps * -4L) + 1L;

			y_cr += (m->y_c >> 8);
		}
		if (y_cr < dt)
			dt = y_cr;
	}

	if (axis_mask & AXIS_Z) {
		if (z_cr < JITTER_CYCLES) {
			// z step
			m->z_steps--;

			if (m->z_steps == 0)
				axis_mask &= ~AXIS_Z;

			// step here
			z_step();

			// accel ramp
			if (z_n != 0) {
				if (z_n == 5)
					m->z_c = fracmult(m->z_c, 0.4056);
				else
					m->z_c = m->z_c - (2 * m->z_c) / z_n;

				z_n += 4L;

				if (m->z_c < m->z_min_c) {
					m->z_c = m->z_min_c;
					z_n = 0;
				}
			}

			if (m->z_steps == m->z_decel_steps)
				z_n = (m->z_steps * -4L) + 1L;

			z_cr += (m->z_c >> 8);
		}
		if (z_cr < dt)
			dt = z_cr;
	}

	if (axis_mask & AXIS_E) {
		if (e_cr < JITTER_CYCLES) {
			// e step
			m->e_steps--;

			if (m->e_steps == 0)
				axis_mask &= ~AXIS_E;

			// step here
			e_step();

			// accel ramp
			if (e_n != 0) {
				if (e_n == 5)
					// todo: check for overflow
					m->e_c = fracmult(m->e_c, 0.4056);
				else
					m->e_c = m->e_c - (2 * m->e_c) / e_n;

				e_n += 4L;

				if (m->e_c < m->e_min_c) {
					m->e_c = m->e_min_c;
					e_n = 0;
				}
			}

			if (m->e_steps == m->e_decel_steps)
				e_n = (m->e_steps * -4L) + 1L;

			e_cr += (m->e_c >> 8);
		}
		if (e_cr < dt)
			dt = e_cr;
	}

	if (m->x_steps == 0 && m->y_steps == 0 && m->z_steps == 0 && m->e_steps == 0) {
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

		update_position_i();

		if (queue_empty() == 0) {
			// start next move
			move_start();
		}
		else {
			// end of buffer
			timer_set(0);
		}
	}
	else
		timer_set(dt);

	unstep();
}
