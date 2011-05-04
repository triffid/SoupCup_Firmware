#ifndef	_TIMER_H
#define	_TIMER_H

#include	<stdint.h>
#include	<avr/io.h>

// time-related constants
#define	US	* (F_CPU / 1000000)
#define	MS	* (F_CPU / 1000)

typedef void (*void_func_ptr)(void);

/*
clock stuff
*/
extern volatile uint8_t	clock_flag;

#define	CLOCK_FLAG_10MS								1
#define	CLOCK_FLAG_250MS							2
#define	CLOCK_FLAG_1S									4
#define	ifclock(F)	for (;clock_flag & (F);clock_flag &= ~(F))

/*
timer stuff
*/
void timer_init(void);

void timer_register_callback(void_func_ptr);

void timer_set(uint32_t);

void timer_stop(void);

#endif	/* _TIMER_H */
