#include	"clock.h"

/** \file
	\brief Do stuff periodically
*/

#include	"timer.h"

/*!	do stuff every 1/4 second

	called from clock_10ms(), do not call directly
*/
void clock_250ms() {
	ifclock(CLOCK_FLAG_1S) {
	}
}

/*! do stuff every 10 milliseconds

	call from ifclock(CLOCK_FLAG_10MS) in busy loops
*/
void clock_10ms() {
	ifclock(CLOCK_FLAG_250MS) {
		clock_250ms();
	}
}
