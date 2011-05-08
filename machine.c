#include	"machine.h"

#include	<math.h>

#include	"sd.h"
#include	"ff.h"
#include	"serial.h"
#include	"timer.h"
#include	"clock.h"

volatile uint8_t sdflags;

DWORD get_fattime() {
	return ((2011UL - 1980UL) << 25) | (4UL << 21) | (28UL << 16) | (21UL << 11) | (20UL << 5) | 34UL;
}

uint8_t sdbuffer[32];
FATFS fatfs;
FIL file;
FILINFO fileinfo;
DIR dir;
FRESULT fr;

int serput_stdio(char c, FILE *f) {
	serial_writechar(c);
	return 0;
}

int serget_stdio(FILE *f) {
	for (;serial_rxchars() == 0;);
	return serial_popchar();
}

int sdput_stdio(char c, FILE *f) {
	return f_write(&file, &c, 1, NULL);
}

int sdget_stdio(FILE *f) {
	uint8_t c;
	f_read(&file, (TCHAR *) &c, 1, NULL);
	return c;
}

FILE mystdio = FDEV_SETUP_STREAM(serput_stdio, serget_stdio, _FDEV_SETUP_RW);
FILE mysdcard = FDEV_SETUP_STREAM(sdput_stdio, sdget_stdio, _FDEV_SETUP_RW);

volatile uint8_t state_flags;

float x_steps_per_mm = (3200.0 / (10.0 * 5.0));
float y_steps_per_mm = (3200.0 / (8.0 * M_PI));
float z_steps_per_mm = (400.0 / 1.25);
float e_steps_per_mm = (800.0 * 11.0 / 39.0 / (6.8 * M_PI));

float x_accel = 100.0;
float y_accel = 100.0;
float z_accel = 10.0;
float e_accel = 100.0;

float x_decel = 100.0;
float y_decel = 100.0;
float z_decel = 10.0;
float e_decel = 100.0;

// position vs endstops
coord f_global = {0, 0, 0, 0, 60};
// local vs global offset
coord f_offset = {0, 0, 0, 0, 0};
// local position. should always be == (global + offset)
coord f_current = {0, 0, 0, 0, 60};

location s_global = {0, 0, 0, 0};
location s_offset = {0, 0, 0, 0};
location s_current = {0, 0, 0, 0};
location s_endpoint = {0, 0, 0, 0};

void update_position_i() {
	// step interrupt updates s_global, update everything else from that
	s_current.X = s_global.X + s_offset.X;
	s_current.Y = s_global.Y + s_offset.Y;
	s_current.Z = s_global.Z + s_offset.Z;
	s_current.E = s_global.E + s_offset.E;
}

void update_position() {
	update_position_i();

	f_global.X = ((float) s_global.X) / x_steps_per_mm;
	f_global.Y = ((float) s_global.Y) / y_steps_per_mm;
	f_global.Z = ((float) s_global.Z) / z_steps_per_mm;
	f_global.E = ((float) s_global.E) / e_steps_per_mm;

	f_current.X = f_global.X + f_offset.X;
	f_current.Y = f_global.Y + f_offset.Y;
	f_current.Z = f_global.Z + f_offset.Z;
	f_current.E = f_global.E + f_offset.E;
}

uint8_t next_tool;

void loopstuff() {
	ifclock(CLOCK_FLAG_10MS)
		clock_10ms();
}
