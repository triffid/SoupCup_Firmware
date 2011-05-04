#ifdef SD
	#include	"diskio.h"
	#include	"ff.h"

	#define SDFLAG_MOUNTED			0x01
	#define	SDFLAG_GET_FILENAME	0x02
	#define SDFLAG_READING			0x10
	#define	SDFLAG_WRITING			0x20
	extern volatile uint8_t sdflags;

	extern uint8_t sdbuffer[32];
	extern FATFS fatfs;
	extern FIL file;
	extern FILINFO fileinfo;
	extern DIR dir;
	extern FRESULT fr;
#endif