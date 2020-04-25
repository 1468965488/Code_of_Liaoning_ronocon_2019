#ifndef __SHOUBING_H
#define __SHOUBING_H
#include "sys.h"
#include "stdio.h"
extern	int shuju[4];

typedef struct 
{ u8 mode;
	u8 x;
	u8 y;
	u8 fun;
}HB;





HB shoubing_init(int fun[4]);




#endif

	 
