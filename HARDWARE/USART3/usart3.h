#ifndef __USART3_H
#define __USART3_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

void USART3_Config(void);
 
void usart5_config(u32 bound);

struct _color{
u8 R;
u8 G;
u8 B;
};
#endif


