#ifndef __CAN2_H__
#define __CAN2_H__	 
#include "stm32f4xx.h"	    


void CAN2_Configuration(void);

u8 Transmit_Motor_Data_can2(void);
u8 Transmit_Motor_Data_can_2(void); 

void get_total_angle(void);

struct _motor_feedback{
	
unsigned short rm2006_201_agree;
unsigned short rm2006_202_agree;
unsigned short rm2006_203_agree;
unsigned short rm2006_204_agree;
unsigned short rm2006_205_agree;
unsigned short rm2006_206_agree;


short rm2006_201_speed;
short rm2006_202_speed;
short rm2006_203_speed;
short rm2006_204_speed;
short rm2006_205_speed;
short rm2006_206_speed;


};

#endif 













