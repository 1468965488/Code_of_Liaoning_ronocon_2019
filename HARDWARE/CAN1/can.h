#ifndef __CAN1_H__
#define __CAN1_H__	 
#include "stm32f4xx.h"	    


void CAN1_Configuration(void);

u8 Transmit_Motor_Data(void);
//u8 Transmit_Motor_Data_2(void);

//void get_total_angle(void);

void get_total_angle_(void);
struct __motor_feedback {
	
unsigned short rm3508_201_agree;
unsigned short rm3508_202_agree;
unsigned short rm3508_203_agree;

short rm3508_201_speed;
short rm3508_202_speed;
short rm3508_203_speed;

short rm3508_201_temp;
short rm3508_202_temp;
short rm3508_203_temp;

};


#endif 













