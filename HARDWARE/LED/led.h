#ifndef __LED_H
#define __LED_H
#include "sys.h"


//LED�˿ڶ���
#define LED0 PEout(3)	// DS0
#define LED1 PEout(4)	// DS1


#define chassis_motor_DIR1 PFout(12)  //0Ϊ˳ʱ����ת 1Ϊ��ʱ����ת 
#define chassis_motor_DIR2 PFout(13)
#define chassis_motor_DIR3 PFout(14)
#define chassis_motor_DIR4 PFout(15)

void LED_Init(void);//��ʼ��		 
void Gpio_init(void);
void Restart_button_Init(void);
#endif
