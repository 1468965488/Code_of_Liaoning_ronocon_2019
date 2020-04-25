#ifndef __LED_H
#define __LED_H
#include "sys.h"


//LED端口定义
#define LED0 PEout(3)	// DS0
#define LED1 PEout(4)	// DS1


#define chassis_motor_DIR1 PFout(12)  //0为顺时针旋转 1为逆时针旋转 
#define chassis_motor_DIR2 PFout(13)
#define chassis_motor_DIR3 PFout(14)
#define chassis_motor_DIR4 PFout(15)

void LED_Init(void);//初始化		 
void Gpio_init(void);
void Restart_button_Init(void);
#endif
