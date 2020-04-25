#include "main.h"



int chassis_motor1_speed;


/*
*电机驱动
*num：.....
*speed_pwm：输入PWM值
*/
void Motor(int num,float speed_pwm,int Motor_err)
{
	chassis_motor1_speed=speed_pwm;   //由于驱动器的原因，PWM在150以内无反应
	
	if(num == 1)
	{
		if(chassis_motor1_speed>=0)
		{ 
			chassis_motor_DIR1=1;
			TIM_SetCompare1(TIM3,chassis_motor1_speed+Motor_err);   //输出加100 减小驱动器低占空比不响应的影响提高调整精度
		}
		if(chassis_motor1_speed<0)
		{
			chassis_motor_DIR1=0;
			TIM_SetCompare1(TIM3,-chassis_motor1_speed+Motor_err);
		}
	}
	
	if(num == 2)
	{
		if(chassis_motor1_speed>=0)
		{ 
			chassis_motor_DIR2=1;
			TIM_SetCompare2(TIM3,chassis_motor1_speed+Motor_err);
		}
		if(chassis_motor1_speed<0)
		{
			chassis_motor_DIR2=0;
			TIM_SetCompare2(TIM3,-chassis_motor1_speed+Motor_err);
		}
	}
	
	if(num == 3)
	{
		if(chassis_motor1_speed>=0)
		{ 
			chassis_motor_DIR3=1;
			TIM_SetCompare3(TIM3,chassis_motor1_speed+Motor_err);
		}
		if(chassis_motor1_speed<0)
		{
			chassis_motor_DIR3=0;
			TIM_SetCompare3(TIM3,-chassis_motor1_speed+Motor_err);
		}
	}
	
	if(num == 4)
	{
		if(chassis_motor1_speed>=0)
		{ 
			chassis_motor_DIR4=1;
			TIM_SetCompare4(TIM3,chassis_motor1_speed+Motor_err);
		}
		if(chassis_motor1_speed<0)
		{
			chassis_motor_DIR4=0;
			TIM_SetCompare4(TIM3,-chassis_motor1_speed+Motor_err);
		}
	}	
}







