#include "main.h"



int chassis_motor1_speed;


/*
*�������
*num��.....
*speed_pwm������PWMֵ
*/
void Motor(int num,float speed_pwm,int Motor_err)
{
	chassis_motor1_speed=speed_pwm;   //������������ԭ��PWM��150�����޷�Ӧ
	
	if(num == 1)
	{
		if(chassis_motor1_speed>=0)
		{ 
			chassis_motor_DIR1=1;
			TIM_SetCompare1(TIM3,chassis_motor1_speed+Motor_err);   //�����100 ��С��������ռ�ձȲ���Ӧ��Ӱ����ߵ�������
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







