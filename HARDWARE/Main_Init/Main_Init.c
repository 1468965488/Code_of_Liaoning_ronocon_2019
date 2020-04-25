#include "main.h"

/**
******************************************************************************
* @file      Main_Init.c
* @author    ���紴���Ŷ�
* @version
* @date      2018/4/5
* @brief     �����ʼ��
******************************************************************************
* @attention	��ŷ�0�Ǳգ�1�ǿ�
******************************************************************************
*/

uint16_t start = 0;
//extern PID_TypeDef motor_pid[6];
void Main_Init(void)
{
	u8 i=0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	
	uart1_init(115200);//��ʼ�����ڲ�����Ϊ115200
	USART3_Config();
 	LCD_Init(); 	
  LED_Init(); 
	KEY_Init(); 
	
	CAN1_Configuration();
	CAN2_Configuration();
	Adc1_Init();
	for(i=0;i<3;i++)
	{
		 pid_init(&rm3508_pid[i]);
		 rm3508_pid[i].f_param_init(&rm3508_pid[i],PID_Speed,20000,4000,10,0,8000,0,20,0.08,0);
	}
	for( i=0; i<6; i++)
  {	
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],PID_Position,16384,0,0,0,8000,0,1,0,40);
  }
	for( i=0; i<2; i++)
  {	
    pid_init(&motor_u3[i]);
    motor_pid[i].f_param_init(&motor_u3[i],PID_Position,1900,1700,20,0,5000,0,0.525,0.012,0);
  }

	//TIM13_Int_Init(1000-1,84-1);    //��ʱ����ʼ��
		TIM3_PWM_Init(20000-1,84-1);  	//500hz
		TIM_SetCompare1(TIM3,700);
		TIM_SetCompare2(TIM3,700);
		TIM_SetCompare3(TIM3,700);
		TIM_SetCompare4(TIM3,700);
		delay_ms(3000);

	usart5_config(9600);


	TIM2_Int_Init(1000-1,84-1);	    //ȫ�ֶ�ʱ�� 1ms

	TIM4_Mode_Config();
	TIM8_Mode_Config();
							//�ڲ�ʱ��168Mhz   -- 1khz	usart5_config(9600);

	TIM_SetCompare1(TIM3,1500);
	TIM_SetCompare2(TIM3,1500);

	delay_ms(11000);
	LCD_DisplayString(10,10,16,"pos_x");
	LCD_DisplayString(120,10,16,"pos_y");
	LCD_DisplayString(10,35,16,"z_angle");
	TIM10_Int_Init(1000-1,168-1); 
	TIM5_In_Init(10-1,83);        	//0.01s����һ��

	Clear_All();
}

