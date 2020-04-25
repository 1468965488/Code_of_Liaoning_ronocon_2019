#include "enc.h"
#include "sys.h"
#include "main.h"
int speed_TIM4;
int speed_TIM8;
int xue_yong4;
int xue_yong8;
int enc_TIM8=0;
int enc_TIM4=0;
 u32 tim5_counter;
long int total_angle_TIM8;
long int total_angle_TIM4;

long int  last_angle_TIM4;
long int  last_angle_TIM8;

extern long int sum_2006_angle_201;
long int last_201_angle=0;
extern  CanTxMsg tx_message;

void TIM4_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef tim;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
		
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	tim.TIM_Period=4096*2-1;
	tim.TIM_Prescaler=0;	
	tim.TIM_ClockDivision=TIM_CKD_DIV1;
	tim.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4,&tim);
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ICStructInit(&TIM_ICInitStructure);   
	TIM_ICInitStructure.TIM_ICFilter=6;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM4->CNT =0;                                   //初值

	TIM_Cmd(TIM4, ENABLE);
	
	
}

void Data_processing_TIM4(int enc)
{
	static int enc_last,enc_now;
	int enc_1,enc_2;
	int data;
	
	enc_now=enc;
	if(enc_last>enc_now)
	{
		enc_1=enc_now+4096-enc_last;
		enc_2=enc_now-enc_last;
	}
	else
	{
		enc_1=enc_now-4096-enc_last;
		enc_2=enc_now-enc_last;
	}
	if(abs(enc_1)>abs(enc_2))
	{
		data=enc_2;
	}
	else data=enc_1;
	
	total_angle_TIM4+=data;
	
	enc_last=enc_now;
	
}

void Date_encoder_TIM4(void)
{
	if(enc_TIM4!=(TIM4->CNT)/2)
	{
		enc_TIM4=(TIM4->CNT)/2;
		Data_processing_TIM4(enc_TIM4);
	}
}

	int ex_speed=0;



void TIM5_In_Init(u16 arr,u16 psc)
{	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	
  NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period=arr; //自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler=psc; //定时器分频
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //自动重载周期值
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);	
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM5,ENABLE);
}


/*TIM5计数器用以编码器数据处理*/
u8 hanser_1=0;
u8 hanser_2=0;

void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) ///溢出中断
	{
			TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //清除中断标志位
			tim5_counter++;
				if(tim5_counter%1000==0)
				{
					Date_encoder_TIM8();
					Date_encoder_TIM4();
				
					xue_yong8=total_angle_TIM8-last_angle_TIM8;
					speed_TIM8=xue_yong8*5.85937;
					last_angle_TIM8=total_angle_TIM8;
					
					xue_yong4=total_angle_TIM4-last_angle_TIM4;
					speed_TIM4=xue_yong4*5.85937;
					last_angle_TIM4=total_angle_TIM4;

//					hanser_1=1;
//					hanser_2=1;
					if(hanser_2)
					{
						motor_u3[0].f_cal_pid(&motor_u3[0],speed_TIM4);
						if(motor_u3[0].output<1170) motor_u3[0].output=1170;
						TIM_SetCompare3(TIM3,motor_u3[0].output);
					}
					else
						TIM_SetCompare3(TIM3,0);
					if(hanser_1)
					{
						motor_u3[1].f_cal_pid(&motor_u3[1],speed_TIM8);
						if(motor_u3[1].output<1170) motor_u3[1].output=1170;
						TIM_SetCompare4(TIM3,motor_u3[1].output);
					}
					else
						TIM_SetCompare4(TIM3,0);
				}
	}
}



void TIM8_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef tim;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	tim.TIM_Period=4096*2-1;
	tim.TIM_Prescaler=0;	
	tim.TIM_ClockDivision=TIM_CKD_DIV1;
	tim.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8,&tim);
	
	TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ICStructInit(&TIM_ICInitStructure);    //????????
	TIM_ICInitStructure.TIM_ICFilter=6;
	TIM_ICInit(TIM8,&TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	TIM8->CNT =0;                                   //初值

	TIM_Cmd(TIM8, ENABLE);
	
	
}


void Data_processing_TIM8(int enc)
{
	static int enc_last_8,enc_now_8;
	int enc_1,enc_2;
	int data;
	
	enc_now_8=enc;
	if(enc_last_8>enc_now_8)
	{
		enc_1=enc_now_8+4096-enc_last_8;
		enc_2=enc_now_8-enc_last_8;
	}
	else
	{
		enc_1=enc_now_8-4096-enc_last_8;
		enc_2=enc_now_8-enc_last_8;
	}
	if(abs(enc_1)>abs(enc_2))
	{
		data=enc_2;
	}
	else data=enc_1;
	
	total_angle_TIM8+=data;
	
	enc_last_8=enc_now_8;
	
}

void Date_encoder_TIM8(void)
{
	if(enc_TIM8!=(TIM8->CNT)/2)
	{
		enc_TIM8=(TIM8->CNT)/2;
		Data_processing_TIM8(enc_TIM8);
	}
}

