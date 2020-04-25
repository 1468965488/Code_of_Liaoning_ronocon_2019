#include "main.h"
 
/**
  ******************************************************************************
  * @file			pwm.c
  * @author		机电创新团队
  * @version	V1.0.0
  * @date			2017/7/4
  * @brief    PWM驱动Maxon RE25电机
  ******************************************************************************
  * @attention 因为RMDS-105驱动器采用光耦隔离方式 CTL1+ CTL12+电路的电压范围为
  *  5.0~26V 所以TIM输出比较极性为高此外robomodule驱动器输入频率在50-1khz 范围内
	*
  ******************************************************************************
  */


//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数

void TIM3_PWM_Init(u32 arr,u32 psc)
{	
   
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTC时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTC时钟	

	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3); //GPIOC7复用为定时器3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3); //GPIOC7复用为定时器3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3); //GPIOC7复用为定时器3
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3); //GPIOC7复用为定时器3

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;           //GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);      	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;           //GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);      	
	
	//初始化PC7

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse=0;
//	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
//  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
 
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
 
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
  
	TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	TIM_Cmd(TIM3, ENABLE);  //使能TIM14
	
	
}  

//TIM9 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM9_TIM_Init(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM3时钟使能    
	

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);//初始化定时器3
	

	
  TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPE使能 
	TIM_Cmd(TIM9, ENABLE);  //使能TIM14
	
	NVIC_InitStructure.NVIC_IRQChannel= TIM8_UP_TIM13_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,ENABLE);  ///使能TIM3时钟
//	
//  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
//	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM13,&TIM_TimeBaseInitStructure);//初始化TIM3
//	
//	TIM_ITConfig(TIM13,TIM_IT_Update,ENABLE); //允许定时器3更新中断
//	TIM_Cmd(TIM13,ENABLE); //使能定时器3
//	
//	NVIC_InitStructure.NVIC_IRQChannel= TIM8_UP_TIM13_IRQn; //定时器3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
