#include "key.h"
#include "delay.h" 
#include "lcd.h"
 /**
  ******************************************************************************
  * @file			key.c
  * @author		机电创新团队
  * @version	V1.0.0
  * @date			2017/6/16
  * @brief    适用于mini板按键设置
  ******************************************************************************
  * @attention
  *    
  ******************************************************************************
  */
extern u8 The_Most_Important_Thing ;
extern u8 BOI_1;
extern u8 BOI_2;
//按键初始化函数
void KEY_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;

   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOF时钟
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOF时钟

  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|\
	                              GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|\
																GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIOF6,7,8,9
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_13; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOF6,7,8,9
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOF6,7,8,9


	
} 
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY2按下 
//4，WKUP按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||WK_UP==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
		else if(KEY2==0)return 3;
		else if(WK_UP==0)return 4;
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==1)key_up=1; 	    
 	return 0;// 无按键按下
}




u8 KEY_Scan_2()
{
	 static u8 key_up=1;
	if(key_up && (basket_1||basket_2||basket_3||basket_4))
	{
		delay_ms(50);
		key_up=0;
		if(basket_1==0)  return 1;
		else if(basket_2==0)  return 2;
		else if(basket_3==0)  return 3;
		else if(basket_4==0)  return 4;
	}
	else if(basket_1&&basket_2&&basket_3&&basket_4)
		key_up=1;
	return 0;
}


void key_scanf()
{
		static u8 ba_1=0,ba_2=0,ok=0;

	u8 key=0;
	key=KEY_Scan_2();
	switch(key)
	{
		case 1:  ba_1++;   break;
		case 2:  ba_2++;   break;
		case 3:  ok+=1;  break;
		case 4:	 BOI_1=0;
						 BOI_2=0;
						 The_Most_Important_Thing=0;
		break;
	}
	BOI_1=ba_1%4+1;
	BOI_2=ba_2%4+1;
	The_Most_Important_Thing=ok%2;
}












