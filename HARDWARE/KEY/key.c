#include "key.h"
#include "delay.h" 
#include "lcd.h"
 /**
  ******************************************************************************
  * @file			key.c
  * @author		���紴���Ŷ�
  * @version	V1.0.0
  * @date			2017/6/16
  * @brief    ������mini�尴������
  ******************************************************************************
  * @attention
  *    
  ******************************************************************************
  */
extern u8 The_Most_Important_Thing ;
extern u8 BOI_1;
extern u8 BOI_2;
//������ʼ������
void KEY_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;

   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOFʱ��
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOFʱ��
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOFʱ��

  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|\
	                              GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|\
																GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13; //KEY0 KEY1 KEY2��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��GPIOF6,7,8,9
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_13; //KEY0 KEY1 KEY2��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOF6,7,8,9
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13; //KEY0 KEY1 KEY2��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOF6,7,8,9


	
} 
//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY2���� 
//4��WKUP���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>WK_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||WK_UP==0))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
		else if(KEY2==0)return 3;
		else if(WK_UP==0)return 4;
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==1)key_up=1; 	    
 	return 0;// �ް�������
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












