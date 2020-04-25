#include "adc.h"
#include "sys.h"
#include "delay.h"

void Adc1_Init(void)
{
	ADC_CommonInitTypeDef adc;
	ADC_InitTypeDef init;
	GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//使能ADC1时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//ADC3复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 


	gpio.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_4;//通道 8  9
  gpio.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &gpio);//初始化  
	
	adc.ADC_Mode=ADC_Mode_Independent;
	adc.ADC_Prescaler=ADC_Prescaler_Div4;
	adc.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
	adc.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&adc);
	
	init.ADC_ContinuousConvMode=DISABLE;//关闭连续转换
	init.ADC_DataAlign=ADC_DataAlign_Right;//右对齐	
	init.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发	
	init.ADC_NbrOfConversion=1;//1个转换在规则序列中 也就是只转换规则序列1 
	init.ADC_Resolution=ADC_Resolution_12b;//12位模式
	init.ADC_ScanConvMode=DISABLE;//非扫描模式	
	ADC_Init(ADC1,&init);//ADC初始化
	
	ADC_Cmd(ADC1,ENABLE);

}

/****************************************************************************
* 名    称: u16 Get_Adc(u8 ch) 
* 功    能：获得ADC值
* 入口参数：ch: 通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
* 返回参数：12位ADC有效值
* 说    明：       
****************************************************************************/
u16 Get_Adc(u8 ch)   
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}


