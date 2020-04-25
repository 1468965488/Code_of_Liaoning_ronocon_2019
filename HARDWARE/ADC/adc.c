#include "adc.h"
#include "sys.h"
#include "delay.h"

void Adc1_Init(void)
{
	ADC_CommonInitTypeDef adc;
	ADC_InitTypeDef init;
	GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//ʹ��ADC1ʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//ADC3��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 


	gpio.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_4;//ͨ�� 8  9
  gpio.GPIO_Mode = GPIO_Mode_AN;//ģ������
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOA, &gpio);//��ʼ��  
	
	adc.ADC_Mode=ADC_Mode_Independent;
	adc.ADC_Prescaler=ADC_Prescaler_Div4;
	adc.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
	adc.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&adc);
	
	init.ADC_ContinuousConvMode=DISABLE;//�ر�����ת��
	init.ADC_DataAlign=ADC_DataAlign_Right;//�Ҷ���	
	init.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������	
	init.ADC_NbrOfConversion=1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
	init.ADC_Resolution=ADC_Resolution_12b;//12λģʽ
	init.ADC_ScanConvMode=DISABLE;//��ɨ��ģʽ	
	ADC_Init(ADC1,&init);//ADC��ʼ��
	
	ADC_Cmd(ADC1,ENABLE);

}

/****************************************************************************
* ��    ��: u16 Get_Adc(u8 ch) 
* ��    �ܣ����ADCֵ
* ��ڲ�����ch: ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
* ���ز�����12λADC��Чֵ
* ˵    ����       
****************************************************************************/
u16 Get_Adc(u8 ch)   
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
  
	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}


