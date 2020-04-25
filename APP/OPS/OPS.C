#include "main.h"



float pos_x=0;
float pos_y=0;
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;
float original_angle=0;
float original_x=0;
float original_y=0;
float err_angle ,err_pos_x,err_pos_y;
uint8_t ppsTalkOk = 0;

void USART1_IRQHandler(void)  
{
	
	static uint8_t ch;
	static union
  {
	 uint8_t data[24];
	 float ActVal[6];
  }posture;
	static uint8_t count=0;
	static uint8_t i=0;
	
	
    
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//接收到数据
	{	 
		//USART_ClearITPendingBit(USART1,USART_IT_RXNE);		
		ch=USART_ReceiveData(USART1);
		
		 switch(count)
		 {
			 case 0:
				 if(ch==0x0d)
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 1:
				 if(ch==0x0a)
				 {
					 ppsTalkOk=1;
					 i=0;
					 count++;
				 }
				 else if(ch==0x0d);
				 else
					 count=0;
				 break;
				 
			 case 2:
				 posture.data[i]=ch;
			   i++;
			   if(i>=24)
				 {
					 i=0;
					 count++;
				 }
				 break;
				 
			 case 3:
				 if(ch==0x0a)
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 4:
				 if(ch==0x0d)
				 {
  				 original_angle=posture.ActVal[0];
	  		   xangle=posture.ActVal[1];
		  	   yangle=posture.ActVal[2];
			     original_x =posture.ActVal[3];//+500;
			     original_y =posture.ActVal[4];//+500;
			     w_z=posture.ActVal[5];
				 }
			   count=0;
				 break;
			 
			 default:
				 count=0;
			   break;		 
		 }
	 	GetXY();
	}
} 


//定位系统初始化
void Clear_All(void)
{
	uint8_t i = 0;
	uint8_t tdata[4];

  tdata[0]='A';
  tdata[1]='C';
	tdata[2]='T';
	tdata[3]='0';
	
	for(i=0;i<4;i++)
		USART_SendData(USART1,tdata[i]);
}
//硬件矫正X值
void CorrectX(float value)
{
	uint8_t i = 0;
	uint8_t tdata[10];
  union{
		float   val;
		uint8_t data[4];
	}valSend;

  tdata[0]='A';
	tdata[1]='C';
	tdata[2]='T';
  tdata[3]='X';
  tdata[8]='\r';
  tdata[9]='\n';
	
	valSend.val=(float)value;
  memcpy(tdata+4,valSend.data,4);
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(1);
		for(i=0;i<10;i++)
			USART_SendData(USART1,tdata[i]);	
	}
}
//硬件矫正Y
void CorrectY(float value)
{
	uint8_t i=0;
	uint8_t tdata[10];
  union{
		float   val;
		uint8_t data[4];
	}valSend;

  tdata[0]='A';
	tdata[1]='C';
	tdata[2]='T';
  tdata[3]='Y';
  tdata[8]='\r';
  tdata[9]='\n';
	
	valSend.val=(float)value;
  memcpy(tdata+4,valSend.data,4);
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(1);
		for(i=0;i<10;i++)
		  USART_SendData(USART1,tdata[i]);
	}
}
//硬件矫正角度
void CorrectAngle(float value)
{
	uint8_t	i = 0;
	
		uint8_t tdata[10];
  union{
		float   val;
		uint8_t data[4];
	}valSend;

	if(value>180.f)
		value=value-360.f;
	else if(value<-180.f)
		value=value+360.f;
	

  tdata[0]='A';
	tdata[1]='C';
	tdata[2]='T';
  tdata[3]='J';
  tdata[8]='\r';
  tdata[9]='\n';
	
	valSend.val=(float)value;
  memcpy(tdata+4,valSend.data,4);
	
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(1);
		for(i=0;i<10;i++)
		 USART_SendData(USART1,tdata[i]);
	}
}


/**
*定位系统数据处理
**/
float Angle_deviation=0;
float RotateX=0,RotateY=0;
float Xerror=0,Yerror=0;
extern u16 err_y;
//旋转坐标系
void GetXY(void)
{
	//旋转坐标系
	RotateX=original_x*cos(Angle_deviation*0.0174f)-original_y*sin(Angle_deviation*0.0174f);
	RotateY=original_y*cos(Angle_deviation*0.0174f)+original_x*sin(Angle_deviation*0.0174f);
	//坐标转化
	pos_x=-(RotateX-Xerror);
	pos_y=-(RotateY-Yerror)+77;
	//角度转化
	zangle=-(original_angle+Angle_deviation);
	if (zangle > 180) zangle -= 360;
	if (zangle < -180)zangle += 360;
}

//角度及Y值矫正
u8 CorrectValue(float Ex_pos_x,float Ex_pos_y,float expectangle)
{
		float sety=0,setx;
		
		GetDeviation(expectangle);
		GetXY();
		sety=RotateY-Ex_pos_y;
		setx=RotateX-Ex_pos_x;
		if(fabs(sety-Yerror)<400)
		SetYerror(sety);
		if(fabs(setx-Xerror)<400)	
		SetXerror(setx);

		return 1;
}
//角度角度矫正（即获取角度偏差）
void GetDeviation(float expectangle)
{
	Angle_deviation = expectangle-original_angle;
}
//获取X偏差
void SetXerror(float SetX)
{
	Xerror=SetX;
}
//获取Y值偏差
void SetYerror(float SetY)
{
	Yerror=SetY;
}

