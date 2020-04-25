#include "main.h"
#include "move_basic.h"

#define pi 3.141592f
#define Dis_Factor      500  	 //¾àÀëÒò×Ó
#define Map_to_angle    57.2957f
#define Map_to_radian   0.017453f
#define Front_wheel_proportion 3

extern struct __motor_feedback motor_feedback; 
extern struct _motor_feedback Motor_feedback;

//¶¨ÒåÎÒµÄ·¢ÉäÃ¤Çø
#define Blind_Zone_1(x,y)  ((x>1600 && y<800) || (x> 800 && y>3200) || (x< -800 && y>3200) || (x<-1600 && y<800))
#define Blind_Zone_2(x,y)  ((x>1600 && y<800) || (x>1600 && y>3200) || (x< -800 && y>3200) || (x<-800 && y<1600))
#define Blind_Zone_3(x,y)  ((x>800 && y<1600) || (x>1600 && y>3200) || (x<-1600 && y>3200) || (x<-800 && y<1600))
#define Blind_Zone_4(x,y)  ((x>800 && y<1600) || (x> 800 && y>3200) || (x<-1600 && y>3200) || (x<-1600 && y<800))
#define Blind_Zone_5(x,y)  ((x>800 && y<1600) || (x>1600 && y>3200) || (x< -800 && y>3200) || (x<-1600 && y<800))
#define Blind_Zone_6(x,y)  ((x>1600 && y<800) || (x> 800 && y>3200) || (x<-1600 && y>3200) || (x<-800 && y<1600))
//Ã¤ÇøÉè¶¨ÇøÓò¿ÉÄÜ²»Ì«ºÏÀí£¬Òª¸ÄµÄ


//¶¨Òå·¢ÉäµÄÄâºÏº¯Êý   
//static int Fire_Function_1(double x) { return 3.367e-09 *x*x*x - 8.796e-05 *x*x + 1.368 *x + 2134;}
//static int Fire_Function_2(double x) { return 3.316e-12 *x*x*x*x - 4.687e-08 *x*x*x + 0.0001939 *x*x + 0.6804 *x + 2913;}
#define Fire_Function_4(x)  x*1
#define Fire_Function_5(x)  x*1
#define Fire_Function_3(x)  x*1

 int All_Function(u8 id,double x)
{
	int speed=0;
	switch(id)
	{
		case 0:			speed=              1.183e-06*x*x*x - 0.007158*x*x + 15.47 *x -7011    ;          break;
		case 1:			speed=x>3500?				(closest_wall()==2||closest_wall()==4)?
																		-6.841e-07 *x*x*x + 0.01032 *x*x - 51.14 *x + 9.043e+04+15+15:
																		-6.841e-07 *x*x*x + 0.01032 *x*x - 51.14 *x + 9.043e+04+30+15+15:
																		(closest_wall()==2||closest_wall()==4)?
																		-2.093e-07*x*x*x + 0.001404 *x*x - 2.195 *x + 5289:
																		-2.093e-07*x*x*x + 0.001404 *x*x - 2.195 *x + 5289-15;  		 			break;
		case 2:			speed=x>3500?				(closest_wall()==2||closest_wall()==4)?
																		 3.381e-07 *x*x*x - 0.005083 *x*x + 26.12 *x - 3.849e+04+13+15:
																		 3.381e-07 *x*x*x - 0.005083 *x*x + 26.12 *x - 3.849e+04+15:
																		(closest_wall()==2||closest_wall()==4)?
																		-7.772e-08 *x*x*x + 0.0004861 *x*x - 0.0421 *x + 3744 -15:   //Î´¸Ä
																		-7.772e-08 *x*x*x + 0.0004861 *x*x - 0.0421 *x + 3744 -30;		 break;
		case 3:			speed=x>3500?     	(closest_wall()==2||closest_wall()==4)?
																		6.742e-07 *x*x*x - 0.01017 *x*x + 51.63 *x - 8.106e+04 +20+12 +15:
																		6.742e-07 *x*x*x - 0.01017 *x*x + 51.63 *x - 8.106e+04 +20+12 +85+15:
																		1.011e-07 *x*x*x - 0.0007563 *x*x + 2.622 *x + 1890;  				break;
		case 4:			speed=x>3500?				(closest_wall()==2||closest_wall()==4)?
																		-3.572e-08 *x*x*x + 0.0005109 *x*x - 1.696 *x + 7400  +15+58+15:
																		-3.572e-08 *x*x*x + 0.0005109 *x*x - 1.696 *x + 7400  +15+35+15:
																		-1.956e-07 *x*x*x + 0.001289 *x*x - 1.864 *x + 5221-58;					break;
	}
	return speed;
}
extern u8 The_Most_Important_Thing ;
extern int target[5][2];         //¿òµÄ×ø±ê£¬
extern u8 BOI_1;
extern u8 BOI_2;
extern u8 edge_g;

extern u8  escape_timer_switch;   //ÌÓÒÝ¶¨Ê±Æ÷¿ª¹Ø
extern u16 escape_timer;

extern float pos_x;
extern float pos_y;
extern float zangle;
extern float chassic_speed;          //µ×ÅÌËÙ¶È




//µ×ÅÌËÙ¶È¿ØÖÆº¯Êý
void Chassic_vel(int speed_L, int speed_R,int speed_F)
{
	rm3508_pid[0].target= speed_L;
	rm3508_pid[1].target=-speed_R;              //Ã»¼Ç´íÓÐÒ»¸öÒª¼Ó¸ººÅµÄ
	rm3508_pid[2].target= speed_F;
}
//ÔË¶¯-½Ç¶È¿ØÖÆ
void M_Angle_control(float ex_angle,int speed)
{
	double adjust=0.f;
	int L_speed=0,R_speed=0,F_speed=0;
	adjust=Angle_control(ex_angle,zangle);
	L_speed=speed+(int)adjust;               //¾ßÌå¼Ó¼õ»¹Ã»ÓÐÈ·¶¨
	R_speed=speed-(int)adjust;

	F_speed=adjust*Front_wheel_proportion;
	Chassic_vel(L_speed,R_speed,F_speed);
}
//µ¹ÍË½Ç¶È±Õ»·
void M_Back_control(float ex_angle,int speed)
{
	double adjust=0.f;
	int L_speed=0,R_speed=0,F_speed=0;
	adjust=Angle_control(ex_angle,zangle);
	L_speed=speed-(int)adjust;              
	R_speed=speed+(int)adjust;
	F_speed=(int)(adjust*Front_wheel_proportion);
	Chassic_vel(-L_speed,-R_speed,F_speed);
}
//½Ç¶ÈÉú³ÉÆ÷
//×¢Òâ²»ÒªÊäÈëÁ½¸öÏàÍ¬µÄµã£¬Ã»ÓÐÐ´´¦ÀíÄÇÖÖÇé¿ö
//¶¨ÒåÐ±ÂÊÖ±ÏßÄ£Ê½
u8 line_mode=0;
float Angle_generator(float start_x,float start_y,float aim_x,float aim_y)
{
	float line_angle=0,slope_line=0;
	if(start_x==aim_x || start_y == aim_y)      //×ßÖ±Ïß
	{
		if(start_x==aim_x)
			line_angle=aim_y-start_y>0?  0  : 180 ;  
		else
			line_angle=aim_x-start_x>0?  90 : -90 ;
		line_mode=0;                               //Ö±ÏßÄ£Ê½0 ±íÊ¾ºáÆ½ÊúÖ±µÄÖ±Ïß
	}
	else
	{
		slope_line=(aim_y-start_y)/(aim_x-start_x);
		if(slope_line>0)
		{
			if(aim_y>start_y)
				line_angle=-atanf(slope_line)*Map_to_angle + 90;
			else
				line_angle=-atanf(slope_line)*Map_to_angle - 90;
		}
		else
		{
			if(aim_y>start_y)
				line_angle=-atanf(slope_line)*Map_to_angle - 90;
			else
				line_angle=-atanf(slope_line)*Map_to_angle + 90;
		}
		line_mode=1;                              //Ö±ÏßÄ£Ê½1 ±íÊ¾Ð±ÏßÄ£Ê½
	}
	return line_angle;
}

//	S=fabs((y2-y1)*(x3-x1)-(y3-y1)*(x2-x1));  ¼ÆËãÃæ»ýµÄ¹«Ê½
//  Ö±Ïß¿ØÖÆ,ÆäÊµÆðÊ¼µãÓëÄ¿±êµã×ø±ê£¬speed±íÊ¾ËÙ¶È£¬Õý±íÊ¾ÍùÇ°£¬¸ºÔò»á±íÊ¾Íùºó£¬²»¿ÉÒÔ¸ø0
//	Ã»ÓÐ×ßËÙ¶È²»ÄÜÐ¡ÓÚ0
void line_move_control(int start_x,int start_y,int aim_x ,int aim_y ,int speed)
{
	float line_angle=0,final_angle,d=0.f,S=0.f;
	//u8 dir_flag=0;
	//dir_flag=speed>0?  1:0;      //1±íÊ¾ÍùÇ°£¬0±íÊ¾Íùºó          
	line_angle=Angle_generator(start_x,start_y,aim_x,aim_y);
	if((start_x==aim_x)||(start_y==aim_y) )  //ºáÆ½ÊúÖ±µÄÖ±Ïß
	{
		if(start_x==aim_x)
		{	
			d=fabs(pos_x-start_x);
			if(aim_y>start_y)
			{
				if(pos_x>start_x)
					final_angle=line_angle-(1-Dis_Factor/(d+Dis_Factor))*90;    //ÎÒ´òËãÊÔÊÔ¶«´óµÄÏë·¨
				else
					final_angle=line_angle+(1-Dis_Factor/(d+Dis_Factor))*90;
			}
			else
			{
				if(pos_x<start_x)
					final_angle=line_angle-(1-Dis_Factor/(d+Dis_Factor))*90;    //ÎÒ´òËãÊÔÊÔ¶«´óµÄÏë·¨
				else
					final_angle=line_angle+(1-Dis_Factor/(d+Dis_Factor))*90;
			}
		}
		else
		{
			d=fabs(pos_y-start_y);
			if(aim_x>start_x)          //ÕâÒ»²¿·ÖÓ¦¸Ã¿ÉÒÔ¼ò»¯
			{
				if(pos_y>aim_y)
					final_angle=line_angle+(1-Dis_Factor/(d+Dis_Factor))*90;
				else
					final_angle=line_angle-(1-Dis_Factor/(d+Dis_Factor))*90;
			}
			else
			{
				if(pos_y<aim_y)
					final_angle=line_angle+(1-Dis_Factor/(d+Dis_Factor))*90;
				else
					final_angle=line_angle-(1-Dis_Factor/(d+Dis_Factor))*90;
			}
				
		}
	}
	else
	{
		//¼ÆËãÃæ»ý¹«Ê½£¬´øÕý¸ººÅ
		S=(aim_y-start_y)*(pos_x-start_x)-(pos_y-start_y)*(aim_x-start_x);
		//µãµ½Ö±Ïß¾àÀë
		d=2*S*invSqrt((start_x-aim_x)*(start_x-aim_x)+(start_y-aim_y)*(start_y-aim_y));
		if(d>15*90)d=15*90;
		if(d<-15*90)d=-15*90;
		final_angle=line_angle-d/15;
	}                               
	M_Angle_control(final_angle,speed);
}


//Ö»ÄÜ×ßºáÆ½ÊúÖ±µÄÖ±ÏßºóÍËµÄµãµ½µãÃ»ÓÐÒâÒå
//ÏòºóÍË×ßÈÎÒâ½Ç¶È¿ÉÒÔÓÃ½Ç¶ÈÉú³ÉÆ÷»òÕßÖ±½Ó¸ø½Ç¶È¾ÍÐÐ
void back_line_control(int start_x,int start_y,int aim_x ,int aim_y ,int speed)
{
		float line_angle=0.f,d=0.f,final_angle=0.f;
		line_angle=Angle_generator(start_x,start_y,aim_x,aim_y);
				if(start_x==aim_x)
		{	
			d=fabs(pos_x-start_x);
			if(aim_y>start_y)
			{
				if(pos_x>start_x)
					final_angle=line_angle+(1-Dis_Factor/(d+Dis_Factor))*90;    //ÎÒ´òËãÊÔÊÔ¶«´óµÄÏë·¨
				else
					final_angle=line_angle-(1-Dis_Factor/(d+Dis_Factor))*90;
			}
			else
			{
				if(pos_x<start_x)
					final_angle=line_angle+(1-Dis_Factor/(d+Dis_Factor))*90;    //ÎÒ´òËãÊÔÊÔ¶«´óµÄÏë·¨
				else
					final_angle=line_angle-(1-Dis_Factor/(d+Dis_Factor))*90;
			}
		}
		else
		{
			d=fabs(pos_y-start_y);
			if(aim_x>start_x)          //ÕâÒ»²¿·ÖÓ¦¸Ã¿ÉÒÔ¼ò»¯
			{
				if(pos_y>aim_y)
					final_angle=line_angle+(1-Dis_Factor/(d+Dis_Factor))*90;
				else
					final_angle=line_angle-(1-Dis_Factor/(d+Dis_Factor))*90;
			}
			else
			{
				if(pos_y<aim_y)
					final_angle=line_angle-(1-Dis_Factor/(d+Dis_Factor))*90;
				else
					final_angle=line_angle-(1-Dis_Factor/(d+Dis_Factor))*90;
			}
				
		}
		M_Back_control(final_angle,zangle);

}
//ÎÒµÄ×ßÔ²º¯Êý£¬Ð´ÍêÁË£¬ºÃ¶Ì
//state±íÊ¾×ßË³Ê±Õë»¹ÊÇÄæÊ±Õë
void My_circle(float x,float y,u16 R,int speed,u8 state)
{
	float d=0.f,direction=0.f,l_angle;
	d=R-sqrt((x-pos_x)*(x-pos_x)+(y-pos_y)*(y-pos_y));
//	if(d> 900) d= 900;
//	if(d<-900) d=-900;
	l_angle=Angle_generator(x,y,pos_x,pos_y);
	if(state)    																			 //1±íÊ¾Ë³Ê±Õë
		direction=l_angle+90-d/10;
	else         																			 //0±íÊ¾ÄæÊ±Õë
		direction=l_angle-90+d/10;
	if(direction> 180)   direction=-360+direction;
	if(direction<-180)   direction= 360+direction;

	M_Angle_control(direction,speed);
}
//µ¹×ßÔ²º¯Êý
void My_back_circle(float x,float y,u16 R,int speed,u8 state)
{
	float d=0.f,direction=0.f,l_angle;
	d=sqrt((x-pos_x)*(x-pos_x)+(y-pos_y)*(y-pos_y))-R;
	l_angle=Angle_generator(x,y,pos_x,pos_y);
	if(state)    																			 //1±íÊ¾Ë³Ê±Õë
		direction=l_angle+90-d/10;
	else         																			 //0±íÊ¾ÄæÊ±Õë
		direction=l_angle-90+d/10;
	if(direction> 180)   direction=-360+direction;
	if(direction<-180)   direction= 360+direction;
	
	M_Back_control(direction,speed);
}
  /**
  ******************************************************************************
  * @file		go_ellipse_cw()	
  * @brief  Ð¡³µË³Ê±Õë×ßÍÖÔ²
  ******************************************************************************
  **/
void My_ellipse(int X,int Y,float a,float b,int speed,u8 state)
{
	
	float expectAngle = 0.f,part_1=0.f,part_2,part_3=0.f,part_4=0.f,act_distance=0.f,ex_Distance=0.f,angle=0.f;
	float adjust_angle=0.f;
		if(pos_x==0)
			X=1;
	part_1=(pos_x-X)/(pos_y-Y);
	part_2=(b*b)/(a*a);
	angle=atan(part_1*part_2)*Map_to_angle;
	if(pos_x==X || pos_y==Y)
	{
		if(pos_x==X)
		{
			if(pos_y>Y)
				expectAngle=state==1? 90:-90;
			else
				expectAngle=state==1? -90:90;
		}
		else
		{
			if(pos_x>X)
				expectAngle=state==1?  180:0;
			else
				expectAngle=state==1?  0:180;
		}
	}
	else
	{
			if (state == 1)    //Ë³Ê±Õë
			{
				if (pos_x > X)
				{
					if (pos_y >= Y)
						expectAngle =  angle + 90;
					if (pos_y < Y)
						expectAngle =  angle - 90;
				}
				if (pos_x < X)
				{
					if (pos_y >= Y)
						expectAngle = 90 + angle;
					if (pos_y < Y)
						expectAngle = angle - 90;
				}
			}
			else          //ÄæÊ±Õë
			{
				if (pos_x > X)
				{
					if (pos_y >= Y)
						expectAngle = -90 + angle;
					if (pos_y < Y)
						expectAngle = 90 + angle;
				}
				if (pos_x < X)
				{
					if (pos_y >= Y)
						expectAngle = -90 + angle;
					if (pos_y < Y)
						expectAngle = 90 + angle;
				}
			}		
}	
			part_3=(a*a)*(b*b)*(1+((pos_y-Y)/(pos_x-X))*((pos_y-Y)/(pos_x-X)));
			part_4=(a*a)*(((pos_y-Y)/(pos_x-X))*((pos_y-Y)/(pos_x-X)))+(b*b);
			ex_Distance=sqrt(part_3/part_4);
			act_distance=sqrt((X-pos_x)*(X-pos_x)+(Y-pos_y)*(Y-pos_y));
			if(state==1)
			adjust_angle=expectAngle-(ex_Distance-act_distance)/12;
			else
			adjust_angle=expectAngle+(ex_Distance-act_distance)/12;
			if(adjust_angle>180)
				adjust_angle-=360;
			if(adjust_angle<-180)
				adjust_angle+=360;
			M_Angle_control(adjust_angle, speed);
			LCD_DisplayNum(100,160,angle,8,24,0);
			LCD_DisplayNum(100,190,-expectAngle,8,24,0);
	
}
//µÃµ½È¦Êý
u8 rso_sunshine=1;
u8 get_circle_num()
{
	static u8 n1=0,n2=0,n3=0,n4=0,circle=0;
	u8 situation=0;
	if(pos_x>0 && pos_y<2400)   n1=1;
	if(pos_x>0 && pos_y>2400)   n2=1;
	if(pos_x<0 && pos_y>2400)   n3=1;
	if(pos_x<0 && pos_y<2400)   n4=1;
		situation=codelf();

		if(n1==1 && n2==1 && n3==1 && n4==1  )
		{
		if(The_Most_Important_Thing==0)    // À¶³¡
		{
					switch(situation)
			{
				case 1:
						if(pos_x>0 && pos_y>2600)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}

						break;
				case 2:
						if(pos_x<-100 && pos_y>2000)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;
					
				case 3:
								if(pos_x<0 && pos_y<2000)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}

				
						break;
					
				case 4:
						
							if(pos_x>400 && pos_y<2100)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}

						break;
					
				case 5:
					if(rso_sunshine==2)
					{
						if(pos_x>600 && pos_y>2700)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;
					}
					else
					{
						if(pos_x<0 && pos_y<2100)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;						
					}
				case 6:
					if(rso_sunshine==2)
					{
						if(pos_x>600 && pos_y>2700)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;
					}
					else
					{
						if(pos_x<0 && pos_y<2100)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;						
					}
			}

		}
		else                      //ºì³¡
		{
					switch(situation)
			{
				case 1:
					
						if(pos_x<0 && pos_y<2000)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;
				case 2:
						if(pos_x>400 && pos_y<2000)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;
					
				case 3:
						if(pos_x>0 && pos_y>2100)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;
					
				case 4:
						if(pos_x<0 && pos_y>2400)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;
					
				case 5:
						if(rso_sunshine==2)
						{							
							if(pos_x>0 && pos_y>2700)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;

						}
						else
						{
							if(pos_x<0 && pos_y<2200)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;
						}
					
				case 6:
						if(rso_sunshine==2)
						{							
							if(pos_x>0 && pos_y>2700)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;

						}
						else
						{
							if(pos_x<0 && pos_y<2200)
						{
								circle++;
								n1=0;n2=0;n3=0;n4=0;
						}
						break;
						}
					
			}

			
		}
	}
	return circle;
}



//ÌÓÒÝº¯Êý
u8 Escape(u8 state)      //Ë³ÄæÊ±Õë£¬Í¬ÒâÓÃ1±íÊ¾Ë³Ê±Õë  £¬0±íÊ¾ÄæÊ±Õë
{
	float distance=0.f;
	u8 finish=0;
	static u8 step=1,allow_1=1,allow_2=1;
	static float err_x=0.f,err_y=0.f,err_angle=0.f; 
	static u8 locate_flag=0;  //ÓÃÀ´±íÊ¾½øÈëÌÓÒÝË²¼äµÄÎ»ÖÃÒÔ¼°½Ç¶È
	distance=sqrt(pos_x*pos_x+(2400-pos_y)*(2400-pos_y));
	
	if(allow_2==1)
	{
		if(distance<1350)
		{
			allow_1=1;
		}
		else
		{
			allow_1=0;
		}
		allow_2=0;
	}
	
						if(allow_1==1)
						{
							
							switch(step)
							{
								case 1:
									escape_timer_switch=1;//¿ªÆô¶¨Ê±Æ÷
									if(locate_flag==0)                         
									{
										err_x=pos_x;
										err_y=pos_y;
										err_angle=zangle;
										locate_flag=1;   //×îºóÒªÇåµôµÄ£¬ËäÈ»¿ÉÄÜ¸ù±¾²»ÐèÒªÕâ¸ö±êÖ¾Î»
										finish=0;
										allow_2=0;        //²»ÈÃ½øÍâÈ¦µÄ²¿·Ö
										step=2;
									}
									break;
									
								case 2:
									if(escape_timer<800)
									{
										if(state==1)   //Èç¹ûË³Ê±Õë
											M_Back_control(err_angle-20,3000);
										else
											M_Back_control(err_angle+20,3000);
									}
									if(escape_timer>=800)
										step=3;
								break;
									
								case 3:
									if(escape_timer>800 &&escape_timer<2100)
									{
										if(state)
											My_circle(err_x,err_y,650,3500,1);
										else
											My_circle(err_x,err_y,650,3500,0);
									}
									if(escape_timer>2100)
										step=4;
									break;
									
								case 4:
									
									escape_timer_switch=0;    //¹Øµô¸÷ÖÖ¶¨Ê±ºÍ±êÖ¾Î»£¬ÌÓÒÝÍê³É±êÖ¾ÖÃ1
									escape_timer=0;
									err_x=0;
									err_y=0;
									err_angle=0;
									locate_flag=0; 
									step=1;
									finish=1;
									allow_2=1;
									break;
							}
						
					}
						

						else	
						{
							
							switch (step)
							{
								case 1:
									escape_timer_switch=1;//¿ªÆô¶¨Ê±Æ÷
									if(locate_flag==0)                         
									{
										err_x=pos_x;
										err_y=pos_y;
										err_angle=zangle;
										locate_flag=1;   //×îºóÒªÇåµôµÄ£¬ËäÈ»¿ÉÄÜ¸ù±¾²»ÐèÒªÕâ¸ö±êÖ¾Î»
										finish=0;
										allow_1=0;
									step=2;
									}
								break;
								
								case 2:
									if(escape_timer<800)
									{
										if(state)   //Èç¹ûË³Ê±Õë
											M_Back_control(err_angle-25,3000);						
										else
											M_Back_control(err_angle+25,3000);
									}
									if(escape_timer>=800)
										step=3;
								break;
								
								case 3:
										if(escape_timer>800  && escape_timer<2100)
									{
										if(state)
											My_circle(err_x,err_y,650,3500,0);
										else
											My_circle(err_x,err_y,650,3500,1);
									}
									if(escape_timer>2100)
										step=4;
									break;

									
								case 4:
									
									escape_timer_switch=0;    //¹Øµô¸÷ÖÖ¶¨Ê±ºÍ±êÖ¾Î»£¬ÌÓÒÝÍê³É±êÖ¾ÖÃ1
									escape_timer=0;
									err_x=0;
									err_y=0;
									err_angle=0;
									locate_flag=0; 
									step=1;
									finish=1;
									allow_2=1;
								break;

							}
						}

		
	return finish;
}
   /**
  ******************************************************************************
  * @file		JudgeStop()	
  * @brief  ÅÐ¶ÏÊÇ·ñÍ£Ö¹
  ******************************************************************************
  */
//Ïëµ½ÁË¸ü¿ÆÑ§µÄ°ì·¨£¬¿ÉÄÜ²»»áÓÃÕâ¸ö¶«Î÷£¬£¬àÅ£¬È·¶¨²»ÓÃÕâ¸ö¶«Î÷ÁË
//int get_speed()
//{
//	static int cuerrnt_x=0 ,cuerrnt_y=0,last_x=0,last_y=0,speed=0;
//	int diff_x=0,diff_y=0;
//	cuerrnt_x=pos_x;
//	cuerrnt_y=pos_y;
//	diff_x=cuerrnt_x-last_x;
//	diff_y=cuerrnt_y-last_y;
//	speed=100*sqrt(diff_x*diff_x+diff_y*diff_y);
//	last_x=pos_x;
//	last_y=pos_y;
// 
//	//LCD_DisplayNum(80,200,speed,4,24,0);
//	return speed;
//	
//}


void clear_edge(u8 state,int speed )
{
	
	if(((pos_x>-1500) && (pos_x<1500)) || ( (pos_y>900 )  && (pos_y<3900) ))
		speed=6000;
	else
		speed=4200;
	
		if(state==0)   //ÄæÊ±Õë                           
	{
		if(pos_x>1300 && pos_y<3700)
		{
			if(pos_x<1900)
				line_move_control(2400-300,0,2400-300,100,speed);
			else
			{
				if(abs(zangle)<7)
					line_move_control(2400-320,0,2400-320,100,speed);
				else
					M_Angle_control(0,speed);	
			}
		}
		if(pos_x>-1300 && pos_y>3700)
		{
			if( pos_y<4300)
				line_move_control(0,4800-300,-100,4800-300,speed);
			else
			{
				if(abs(zangle+90)<10 )
					line_move_control(0,4800-280,-100,4800-280,speed);
				else
					M_Angle_control(-90,speed);	
			}
		}
		if(pos_x<-1300 && pos_y>1100)
		{
			if(pos_x>-1900)
				line_move_control(-2400+300,100,-2400+300,0,speed);
			else
			{
				if((zangle>173 && zangle<=180 )|| (zangle>=-180 && zangle<-173) ) 
					line_move_control(-2400+320,100,-2400+320,0,speed);
				else
					M_Angle_control(180,speed);	
			}
		}
		if(pos_x<1300  && pos_y<1100)
		{
			if(pos_y>500)
				line_move_control(0,0+340,100,0+340,speed);
				else
			{
				if(abs(zangle-90)<7 )
					line_move_control(0,0+320,100,0+320,speed);
				else
					M_Angle_control(90,speed);	
			}
		}
	}
	else     //Ë³Ê±Õë                              
	{
		if(pos_x>1000 && pos_y>1200)
		{
			line_move_control(2400-300,100,2400-300,0,speed);
		}
		if(pos_x<1400 && pos_y>3800)
		{
			line_move_control(0,4800-300,100,4800-300,speed);
		}
		if(pos_x<-1400 && pos_y<3800)
		{
			line_move_control(-2400+300,0,-2400+300,100,speed);
		}
		if(pos_x>-1400 && pos_y<1200)
		{
			line_move_control(100,0+340,0,0+340,speed);
		}

	}
}

//É¨±ßº¯Êý£¬Ô¤ÏëÊÇÁ½¸öÉ¨±ßµÄ½øÈë¹ÕÍäµÄÇøÓò²»Ò»Ñù
//µÚÈý¸ö²ÎÊý±íÊ¾
void clear_edge_2(u8 state,int speed)       //ÖÐÈ¦É¨±ß
{
	
	if(((pos_x>-1000) && (pos_x<1000)) || ( (pos_y>1400 )  && (pos_y<3400) ))
		speed=6000;
	else
		speed=4500;
	
		if(state==0)   //ÄæÊ±Õë                           
	{
		if(pos_x>1000 && pos_y<3400)
		{
			line_move_control(2400-700,0,2400-700,100,speed);
		}
		if(pos_x>-1000 && pos_y>3400)
		{
			line_move_control(0,4800-700,-100,4800-700,speed);
		}
		if(pos_x<-1000 && pos_y>1400)
		{
			line_move_control(-2400+700,100,-2400+700,0,speed);
		}
		if(pos_x<1000  && pos_y<1400)
		{
			line_move_control(0,0+700,100,0+700,speed);
		}
	}
	else     //Ë³Ê±Õë                              
	{
		if(pos_x>1000 && pos_y>1000)
		{
			line_move_control(2400-700,100,2400-700,0,speed);
		}
		if(pos_x<1400 && pos_y>3800)
		{
			line_move_control(0,4800-700,100,4800-700,speed);
		}
		if(pos_x<-1400 && pos_y<3800)
		{
			line_move_control(-2400+700,0,-2400+700,100,speed);
		}
		if(pos_x>-1400 && pos_y<1000)
		{
			line_move_control(100,0+700,0,0+700,speed);
		}

	}
	
}


  
//×ßÐÎº¯Êý
void De_ja_vu(int speed,u8 state,int r)
{ 
	if(r<1800)	
	{
		My_circle(0,2400,r,speed,state);
		edge_g=0;
	}
	else
	{
		clear_edge(state,speed);
		edge_g=1;
	}
}


  /**
  ******************************************************************************
  * @file		choose_closerwall()	
  * @brief  ÅÐ¶ÏÐ¡³µÀëÄÇ¸öÇ½×î½ü
						±àºÅÎªÄæÊ±Õë·½Ïò ³ö·¢ÇøÎª1
  ******************************************************************************
  **/

u8 closest_wall(void)
{
	u8 wall=0;
	if(fabs(pos_x)>fabs(pos_y-2400+116))
	{
		if(pos_x>0)
			wall=2;
		else
			wall=4;
	}
	if(fabs(pos_x)<fabs(pos_y-2400+116))
	{
		if(pos_y-2400>0)
			wall=3;
		else 
			wall=1;
	}
	return wall;
}


//³éÇ©Çé¿ö·ÖÎö
u8 codelf(void)
{
		u8 situation=0;
		if((BOI_1+BOI_2)==4 || (BOI_1+BOI_2)==6)   //¶Ô½Ç¿ò
		{
			if(BOI_1==1 || BOI_2==1 )                //Ñ¡ÖÐ 1 3 ¿ò
				situation=5;	
			else                                     //Ñ¡ÖÐ2 4¿ò
				situation=6;	
		}
		else                                       //·Ç¶Ô½Ç¿ò
		{
			if((BOI_1+BOI_2)==3)                       //Ñ¡ÖÐ3£¬4£¬¿ò
				situation=3;	
			if((BOI_1+BOI_2)==7)                       //Ñ¡ÖÐ3£¬4£¬¿ò
				situation=1;	
			if((BOI_1+BOI_2)==5)
			{
				if( BOI_1==1 || BOI_2==1 )             //1 4
					situation=4;	
				else
					situation=2;
			}
		}
		
		return situation;
}


extern u8  edg_switch;             //×¼±¸·¢ÉäËùÓÃ¶¨Ê±Æ÷
extern u16 edg_time;
/*
u8 Ready_To_Endgame(u8 situation,u8 state)
{
	u8 wall=0,done=0;
	int except_angle=0;
	wall=closest_wall();            //¸ù¾Ý³µµÄÎ»ÖÃÅÐ¶¨×î½üµÄÇ½
	if(situation==1)                //³éµ½µÄÊÇÇé¿ö1
	{
		if(Blind_Zone_1(pos_x,pos_y))    //Èç¹ûÔÚ·¢ÉäÃ¤Çø
		{
			My_circle(0,2400,1500,3500,state);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(edg_time>400)
				{
					done=2;                        //2±íÊ¾ÔÚÃ¤ÇøµÄÊ±ºò¸ú±ðµÄ³µ×²ÉÏÁË£¬ÎÒ¸øµÄ°ë¾¶²»ÄÜ×²Ç½
					edg_switch=0;
					edg_time=0;
				}
			}
			else
			{
					edg_switch=0;
					edg_time=0;				   //¿ªÊ¼ÅÜÁË¾Í°Ñ¶¨Ê±Æ÷ºÍ¼ÆÊýÈ¥µô
			}
			
			//ÕâÀïÒ²Ó¦¸Ã¼ÓÌÓÒÝ
		}
		else                            //ÔÚ¿É·¢ÉäÇøÓò
		{
			switch(wall)
			{
				case 1:   except_angle=180 ;  break;
				case 2:   except_angle= 90 ;  break;
				case 3:   except_angle=  0 ;  break;
				case 4:   except_angle=-90 ;  break;
			}
			M_Angle_control(except_angle,3000);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(1               )                    //Èç¹û·ÇÁ½¸ö¸´Î»¿ª¹Ø¶¼´¥·¢
				{	
					Correction_switch=0;   //¹Øµô×¼±¸¸´Î»¶¨Ê±Æ÷
					Correction_time=0;
					if(edg_time>500)       //Í£ÏÂ300ºÁÃë
					{
						done=3;              //·µ»Ø3£¬£¬ÈÏÎª¸´Î»Ê§°Ü£¬È¥ÏÂÒ»¸öµØ·½¸´Î»
						edg_switch=0;
						edg_time=0;
						
						//¿ÉÄÜÒª¼ÇÂ¼ÒÔÏÂÕâÊÇÄÄÒ»¸öÇ½£¬È»ºó×ßÈ¦£¬µ±Ç½±äÁËµÄÊ±ºò£¬ÔÙÈ¥½ÃÕý
					}
				}
				else                    //Á½¸öÐÐ³Ì¿ª¹ØÍ¬Ê±°´ÏÂ
				{
					edg_switch=0;          //¹ØµôÁíÒ»¸ö¶¨Ê±Æ÷
					edg_time=0;
					Correction_switch=1;            //´ò¿ªÁíÒ»¸ö¶¨Ê±Æ÷£¬È»ºó£¬Èç¹ûÐÐ³Ì¿ª¹Ø´¥·¢´ïµ½Ò»¶¨Ê±¼ä£¬ÈÏÎª¸´Î»³É¹¦
					if(Correction_time>300)   //Á¬Ðø°´ÏÂ´ïµ½300msÖ®ºó¿ªÊ¼½øÈë½ÃÕý
					{
						done=1;
						Correction_switch=0;
						Correction_time=0;
					}
				}
			}
			else
			{
				edg_switch=0;           //Èç¹ûËÙ¶È´óÓÚ150¹Ø±Õ¶¨Ê±Æ÷
				edg_time=0;
			}
		}
	}
	
	
	
	if(situation==2)
	{
		if(Blind_Zone_2(pos_x,pos_y))    //Èç¹ûÔÚ·¢ÉäÃ¤Çø
		{
			My_circle(0,2400,1500,3500,state);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(edg_time>400)
				{
					done=2;                        //2±íÊ¾ÔÚÃ¤ÇøµÄÊ±ºò¸ú±ðµÄ³µ×²ÉÏÁË£¬ÎÒ¸øµÄ°ë¾¶²»ÄÜ×²Ç½
					edg_switch=0;
					edg_time=0;
				}
			}
			else
			{
					edg_switch=0;
					edg_time=0;				   //¿ªÊ¼ÅÜÁË¾Í°Ñ¶¨Ê±Æ÷ºÍ¼ÆÊýÈ¥µô
			}
			
			//ÕâÀïÒ²Ó¦¸Ã¼ÓÌÓÒÝ
		}
		else                            //ÔÚ¿É·¢ÉäÇøÓò
		{
			switch(wall)
			{
				case 1:   except_angle=180 ;  break;
				case 2:   except_angle= 90 ;  break;
				case 3:   except_angle=  0 ;  break;
				case 4:   except_angle=-90 ;  break;
			}
			M_Angle_control(except_angle,3000);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(1               )                    //Èç¹û·ÇÁ½¸ö¸´Î»¿ª¹Ø¶¼´¥·¢
				{	
					Correction_switch=0;   //¹Øµô×¼±¸¸´Î»¶¨Ê±Æ÷
					Correction_time=0;
					if(edg_time>500)       //Í£ÏÂ300ºÁÃë
					{
						done=3;              //·µ»Ø3£¬£¬ÈÏÎª¸´Î»Ê§°Ü£¬È¥ÏÂÒ»¸öµØ·½¸´Î»
						edg_switch=0;
						edg_time=0;
						
						//¿ÉÄÜÒª¼ÇÂ¼ÒÔÏÂÕâÊÇÄÄÒ»¸öÇ½£¬È»ºó×ßÈ¦£¬µ±Ç½±äÁËµÄÊ±ºò£¬ÔÙÈ¥½ÃÕý
					}
				}
				else                    //Á½¸öÐÐ³Ì¿ª¹ØÍ¬Ê±°´ÏÂ
				{
					edg_switch=0;          //¹ØµôÁíÒ»¸ö¶¨Ê±Æ÷
					edg_time=0;
					Correction_switch=1;            //´ò¿ªÁíÒ»¸ö¶¨Ê±Æ÷£¬È»ºó£¬Èç¹ûÐÐ³Ì¿ª¹Ø´¥·¢´ïµ½Ò»¶¨Ê±¼ä£¬ÈÏÎª¸´Î»³É¹¦
					if(Correction_time>300)   //Á¬Ðø°´ÏÂ´ïµ½300msÖ®ºó¿ªÊ¼½øÈë½ÃÕý
					{
						done=1;
						Correction_switch=0;
						Correction_time=0;
					}
				}
			}
			else
			{
				edg_switch=0;           //Èç¹ûËÙ¶È´óÓÚ150¹Ø±Õ¶¨Ê±Æ÷
				edg_time=0;
			}
		}
		
	}
	
	
	
	if(situation==3)
	{
		if(Blind_Zone_3(pos_x,pos_y))    //Èç¹ûÔÚ·¢ÉäÃ¤Çø
		{
			My_circle(0,2400,1500,3500,state);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(edg_time>400)
				{
					done=2;                        //2±íÊ¾ÔÚÃ¤ÇøµÄÊ±ºò¸ú±ðµÄ³µ×²ÉÏÁË£¬ÎÒ¸øµÄ°ë¾¶²»ÄÜ×²Ç½
					edg_switch=0;
					edg_time=0;
				}
			}
			else
			{
					edg_switch=0;
					edg_time=0;				   //¿ªÊ¼ÅÜÁË¾Í°Ñ¶¨Ê±Æ÷ºÍ¼ÆÊýÈ¥µô
			}
			
			//ÕâÀïÒ²Ó¦¸Ã¼ÓÌÓÒÝ
		}
		else                            //ÔÚ¿É·¢ÉäÇøÓò
		{
			switch(wall)
			{
				case 1:   except_angle=180 ;  break;
				case 2:   except_angle= 90 ;  break;
				case 3:   except_angle=  0 ;  break;
				case 4:   except_angle=-90 ;  break;
			}
			M_Angle_control(except_angle,3000);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(1               )                    //Èç¹û·ÇÁ½¸ö¸´Î»¿ª¹Ø¶¼´¥·¢
				{	
					Correction_switch=0;   //¹Øµô×¼±¸¸´Î»¶¨Ê±Æ÷
					Correction_time=0;
					if(edg_time>500)       //Í£ÏÂ300ºÁÃë
					{
						done=3;              //·µ»Ø3£¬£¬ÈÏÎª¸´Î»Ê§°Ü£¬È¥ÏÂÒ»¸öµØ·½¸´Î»
						edg_switch=0;
						edg_time=0;
						
						//¿ÉÄÜÒª¼ÇÂ¼ÒÔÏÂÕâÊÇÄÄÒ»¸öÇ½£¬È»ºó×ßÈ¦£¬µ±Ç½±äÁËµÄÊ±ºò£¬ÔÙÈ¥½ÃÕý
					}
				}
				else                    //Á½¸öÐÐ³Ì¿ª¹ØÍ¬Ê±°´ÏÂ
				{
					edg_switch=0;          //¹ØµôÁíÒ»¸ö¶¨Ê±Æ÷
					edg_time=0;
					Correction_switch=1;            //´ò¿ªÁíÒ»¸ö¶¨Ê±Æ÷£¬È»ºó£¬Èç¹ûÐÐ³Ì¿ª¹Ø´¥·¢´ïµ½Ò»¶¨Ê±¼ä£¬ÈÏÎª¸´Î»³É¹¦
					if(Correction_time>300)   //Á¬Ðø°´ÏÂ´ïµ½300msÖ®ºó¿ªÊ¼½øÈë½ÃÕý
					{
						done=1;
						Correction_switch=0;
						Correction_time=0;
					}
				}
			}
			else
			{
				edg_switch=0;           //Èç¹ûËÙ¶È´óÓÚ150¹Ø±Õ¶¨Ê±Æ÷
				edg_time=0;
			}
		}
		
	}
	
	
	
	if(situation==4)
	{
		if(Blind_Zone_4(pos_x,pos_y))    //Èç¹ûÔÚ·¢ÉäÃ¤Çø
		{
			My_circle(0,2400,1500,3500,state);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(edg_time>400)
				{
					done=2;                        //2±íÊ¾ÔÚÃ¤ÇøµÄÊ±ºò¸ú±ðµÄ³µ×²ÉÏÁË£¬ÎÒ¸øµÄ°ë¾¶²»ÄÜ×²Ç½
					edg_switch=0;
					edg_time=0;
				}
			}
			else
			{
					edg_switch=0;
					edg_time=0;				   //¿ªÊ¼ÅÜÁË¾Í°Ñ¶¨Ê±Æ÷ºÍ¼ÆÊýÈ¥µô
			}
			
			//ÕâÀïÒ²Ó¦¸Ã¼ÓÌÓÒÝ
		}
		else                            //ÔÚ¿É·¢ÉäÇøÓò
		{
			switch(wall)
			{
				case 1:   except_angle=180 ;  break;
				case 2:   except_angle= 90 ;  break;
				case 3:   except_angle=  0 ;  break;
				case 4:   except_angle=-90 ;  break;
			}
			M_Angle_control(except_angle,3000);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(1               )                    //Èç¹û·ÇÁ½¸ö¸´Î»¿ª¹Ø¶¼´¥·¢
				{	
					Correction_switch=0;   //¹Øµô×¼±¸¸´Î»¶¨Ê±Æ÷
					Correction_time=0;
					if(edg_time>500)       //Í£ÏÂ300ºÁÃë
					{
						done=3;              //·µ»Ø3£¬£¬ÈÏÎª¸´Î»Ê§°Ü£¬È¥ÏÂÒ»¸öµØ·½¸´Î»
						edg_switch=0;
						edg_time=0;
						
						//¿ÉÄÜÒª¼ÇÂ¼ÒÔÏÂÕâÊÇÄÄÒ»¸öÇ½£¬È»ºó×ßÈ¦£¬µ±Ç½±äÁËµÄÊ±ºò£¬ÔÙÈ¥½ÃÕý
					}
				}
				else                    //Á½¸öÐÐ³Ì¿ª¹ØÍ¬Ê±°´ÏÂ
				{
					edg_switch=0;          //¹ØµôÁíÒ»¸ö¶¨Ê±Æ÷
					edg_time=0;
					Correction_switch=1;            //´ò¿ªÁíÒ»¸ö¶¨Ê±Æ÷£¬È»ºó£¬Èç¹ûÐÐ³Ì¿ª¹Ø´¥·¢´ïµ½Ò»¶¨Ê±¼ä£¬ÈÏÎª¸´Î»³É¹¦
					if(Correction_time>300)   //Á¬Ðø°´ÏÂ´ïµ½300msÖ®ºó¿ªÊ¼½øÈë½ÃÕý
					{
						done=1;
						Correction_switch=0;
						Correction_time=0;
					}
				}
			}
			else
			{
				edg_switch=0;           //Èç¹ûËÙ¶È´óÓÚ150¹Ø±Õ¶¨Ê±Æ÷
				edg_time=0;
			}
		}
		
	}
	
	
	if(situation==5)
	{
		if(Blind_Zone_5(pos_x,pos_y))    //Èç¹ûÔÚ·¢ÉäÃ¤Çø
		{
			My_circle(0,2400,1500,3500,state);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(edg_time>400)
				{
					done=2;                        //2±íÊ¾ÔÚÃ¤ÇøµÄÊ±ºò¸ú±ðµÄ³µ×²ÉÏÁË£¬ÎÒ¸øµÄ°ë¾¶²»ÄÜ×²Ç½
					edg_switch=0;
					edg_time=0;
				}
			}
			else
			{
					edg_switch=0;
					edg_time=0;				   //¿ªÊ¼ÅÜÁË¾Í°Ñ¶¨Ê±Æ÷ºÍ¼ÆÊýÈ¥µô
			}
			
			//ÕâÀïÒ²Ó¦¸Ã¼ÓÌÓÒÝ
		}
		else                            //ÔÚ¿É·¢ÉäÇøÓò
		{
			switch(wall)
			{
				case 1:   except_angle=180 ;  break;
				case 2:   except_angle= 90 ;  break;
				case 3:   except_angle=  0 ;  break;
				case 4:   except_angle=-90 ;  break;
			}
			M_Angle_control(except_angle,3000);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(1               )                    //Èç¹û·ÇÁ½¸ö¸´Î»¿ª¹Ø¶¼´¥·¢
				{	
					Correction_switch=0;   //¹Øµô×¼±¸¸´Î»¶¨Ê±Æ÷
					Correction_time=0;
					if(edg_time>500)       //Í£ÏÂ300ºÁÃë
					{
						done=3;              //·µ»Ø3£¬£¬ÈÏÎª¸´Î»Ê§°Ü£¬È¥ÏÂÒ»¸öµØ·½¸´Î»
						edg_switch=0;
						edg_time=0;
						
						//¿ÉÄÜÒª¼ÇÂ¼ÒÔÏÂÕâÊÇÄÄÒ»¸öÇ½£¬È»ºó×ßÈ¦£¬µ±Ç½±äÁËµÄÊ±ºò£¬ÔÙÈ¥½ÃÕý
					}
				}
				else                    //Á½¸öÐÐ³Ì¿ª¹ØÍ¬Ê±°´ÏÂ
				{
					edg_switch=0;          //¹ØµôÁíÒ»¸ö¶¨Ê±Æ÷
					edg_time=0;
					Correction_switch=1;            //´ò¿ªÁíÒ»¸ö¶¨Ê±Æ÷£¬È»ºó£¬Èç¹ûÐÐ³Ì¿ª¹Ø´¥·¢´ïµ½Ò»¶¨Ê±¼ä£¬ÈÏÎª¸´Î»³É¹¦
					if(Correction_time>300)   //Á¬Ðø°´ÏÂ´ïµ½300msÖ®ºó¿ªÊ¼½øÈë½ÃÕý
					{
						done=1;
						Correction_switch=0;
						Correction_time=0;
					}
				}
			}
			else
			{
				edg_switch=0;           //Èç¹ûËÙ¶È´óÓÚ150¹Ø±Õ¶¨Ê±Æ÷
				edg_time=0;
			}
		}
		
	}

	if(situation==6)
	{
		if(Blind_Zone_6(pos_x,pos_y))    //Èç¹ûÔÚ·¢ÉäÃ¤Çø
		{
			My_circle(0,2400,1500,3500,state);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(edg_time>400)
				{
					done=2;                        //2±íÊ¾ÔÚÃ¤ÇøµÄÊ±ºò¸ú±ðµÄ³µ×²ÉÏÁË£¬ÎÒ¸øµÄ°ë¾¶²»ÄÜ×²Ç½
					edg_switch=0;
					edg_time=0;
				}
			}
			else
			{
					edg_switch=0;
					edg_time=0;				   //¿ªÊ¼ÅÜÁË¾Í°Ñ¶¨Ê±Æ÷ºÍ¼ÆÊýÈ¥µô
			}
			
			//ÕâÀïÒ²Ó¦¸Ã¼ÓÌÓÒÝ
		}
		else                            //ÔÚ¿É·¢ÉäÇøÓò
		{
			switch(wall)
			{
				case 1:   except_angle=180 ;  break;
				case 2:   except_angle= 90 ;  break;
				case 3:   except_angle=  0 ;  break;
				case 4:   except_angle=-90 ;  break;
			}
			M_Angle_control(except_angle,3000);
			
			if(chassic_speed<150)
			{
				edg_switch=1;
				if(1               )                    //Èç¹û·ÇÁ½¸ö¸´Î»¿ª¹Ø¶¼´¥·¢
				{	
					Correction_switch=0;   //¹Øµô×¼±¸¸´Î»¶¨Ê±Æ÷
					Correction_time=0;
					if(edg_time>500)       //Í£ÏÂ300ºÁÃë
					{
						done=3;              //·µ»Ø3£¬£¬ÈÏÎª¸´Î»Ê§°Ü£¬È¥ÏÂÒ»¸öµØ·½¸´Î»
						edg_switch=0;
						edg_time=0;
						
						//¿ÉÄÜÒª¼ÇÂ¼ÒÔÏÂÕâÊÇÄÄÒ»¸öÇ½£¬È»ºó×ßÈ¦£¬µ±Ç½±äÁËµÄÊ±ºò£¬ÔÙÈ¥½ÃÕý
					}
				}
				else                    //Á½¸öÐÐ³Ì¿ª¹ØÍ¬Ê±°´ÏÂ
				{
					edg_switch=0;          //¹ØµôÁíÒ»¸ö¶¨Ê±Æ÷
					edg_time=0;
					Correction_switch=1;            //´ò¿ªÁíÒ»¸ö¶¨Ê±Æ÷£¬È»ºó£¬Èç¹ûÐÐ³Ì¿ª¹Ø´¥·¢´ïµ½Ò»¶¨Ê±¼ä£¬ÈÏÎª¸´Î»³É¹¦
					if(Correction_time>300)   //Á¬Ðø°´ÏÂ´ïµ½300msÖ®ºó¿ªÊ¼½øÈë½ÃÕý
					{
						done=1;
						Correction_switch=0;
						Correction_time=0;
					}
				}
			}
			else
			{
				edg_switch=0;           //Èç¹ûËÙ¶È´óÓÚ150¹Ø±Õ¶¨Ê±Æ÷
				edg_time=0;
			}
		}
		
	}

	return done;
	
}




*/



/*
************************************ÆÆÇ§ÐÐ¾¯¸æ**********************************************************
*/
u8 In_fire=0;

void Ready_to_Endgame_2(u8 situation,u8 state,u8 tmit)
{
	u8 wall=0;
	int except_angle=0,speed=0;
	wall=closest_wall();
	In_fire=1;
	if(abs(pos_x)>1900 || abs(pos_y-2400)>1900)
		speed=1000;
	else
		speed=4500;
	if(tmit==1)
	{
			if(situation==1)                         //Ð´µÄºÜ¼òµ¥£¬ÕâÊÇµÚÒ»²½£¬ÅÐ¶ÏÊÇ·ñÔÚÃ¤Çø£¬ÔÚµÄ»°
			{																				 //ÏÈ×ßÈ¦£¬×ß³öÃ¤ÇøÔÙÈ¥¿¿Ç½
				if(Blind_Zone_3(pos_x,pos_y))          //Èç¹ûÖ±½Ó¿¿Ç½×ßÐÎ²»ºÃ¿´¿ÉÒÔÔÚ¼ÓÒ»¾ä£¬Èç¹ûÀëÔ²ÐÄÌ«Ô¶
																							 //¿ÉÒÔÏÈ×ßÔ²£¬Õâ¸öÔ²°ë¾¶¿ÉÒÔÐ¡Ò»µã£¬£¬
				{																			 //µÈ³öÀ´Ò»¶Î¾àÀëÔÙ¿¿Ç½
					My_circle(0,2400,1500,4500,state);
				}
				else
				{
						switch(wall)
					{
						case 1:   except_angle=180 ;  break;
						case 2:   except_angle= 90 ;  break;
						case 3:   except_angle=  0 ;  break;
						case 4:   except_angle=-90 ;  break;
					}
					M_Angle_control(except_angle,speed);
					
				}
			}
			if(situation==2)
			{
				if(Blind_Zone_4(pos_x,pos_y))
				{
					My_circle(0,2400,1500,4500,state);
				}
				else
				{
						switch(wall)
					{
						case 1:   except_angle=180 ;  break;
						case 2:   except_angle= 90 ;  break;
						case 3:   except_angle=  0 ;  break;
						case 4:   except_angle=-90 ;  break;
					}
					M_Angle_control(except_angle,speed);
					
				}
			}

				if(situation==3)
			{
			//	LCD_DisplayNum(100,135,Blind_Zone_3(pos_x,pos_y),5,24,0);
				if(Blind_Zone_1(pos_x,pos_y))
				{
					My_circle(0,2400,1500,4500,state);
				}
				else
				{
						switch(wall)
					{
						case 1:   except_angle=180 ;  break;
						case 2:   except_angle= 90 ;  break;
						case 3:   except_angle=  0 ;  break;
						case 4:   except_angle=-90 ;  break;
					}
					M_Angle_control(except_angle,speed);
					
				}
			}

				if(situation==4)
			{
				if(Blind_Zone_2(pos_x,pos_y))
				{
					My_circle(0,2400,1500,4500,state);
				}
				else
				{
						switch(wall)
					{
						case 1:   except_angle=180 ;  break;
						case 2:   except_angle= 90 ;  break;
						case 3:   except_angle=  0 ;  break;
						case 4:   except_angle=-90 ;  break;
					}
					M_Angle_control(except_angle,speed);
					
				}
			}

				if(situation==5)
			{
				if(Blind_Zone_5(pos_x,pos_y))
				{
					My_circle(0,2400,1500,4500,state);
				}
				else
				{
						switch(wall)
					{
						case 1:   except_angle=180 ;  break;
						case 2:   except_angle= 90 ;  break;
						case 3:   except_angle=  0 ;  break;
						case 4:   except_angle=-90 ;  break;
					}
					M_Angle_control(except_angle,speed);
					
				}
			}

				if(situation==6)
			{
				if(Blind_Zone_6(pos_x,pos_y))
				{
					My_circle(0,2400,1500,4500,state);
				}
				else
				{
						switch(wall)
					{
						case 1:   except_angle=180 ;  break;
						case 2:   except_angle= 90 ;  break;
						case 3:   except_angle=  0 ;  break;
						case 4:   except_angle=-90 ;  break;
					}
					M_Angle_control(except_angle,speed);
					
				}
			}

	}
	
	else         //À¶³¡
	{
				if(situation==1)                         
				{																				 
					if(Blind_Zone_1(pos_x,pos_y))         
																								 
					{																			 
						My_circle(0,2400,1500,speed,state);
					}
					else
					{
							switch(wall)
						{
							case 1:   except_angle=180 ;  break;
							case 2:   except_angle= 90 ;  break;
							case 3:   except_angle=  0 ;  break;
							case 4:   except_angle=-90 ;  break;
						}
						M_Angle_control(except_angle,speed);
						
					}
				}
				if(situation==2)
				{
					if(Blind_Zone_2(pos_x,pos_y))
					{
						My_circle(0,2400,1500,speed,state);
					}
					else
					{
							switch(wall)
						{
							case 1:   except_angle=180 ;  break;
							case 2:   except_angle= 90 ;  break;
							case 3:   except_angle=  0 ;  break;
							case 4:   except_angle=-90 ;  break;
						}
						M_Angle_control(except_angle,speed);
						
					}
				}

					if(situation==3)
				{
					if(Blind_Zone_1(pos_x,pos_y))
					{
						My_circle(0,2400,1500,speed,state);
					}
					else
					{
							switch(wall)
						{
							case 1:   except_angle=180 ;  break;
							case 2:   except_angle= 90 ;  break;
							case 3:   except_angle=  0 ;  break;
							case 4:   except_angle=-90 ;  break;
						}
						M_Angle_control(except_angle,speed);
						
					}
				}

					if(situation==4)
				{
					if(Blind_Zone_2(pos_x,pos_y))
					{
						My_circle(0,2400,1500,speed,state);
					}
					else
					{
							switch(wall)
						{
							case 1:   except_angle=180 ;  break;
							case 2:   except_angle= 90 ;  break;
							case 3:   except_angle=  0 ;  break;
							case 4:   except_angle=-90 ;  break;
						}
						M_Angle_control(except_angle,speed);
						
					}
				}

					if(situation==5)
				{
					if(Blind_Zone_5(pos_x,pos_y))
					{
						My_circle(0,2400,1500,speed,state);
					}
					else
					{
							switch(wall)
						{
							case 1:   except_angle=180 ;  break;
							case 2:   except_angle= 90 ;  break;
							case 3:   except_angle=  0 ;  break;
							case 4:   except_angle=-90 ;  break;
						}
						M_Angle_control(except_angle,speed);
						
					}
				}

					if(situation==6)
				{
					if(Blind_Zone_6(pos_x,pos_y))
					{
						My_circle(0,2400,1500,speed,state);
					}
					else
					{
							switch(wall)
						{
							case 1:   except_angle=180 ;  break;
							case 2:   except_angle= 90 ;  break;
							case 3:   except_angle=  0 ;  break;
							case 4:   except_angle=-90 ;  break;
						}
						M_Angle_control(except_angle,speed);
						
					}
				}
				
		
		
	}
	
}

//µÃµ½¾à¶¨Î»ÏµÍ³µÄ¾àÀë
 float get_distance(int x,int y)
{
	float distance=0.f;
	distance=sqrt((pos_x-x)*(pos_x-x)+(pos_y-y)*(pos_y-y));
	return distance;
}

extern int pwm_channel_1;
extern int pwm_channel_2;
u8 zhangyucheng=1;
void Ready_to_Endgame_3(u8 state)
{
			int ex_angle=0;
	
//		if(!center(pos_x,pos_y))
//		{
//			My_circle(0,2400,1750,5500,state);
//		}
		if(zhangyucheng==1)
		{
					pwm_channel_1=1400;
					pwm_channel_2=1400;
			
			if(corner(pos_x,pos_y))
			{
				My_circle(0,2400,1450,5500,state);
			}
			else
			{
				if(get_distance(0,2400)>1700)
					My_circle(0,2400,1750,5500,state);
				else
				{
					zhangyucheng=2;
				}

			}
		}
	else
	{
			if(corner(pos_x,pos_y))
				zhangyucheng=1;
			else
			{
					switch(closest_wall())
					{
						case 1:   ex_angle=180 ;  break;
						case 2:   ex_angle= 90 ;  break;
						case 3:   ex_angle=  0 ;  break;
						case 4:   ex_angle=-90 ;  break;
					}

			if((abs(pos_x)>1800) || (abs(pos_y-2400-76)>1800))
				M_Angle_control(ex_angle,2000);
			else
				M_Angle_control(ex_angle,4500);
			
			if(left_diaplasis_sw==1 || right_diaplasis_ww==1)   //Î´¿¿ÉÏÇ½
			{
					pwm_channel_1=1550;
					pwm_channel_2=1550;
			}
			else
			{
					pwm_channel_1=1500;
					pwm_channel_2=1500;
			}
		}
	}
	
}
float laser_left=0.f;
float laser_right=0.f;

void laser_line(float ex_angle,int speed,float dis)  //¼¤¹âÉ¨±ßµÄº¯Êý
{
	float angle_diff=0.f;
	angle_diff=ex_angle-zangle;
	if(angle_diff>180)  angle_diff-=360;
	if(angle_diff<-180) angle_diff+=360;
	
	if(abs(angle_diff<30))             //Èç¹û½Ç¶ÈÆ«²î½ÏÐ¡¾Í×ß½Ç¶È±Õ»·£¬½Ï´ó¾Í×ß½Ç¶È±Õ»·
		ex_angle=ex_angle-(dis-laser_right)/10;

	M_Angle_control(ex_angle,speed);
}



void Ready_to_Endgame_4(u8 state)
{
	
	int ex_angle=0;
	if(zhangyucheng==1)
	{
	if(center(pos_x,pos_y))
	{
		if(abs(pos_x)>1200 || abs(pos_y-2400+116)>1200)
			zhangyucheng=2;
		else
			My_circle(0,2400,1450,5500,state);

	}
	else
	{
		My_circle(0,2400,1450,5500,state);
	}
}
	else
	{
			switch(closest_wall())
		{
			case 1:   ex_angle=  0 ;  break;
			case 2:   ex_angle=-90 ;  break;
			case 3:   ex_angle=180 ;  break;
			case 4:   ex_angle= 90 ;  break;
		}
			laser_line(ex_angle,2000,2210);

	}
	
			
		

	
	
	
}
//u8 Redy_to_Endgame_3(u8 situation,u8 state)
//{
//		u8 wall=0;
//	int except_angle=0;
//	wall=closest_wall();
//	In_fire=1;

//	

//}


//extern u8  err3_switch;
//extern u16 err3_time;
//u8 err3_process(u8 state)    //´¦Àí´íÎó3    È¥ÏÂÒ»¸öÇ½·¢ÉäµÄ¹ý³ÌÖÐÒ²¿ÉÄÜ»á±»×°£¬Í£ÏÂ£¬ÐèÒªÌÓÒÝµÄ
//{
//	static u8 wall_last=0,flag=0,done=0;
//	u8 wall=0;
//	short int ex_angle=0;
//	if(flag==0)
//	{
//		wall_last=closest_wall();
//		flag=1;
//		done=0;
//		err3_switch=1;
//	}
//	wall=closest_wall();
//	switch(wall)
//	{
//			case 1:   ex_angle=180 ;  break;
//			case 2:   ex_angle= 90 ;  break;
//			case 3:   ex_angle=  0 ;  break;
//			case 4:   ex_angle=-90 ;  break;
//	}
//	if(wall!=wall_last)       //¹ý½çÏÞÁË
//	{
//		done=1;
//		err3_switch=0;
//		err3_time=0;
//	}
//	else
//	{
//		if(err3_time<800)
//			M_Back_control(ex_angle,3500);
//		else
//			My_circle(0,2400,1500,3500,state);    //ÌÓÒÝÔÚÍâ±ßÐ´
//		done=0;
//	}
//		
//		return done;
//}



/*u8 ready_to_endgame(u8 state)
{
	u8 wall=0;
	static u8 done=0;
	wall=closest_wall();
	if(The_Most_Important_Thing==1)             //ºì³¡
	{
		if((BOI_1+BOI_2)==4 || (BOI_1+BOI_2)==6)   //¶Ô½Ç¿ò
		{
			if(BOI_1==1 || BOI_2==1 )                //Ñ¡ÖÐ 1 3 ¿ò
			{
				
				switch(wall)
				{
					case 1:
						if(pos_x>-1400 && pos_x<1000)
							M_Angle_control(180,3000);
						else
							My_circle(0,2400,1500,5000,state);
						//¼ÓÉÏ¿¿Ç½Ö®ºóµÄ¼ì²âÐÐ³Ì¿ª¹Ø´¥·¢
						break;
					
					case 2:
						if(pos_y>-1000 && pos_y<4000)
							M_Angle_control(90,3000);
						else
							My_circle(0,2400,1500,5000,state);

						break;
					
					case 3:
						if(pos_x>-1000 && pos_x<1600)
							M_Angle_control(180,3000);
						else
							My_circle(0,2400,1500,5000,state);
						
						break;                           //´ý²¹³ä
					
					case 4:
						if(pos_y>800 && pos_y<3800)
							M_Angle_control(180,3000);
						else
							My_circle(0,2400,1500,5000,state);
						
						break;
				}
				
			}
			else                                     //Ñ¡ÖÐ2 4¿ò
			{
				
				switch(wall)
				{
					case 1:
						if(pos_x>-1000 && pos_x<1600)
							M_Angle_control(180,3000);
						else
							My_circle(0,2400,1500,5000,state);
						//¼ÓÉÏ¿¿Ç½Ö®ºóµÄ¼ì²âÐÐ³Ì¿ª¹Ø´¥·¢
						break;
					
					case 2:
						if(pos_y>800 && pos_y<3800)
							M_Angle_control(180,3000);
						else
							My_circle(0,2400,1500,5000,state);
						
						break;
					
					case 3:
						if(pos_x>-1400 && pos_x<1000)
							M_Angle_control(180,3000);
						else
							My_circle(0,2400,1500,5000,state);
						
						break;
					
					case 4:
						if(pos_y>-1000 && pos_y<4000)
							M_Angle_control(90,3000);
						else
							My_circle(0,2400,1500,5000,state);
						
						break;
				}

				
				
			}
				
		}
		else                                       //·Ç¶Ô½Ç¿ò
		{
			if((BOI_1+BOI_2)==3)                       //Ñ¡ÖÐ1£¬2¿ò   £¬12¿òÖÐ¼ä¼ÐµÃÊÇÇ½3
			{
				switch(wall)
				{
					case 1:
						My_circle(0,2400,1500,5000,state);
					break;
					
					case 2:
						My_circle(0,2400,1500,5000,state);
					break;
					
					case 3:
						if(1)
					break;
					
					case 4:
						My_circle(0,2400,1500,5000,!state);
						break;
				}
			}
			
			if((BOI_1+BOI_2)==7)                       //Ñ¡ÖÐ3£¬4£¬¿ò
			{
				
			}
			
			if((BOI_1+BOI_2)==5)
			{
				if( BOI_1==1 || BOI_2==1 )             //1 4
				{
					
				}
				else
				{
					
				}
			}
		}
	}
	else                              //À¶³¡
	{
		
	}
	
}
*/

#define position_prop        3276.8f    //ÔÆÌ¨µç»úÒª×ªµÄÎ»ÖÃÏµÊý
#define Initial_angle        21
#define r                    107
float Lingyuan_yousa_1(u8 id)
{
	float real_x=0.f ,real_y=0.f,final_angle=0.f,position=0.f,angle,angle_1=0.f;
	double current_angle=0.f;
	u8 wall=0;
	int init_angle=0;
	wall=closest_wall();
	
	switch(wall)
	{
		case 1:  angle=180 ;  init_angle=  0;  break;
		case 2:  angle=90  ;  init_angle= 90;  break;
		case 3:  angle=0   ;  init_angle=180;  break;
		case 4:  angle=-90 ;  init_angle=-90;  break;
	}
	angle_1=angle-zangle;
	if(angle_1> 180)  angle_1 -=360;
	if(angle_1<-180)  angle_1 +=360;
	
	current_angle=Initial_angle-zangle;
	if(current_angle>360)                              //current_angleÎªÔÆÌ¨Óë¶¨Î»ÏµÍ³µÄÊµÊ±Ïà¶Ô½Ç¶È
		current_angle-=360;

	real_x=pos_x+r*cos(current_angle*pi/180);
	real_y=pos_y+r*sin(current_angle*pi/180);        // ÔÆÌ¨µÄÕæÊµ×ø±ê

	final_angle=Angle_generator(real_x,real_y,target[id][0],target[id][1])+angle_1+init_angle;
	if (final_angle >= 180) final_angle -= 360;
	if (final_angle <=- 180) final_angle += 360;
	position=final_angle*position_prop;
	return position;
	
}

//float Lingyuan_yousa_2()
//{
//	float real_x=0.f ,real_y=0.f,current_angle=0.f,final_angle=0.f,position=0.f;
//	
//		current_angle=Initial_angle+zangle;
//		if(current_angle>360)                              //current_angleÎªÔÆÌ¨Óë¶¨Î»ÏµÍ³µÄÊµÊ±Ïà¶Ô½Ç¶È
//			current_angle-=360;
//	
//		real_x=pos_x+r*cos(current_angle*0.017453);
//		real_y=pos_y+r*sin(current_angle*0.017453);        // ÔÆÌ¨µÄÕæÊµ×ø±ê
//	
//		final_angle=Angle_generator(real_x,real_y,target[BOI_2][0],target[BOI_2][1]);
//		position=final_angle*position_prop;
//		return position;
//}


#define M_initial_angle      159
float Lingyuan_yousa_2(void)
{
	float real_x=0.f ,real_y=0.f,final_angle=0.f,position=0.f,angle,angle_1=0.f;
	double current_angle=0.f;
	u8 wall=0;
	int init_angle=0;
	wall=closest_wall();
	
	switch(wall)
	{
		case 1:  angle=180 ;  init_angle=  0;  break;
		case 2:  angle=90  ;  init_angle= 90;  break;
		case 3:  angle=0   ;  init_angle=180;  break;
		case 4:  angle=-90 ;  init_angle=-90;  break;
	}
	angle_1=angle-zangle;
	if(angle_1> 180)  angle_1 -=360;
	if(angle_1<-180)  angle_1 +=360;
	
	current_angle=M_initial_angle-zangle;
	if(current_angle>360)                                  //current_angleÎªÔÆÌ¨Óë¶¨Î»ÏµÍ³µÄÊµÊ±Ïà¶Ô½Ç¶È
		current_angle-=360;

	real_x=pos_x+r*cos(current_angle*pi/180);
	real_y=pos_y+r*sin(current_angle*pi/180);        // ÔÆÌ¨µÄÕæÊµ×ø±ê

	final_angle=Angle_generator(real_x,real_y,0,2400)+init_angle+angle_1;
	if (final_angle >= 180) final_angle  -= 360;
	if (final_angle <=- 180) final_angle += 360;
	position=final_angle*position_prop;
	return position;

}



u8 angle_range()
{
	u8 wall=0,True=0;
	switch(wall)
	{
		case 1: 
			if( ( (zangle>150) && (zangle<180) ) ||  ( (zangle>-180) && (zangle<-150) )  )
				True=0;
			else
				True=1;
			break;
			
		case 2:
			if(abs(zangle-90)<30)
				True=0;
			else
				True=1;
					break;
			
		case 3:
			if(abs(zangle<30))
				True=0;
			else
				True=1;
			break;
			
		case 4:
			if((zangle+90)<30)
				True=0;
		else
				True=1;
			break;
		
	}
	return True;
}

void stop()
{
	rm3508_pid[0].target=0;
	rm3508_pid[1].target=0;
	rm3508_pid[2].target=0;
}
extern DBUS dbus;

extern u8 pink_in_repository;       //¶þ¼¶´¢²ØÊÒÖÐµÄ·ÛÇòÊýÄ¿
extern u8 good_in_repository;			 //¶þ¼¶´¢²ØÊÒÖÐºÚ°×ÇòÊýÄ¿
extern int speed_TIM4;
extern int speed_TIM8;
extern u8 hanser_1;
extern u8 hanser_2;

static float get_dis(u8 id)
{
	float dis=0.f;
	if(id!=0)
		dis=sqrt((pos_y-target[id][1])*(pos_y-target[id][1])+(pos_x-target[id][0])*(pos_x-target[id][0]));
	else
		dis=sqrt((pos_y-2400)*(pos_y-2400)+pos_x*pos_x);
	return dis;
}

/*		
		ÓÐ¸öÏë·¨£¬×ó±ßÔÆÌ¨³¯Ç°¸´Î»£¬ÓÒÔÆÌ¨³¯ºó¸´Î»
*/
u8 sunshine=0;     
extern u8  fire_switch_1;         
extern u16 fire_time_1;
extern u8  fire_switch_2;          
extern u16 fire_time_2;
extern u32 running_time;
float real_dis_1=0.f;
float real_dis_2=0.f;
extern u16 deadline_1;
extern u16 deadline_2;

u8 I_Am_Champion(u8 situation,u8 state)
{
	//                  ·¢ÉäË³Ðò          ·¢Éä¼ÆÊý£¬ÒòÎªÒªÁ½¸ö¿òÂÖÁ÷·¢Éä
		static u8 step=1,counting=2,shoot_1=0,shoot_2=0;
		u8 finish=0,id=0;
		float dis_1=0.f,dis_2=0.f;      
		switch(step)
		{
			case 1:            
				stop();
				coordinate_correction();          //¶¨Î»ÏµÍ³×ø±ê½ÃÕý
				pwm_channel_1=1500;
				pwm_channel_2=1500;
				step=3;
			break;
									
					case 3:
						
						if( pink_in_repository>0 || good_in_repository>0 )                        //ÓÐÇò
						{
								if(corner2(pos_x,pos_y))   					 //Èç¹û±»×²½øÃ¤Çø»òÕß½Ç¶È±»×²Æ«Ä³¸ö½Ç¶ÈÒÔÉÏ
										finish=2;																							//·µ»Ø´íÎó£¬ÖØÐÂ½øÈë½ÃÕý£¬¶ÔµÄ£¬È»ºóÔÚÃ¤Çø¾Í»ØÈ¥ÅÜ£¬²»ÔÚ¾Í»á¼ÌÐøÈ¥¿¿Ç½¸´Î»
								else
								{
									stop();
									In_fire=1;
										switch(situation)
										{
											case 1:           //  Ö»ÓÐ 3 4¿ò
														dis_1=get_dis(3);
														dis_2=get_dis(4);
														id=	counting%2==0?  dis_1<dis_2?  3:4:  dis_1>dis_2?3:4;
											break;
												
											case 2:
														dis_1=get_dis(2);
														dis_2=get_dis(3);
														id=	counting%2==0?  dis_1<dis_2?  2:3:  dis_1>dis_2?2:3;
											break;
											
											case 3:
														dis_1=get_dis(1);
														dis_2=get_dis(2);
														id=	counting%2==0?  dis_1<dis_2?  1:2:  dis_1>dis_2?1:2;
											break;
											
											case 4:
														dis_1=get_dis(1);
														dis_2=get_dis(4);
														id=	counting%2==0?  dis_1<dis_2?  1:4:  dis_1>dis_2?1:4;
											break;
											
											case 5:
														dis_1=get_dis(1);
														dis_2=get_dis(3);
														id=	counting%2==0?  dis_1<dis_2?  1:3:  dis_1>dis_2?1:3;
											break;
											
											case 6:
														dis_1=get_dis(2);
														dis_2=get_dis(4);
														id=	counting%2==0?  dis_1<dis_2?  2:4:  dis_1>dis_2?2:4;
											break;
										}
											
											motor_pid[1].target=Lengniao_yousa(id);
											motor_pid[0].target=Lengniao_yousa_2();
		
											hanser_1=pink_in_repository;
											hanser_2=good_in_repository;
										
											if(hanser_2>0) motor_u3[0].target=All_Function(0,real_dis_2);
											if(hanser_1>0) motor_u3[1].target=All_Function(id,real_dis_1);
											

//  ÅÐ¶Ï½ø¹©µ¯µÄ²¿·Ö					
									
									if(hanser_2>0)
									{
										if(shoot_1==0)
										{
												fire_switch_1=1;
												if(abs(motor_u3[0].target-speed_TIM4)>30)
														fire_time_1=0;
												if(fire_time_1>1000  || deadline_1>6000)
													shoot_1=1;

										}
									}
									if(hanser_1>0)
									{
										if(shoot_2==0)
										{		
												fire_switch_2=1;
												if(abs(speed_TIM8-motor_u3[1].target)>30)
														fire_time_2=0;											

												if(fire_time_2>1000 || deadline_2>6000)
													shoot_2=1;

										}
									}
										
										
										if(shoot_1==1)
										{
											motor_pid[2].target=-5000;
											if(fire_time_1>1500)           //·¢ÉäºóÍ£Áô400ms,»Øµ½³õÊ¼Î»ÖÃ·ÅÇò½ø¹©µ¯
											{
												motor_pid[2].target=-280000;
											}
											if(fire_time_1>1700)
											{												
												good_in_repository-=1;
												fire_switch_1=0;
												fire_time_1=0;
												deadline_1=0;
												shoot_1=0;

											}
										}
										
										
										if(shoot_2==1)
										{
												motor_pid[3].target=5000;
											if(fire_time_2>1500)
											{
												motor_pid[3].target=300000;
											}
											if(fire_time_2>1700)
											{
												pink_in_repository-=1;
												deadline_2=0;
												fire_switch_2=0;
												fire_time_2=0;
												shoot_2=0;
												counting++;

											}
										}

								}
							
						}
						else
						{
							In_fire=0;
							fire_switch_2=0;
							fire_switch_1=0;
							fire_time_1=0;
							fire_time_2=0;
							deadline_1=0;
							deadline_2=0;
							shoot_1=0;
							shoot_2=0;
							counting=2;
							step=4;  //·¢Éä³É¹¦

						}
					
						break;
						
						
						//µÚËÄ²½
							case 4:
								sunshine=0;
								motor_pid[0].target=0;
								motor_pid[1].target=0;
								pwm_channel_1=1400;
								pwm_channel_2=1400;
								zhangyucheng=1;
								fire_switch_1=1;
								if(fire_time_1<600)
									M_Back_control(zangle,4000);
								else
								{
									rso_sunshine+=1;
									finish=1;
									step=1;
									fire_switch_1=0;
									fire_time_1=0;
								}
																									
							
								break;
								
			
		}

	return finish;	
}




//ÇóÈý¸öÊý¼«²î
u8 absdiff(u8 a,u8 b,u8 c)
{
	u8 max=0,min=0;
	max=a>b? a>c? a:c :b>c? b:c; 
	min=a<b? a<c? a:c :b<c? b:c; 
	return max-min;
}

//***********************************ÆÆ2000ÐÐ¾¯¸æ****************************************************************


extern u8  detect_switch;          //¼ì²âÑÕÉ«È¥¶¶¶¨Ê±Æ÷
extern u16 detect_time;
extern u8  black_switch;          //¼ì²âÑÕÉ«È¥¶¶¶¨Ê±Æ÷
extern u16 black_time;
extern u8  white_switch;          //¼ì²âÑÕÉ«È¥¶¶¶¨Ê±Æ÷
extern u16 white_time;
extern u8  desert_switch;
extern u16 desert_time;
#define none  0
#define pink  1 
#define black 2
#define white 3
#define unknown 4
u8 color_detect(u8 R,u8 G,u8 B)
{
	u8 diff=0, aver=0,color=0,final_color=0;
	aver=R/3+G/3+B/3;
	diff=absdiff(R,G,B);
	if(diff<17)
	{
		if(aver>60)
		{
			if(R>60 && G>60  && B>60)
				color=white;
			else
				color=unknown;
		}
		else
		{
			if(R<40 && G<40 && B<40)
				color=black;
			else
				color=unknown;
		}
	}
	else
	{
		if(B>150)
			color=none;
		else
		{
			if(R>=60 && G<60 && B<60)
				color=pink;
			else
				color=unknown;
		}
	}
	
	
//	if(aver<40)
//	{
//		color=black;
//	}
//	else
//	{
//		if(B>150)
//			color=none;
//		else
//		{
//			if(R>=60 && G<60 && B<60)
//			color=pink;
//			if(R>60 && G>60  && B>60)
//			color=white;

//		}
//	}
	
	//È¥¶¶²¿·Ö
	if(color==black)
	{
		black_switch=1;
		if(black_time>350)
		{
			final_color=black;
		}
		else
			final_color=none;
	}
	else
	{
		black_switch=0;
		black_time=0;
	}
	
		if(color==white)
	{
		white_switch=1;
		if(white_time>400)
		{
			final_color=white;
		}
		else
			final_color=none;
	}
	else
	{
		white_switch=0;
		white_time=0;
	}

	
		if(color==pink)
	{
		detect_switch=1;
		if(detect_time>400)
		{
			final_color=pink;
		}
		else
			final_color=none;
	}
	else
	{
		detect_switch=0;
		detect_time=0;
	}


	if(color==unknown)
	{
		desert_switch=1;
		if(desert_time>1500)
		{
			final_color=unknown;
		}
		else
		{
			final_color=none;
		}
	}	
	else
	{
				desert_switch=0;
				desert_time=0;

	}
	
	
	return final_color;
	
}




extern u8  squeeze_switch;
extern u16 squeeze_time;
extern struct _color color;
#define direction  80000
void squeeze_ball()
{
	static u8 step=0,allow=1,desert_count=0;
	static u8 ball=0;
	static u32 pos_last=0;
	if(The_Most_Important_Thing==1)                            //ºì³¡,ºì³¡µÃ·ÖÇòÎª°×É«
	{
		switch(step)
		{
			case 0:
				ball=color_detect(color.R,color.G,color.B);
			if(ball)   //Èç¹ûÓÐÇò
			{
				step=1;
			}
			break;
			case 1:
					squeeze_switch=1;
				if(squeeze_time<350)
				{
					if(ball==pink)
						motor_pid[4].target=-direction;
					if(ball==black)
						motor_pid[4].target=0;
					if(ball==white)
						motor_pid[4].target=direction;
					if(ball==unknown)
						motor_pid[4].target=0;
					
					//ÅÐ¶Ï±ä»»´æÇò·½ÏòµÄÌõ¼þ
										if(desert_count<=12)  //ÈÏÎª×î¶à¿ÉÒÔ·Å10¸öÇò
										{
											if(pink_in_repository>2 )  //·ÛÇòÂú
												if(ball==pink)
													motor_pid[4].target=0;           //ÍùµØÏÂ·Ö
											if(good_in_repository>2)
												if(ball==white)
													motor_pid[4].target=0;
										}
										else          //µØÏÂ´¢²ØÊÒÂú
										{
											if(ball==black)
											if(pink_in_repository<3)
													motor_pid[4].target=-direction;                //°ÑºÚÇò·Öµ½·ÛÉ«
										}
										
				}
				else
				{
					
							//ÅÐ¶ÏÃ»ÊÕ¼·ÇòÈ¨ÏÞµÄÌõ¼þ
							if(desert_count<=12  && pink_in_repository>2 && good_in_repository>2)					
								if(ball!=black)
									allow=0;
							if(desert_count>=12 )
							{
								if(good_in_repository>2 || pink_in_repository>2)              
								{
										if(good_in_repository>2 && ball==white)
											allow=0;
										if(pink_in_repository>2 && (ball==pink||ball==black))
											allow=0;
								}
									
									
							}
							step=2;
						
				}
				break;
				
			case 2:               
						
					if(allow==1)
					{
						if(motor_pid[4].target==-direction)
							pink_in_repository++;             //²Ö¿âÖÐ·ÛÇòÊýÔö¼Ó     
						if(motor_pid[4].target==direction)                     //²Ö¿âÖÐ°×ÇòÊýÔö¼Ó
							good_in_repository++;
						if(motor_pid[4].target==0)
							desert_count++;
					}
					step=3;
			break;
			case 3:
			{
				if(squeeze_time<850)//allow=1±íÊ¾ÔÊÐí±ä»»£¬·ñÔò²»ÔÊÐí±ä»»Î»ÖÃ
				{
					if(allow==1)
					{
						if(pos_last==0)   
							motor_pid[5].target=295994;
						if(pos_last==295994)
							motor_pid[5].target=0;
					}
				}
				else
				{
					if(allow==1)   							 //Èç¹û¼·ÇòÁË  ,¸üÐÂ¼ÇÂ¼
						pos_last=motor_pid[5].target;
					
						squeeze_switch=0;
						squeeze_time=0;
						step=0;
						allow=1;               //¸üÐÂÏÂÒ»´Î¼·ÇòÈ¨ÏÞ
				}			
			}
			break;
		}
	}
	

	
	else                                           //À¶³¡µÃ·ÖÇòÎªºÚÉ«
	{
		switch(step)
		{
			case 0:
				ball=color_detect(color.R,color.G,color.B);
			if(ball)   //Èç¹ûÓÐÇò
			{
				step=1;
			}

			break;
			case 1:
					squeeze_switch=1;
				if(squeeze_time<350)
				{
					if(ball==pink)
						motor_pid[4].target=-direction;
					if(ball==white)
						motor_pid[4].target=0;
					if(ball==black)
						motor_pid[4].target=direction;
					if(ball==unknown)
						motor_pid[4].target=0;
											if(desert_count<=12)  //ÈÏÎª×î¶à¿ÉÒÔ·Å10¸öÇò
										{
											if(pink_in_repository>2 )  //·ÛÇòÂú
												if(ball==pink)
													motor_pid[4].target=0;           //ÍùµØÏÂ·Ö
											if(good_in_repository>2)
												if(ball==black)
													motor_pid[4].target=0;
										}
										else          //µØÏÂ´¢²ØÊÒÂú
										{
											if(pink_in_repository<3)
												if(ball==white)
													motor_pid[4].target=-direction;                //°ÑºÚÇò·Öµ½·ÛÉ«
										}

					
					
				}
				else
				{
							//ÅÐ¶ÏÃ»ÊÕ¼·ÇòÈ¨ÏÞµÄÌõ¼þ
							if(desert_count<=12  && pink_in_repository>2 && good_in_repository>2)					
							{
								if(ball!=white)
									allow=0;
							}
							if(desert_count>=12 )
							{
								if(good_in_repository>2 || pink_in_repository>2)              
								{
										if(good_in_repository>2 && ball==black)
											allow=0;
										if(pink_in_repository>2 && ball!=black)
											allow=0;
								}
									
							}
							
							step=2;

				}
				break;
				
			case 2:
				if(allow)
				{
					if(motor_pid[4].target==-direction)						//²Ö¿âÖÐ·ÛÇòÊýÔö¼Ó     
						pink_in_repository++;             
					if(motor_pid[4].target==direction)               //²Ö¿âÖÐ°×ÇòÊýÔö¼Ó
						good_in_repository++;
					if(motor_pid[4].target==0)                 //ÆúÇòÊý×ÔÔö
						desert_count++;
					

				}
			step=3;
				break;
			case 3:
			{
						if(squeeze_time<850)//allow=1±íÊ¾ÔÊÐí±ä»»£¬·ñÔò²»ÔÊÐí±ä»»Î»ÖÃ
				{
					if(allow==1)
					{
						if(pos_last==0  )   
							motor_pid[5].target=295994;
						if(pos_last==295994)
							motor_pid[5].target=0;
					}
				}
				else
				{
					if(allow==1)   							 //Èç¹û¼·ÇòÁË  ,¸üÐÂ¼ÇÂ¼
						pos_last=motor_pid[5].target;
					
						squeeze_switch=0;
						squeeze_time=0;
						step=0;
						allow=1;               //¸üÐÂÏÂÒ»´Î¼·ÇòÈ¨ÏÞ
				}			
			}
			break;
		}

	}

}




//ÕâÊÇÒ»¸ö×ßÐÎº¯Êý£¬ÎÒÏëÈÃÕâ¸öº¯ÊýÒ»ÐÐ¾Í¿ÉÒÔÅÜ3·ÖÖÓ
//´þÏº»§
//Õâ¸öÊÇÖ®Ç°µÃµÚÒ»°æ×ßÐÎ£¬×ßµÄºÜ¼òµ¥£¬Ë³Ê±Õë×ßÎåÈ¦£¬ÄæÊ±Õë×ß5È¦£¬´øÉ¨±ßµÃ,È»¶øºÃÏñÓÐÐ©ÎÊÌâ
void De_ja_vu_2(u8 state,int speed)
{
	static u8 step=1,num=0;
	int R;
		if(step==1)
	{
		R=1000+(get_circle_num()-num)*200;
		De_ja_vu(4500,state,R);   //0±íÊ¾ÄæÊ±Õë
	//	if(get_circle_num()>=6)
		if(get_circle_num()%5==0 && get_circle_num()>1 && get_circle_num()%10!=0 )
		{
			state=1-state;
			step=2;
			num=get_circle_num();
		}
	}
	if(step==2)
	{
		R=900+(get_circle_num()-num)*200;
		De_ja_vu(4500,state,R);   //0±íÊ¾ÄæÊ±Õë                                          

		if(get_circle_num()%10==0 && get_circle_num()>1)
		{
			state=1-state;
			step=1;
			num=get_circle_num();
		}
	}


}


//Ë¼¿¼ÏÂÒ»¸ö×ßÐÎ
//void De_ja_vu_3(u8 state,int speed)
//{
//	static u8 step=1,num=0;
//	int R;
//	if(step==1)
//	{
//		num=get_circle_num();
//		if(num==0)
//		{
//			My_ellipse(0,2400,850,500,3000,state);    //1±íÊ¾Ë³Ê±Õë  0±íÊ¾ÄæÊ±Õë
//		}
//		if(num==1)
//		{
//			My_ellipse(0,2400,200,850,3000,state);
//		}
//		if(num>=2)
//		{
//			De_ja_vu_2(state,speed);
//			
//		}
//	}
//	
//}




//¿ÉÒÔ¿¼ÂÇ¼ÓÒ»¸öÈë¿Ú²ÎÊý£¬±íÊ¾µÚÒ»È¦×ßÄÚÈ¦»¹ÊÇÖÐÈ¦
void de_ja_vu_4(int speed,u8 state)
{
	u8 num=0;
	unsigned int R=0;
	static u8 running_step=1,last_num=0;
	if(running_step==1)   					 //µÚÒ»²½±íÊ¾µÚÒ»È¦µÄ×ßÐÐ£¬
	{
		num=get_circle_num();
		R=800+400*num;
		My_circle(0,2400,R,speed,state);
		
		if(num-last_num>0 && last_num!=0)             //É¨±ßÒ»È¦ºó½øÈëµÚ¶þ²½
		{
			running_step=2;
			last_num=num;
		}

		if(R>1850) 
		{
			last_num=num;
			clear_edge(state,speed);
			edge_g=1;
		}
		else
		{
			edge_g=0;
		}
	}
	
	if(running_step==2)
  {
		num=get_circle_num()-last_num;    //´Ó0¿ªÊ¼¼ÆÈ¦
		R=1000+num*450;
		My_circle(0,2400,R,speed,state);
		if(R>1850)
		{
			clear_edge(state,speed);
			if(R>2000)
			{
				last_num=get_circle_num();
				running_step=3;
				edge_g=1;
			}
			else
			{
				edge_g=0;
			}
		}
		
	}
	
	if(running_step==3)    //Õâ¸öº¯ÊýÓÐÁ½¸öÉ¨±ß£¬£¬ÆäÊµÒ²ÓÐ¿ÉÄÜ°ÑÕâ¸öµ±³ÉµÚ¶þ²½
	{
		num=get_circle_num()-last_num;
		R=1100+num*350;
		My_circle(0,2400,R,speed,state);
		if(R>1500)
		{
			if(R>2000)
			{
				clear_edge(state,speed);
				if(R>2100)
				{
				last_num=get_circle_num();
				running_step=2;					
				}
				
			}
			else
			edge_g=1;
			clear_edge_2(state,speed);
		}
			else
		{
			edge_g=0;
		}		
	}
	
//	LCD_DisplayNum(100,100,edge_g,5,24,0);
}



//Õâ¸öº¯ÊýÓÃÀ´½øÐÐ½ÃÕýµÄÊ±ºò³öÏÖ±»¸ÉÈÅµÄÇé¿öÒªÍùÏÂÒ»¸öÇ½ÅÜµÄÇé¿ö

u8 To_next_wall(u8 state)
{
	static u8 wall_start=0,step=1;
	u8 wall=0,finish=0;
	wall=closest_wall();
	switch(step)
	{
		case 1:
			wall_start=wall;
			step=2;
		break;
		case 2:
			My_circle(0,2400,1500,4000,state);
		if(wall!=wall_start)
			step=3;
		break;
		case 3:
			wall_start=0;
			finish=1;
			step=1;
			break;
		
	}
	return finish;       //Íê³ÉÖ®ºó½øÐÐ·µ»Ø1
	
}



//Initial_angle

//±ß×ß±ßÉäµÄÖ±Ïß×ßÐÎ     ÏÖÔÚÓ¦¸ÃÒ²·ÏÆúÁË
void Tilted_rect(int speed ,u8 state)    
{
	float diff_x=0.f,diff_y=0.f;
		
		diff_x=r*sin((Initial_angle-zangle)*pi/180);
		diff_y=r*cos((Initial_angle-zangle)*pi/180);
	//	id=sure_id(state);
	
	if (state==0)          //ÄæÊ±Õë
	{
		if(pos_x>900 && pos_y<3300)
		{
			line_move_control(900,700,2200+diff_x,4600+diff_y,speed);
		}
			if(pos_x>-900 && pos_y>3300)
		{
			line_move_control(1700,3300,-2200+diff_x,4600+diff_y,speed);
		}
			if(pos_x<-900 && pos_y>1500)
		{
			line_move_control(-900,4100,-2200+diff_x,200+diff_y,speed);
		}
		if(pos_x<900 && pos_y<1500)
		{
			line_move_control(-1700,1500,2200+diff_x,200+diff_y,speed);
		}
	}
	else               //Ë³Ê±Õë
	{
			if(pos_x>-900 && pos_y<1500)
		{
			line_move_control(1700,1500,-2200+diff_x,200+diff_y,speed);
		}
			if(pos_x<-900 && pos_y<3300)
		{
			line_move_control(-900,700,-2200+diff_x,4600+diff_y,speed);
		}
			if(pos_x<900 && pos_y>3300)
		{
			line_move_control(-1700,3300,2200+diff_x,4600+diff_y,speed);
		}
		if(pos_x>900 && pos_y>1500)
		{
			line_move_control(900,4100,2200+diff_x,200+diff_y,speed);
		}
	}
	
	if(pos_x<900 && pos_x>-900 && pos_y>1500 && pos_y<3300)
		My_circle(0,2400,1100,4500,state);
	//LCD_DisplayNum(100,100,id,8,24,0);
}


void Tilted_rect_2(int speed,u8 state)
{
	
		float diff_x=0.f,diff_y=0.f;
		
		diff_x=r*sin((Initial_angle-zangle)*pi/180);
		diff_y=r*cos((Initial_angle-zangle)*pi/180);
	
	if (state==0)          //ÄæÊ±Õë
	{
		if(pos_x>700 && pos_y<3100)
		{
			line_move_control(700,1100,2200+diff_x,4600+diff_y,speed);
		}
			if(pos_x>-700 && pos_y>3100)
		{
			line_move_control(1300,3100,-2200+diff_x,4600+diff_y,speed);
		}
			if(pos_x<-700 && pos_y>1700)
		{
			line_move_control(-700,3700,-2200+diff_x,200+diff_y,speed);
		}
		if(pos_x<700 && pos_y<1700)
		{
			line_move_control(-1300,1300,2200+diff_x,200+diff_y,speed);
		}
	}
	else               //Ë³Ê±Õë
	{
			if(pos_x>-700 && pos_y<1700)
		{
			line_move_control(1300,1700,-2200+diff_x,200+diff_y,speed);
		}
			if(pos_x<-700 && pos_y<3100)
		{
			line_move_control(-700,1100,-2200+diff_x,4600+diff_y,speed);
		}
			if(pos_x<700 && pos_y>3100)
		{
			line_move_control(-1300,3100,2200+diff_x,4600+diff_y,speed);
		}
		if(pos_x>700 && pos_y>1700)
		{
			line_move_control(700,3700,2200+diff_x,200+diff_y,speed);
		}
	}
	
	if(pos_x<700 && pos_x>-700 && pos_y>1700 && pos_y<3100)
		My_circle(0,2400,1100,4500,state);

	
}
	

#define angle_declination_1 -19
#define angle_declination_0 19
#define compensate 440
//ÐÂÐ´µÄ¸´ºÏ¶à±ßÐÎ
void Compound_polygon(u8 state,int speed)
{
	float new_x=0,new_y=0;
		float diff_x=0.f,diff_y=0.f;
		diff_x=r*sin((Initial_angle-zangle)*pi/180);
		diff_y=r*cos((Initial_angle-zangle)*pi/180);

	if(state==0)            //0±íÊ¾ÄæÊ±Õë Ë³ÄæÊ±ÕëµÄ×ø±êÏµÐý×ª½Ç¶È²»Ò»ÑùµÄ
	{
		new_x=pos_x*cos(angle_declination_0*pi/180)+(pos_y-2400)*sin(angle_declination_0*pi/180);
		new_y=(pos_y-2400)*cos(angle_declination_0*pi/180)-pos_x*sin(angle_declination_0*pi/180);
		
		if(new_x>0 && new_y>0)
		{
			line_move_control(2400,2400,-2200+diff_x,4600+diff_y,speed);
		}
		if(new_x>0 && new_y<0)
		{
				line_move_control(0,0,2200+diff_x,4600+diff_y,speed);	
		}
		if(new_x<0  && new_y<0)
		{
				line_move_control(-2400,2400,2200+diff_x,200+diff_y,speed);
		}
		if(new_x<0 && new_y>0)
		{
				line_move_control(0,4800,-2200+diff_x,200+diff_y,speed);
		}

		
	}
	else
	{
			new_x=pos_x*cos(angle_declination_1*pi/180)+(pos_y-2400)*sin(angle_declination_1*pi/180);
			new_y=(pos_y-2400)*cos(angle_declination_1*pi/180)-pos_x*sin(angle_declination_1*pi/180);

		if(new_x>0 && new_y>0)
		{
			line_move_control(2400,2400,-2200+diff_y,4600+diff_y,speed);
		}
		if(new_x>0 && new_y<0)
		{
				line_move_control(0,0,2200+diff_y,4600+diff_y,speed);	
		}
		if(new_x<0  && new_y<0)
		{
				line_move_control(-2400,2400,2200+diff_y,200+diff_y,speed);
		}
		if(new_x<0 && new_y>0)
		{
				line_move_control(0,4800,-2200+diff_y,200+diff_y,speed);
		}

	}
}


//#define angle_declination2_1 -14
//#define angle_declination2_0  14
//¸´ºÏ¶à±ßÐÎ2ºÅ£¬£¬Ð¡È¦
u8 in_advance=0;
#define desire_speed_1   3750
#define desire_speed_2   4500
#define desire_speed_3   4200
#define desire_speed_4   4500
u8 Compound_polygon_2(u8 state,int speed)
{
	static u8 fire=0,allow_1=1,allow_2=1;
	int desire_speed;
	u8 done=0, current_id=0;
	static float dis_last=0,dis_2=0;
	static u8 n1=0,n2=0,n3=0,n4=0;
	if(!state)            //0±íÊ¾ÄæÊ±Õë  Ë³ÄæÊ±ÕëµÄ×ø±êÏµÐý×ª½Ç¶È²»Ò»ÑùµÄ
	{
	
/*************************×ßÐÐ²¿·Ö*****************************************/
			if(pos_x>487 && pos_y<2875)
			{
				if(pos_y<2475)
				{
					line_move_control(0,0,2200-200,4600,speed);
					current_id=The_Most_Important_Thing==1?2:4;
					fire=1;
				}
				else
				{
					line_move_control(2400,2400,-2200,4600,speed);	
					n1=1;
					fire=0;
				}
				desire_speed=desire_speed_1;
			}
			if(pos_x>-487 && pos_y>2875)
			{
				if(pos_x>-87)
				{
					fire=1;
					line_move_control(2400,2400,-2200,4600-200,speed);	
					current_id=The_Most_Important_Thing==1?1:3;
				}
				else
				{
					line_move_control(0,4800,-2200,200,speed);
					n2=1;
					fire=0;
				}
				desire_speed=desire_speed_2;					
			}
			if(pos_x<-487 && pos_y>1912)
			{
				if(pos_y>2312)
				{
					fire=1;
					line_move_control(0,4800,-2200+200,200,speed);
					current_id=The_Most_Important_Thing==1?4:2;
				}
				else
				{
					line_move_control(-2400,2400,2200,200,speed);
					if(n2==1)  n3=1;
					fire=0;
				}
					desire_speed=desire_speed_3;
			}
			if(pos_x<487 && pos_y<1912)
			{
				if(pos_x<87)
				{
					line_move_control(-2400,2400,2200,200+200,speed);
					current_id=The_Most_Important_Thing==1?3:1;
					fire=1;
				}
				else
				{
					line_move_control(0,0,2200,4600,speed);
					n4=1;
					fire=0;
				}
						desire_speed=desire_speed_4;
			}
		
		if(n1==1 && n2==1 && n3==1 && n4==1)
		{
			done=1;
			n1=0;
			n2=0;
			n3=0;
			n4=0;
		}
		
/*********************ÔÆÌ¨Ö¸Ïò²¿·Ö*****************************/		
			motor_pid[1].target=0;		
			motor_pid[0].target=124518;        //Ö¸Ïò90¶È·½Ïò

/**********************·¢Éä²¿·Ö********************************/		
		
		if(current_id==BOI_1 || current_id==BOI_2)   //Èç¹ûÕâ¸ö¿òÎÒÒª
		{
				if(fire==1 && pink_in_repository)   //Èç¹û¿ÉÒÔ·¢Éä
				if(get_dis(current_id)<=2700 && dis_last>=2700)    //ÓÐÇò£¬ËÙ¶ÈÎÈ¶¨£¬½Ç¶ÈÎÈ¶¨£¬µ×ÅÌËÙ¶ÈÎÈ¶¨
				{
					motor_pid[3].target=10000;
					if(allow_1==1)
					{
						pink_in_repository-=1;
						allow_1=0;
					}
				}			
					dis_last=get_dis(current_id);
				
				if(dis_last>=2800)
				{
					motor_pid[3].target=300000;
					allow_1=1;
				}
		}
		else
		{
			motor_pid[3].target=300000;//·ÇÄ¿±ê¿òÊ±ÊÕÇÊ
		}
		
		if(good_in_repository)             //Èç¹ûÓÐºÚ°×Çò
		{
				if(get_dis(current_id)<=2700 && dis_2>=2700)				
				{
					motor_pid[2].target=10000;
						if(allow_2==1)
					{
						good_in_repository-=1;
						allow_2=0;
					}

				}
					dis_2=get_dis(current_id);
				
				if(dis_2>=2800)
				{
					motor_pid[2].target=-300000;
					allow_2=1;
				}
		}
		else
		{
			motor_pid[2].target=-300000;
		}
/************************·¢ÉäËÙ¶È¿ØÖÆ************************/		
		hanser_1=1;
		hanser_2=1;
		
//		ex_speed8=desire_speed;
//		ex_speed4=3700; //×ó±ßÔÆÌ¨ËÙ¶ÈÓ¦¸ÃÊÇ¹Ì¶¨µÄ
		
		//ÖÐ¼ä¿ò¾Í»¹ÓÃÄÇ¸ö¾àÀëÅÐ¶Ï
		
	}
	else           //Ë³Ê±Õë
	{

	}
	
	return done;
}





//3ºÅ¶à±ßÐÎ£¬´óÈ¦
u8 Compound_polygon_3(u8 state,int speed)
{
	static u8 n1=0,n2=0,n3=0,n4=0,allow_1=1,allow_2=1,fire=0;
	u8 done=0,current_id=0;
	int desire_speed=0;
	static float dis_last=0,dis_2=0;
	if(!state)   //ÄæÊ±Õë
	{
/************************×ßÐÐ²¿·Ö*******************************/		
		if(pos_x>800 && pos_y<3200)
		{
			if(pos_y<2800)
			{
				fire=1;
				line_move_control(600,0,2200-400,4600,speed);	
				current_id=The_Most_Important_Thing==1?2:4;
			}
			else
			{
				fire=0;
				line_move_control(2400,3200,-2200,4600,speed);
				n1=1;
			}
			desire_speed=desire_speed_1;					
		}
		if(pos_x>-800 && pos_y>3200)
		{
			if(pos_x>-400)
			{
				fire=1;
				line_move_control(2400,3200,-2200,4600-400,speed);	
				current_id=The_Most_Important_Thing==1?1:3;
			}
			else
			{
				fire=0;
				line_move_control(-600,4800,-2200,200,speed);	
				n2=1;
			}
			desire_speed=desire_speed_2;					
		}		
		if(pos_x<-800 && pos_y>1600)
		{
			if(pos_y>1800)
			{
				fire=1;
				line_move_control(-600,4800,-2200+400,200,speed);	
				current_id=The_Most_Important_Thing==1?4:2;
			}
			else
			{
				fire=0;
				line_move_control(-2400,1600,2200,200,speed);
				if(n2==1)n3=1;
			}
			desire_speed=desire_speed_3;					
		}
		if(pos_x<800 && pos_y<1600)
		{
			if(pos_x<400)
			{
				fire=1;
				line_move_control(-2400,1600,2200,200+400,speed);	
				current_id=The_Most_Important_Thing==1?3:1;
			}
			else
			{
				fire=0;
				line_move_control(600,0,2200,4600,speed);	
				n4=1;
			}
				desire_speed=desire_speed_4;					
		}
		
		if(n1==1 && n2==1 && n3==1 && n4==1)
		{
			done=1;
			n1=0,n2=0;n3=0;n4=0;
		}
		
/*********************ÔÆÌ¨Ö¸Ïò²¿·Ö*****************************/		
			motor_pid[1].target=0;
			motor_pid[0].target=147456;       		 //Ö¸Ïò38¶È·½Ïò

/**********************·¢Éä²¿·Ö********************************/		
		
		if(current_id==BOI_1 || current_id==BOI_2)   //Èç¹ûÕâ¸ö¿òÎÒÒª
		{

				if(fire==1 && pink_in_repository)   //Èç¹û¿ÉÒÔ·¢Éä
				if(get_dis(current_id)<=2700 && dis_last>=2700)    //ÓÐÇò£¬ËÙ¶ÈÎÈ¶¨£¬½Ç¶ÈÎÈ¶¨£¬µ×ÅÌËÙ¶ÈÎÈ¶¨
				{
						motor_pid[3].target=10000;
					if(allow_1==1)
					{
						pink_in_repository-=1;
						allow_1=0;
					}

				}
				dis_last=get_dis(current_id);
				
					if(dis_last>2800)
					{
						motor_pid[3].target=300000;
						allow_1=1;
					}
		}
		
		if(good_in_repository)             //Èç¹ûÓÐºÚ°×Çò
		{
				if(get_dis(current_id)<=2700 && dis_2>=2700)		//ËÙ¶ÈÎÈ¶¨£¬µ×ÅÌËÙ¶ÈÎÈ¶¨£¬½Ç¶ÈÎÈ¶¨
				{
					motor_pid[2].target=10000;
					if(allow_2==1)
					{
						good_in_repository-=1;
						allow_2=0;
					}
				}
					
					dis_2=get_dis(current_id);
				
				if(dis_2>=2800)
				{
					motor_pid[2].target=-300000;
					allow_2=1;
				}
		}
		else
		{
			motor_pid[2].target=-300000;
		}
/**********************·¢ÉäËÙ¶È¿ØÖÆ*****************************/		
		//·¢ÉäÊ¹ÄÜ
		hanser_1=1;
		hanser_2=1;
		
//		ex_speed8=desire_speed;
//		ex_speed4=3700;
		
		//ÖÐ¼ä¿ò¾Í»¹ÓÃÄÇ¸ö¾àÀëÅÐ¶Ï
		
	}
	else
	{
		
		
	}
	return done;
}


u8 advanced_walking(int speed,u8 state)
{
	static u8 step=1;
	u8 num=0,done=0;
	switch(step)
	{
		case 1:
			num=get_circle_num();
			My_circle(0,2400,1000,speed,state);
		if(num>=1 && pos_y<2100)  //ÅÜ¹»Ò»È¦¶øÇÒ×ø±ê´ïµ½Ä³¸öÎ»ÖÃ£¬
			step=2;
		break;
		
		case 2:
			if(Compound_polygon_2(state,speed) )
				step=3;
		break;
		case 3:
			if(Compound_polygon_3(state,speed))
			step=4;			
			
			break;
		
		case 4:
			done=1;
			step=2;
			break;
		
	}
	
	return done;
}




//±ß×ß±ßÉäËùÓÃµÄÔÆÌ¨½Ç¶È
//Ä¬ÈÏ
float yousa_while_walking(u8 id)
{
	
	float real_x=0.f,real_y=0.f,current_angle=0.f,init_angle=0.f,real_angle=0.f;
	
	current_angle=Initial_angle-zangle;
		if(current_angle>360)                              //current_angleÎªÔÆÌ¨Óë¶¨Î»ÏµÍ³µÄÊµÊ±Ïà¶Ô½Ç¶È
			current_angle-=360;

	real_x=pos_x+107*cos(current_angle*pi/180);
	real_y=pos_y+107*sin(current_angle*pi/180);        // ÔÆÌ¨µÄÕæÊµ×ø±ê

		//	LCD_DisplayNum(100,210,real_x,8,24,0);
		//		LCD_DisplayNum(100,190,real_dis_1,8,24,0);
		real_dis_1=sqrt((real_x-target[id][0])*(real_x-target[id][0])+(real_y-target[id][1])*(real_y-target[id][1]));

	init_angle=atan2f(target[id][0]-real_x,target[id][1]-real_y) * 180/pi;
	real_angle=init_angle-zangle;
		
	if(real_angle>180)  real_angle-=360;
	if(real_angle<-180) real_angle+=360;            //¼ÓÉÏÕâÁ½ÐÐµÄÐ¡¹û¾ÍÊÇÂ·¹ý180µÄÊ±ºò»áÓÐÍ»±ä£¬¶øÕâ¸öÍ»±ä¶ÔÎÒÃÇÀ´ËµÊ±±ØÒªµÄ£¬ÒòÎªÅÂ³¶×ÅÏß
	
	return real_angle*position_prop;
}
	

float yousa_while_walking_2(void)
{
	
	float real_x=0.f,real_y=0.f,current_angle=0.f,init_angle=0.f,real_angle=0.f;
	
	current_angle=M_initial_angle-zangle;
		if(current_angle>360)                              
			current_angle-=360;

	real_x=pos_x+r*cos(current_angle*pi/180);
	real_y=pos_y+r*sin(current_angle*pi/180);        // ÔÆÌ¨µÄÕæÊµ×ø±ê

	init_angle=atan2f(0-real_x,2400-real_y) * 180/pi;
	real_angle=init_angle-zangle;
		
	if(real_angle>180)  real_angle-=360;
	if(real_angle<-180) real_angle+=360;            
	
	return real_angle*position_prop;
}
	


extern u8 choice_203;
extern u8 choice_204;

extern long int sum_2006_angle_203;
extern long int sum_2006_angle_204;

u8 self_203_init()
{
	static u8 flag=0;
	//u8 done=0;
	if(flag==0)
	{
	if(switch_203==0)
	{
		choice_203=1;
		sum_2006_angle_203=0;
		flag=1;
	}
	}
	if(flag==1)
	motor_pid[2].target=-290000;
	return flag;
}
u8 self_204_init()
{
	static u8 flag=0;
	//u8 done=0;
	if(flag==0)
	{
	if(switch_204==0)
	{
		choice_204=1;
		sum_2006_angle_204=0;
		flag=1;
	}
	}
	if(flag==1)
	motor_pid[3].target=300000;
	return flag;
}




void Arbitrary_square(int dis ,u8 state,int speed)
{
	if(state)
	{
		stop();
	}
	else
	{
		if(pos_x<275 && pos_y<2125)
		{
			line_move_control(0,2125-dis,100,2125-dis,speed);
		}
		if(pos_x>275 && pos_y<2675)
		{
			line_move_control(275+dis,0,275+dis,100,speed);
		}
		if(pos_x>-275 && pos_y>2675)
		{
			line_move_control(0,2675+dis,-100,2675+dis,speed);
		}
		if(pos_x<-275 && pos_y>2125)
		{
			line_move_control(-275-dis,0,-275-dis,-100,speed);
			
		}
	}
}
//What is dead may never die ,but rise harder and stronger
void Theon_Greyjoy()
{
	
}


//ÔÆÌ¨³¯ºó¸´Î»ÓÃµÄÔÆÌ¨º¯Êý
float Lengniao_yousa(u8 id)
{
		float real_x=0.f,real_y=0.f,current_angle=0.f,init_angle=0.f,real_angle=0.f;
	
	current_angle=Initial_angle-zangle;
		if(current_angle>360)                              //current_angleÎªÔÆÌ¨Óë¶¨Î»ÏµÍ³µÄÊµÊ±Ïà¶Ô½Ç¶È
			current_angle-=360;

	real_x=pos_x+r*cos(current_angle*pi/180);
	real_y=pos_y+r*sin(current_angle*pi/180);        // ÔÆÌ¨µÄÕæÊµ×ø±ê

	real_dis_1=sqrt((real_x-target[id][0])*(real_x-target[id][0])+(real_y-target[id][1])*(real_y-target[id][1]));
	
		init_angle=real_dis_1>3500?Angle_generator(real_x,real_y,target[id][0],target[id][1]) +180-3.5:
																Angle_generator(real_x,real_y,target[id][0],target[id][1]) +180;
	real_angle=init_angle-zangle;
		
		//LCD_DisplayNum(30,160,real_x,4,24,0);
		LCD_DisplayNum(100,210,real_dis_1,4,24,0);

	if(real_angle>180)  real_angle-=360;
	if(real_angle<-180) real_angle+=360;            //¼ÓÉÏÕâÁ½ÐÐµÄÐ¡¹û¾ÍÊÇÂ·¹ý180µÄÊ±ºò»áÓÐÍ»±ä£¬¶øÕâ¸öÍ»±ä¶ÔÎÒÃÇÀ´ËµÊ±±ØÒªµÄ£¬ÒòÎªÅÂ³¶×ÅÏß
	
	return real_angle*position_prop;

}
//ÔÆÌ¨³¯ºó¸´Î»ÓÃµÄÔÆÌ¨º¯Êý
float Lengniao_yousa_2(void)
{
	float real_x=0.f,real_y=0.f,current_angle=0.f,init_angle=0.f,real_angle=0.f;
	
	current_angle=M_initial_angle-zangle;
		if(current_angle>360)                              //current_angleÎªÔÆÌ¨Óë¶¨Î»ÏµÍ³µÄÊµÊ±Ïà¶Ô½Ç¶È
			current_angle-=360;

	real_x=pos_x+r*cos(current_angle*pi/180);
	real_y=pos_y+r*sin(current_angle*pi/180);        // ÔÆÌ¨µÄÕæÊµ×ø±ê

			real_dis_2=sqrt(real_x*real_x+(real_y-2400)*(real_y-2400));

		init_angle=Angle_generator(real_x,real_y,0,2400)+180-3;
		real_angle=init_angle-zangle;
		
			//LCD_DisplayNum(100,190,real_dis_2,5,24,0);
	if(real_angle>180)  real_angle-=360;
	if(real_angle<-180) real_angle+=360;            //¼ÓÉÏÕâÁ½ÐÐµÄÐ¡¹û¾ÍÊÇÂ·¹ý180µÄÊ±ºò»áÓÐÍ»±ä£¬¶øÕâ¸öÍ»±ä¶ÔÎÒÃÇÀ´ËµÊ±±ØÒªµÄ£¬ÒòÎªÅÂ³¶×ÅÏß
	
	return real_angle*position_prop;

}


void Laser_edge(u8 state,int speed ,u16 dis)
{
	if(((pos_x>-1100) && (pos_x<1100)) || ( (pos_y>1300 )  && (pos_y<3500) ))
		speed=4500;
	else
		speed=3000;
		if(state==0)   //ÄæÊ±Õë                           
	{
		if(pos_x>1550 && pos_y<3950)
		{
			laser_line(0,speed,dis);
		}
		if(pos_x>-1550 && pos_y>3950)
		{
			laser_line(-90,speed,dis);
		}
		if(pos_x<-1550 && pos_y>850)
		{
			laser_line(180,speed,dis);
		}
		if(pos_x<1550  && pos_y<850)
		{
			laser_line(90,speed,dis);
		}
		
		if(abs(pos_x<1550) && pos_x<3950 && pos_y>850)
			My_circle(0,2400,2000,speed,0);
	}
	else     //Ë³Ê±Õë                              
	{
		if(pos_x>1550 && pos_y>850)
		{
			laser_line(180,speed,dis);
		}
		if(pos_x<1550 && pos_y>3950)
		{
			laser_line(90,speed,dis);
		}
		if(pos_x<-1550 && pos_y<3950)
		{
			laser_line(0,speed,dis);
		}
		if(pos_x>-1550 && pos_y<850)
		{
			laser_line(-90,speed,dis);
		}

	if(abs(pos_x<1550) && pos_x<3950 && pos_y>850)
			My_circle(0,2400,2000,speed,1);
	}
	
}


u8 come_and_get_your_love()
{
	u8 ohhhhhhhhhh=0;
	static u8 firstips=0;
	if(!firstips)  //20ÃëÄÚÍê³ÉµÚÒ»´Î·¢Éä
	{
		if(running_time>20000 ||( pink_in_repository>=2 && good_in_repository>=1))
		{
			ohhhhhhhhhh=1;
			firstips=1;
		}
	}
	else    //µÚÒ»´Î·¢ÉäÍê³É
	{
		if(running_time<80000)  //80ÃëÄÚ
		{
			if(pink_in_repository && good_in_repository)
				ohhhhhhhhhh=1;
		}
		else
		{
			if(pink_in_repository || good_in_repository)
				ohhhhhhhhhh=1;
		}
	}
	
	//×î¸ßÓÅÏÈ¼¶£¬ÊÕÇòÊý´óÓÚµÈÓÚÈý¸ö±ØÈ»È¥·¢Éä
	if(pink_in_repository+good_in_repository>=3)
		ohhhhhhhhhh=1;
	return ohhhhhhhhhh;
}

double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)//0.01	0.5	0
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;       

   x_mid=x_last; 
   p_mid=p_last+Q; 
   kg=p_mid/(p_mid+R); 
   x_now=x_mid+kg*(ResrcData-x_mid);
               
   p_now=(1-kg)*p_mid;  

   p_last = p_now; 
   x_last = x_now; 
				
   return x_now;  
}

double KalmanFilter_2(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)//0.01	0.5	0
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;       

   x_mid=x_last; 
   p_mid=p_last+Q; 
   kg=p_mid/(p_mid+R); 
   x_now=x_mid+kg*(ResrcData-x_mid);
               
   p_now=(1-kg)*p_mid;  

   p_last = p_now; 
   x_last = x_now; 
				
   return x_now;  
}

//¼¤¹âË¢ÐÂº¯Êý
void laser_refersh()
{
	static float adc_1=0.f,adc_2=0.f;
	static u8 i=0;
	//float mid1=0;//,mid3=0;
	adc_1+=Get_Adc(ADC_Channel_4);
	adc_2=Get_Adc(ADC_Channel_5)*1.1776;
			laser_right=KalmanFilter(adc_2,0.01,0.5,0);
		//	laser_right=KalmanFilter_2(mid1,0.01,0.5,0);

	i+=1;
	if(i>=10)
	{
		laser_left=adc_1/i*1.1776;
		i=0;
		adc_1=0;
//		adc_2=0;
	}
}


u8 coordinate_correction()
{
	u8 done=1;
	float real_pos_2=0.f;
	if(laser_left+laser_right+316>4400)   //Èç¹ûÃ»ÓÐ±»³µ
	{
		real_pos_2=laser_right;//-(4484-laser_left-laser_right)/2;
			
		switch(closest_wall())
		{
			case 1:      CorrectValue(-(real_pos_2-2230),-233,180);  break;
			
			case 2: 		 CorrectValue(-2090,-(real_pos_2+170-77),-90);  break;
							
			case 3:   	 CorrectValue(-(2230-real_pos_2),-4413,0 );  break;
							
			case 4:			 CorrectValue(2090,-(4630-real_pos_2-77),90 );  break;				
		}
		/*		switch(wall)
				{
					case 1:           
						CorrectValue(-(real_pos_2-2235),-230,180)
						break;
					case 2:
						if(CorrectValue(-2170,-(adc5+165),-90))
						break;
					case 3:
						if(CorrectValue(-(2235-adc5),-4570,0))
						break;
					case 4:
						if(	CorrectValue(2170,-(4635-adc5),90))
						break;
				}
*/
	}	
	else        
	{
		
		switch(closest_wall())
		{
			case 1:      
				if(abs(laser_left-2400-pos_x)<400)   //Èç¹û×ó±ß¼¤¹âÃ»ÓÐ±»µ²ÉÏ
					CorrectValue((laser_left-2230),-233,180);  
				else
					CorrectValue(-(laser_right-2230),-233,180);
			break;
			
			case 2: 		 
				if(abs(laser_left+pos_y-4800)<400)  //Èç¹û×ó±ß¼¤¹âÃ»ÓÐ±»µ²ÉÏ
					CorrectValue(-2090,-(4630-laser_left-77),-90);  
				else
					CorrectValue(-2090,-(laser_right+159-77),-90);  
			break;
							
			case 3:   	 
				if(abs(laser_left+pos_x-2400)<400)  //Èç¹û×ó±ß¼¤¹âÃ»ÓÐ±»µ²ÉÏ
					CorrectValue(-(laser_left-2241),-4413,0 );  
				else
					CorrectValue(-(2241-laser_right),-4413,0 );  		
			break;
							
			case 4:			 
				if(abs(laser_left-pos_y)<400)
					CorrectValue(2090,-(laser_left+170),90 );  
				else
					CorrectValue(2090,-(4630-laser_right-77),90 );  
			break;				
		}
	}
	return done;
}

u8 sp_escape(u8 state)
{
	u8 wall=0,finish=0;
	int ex_angle=0;
	static u8 step=1;
	
		wall=closest_wall();
												//     1±íÊ¾Ë³Ê±Õë  £¬0±íÊ¾ÄæÊ±Õë
	
		switch(step)
		{
			case 1:
				escape_timer_switch=1;
				finish=0;
//				switch(wall)
//				{
//					case 1: ex_angle =  state==1?  -120: 120;  break;
//					case 2: ex_angle =  state==1?   150:-150;  break;
//					case 3: ex_angle =  state==1?    60: -60;  break;
//					case 4: ex_angle =  state==1?   -30:  30;  break;
//				}
//				break;
			if(wall==1)
				{	if(state==1)ex_angle=-120; else ex_angle=120;	}
			if(wall==2)
				{	if(state==1)ex_angle=150; else ex_angle=-120;	}
			if(wall==3)
				{	if(state==1)ex_angle=60; else ex_angle=-60;	}
			if(wall==4)
				{	if(state==1)ex_angle=-30; else ex_angle=30;	}
			
			case 2:
				
			if(escape_timer<700)	
				M_Back_control(ex_angle,3600);
			else
			{
				step=3;
			}
				break;
			
			case 3:
				
			if(escape_timer<1800)
			{
				My_circle(0,2400,1650,3600,state);			
				}
			else
			{
				step=4;
			}
				break;
			
			
				case 4:
					escape_timer_switch=0;    
					escape_timer=0;
					ex_angle=0;
					step=1;
					finish=1;
					break;
			
		}
		return finish;
}

//°ë¾¶Ö»ÓÐÁ½¸ö£¬Ò²¾ÍÊÇÖ®ºóÁ½È¦£¬µÚÒ»È¦Ê²Ã´¶¼Ã»ÓÐ£¬
void round_fire(int R,int speed,int state)
{
	u8 id=0,wall=0;
	static u8 allow_1=1,allow_2=1;
	int desire_speed_lt=0,desire_speed_rh=0;
	float angle=0,dire_lt=0.f,dire_rh=0.f;
	static int ang_1=0;
/************×ßÐÐ²¿·Ö***************************/
	My_circle(0,2400,R,speed,state);
/************È·¶¨ID*****************************/
	wall=closest_wall();
	switch(wall)
	{
		case 1:   id=The_Most_Important_Thing==0?1:3;  break;
		case 2:   id=The_Most_Important_Thing==0?4:2;  break;
		case 3:   id=The_Most_Important_Thing==0?3:1;  break;
		case 4:   id=The_Most_Important_Thing==0?2:4;  break;
	}
/**************È·¶¨ËÙ¶È¼°½Ç¶È********************/
	switch(R)
	{		
		case 1350:
			dire_lt=47;
			desire_speed_lt=4550;
				switch(id)
			{
				case 1:  desire_speed_rh=4100; dire_rh=-139; break;
				case 2:  desire_speed_rh=4500; dire_rh=-137; break;
				case 3:  desire_speed_rh=4260; dire_rh=-138; break;
				case 4:  desire_speed_rh=4680; dire_rh=-138; break;
			}

		break;
		
		case 1550:
			dire_lt=48;
			desire_speed_lt=4600;
			switch(id)
			{
				case 1:  desire_speed_rh=4050; dire_rh=-137; break;
				case 2:  desire_speed_rh=4500; dire_rh=-139; break;
				case 3:  desire_speed_rh=4260; dire_rh=-138; break;
				case 4:  desire_speed_rh=4680; dire_rh=-144; break;
			}

		break;
	}
	
/*************ÔÆÌ¨Ö¸Ïò***************************/
		motor_pid[0].target=3276.8*dire_lt; 
		motor_pid[1].target=3276.8*dire_rh;	     
/***********×ªËÙ²¿·Ö*************/
		hanser_1=1;
		hanser_2=1;
		
		motor_u3[0].target=desire_speed_lt;
		motor_u3[1].target=desire_speed_rh;
/*********·¢Éä²¿·Ö****************************/	
	angle=atan2f(pos_x,(pos_y-2400+77))*180/pi;
	if(angle<0)   angle+=360;
	angle=(int)angle%90;

		if((id==1||id==3) && pink_in_repository  &&  (id==BOI_1||id==BOI_2) )  //ÊÇÄ¿±ê¿ò
		{
			if(angle<=88 && ang_1>=88)   //´¥·¢¹©µ¯µÄÌõ¼þ
			{
				if( abs(motor_u3[1].target-desire_speed_rh)<45   && \
						chassic_speed>500)     //ÆäËü¸½¼ÓÌõ¼þ£¬µ×ÅÌËÙ¶È£    ³µµÄ½Ç¶ÈµÈ    ÔÆÌ¨µÄ×ªËÙ     
					{
					motor_pid[3].target=10000;
					if(allow_1==1)
					{
						pink_in_repository-=1;
						allow_1=0;
					}
				}
			}
		}
		
		if(angle<=88 && ang_1>=88 )
		{
			if(good_in_repository && abs(motor_u3[0].target-desire_speed_lt)<35           &&     (id==BOI_1||id==BOI_2) &&\
				chassic_speed>500)     //Èç¹û¸÷¸öÌõ¼þÎÈ¶¨
			{
				motor_pid[2].target=-10000;
				if(allow_2==1)
				{
					good_in_repository-=1;
					allow_2=0;
				}
			}
		}
		ang_1=angle;
		
		if(ang_1<65)
		{
			motor_pid[3].target=300000;
			motor_pid[2].target=-280000;
			allow_2=1;   										   //ÔÊÐí¼õÇò
			allow_1=1;
		}

}

//´ó¸çµÄ²Êºç×ßÐÐ
u8 numlast=0;
void rainbow(int speed,u8 state)
{
	int R=0;
	if(!sunshine)    //sunshinµÈÓÚ1±íÊ¾¿ÉÒÔÈ¥×ßÐÐ
	{
		switch(rso_sunshine)
		{
			case 1:
				if(get_circle_num()==1)
					Arbitrary_square(280,state,speed);
				else if(get_circle_num()==0)
					My_circle(0,2400,1150,speed,state);
				else if(get_circle_num()==2)
					My_circle(0,2400,1450,speed,state);
				else if(get_circle_num()>=1){
					sunshine=1;
					numlast=get_circle_num();}
			break;
			
			case 2:
				
				R=2000-(get_circle_num()-numlast)*700;
			if(R>1800)
				clear_edge(state,speed);
			else
			{
//				if(R==1600)
//					round_fire(1550,speed,state);
//				else
					My_circle(0,2400,R,speed,state);
			}
			
			if(R<1000)
			{
				numlast=get_circle_num();
				sunshine=1;
			}
			break;
			
			case 3:
				
			R=1100+(get_circle_num()-numlast)*280;
			
			if(R>1800)
				clear_edge(state,speed);
			else
			{
				if(R==1500 || R==1700)
					round_fire(R,speed,state);
				else
					My_circle(0,2400,R,speed,state);
			}
			if(R>1800)
			{
				numlast=get_circle_num();
				sunshine=1;
			}
			break;
			
	}
		
	//		LCD_DisplayNum(100,130,rso_sunshine,5,24,0);

	
}
	
	
}



void rainbow_2(int speed,u8 state)
{
	int R=0;
	if(!sunshine)    //sunshinµÈÓÚ1±íÊ¾¿ÉÒÔÈ¥×ßÐÐ
	{
		switch(rso_sunshine)
		{
			case 1:
				if(get_circle_num()==0)
					Arbitrary_square(580,state,speed);
				else if(get_circle_num()==1)
					My_circle(0,2400,1550,speed,state);
				else if(get_circle_num()>=2){
					sunshine=1;
					numlast=get_circle_num();}
			break;
			
			case 2:
				
				R=2000-(get_circle_num()-numlast)*400;
			if(R>1800)
				clear_edge(state,speed);
			else
			{
//				if(R==1600)
//					round_fire(1550,speed,state);
//				else
					My_circle(0,2400,R,speed,state);
			}
			
			if(R<1300)
			{
				numlast=get_circle_num();
				sunshine=1;
			}
			break;
			
			case 3:
				
				
			R=1100+(get_circle_num()-numlast)*280;
			
			if(R>1800)
				clear_edge(state,speed);
			else
			{
				if(R==1500 || R==1700)
					round_fire(R,speed,state);
				else
					My_circle(0,2400,R,speed,state);
			}
			if(R>1800)
			{
				numlast=get_circle_num();
				sunshine=1;
			}
			break;
			
	}
		
	//		LCD_DisplayNum(100,130,rso_sunshine,5,24,0);

	
}
	
	
}


u8 come_and_get_your_love_2()    //ºÜ¼òµ¥µÄË¼Â·
{
	u8 ohhhhhhhhhhh=0;

	if(rso_sunshine<=2)
	{
		if(sunshine==1 )
			ohhhhhhhhhhh=1;
	}
	else
	{		
		if(running_time<90000) 
	{
		if((good_in_repository>=2) || pink_in_repository)
			ohhhhhhhhhhh=1;
	}
	else
	{
		if(good_in_repository || pink_in_repository)
			ohhhhhhhhhhh=1;
	}
}
	

if(running_time>60000)
{
			if((good_in_repository>=2) || pink_in_repository)
			ohhhhhhhhhhh=1;

}
	return ohhhhhhhhhhh;
}

void de_ja_vu_5(u8 state,int speed)
{
	static u8 step=1;
	int R=0;
	switch(step)
	{
			case 2:
				R=2000-(get_circle_num()-numlast)*300;
			if(R>1900)
				clear_edge(state,speed);
			else
			{
				if(R>1300)
							My_circle(0,2400,R,speed,state);
				else
						 Arbitrary_square(280,state,4500);
			}
			if(R<1000)
			{
				numlast=get_circle_num();
				step=1;
			}
			break;
			
			case 1:
				
			R=1100+(get_circle_num()-numlast)*450;
			
			if(R>1600)
				clear_edge_2(state,speed);
			else
			{
				if(R==1500 || R==1700)
					round_fire(R,speed,state);
				else
					My_circle(0,2400,R,speed,state);
			}
			if(R>2100)
			{
				numlast=get_circle_num();
				step=2;
			}
			break;

		
	}
}	
	
