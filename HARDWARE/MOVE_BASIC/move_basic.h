#ifndef _MOVE_BASIC_
#define _MOVE_BASIC_

#include "main.h"

#define center(x,y)   (fabs(x)<600  || fabs(y-2400)<400)
#define corner(x,y)   (x>=500 && y<=1900) || (x>=500 && y>=2900) ||(x<=-500 && y<=1900) || (x<=-900 && y>=2900)
#define corner2(x,y)  ((x>800 && y<1600) ||(x>800 && y>3600)  ||(x<-800 && y<1600) || (x<-800 && y>3600))


//���л�����
void Chassic_vel(int speed_L, int speed_R,int speed_F);
void M_Angle_control(float ex_angle,int speed);
void M_Back_control(float ex_angle,int speed);
void My_back_circle(float x,float y,u16 R,int speed,u8 state);
void My_circle(float x,float y,u16 R,int speed,u8 state);
void line_move_control(int start_x,int start_y,int aim_x ,int aim_y ,int speed);
void back_line_control(int start_x,int start_y,int aim_x ,int aim_y ,int speed);
void My_ellipse(int X,int Y,float a,float b,int speed,u8 state);
float Angle_generator(float start_x,float start_y,float aim_x,float aim_y);


//������

void clear_edge(u8 state,int speed);
void clear_edge_2(u8 state,int speed);

void De_ja_vu(int speed,u8 state,int r);
void De_ja_vu_2(u8 state,int speed);
void de_ja_vu_4(int speed,u8 state);
void de_ja_vu_5(u8 state,int speed);

void Tilted_rect(int speed ,u8 state);   				//��б����    ����Ҳ�������������
void Tilted_rect_2(int speed,u8 state);  				//��б���ζ��ţ��ȵ�һ��СһȦ  ʵ��Ч�����ã��ߵĺܳ�

void Arbitrary_square(int dis ,u8 state,int speed); //�����Σ������ɨ�ڿ��õ�


void laser_line(float ex_angle,int speed,float dis);  //����ɨ�ߵĺ���
void Laser_edge(u8 state,int speed ,u16 dis);
u8 Compound_polygon_3(u8 state,int speed);
u8 advanced_walking(int speed,u8 state);

void round_fire(int R,int speed,int state);
void rainbow(int speed,u8 state);
void rainbow_2(int speed,u8 state);

//����
u8 Escape(u8 state);


//��λ��
u8 Ready_To_Endgame(u8 situation,u8 state);  //ȥ��ǽ
u8 To_next_wall(u8 state);
u8 I_Am_Champion(u8 situation,u8 state);
void Ready_to_Endgame_2(u8 situation,u8 state,u8 tmit);
void Ready_to_Endgame_3(u8 state);
u8 coordinate_correction(void);
void Ready_to_Endgame_4(u8 state);
 int All_Function(u8 id,double x);



//��̨��
float Lingyuan_yousa_1(u8 id);
float Lingyuan_yousa_2(void);

float yousa_while_walking(u8 id);
float yousa_while_walking_2(void);

float Lengniao_yousa(u8 id);
float Lengniao_yousa_2(void);

//ͨ����
u8 codelf(void);
u8 get_circle_num(void);
u8 closest_wall(void);
u8 come_and_get_your_love(void);
u8 come_and_get_your_love_2(void);
void laser_refersh(void);


//������
u8 absdiff(u8 a,u8 b,u8 c);
u8 color_detect(u8 R,u8 G,u8 B);
void squeeze_ball(void);
u8 self_203_init(void);
u8 self_204_init(void);


//������   ����

void Compound_polygon(u8 state,int speed); 	 		//���϶����
u8 Compound_polygon_2(u8 state,int speed); 		//���϶����2  СȦ


#endif

