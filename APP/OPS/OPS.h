#ifndef _OPS_H
#define _OPS_H

#include "sys.h"


#define OPS_TO_

//��λϵͳ��ʼ��
void Clear_All(void);
//��ת����ϵ
void GetXY(void);

/*�������*/
//�Ƕȼ�Yֵ����
u8 CorrectValue(float Ex_pos_x,float Ex_pos_y,float expectangle);
//Yֵ����
u8 CorrectValue_Y(float Ex_pos_y);
//Xֵ����
u8 CorrectValue_X(float Ex_pos_x);
//�ǶȽǶȽ���������ȡ�Ƕ�ƫ�
void GetDeviation(float expectangle);
//��ȡXֵƫ��
void SetXerror(float SetX);
//��ȡYֵƫ��
void SetYerror(float SetY);

/*Ӳ�����������������뵽��λϵͳ�н��н���*/
//�������X
void CorrectX(float value);
//�������y
void CorrectY(float value);
//��������Ƕ�
void CorrectAngle(float value);
#endif

