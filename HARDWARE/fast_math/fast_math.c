#include <fast_math.h>
#include "main.h"
int min(int a,int b)
{
	return a>=b? b:a;
}
int max(int a,int b)
{
	return a>=b? a:b;
}
//������
float f_min(float a,float b)
{
	return a>b? b:a;
}
float f_max(float a,float b)
{
	return a>b? a:b;
}
short temp_max(short a,short b,short c)
{
	return a>b? a>c? a:c :b>c? b:c; 
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


//��ʵ֤��������ļ���Ķ�������ûʲô��
