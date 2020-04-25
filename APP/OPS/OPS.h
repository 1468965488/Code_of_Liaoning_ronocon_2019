#ifndef _OPS_H
#define _OPS_H

#include "sys.h"


#define OPS_TO_

//定位系统初始化
void Clear_All(void);
//旋转坐标系
void GetXY(void);

/*软件矫正*/
//角度及Y值矫正
u8 CorrectValue(float Ex_pos_x,float Ex_pos_y,float expectangle);
//Y值矫正
u8 CorrectValue_Y(float Ex_pos_y);
//X值矫正
u8 CorrectValue_X(float Ex_pos_x);
//角度角度矫正（即获取角度偏差）
void GetDeviation(float expectangle);
//获取X值偏差
void SetXerror(float SetX);
//获取Y值偏差
void SetYerror(float SetY);

/*硬件矫正，将数据输入到定位系统中进行矫正*/
//输入矫正X
void CorrectX(float value);
//输入矫正y
void CorrectY(float value);
//输入矫正角度
void CorrectAngle(float value);
#endif

