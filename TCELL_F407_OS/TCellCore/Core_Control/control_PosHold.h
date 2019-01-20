#ifndef _CONTROL_POSHOLD_H_
#define _CONTROL_POSHOLD_H_

#include "sys_Platform.h"
#include "control_Config.h"

/*定点模式下,遥杆回中后,先用水平速度控制刹车,待刹停后再赋值GPS位置选点*/
SYS_RETSTATUS horizontal_GPS_Get_Stop_Point_XY(Vector2f_Nav *stopPoint);

/*定点模式下,遥杆回中后,先用水平速度控制刹车,待刹停后再赋值光流位置选点*/
SYS_RETSTATUS horizontal_OpticFlow_Get_Stop_Point_XY(Vector2f *stopPoint);

/*水平方向的加速度换算倾角*/
Vector2f *horizontal_Acc_Convert_To_Dip_Angle(Vector2f acc2f, Vector2f *angle2f);

/*水平位置控制器*/
void horizontal_Control_PosHold(fp32 controlDeltaT);

/*水平GPS位置控制器*/
void horizontal_Control_GPS_PosHold(fp32 controlDeltaT);

/*水平光流位置控制器*/
void horizontal_Control_OpticFlow_PosHold(fp32 controlDeltaT);

extern Vector2f g_sHorizontalExpectAcc; /*水平期望加速度*/
extern Vector2f g_sHorizontalExpectAngle; /*水平期望角度*/

#endif
