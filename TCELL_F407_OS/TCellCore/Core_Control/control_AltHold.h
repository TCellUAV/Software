#ifndef _CONTROL_ALTHOLD_H_
#define _CONTROL_ALTHOLD_H_

#include "sys_Platform.h"
#include "control_Config.h"

/*油门推动和中位死区关系*/
typedef enum
{
	/*单次油门状态*/
	CONTROL_THROTTLE_PUSH_LOCATE_MIDDEAD	  = 0,  /*油门位于中位死区*/
	CONTROL_THROTTLE_PUSH_LOCATE_UP_MIDDEAD   = 1,	/*油门位于死区上方*/
	CONTROL_THROTTLE_PUSH_LOCATE_DOWN_MIDDEAD = 2,	/*油门位于死区下方*/
	
	/*结合本次和上次的状态*/
	CONTROL_THROTTLE_PUSH_DOWM_TO_MIDDEAD     = 3,  /*向下推到中位死区*/
	CONTROL_THROTTLE_PUSH_DOWM_OVER_MIDDEAD   = 4,  /*向下推过中位死区*/
	CONTROL_THROTTLE_PUSH_UP_TO_MIDDEAD	      = 5,  /*向上推到中位死区*/
	CONTROL_THROTTLE_PUSH_UP_OVER_MIDDEAD     = 6,  /*向上推过中位死区*/
	CONTROL_THROTTLE_PUSH_UPDOWN_DEADBAND     = 7,  /*油门暂位于中位死区*/
	CONTROL_THROTTLE_PUSH_UPDOWN_UPBAND       = 8,  /*油门暂位于上位区*/
	CONTROL_THROTTLE_PUSH_UPDOWN_DOWMBAND     = 9,  /*油门暂位于下位区*/	
}CONTROL_THROTTLE_PUSH;

/*油门推动和中位死区关系*/
typedef struct
{
	volatile CONTROL_THROTTLE_PUSH LAST_STATUS;	/*上次油门相对死区状态*/
	volatile CONTROL_THROTTLE_PUSH CUR_STATUS;	/*本次油门相对死区状态*/
	volatile CONTROL_THROTTLE_PUSH REAL_STATUS;	/*真实油门相对死区状态(结合上次和本次油门状态)*/
}ControlThrottlePush;

/*获取油门相对中位死区推动情况*/
CONTROL_THROTTLE_PUSH altitude_Get_Throttle_Relative_MidDead(ControlThrottlePush *controlThrottlePush);

/*竖直高度(定高)控制器*/
void vertical_Control_AltHold(fp32 controlDeltaT);


extern fp32 g_HeightYawSpeedFeedforwardOutput;   /*竖直速度前馈控制器输出*/
extern fp32 g_HeightYawSpeedFeedforwardRate;   	 /*竖直速度前馈控制器,APM里面为1、0.45*/
extern fp32 g_HeightYawSpeedFeedforwardDelta;    /*竖直期望速度变化率*/
extern fp32 g_CurHeightYawSpeedExpect;   		 /*本次竖直速度期望*/
extern fp32 g_LastHeightYawSpeedExpect;			 /*上次竖直速度期望*/

#endif
