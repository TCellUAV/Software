#ifndef _CONTROL_AIRCRAFT_H_
#define _CONTROL_AIRCRAFT_H_

#include "sys_Platform.h"
#include "math_Function.h"
#include "bsp_BoardLib.h"
#include "attitude_Aircraft.h"
#include "remot_DataAnaly.h"
#include "status_Aircraft.h"
#include "ahrs_Caculation.h"
#include "sins_Strapdown.h"
#include "bsp_BoardLib.h"

/*定高/定点/返航(降落)*/
#include "control_Config.h"

/*控制系统参数读取结果*/
typedef enum
{
	CTRL_SYSTEM_PARA_INIT_SUCC = 0, /*初始化成功*/
	CTRL_SYSTEM_PARA_INIT_FAIL = 1, /*初始化失败*/
}CTRL_SYSTEM_PARA_INIT_STATUS;

/*控制算法初始化状态*/
typedef struct
{
	/*PID算法*/
	volatile CTRL_SYSTEM_PARA_INIT_STATUS pid;
	
	/*ADRC算法*/
	volatile CTRL_SYSTEM_PARA_INIT_STATUS adrc;
}CTRL_SysStatus;

/*飞行器飞行过程中,运动趋势*/
typedef enum
{
	/*竖直方向*/
	CTRL_AIRCRAFT_MOVE_VER_UP    = 0, /*竖直向上运动*/
	CTRL_AIRCRAFT_MOVE_VER_HOLD  = 1, /*竖直保持中立*/
	CTRL_AIRCRAFT_MOVE_VER_DOWN  = 2, /*竖直向下运动*/
	
	/*水平方向*/	
	CTRL_AIRCRAFT_MOVE_HOR_FRONT = 3, /*水平向前运动*/
	CTRL_AIRCRAFT_MOVE_HOR_BACK  = 4, /*水平向后运动*/
	CTRL_AIRCRAFT_MOVE_HOR_LEFT  = 5, /*水平向左运动*/	
	CTRL_AIRCRAFT_MOVE_HOR_RIGHT = 6, /*水平向右运动*/	
	CTRL_AIRCRAFT_MOVE_HOR_HOLD  = 7, /*水平保持中立*/
}CTRL_AIRCRAFT_MOVE_TREND;

/*自动返航功能开关*/
typedef enum
{
	CTRL_AIRCRAFT_GO_HOME_DISABLE = 0, /*不允许自动返航*/
	CTRL_AIRCRAFT_GO_HOME_ENABLE  = 1, /*允许自动返航*/
}CTRL_AIRCRAFT_GO_HOME_STATUS;

/*自动返航设置状态*/
typedef enum
{
	CTRL_AIRCRAFT_GO_HOME_SET    = 1, /*自动返航已设置*/
	CTRL_AIRCRAFT_GO_HOME_NOTSET = 0, /*自动返航未设置*/
}CTRL_AIRCRAFT_GO_HOME_SWITCH;

/*电机PWM输出值*/
typedef struct
{
	u16 channle1;
	u16 channle2;	
	u16 channle3;	
	u16 channle4;
	u16 channle5;
	u16 channle6;	
	u16 channle7;	
	u16 channle8;	
}MotorPWMOutput;

/*控制相关*/
typedef struct
{
	/*飞行器状态*/
    u16 			        	 	         heightHoldThrottle;        /*高度保持油门(用于定高)*/
	u16					     		         throttleUpDownJudgeValue;	/*油门杆上推或者下推判断量*/
	u16					     		         ctrlThrottle;				/*本次油门(非遥控期望油门,非PWM输出油门)*/	
	u16					     		         lastCtrlThrottle;			/*上次油门(非遥控期望油门,非PWM输出油门)*/	
	u16					     		         throttleOutput;			/*油门最终输出*/
	Remot_Expect_Angle     		             RemotExpectAngle;			/*遥控值转期望角度值*/
	ImuAttitude			     		         RemotExpectAutoAngle;		/*遥控期望自稳俯仰角,横滚角*/
	GimbalAngle              		  	     GimbalExpectAngle;		    /*万向节云台期望角度*/
	volatile CTRL_AIRCRAFT_MOVE_TREND 	     AIRCRAFT_VER_MOVE_TREND;	/*飞行器竖直方向运动趋势*/
	volatile CTRL_AIRCRAFT_MOVE_TREND        AIRCRAFT_HOR_MOVE_TREND;	/*飞行器水平方向运动趋势*/
	volatile CTRL_AIRCRAFT_GO_HOME_STATUS    GO_HOME_STATUS;			/*失联自动返航使能状态*/
	volatile CTRL_AIRCRAFT_GO_HOME_SWITCH    GO_HOME_SET;				/*自动返航设置状态*/
	
	/*电机PWM输出*/
	MotorPWMOutput		 	 		         CurMotorPwmOutput;			/*当前电机PWM输出*/
	MotorPWMOutput	     	 		         LastMotorPwmOutput;			/*上次电机PWM输出*/
}ControlAircraft;


/*控制算法参数初始化*/
SYS_RETERR ctrl_autocontrol_para_init(CTRL_SysStatus *sysStatus);

/*飞行器自动控制系统*/
void ctrl_auto_control_system_dp(void);

/*飞行模式及飞行任务选择*/
void ctrl_Control_Mode_Select(Uav_Status *uavStatus);

/*姿态控制器: 角度外环+角速度内环*/
void ctrl_Attitude_Control_Dp(void);
/*角度外环控制*/
void ctrl_Attitude_Angle_Control_Dp(void);
/*角速度内环控制*/
void ctrl_Attitude_Gyro_Control_Dp(void);

/*主导控制器: 自稳+定高&定点*/
void ctrl_MainLeading_Control_Dp(void);

/*油门倾角补偿*/
void ctrl_Throttle_DipAngle_Compensate(void);

/*自控系统控制器输出*/
void ctrl_auto_control_system_output(void);

extern CTRL_SysStatus g_sCtrlSysStatus;
extern CTRL_SysStatus *g_psCtrlSysStatus;

extern ControlAircraft g_sControlAircraft;
extern ControlAircraft *g_psControlAircraft;

#endif
