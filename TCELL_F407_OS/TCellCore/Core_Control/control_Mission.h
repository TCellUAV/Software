#ifndef _CONTROL_MISSION_H_
#define _CONTROL_MISSION_H_

#include "sys_Platform.h"
#include "control_Config.h"
#include "status_Aircraft.h"

/*拨键有效性状态*/
typedef enum
{
	CONTROL_MISSION_SWITCH_DISAVA = 0, /*无效*/
	CONTROL_MISSION_SWITCH_AVA    = 1, /*有效*/
}CONTROL_MISSION_SWITCH_STATUS;

/*倒计时状态*/
typedef enum
{
	CONTROL_MISSION_COUNTDOWN_FINISH = 0,	/*倒计时结束*/	
	CONTROL_MISSION_COUNTDOWN_START  = 1,	/*倒计时开始*/
}CONTROL_MISSION_COUNTDOWN_STATUS;

/*开始检测使能状态*/
typedef enum
{
	CONTROL_MISSION_START_CHECK_DISABLE = 0,  /*失能检测*/	
	CONTROL_MISSION_START_CHECK_ENABLE  = 1,  /*使能开始检测*/
}CONTROL_MISSION_START_CHECK_STATUS;

typedef struct
{
	volatile REMOT_SWITCH_EDGE_AVA_STATUS       HIGH_EDGE_AVA_STATUS;  /*高沿有效状态*/
	volatile REMOT_SWITCH_EDGE_AVA_STATUS       LOW_EDGE_AVA_STATUS;   /*低沿有效状态*/
	volatile CONTROL_MISSION_COUNTDOWN_STATUS   COUNTDOWN_STATUS;	   /*倒计时状态*/
	volatile CONTROL_MISSION_START_CHECK_STATUS CHECK_STATUS;		   /*开始检测使能状态*/
	vu8				  			  		        edge_cnt;  		       /*拨动次数*/
	SimulateWatchDog					        count_down;			   /*倒计时*/
}Control_Fly_Mission_Switch_Motion;

/*喂任务检测狗*/
void feed_control_fly_mission_check_dog(u8 _10msFoc, SimulateWatchDog *mission_check_dog);

/*获取任务检测狗存活状态*/
SIM_WATCH_DOG_STATUS get_control_fly_mission_dog_status(SimulateWatchDog *mission_check_dog);

/*=== 设置和获取任务 ===*/
/*任务状态机初始化*/
void control_fly_mission_machine_reset(Uav_Status *uavStatus, UAV_FLY_MISSION TARG_MISSION);

/*清除任务*/
void control_fly_mission_clear(Uav_Status *uavStatus, UAV_FLY_MISSION TARG_MISSION);

/*清除上一个任务,设置当前任务*/
void control_fly_mission_set(Uav_Status *uavStatus, UAV_FLY_MISSION SET_MISSION, UAV_MISSION_ATTITUDE_CTRL_STATUS ATTITUDE_CTRL_STATUS);

/*获取当前任务*/
UAV_FLY_MISSION control_fly_mission_get(void);

/*=== 常规任务 ===*/
/*一键起飞*/
void control_fly_mission_onekey_fly(Uav_Status *uavStatus);

/*一键/失联返航/降落*/
void control_fly_mission_gohome(void);

/*=== 需要超声波+光流定点的任务 ===*/
typedef enum
{
	MISSION_OPFLOW_FOLLOW_POINT = 2,	/*光流追点*/
	MISSION_OPFLOW_FOLLOW_LINE  = 4,	/*光流循线*/
}MISSION_ULTR_OPFLOW_NBR;

/*任务检测*/
UAV_FLY_MISSION control_fly_mission_check_ultr_opflow_pos(Uav_Status *uavStatus, u32 checkPeriodMS);

/*光流追点*/
void control_fly_mission_ultr_opflow_follow_point(void);

/*光流循线*/
void control_fly_mission_ultr_opflow_follow_line(void);

/*=== 需要GPS+气压计定点的任务 ===*/
typedef enum
{
	MISSION_GPS_WRITE_POINT = 2,	/*GPS记录当前点*/
	MISSION_GPS_PATROL_SKY  = 4,	/*GPS按记录点巡天*/
}MISSION_BERO_GPS_NBR;

/*任务检测*/
UAV_FLY_MISSION control_fly_mission_check_bero_gps_pos(Uav_Status *uavStatus, u32 checkPeriodMS);

/*GPS记录当前座标*/
void control_fly_mission_bero_gps_write_pos(Uav_Status *uavStatus);

/*GPS巡天*/
void control_fly_mission_bero_gps_patrol_sky(Uav_Status *uavStatus);

/*=== 需要OPENMV的任务 ===*/

#endif
