#ifndef _CONTROL_MISSION_H_
#define _CONTROL_MISSION_H_

#include "sys_Platform.h"
#include "control_Config.h"
#include "status_Aircraft.h"

/*任务清除拨键有效性状态*/
typedef enum
{
	MISSION_CLEAR_SWITCH_DISABLE = 0, /*无效*/
	MISSION_CLEAR_SWITCH_ENABLE  = 1, /*有效*/
}MISSION_CLEAR_SWITCH_STATUS;

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

/*飞行器飞行任务*/
typedef enum
{	
	/*NULL*/
	UAV_FLY_MISSION_NULL                = 00,   /*没有任务*/
	
	/*一键*/
	UAV_FLY_MISSION_ONEKEY_FLY			= 01,	/*一键起飞*/
	UAV_FLY_MISSION_ONEKEY_LAND_HOME    = 02,	/*一键返航/降落*/
	
	/*超声波+光流定点模式下*/
	UAV_FLY_MISSION_OPFLOW_FOLLOW_POINT = 06,	/*光流追点*/
	UAV_FLY_MISSION_OPFLOW_FOLLOW_LINE  = 07,	/*光流巡线*/
	
	/*气压计+GPS定点模式下*/
	UAV_FLY_MISSION_GPS_WRITE_POS		= 10,   /*GPS写入当前点位*/
	UAV_FLY_MISSION_GPS_CLEAR_ALL_POS	= 11,   /*清除所有点位*/	
	UAV_FLY_MISSION_GPS_PATROL_SKY      = 12,	/*GPS巡天*/
}UAV_FLY_MISSION;

/*任务有效状态*/
typedef enum
{
	UAV_MISSION_DISABLE = 0,	/*任务无效*/	
	UAV_MISSION_ENABLE  = 1,	/*任务有效*/
}UAV_MISSION_ENABLE_STATUS;

/*任务中目标点设置状态*/
typedef enum
{
	UAV_MISSION_TARG_SET_NO = 0, /*目标点未设置*/
	UAV_MISSION_TARG_SET_OK = 1, /*目标点已设置*/
}UAV_MISSION_TARG_SET_STATUS;	

/*任务中目标点到达状态*/
typedef enum
{
	UAV_MISSION_TARG_REACH_NO = 0, /*目标点未到达*/
	UAV_MISSION_TARG_REACH_OK = 1, /*目标点已到达*/
}UAV_MISSION_TARG_REACH_STATUS;

/*任务是否正在执行*/
typedef enum
{
	UAV_MISSION_EXECUTE_NOT = 0, /*没执行*/
	UAV_MISSION_EXECUTE_YES = 1, /*正在执行*/
}UAV_MISSION_EXECUTE_STATUS;

typedef struct
{
	UAV_MISSION_ENABLE_STATUS        ENABLE_STATUS;
	UAV_MISSION_TARG_SET_STATUS      TARG_SET_STATUS;
	UAV_MISSION_TARG_REACH_STATUS    TARG_REACH_STATUS;
	UAV_MISSION_EXECUTE_STATUS       EXECUTE_STATUS;
}Uav_Mission_Status;

/*一键任务*/
typedef struct
{
	Uav_Mission_Status FixedHeightFly;
	Uav_Mission_Status LandHome;
}Uav_Onekey_Mission;

/*写入点位任务*/
typedef struct
{
	UAV_MISSION_TARG_SET_STATUS   SET_STATUS; /*点的状态*/
	Vector3f_Nav                  Pos;    /*点*/
}Uav_Limitless_Pos;

typedef struct
{
	vu16 			   index; 	   											/*位置点的顺序*/
	vu16			   posTotalNbr;											/*写入点的总个数*/
	Uav_Limitless_Pos  Limitless_Pos[MISSION_LIMITLESS_POS_WRITE_MAX_NBR];  /*位置点*/
}Uav_Write_Gps_Mission;

/*是否允许同时执行任务外的姿态控制*/
typedef enum
{
	UAV_MISSION_ATTITUDE_CTRL_ENABLE  = 1, /*允许同时执行任务外的姿态控制*/
	UAV_MISSION_ATTITUDE_CTRL_DISABLE = 0, /*禁止同时执行任务外的姿态控制*/
}UAV_MISSION_ATTITUDE_CTRL_STATUS;

typedef struct
{
	/*当前飞行任务*/
	UAV_FLY_MISSION    			     CURRENT_FLY_MISSION;
	
	/*上次飞行任务*/
	UAV_FLY_MISSION    				 LAST_FLY_MISSION;
	
	/*是否允许同时执行姿态控制*/
	UAV_MISSION_ATTITUDE_CTRL_STATUS ATTITUDE_CTRL_STATUS;
	
	/*任务清除拨键有效性状态*/
	MISSION_CLEAR_SWITCH_STATUS      CLEAR_SWITCH_STATUS;
	
	/*一键任务*/
	Uav_Onekey_Mission 			     Onekey_Mission;
	
	/*写入点位任务*/
	Uav_Write_Gps_Mission 		     Write_Gps_Mission;
}Uav_Fly_Mission;


/*喂任务检测狗*/
void feed_control_fly_mission_check_dog(u8 _10msFoc, SimulateWatchDog *mission_check_dog);

/*获取任务检测狗存活状态*/
SIM_WATCH_DOG_STATUS get_control_fly_mission_dog_status(SimulateWatchDog *mission_check_dog);

/*=== 设置和获取任务 ===*/
/*任务状态机初始化*/
void control_fly_mission_machine_reset(Uav_Fly_Mission *uav_fly_mission, UAV_FLY_MISSION TARG_MISSION);

/*清除任务*/
void control_fly_mission_clear(Uav_Fly_Mission *uav_fly_mission, UAV_FLY_MISSION TARG_MISSION);

/*清除上一个任务,设置当前任务*/
void control_fly_mission_set(Uav_Fly_Mission *uav_fly_mission, UAV_FLY_MISSION SET_MISSION, UAV_MISSION_ATTITUDE_CTRL_STATUS ATTITUDE_CTRL_STATUS);

/*获取当前任务*/
UAV_FLY_MISSION control_fly_mission_get(void);

/*=== 常规任务 ===*/
/*一键起飞*/
void control_fly_mission_onekey_fly(Uav_Fly_Mission *uav_fly_mission, Uav_Status *uavStatus);

/*一键/失联返航/降落*/
void control_fly_mission_auto_gohome(Uav_Fly_Mission *uav_fly_mission, Uav_Status *uavStatus, fp32 controlDeltaT);

/*=== 需要超声波+光流定点的任务 ===*/
typedef enum
{
	MISSION_OPFLOW_FOLLOW_POINT = 2,	/*光流追点*/
	MISSION_OPFLOW_FOLLOW_LINE  = 4,	/*光流循线*/
}MISSION_ULTR_OPFLOW_NBR;

/*任务检测*/
UAV_FLY_MISSION control_fly_mission_check_ultr_opflow_pos(Uav_Fly_Mission *uav_fly_mission, u32 checkPeriodMS);

/*光流追点*/
void control_fly_mission_ultr_opflow_follow_point(Uav_Fly_Mission *uav_fly_mission, Uav_Status *uavStatus);

/*光流循线*/
void control_fly_mission_ultr_opflow_follow_line(Uav_Fly_Mission *uav_fly_mission, Uav_Status *uavStatus);

/*=== 需要GPS+气压计定点的任务 ===*/
typedef enum
{
	MISSION_GPS_WRITE_LIMITLESS_POS     = 2,	/*GPS记录当前点*/
	MISSION_GPS_PATROL_SKY              = 4,	/*GPS按记录点巡天*/
	MISSION_GPS_CLEAR_ALL_LIMITLESS_POS = 6,	/*清除所有记录点*/	
}MISSION_BERO_GPS_NBR;

/*任务检测*/
UAV_FLY_MISSION control_fly_mission_check_bero_gps_pos(Uav_Fly_Mission *uav_fly_mission, u32 checkPeriodMS);

/*GPS记录当前座标*/
void control_fly_mission_bero_gps_write_pos(Uav_Fly_Mission *uav_fly_mission, Uav_Status *uavStatus);

/*GPS清除所有记录的坐标*/
void control_fly_mission_bero_gps_clear_all_pos(Uav_Fly_Mission *uav_fly_mission, Uav_Status *uavStatus);

/*GPS巡天*/
void control_fly_mission_bero_gps_patrol_sky(Uav_Fly_Mission *uav_fly_mission, Uav_Status *uavStatus);

/*=== 需要OPENMV的任务 ===*/


extern Uav_Fly_Mission gs_Uav_Fly_Mission;

#endif
