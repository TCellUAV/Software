#ifndef _STATUS_AIRCRAFT_H_
#define _STATUS_AIRCRAFT_H_

#include "sys_Platform.h"
#include "bsp_BoardLib.h"

#define AIRCRAFT_LOCK_CONTINU_TIME_MS		(100)	 /*动作持续100ms(0.1s)以上锁定*/
#define AIRCRAFT_UNLOCK_CONTINU_TIME_MS		(2000)	 /*动作持续2000ms(2s)以上解锁*/
#define AIRCRAFT_AUTOLOCK_CONTINU_TIME_MS	(10000)	 /*未飞前未操作持续15000ms(15s)自动上锁*/

/*= 1.飞行器本身的状态 =*/	
/*飞行器锁定状态*/
typedef enum
{
	UAV_LOCK_YES  = 0,	/*已锁定*/
	UAV_LOCK_NOT  = 1,	/*未锁定(解锁)*/		
}UAV_LOCK_STATUS;

/*飞行器着陆状态*/
typedef enum
{
	UAV_LAND_YES = 1,	/*飞行器处于着陆状态*/
	UAV_LAND_NOT = 2,	/*飞行器处于飞行状态*/
}UAV_LAND_STATUS;

typedef struct
{
	UAV_LAND_STATUS ThisTime;	/*本次*/
	UAV_LAND_STATUS LastTime;	/*上次*/
}Uav_Land_Status;

/*飞行器飞行状态*/
typedef enum
{
	UAV_FLY_TYPE_ATTITUDE   = 00, /*纯姿态*/
	UAV_FLY_TYPE_FIX_HEIGHT = 01, /*定高*/
	UAV_FLY_TYPE_FIX_POS    = 02, /*定点*/
}UAV_FLY_TYPE;

typedef struct
{
	UAV_FLY_TYPE CURRENT;				/*当前*/
	UAV_FLY_TYPE BEFORE_WIRELESS_MISS;	/*失联前*/
}Uav_Fly_Type;

/*Home点设置状态*/
typedef enum
{
	UAV_HOME_SET_NOT = 0, /*飞行器Home点未设置*/	
	UAV_HOME_SET_YES = 1, /*飞行器Home点已设置*/
}UAV_HOME_SET_STATUS;

/*遥控和飞行器通信状态*/
typedef enum
{
	UAV_WIRELESS_CMC_FAIL = 0,	/*遥控和飞行器通信失败*/		
	UAV_WIRELESS_CMC_SUCC = 1,	/*遥控和飞行器通信成功*/
}UAV_WIRELESS_CMC_STATUS;

/*人机交互-OLED显示页面状态*/
typedef enum
{
	/*页面显示*/
	UAV_HCI_SHOW_HOLD_PAGE = 0,	/*OLED锁定当前页面*/
	UAV_HCI_SHOW_FREE_PAGE = 1,	/*解除当前页锁定*/
	UAV_HCI_SHOW_LAST_PAGE = 2,	/*OLED显示上一个页面*/
	UAV_HCI_SHOW_NEXT_PAGE = 3,	/*OLED显示下一个页面*/
	
	/*显示开关*/
	UAV_HCI_SHOW_ENABLE    = 7,	/*允许显示*/
	UAV_HCI_SHOW_DISABLE   = 8,	/*不允许显示*/
}UAV_HCI_SHOW_STATUS;

/*= 2.飞行器工作方式状态 =*/	
/*飞行器飞行任务*/
typedef enum
{	
	/*NULL*/
	UAV_FLY_MISSION_NULL                = 00,   /*没有任务*/
	
	/*一键*/
	UAV_FLY_MISSION_ONEKEY_FLY			= 01,	/*一键起飞*/
	UAV_FLY_MISSION_ONEKEY_LAND_HOME    = 02,	/*一键返航/降落*/
	
	/*姿态模式下*/
	UAV_FLY_MISSION_OPFLOW_FOLLOW_POINT = 06,	/*光流追点*/
	UAV_FLY_MISSION_OPFLOW_FOLLOW_LINE  = 07,	/*光流巡线*/
	
	/*定点模式下*/
	UAV_FLY_MISSION_GPS_WRITE_POS		= 10,   /*GPS写入当前点位*/
	UAV_FLY_MISSION_GPS_PATROL_SKY      = 11,	/*GPS巡天*/
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

/*任务中改变系统参数状态(退出任务时需要恢复)*/
typedef enum
{
	UAV_MISSION_CHANGE_SYSTEM_YES = 0, /*有改变*/
	UAV_MISSION_CHANGE_SYSTEM_NOT = 1, /*无改变*/
}UAV_MISSION_CHANGE_SYSTEM_STATUS;

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

typedef struct
{
	Uav_Mission_Status FixedHeightFly;
	Uav_Mission_Status LandHome;
}Uav_Onekey_Mission;

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
	
	/*一键任务*/
	Uav_Onekey_Mission 			     Onekey_Mission;
}Uav_Current_Fly_Mission;

/*竖直方向控制模式*/
typedef enum
{
	UAV_VERTICAL_CONTROL_SELFAUTO   = 0, /*纯姿态自稳*/
	UAV_VERTICAL_CONTROL_FIX_HEIGHT = 1, /*定高控制*/
}UAV_VERTICAL_CONTROL_MODE;

typedef struct
{
	UAV_VERTICAL_CONTROL_MODE ThisTime;
	UAV_VERTICAL_CONTROL_MODE LastTime;
}Uav_Vertical_Control_Mode_Switch;

/*水平方向控制模式*/
typedef enum
{
	UAV_HORIZONTAL_CONTROL_SELFAUTO = 0, /*纯姿态自稳*/
	UAV_HORIZONTAL_CONTROL_FIX_POS  = 1, /*定点控制*/
}UAV_HORIZONTAL_CONTROL_MODE;

typedef struct
{
	UAV_HORIZONTAL_CONTROL_MODE ThisTime;
	UAV_HORIZONTAL_CONTROL_MODE LastTime;
}Uav_Horizontal_Control_Mode_Switch;

/*状态切换后参考点设置状态*/
typedef enum
{
	UAV_SWITCH_REFERENCE_SET_NO = 0,	/*参考点未设置*/
	UAV_SWITCH_REFERENCE_SET_OK = 1,	/*参考点设置OK*/
}UAV_SWITCH_REFERENCE_SET_STATUS;

typedef struct
{
	Uav_Vertical_Control_Mode_Switch Mode_Switch;		   /*模式切换*/
	UAV_VERTICAL_CONTROL_MODE        EXPECT_CONTROL_MODE;  /*期望控制模式*/
	UAV_VERTICAL_CONTROL_MODE 		 CONTROL_MODE;         /*实际控制模式*/
	UAV_SWITCH_REFERENCE_SET_STATUS  REFERENCE_SET_STATUS; /*参考点设置状态*/
}Uav_Vertical_Control_Mode;

typedef struct
{
	Uav_Horizontal_Control_Mode_Switch Mode_Switch;			 /*模式切换*/
	UAV_HORIZONTAL_CONTROL_MODE        EXPECT_CONTROL_MODE;	 /*期望控制模式*/
	UAV_HORIZONTAL_CONTROL_MODE 	   CONTROL_MODE;		 /*实际控制模式*/
	UAV_SWITCH_REFERENCE_SET_STATUS    REFERENCE_SET_STATUS; /*参考点设置状态*/
}Uav_Horizontal_Control_Mode;

typedef struct
{
	Uav_Vertical_Control_Mode   Vertical;
	Uav_Horizontal_Control_Mode Horizontal;
}Uav_Control_Mode;

/*=== 传感器数据状态 ===*/
/*传感器/模块存在状态*/
typedef enum
{
	UAV_SENMOD_EXIST_NO = 0,	/*不存在*/
	UAV_SENMOD_EXIST_OK = 1,	/*存在*/
}UAV_SENMOD_EXIST_STATUS;

/*传感器/模块数据状态*/
typedef enum
{
	UAV_SENMOD_DATA_NO   = 0,		/*无效*/
	UAV_SENMOD_DATA_OK   = 1,		/*有效*/
}UAV_SENMOD_DATA_STATUS;

/*传感器/模块第一次使用状态*/
typedef enum
{
	UAV_SENMOD_FIRST_USE_AVA_NO = 0, /*无效*/	
	UAV_SENMOD_FIRST_USE_AVA_OK = 1, /*有效*/
}UAV_SENMOD_FIRST_USE_AVA_STATUS;

/*传感器/模块参考点设置状态*/
typedef enum
{
	UAV_SENMOD_ZERO_REFERENCE_SET_NO = 0, /*未设置*/	
	UAV_SENMOD_ZERO_REFERENCE_SET_OK = 1, /*已设置*/
}UAV_SENMOD_ZERO_REFERENCE_SET_STATUS;	

/*传感器/模块数据融合状态*/
typedef enum
{
	UAV_SENMOD_FUSION_NO = 0,	/*未成功*/	
	UAV_SENMOD_FUSION_OK = 1,	/*已成功*/
}UAV_SENMOD_FUSION_STATUS;

/*传感器/模块可用于控制状态*/
typedef enum
{
	UAV_SENMOD_USE_CONTROL_DISALLOW = 0, /*禁止*/
	UAV_SENMOD_USE_CONTROL_ALLOW    = 1, /*允许*/
}UAV_SENMOD_USE_CONTROL_STATUS;

/*不用数据融合的传感器状态*/
typedef struct
{
	UAV_SENMOD_EXIST_STATUS 			 EXIST_STATUS;					/*存在状态*/
	UAV_SENMOD_DATA_STATUS  			 DATA_STATUS;					/*数据状态*/
	UAV_SENMOD_ZERO_REFERENCE_SET_STATUS ZERO_REFERENCE_SET_STATUS;		/*零参考点设置状态*/
	UAV_SENMOD_FIRST_USE_AVA_STATUS      FIRST_USE_AVA_STATUS;			/*第一次使用有效状态*/
	UAV_SENMOD_USE_CONTROL_STATUS        USE_CONTROL_STATUS;			/*能否用于控制状态*/
}Uav_Nofusion_Senmod_Status;

/*需要数据融合的传感器状态*/
typedef struct
{
	UAV_SENMOD_EXIST_STATUS 			 EXIST_STATUS;					/*存在状态*/
	UAV_SENMOD_DATA_STATUS  			 DATA_STATUS;					/*数据状态*/
	UAV_SENMOD_ZERO_REFERENCE_SET_STATUS ZERO_REFERENCE_SET_STATUS;		/*零参考点设置状态*/
	UAV_SENMOD_FIRST_USE_AVA_STATUS      FIRST_USE_AVA_STATUS;			/*第一次使用有效状态*/
	UAV_SENMOD_USE_CONTROL_STATUS        USE_CONTROL_STATUS;			/*能否用于控制状态*/	
	UAV_SENMOD_FUSION_STATUS			 FUSION_STATUS;					/*融合状态*/
}Uav_Fusion_Senmod_Status;

/*当前使用传感器/模块类型*/
/*传感器工作形式*/
typedef enum
{
	UAV_SENMOD_WORK_LIMITLESS   = 0,	/*无限制传感器:如BERO、GPS(仅使用)*/
	UAV_SENMOD_WORK_AUTO_SWITCH = 1,	/*有限制传感器:如ULTR、OpticFlow*/
}UAV_SENMOD_WORK_STATUS;

/*当前使用的传感器/模块*/
typedef enum
{
	UAV_VERTICAL_SENMOD_CURRENT_NULL = 0,       /*NULL*/
	UAV_VERTICAL_SENMOD_CURRENT_BERO = 1,		/*BERO*/
	UAV_VERTICAL_SENMOD_CURRENT_ULTR = 2,	   	/*ULTR*/
}UAV_VERTICAL_SENMOD_CURRENT_USE;

typedef enum
{
	UAV_HORIZONTAL_SENMOD_CURRENT_NULL      = 0,    /*NULL*/	
	UAV_HORIZONTAL_SENMOD_CURRENT_GPS       = 1,	/*GPS*/
	UAV_HORIZONTAL_SENMOD_CURRENT_OPTICFLOW = 2,	/*光流*/
}UAV_HORIZONTAL_SENMOD_CURRENT_USE;

typedef struct
{
	UAV_SENMOD_WORK_STATUS          WORK_STATUS; /*工作状态*/     
	UAV_VERTICAL_SENMOD_CURRENT_USE LAST_USE;    /*上次使用*/
	UAV_VERTICAL_SENMOD_CURRENT_USE CURRENT_USE; /*当前使用*/	
	Uav_Nofusion_Senmod_Status      Bero;		 /*气压计*/
	Uav_Nofusion_Senmod_Status 	    Ultr;		 /*超声波*/
}Uav_Vertical_Senmod_Status;

typedef struct
{
	UAV_SENMOD_WORK_STATUS            WORK_STATUS; /*工作状态*/  	
	UAV_HORIZONTAL_SENMOD_CURRENT_USE LAST_USE;	   /*上次使用*/
	UAV_HORIZONTAL_SENMOD_CURRENT_USE CURRENT_USE; /*当前使用*/	
	Uav_Nofusion_Senmod_Status 		  Opticflow;   /*光流*/
	Uav_Fusion_Senmod_Status   		  Gps;		   /*GPS*/
}Uav_Horizontal_Senmod_Status;

typedef struct
{
	Uav_Vertical_Senmod_Status   Vertical;		/*竖直方向传感器*/
	Uav_Horizontal_Senmod_Status Horizontal;	/*水平方向传感器*/
}Uav_Senmod_Status;

/*= 3.遥控操作状态 =*/
typedef enum
{
	UAV_REMOT_OPERATE_NOT_UNLOCK           = 0, /*未解锁*/
	UAV_REMOT_OPERATE_SWITCH_BEFORE_UNLOCK = 1, /*拨键在解锁前*/
	UAV_REMOT_OPERATE_UNLOCK_BEFORE_SWITCH = 2, /*解锁在拨键前*/
}UAV_REMOT_OPERATE_STATUS;

/*= 4.程序运行状态 =*/
/*程序初始化运行状态*/
typedef enum
{
	UAV_PROGRAME_INIT_START  = 0, 	/*程序初始化开始运行*/
	UAV_PROGRAME_INIT_FINISH = 1,	/*程序初始化运行完毕*/
}UAV_PROGRAME_INIT_STATUS;

/*程序CPU使用率*/
typedef struct
{
	vu8	major;	/*CPU 使用率 整数*/
	vu8	minor;	/*CPU 使用率 小数*/	
}Uav_Programe_Cpu_Use;

typedef struct
{
	UAV_PROGRAME_INIT_STATUS INIT_STATUS;		/*程序初始化状态*/
	Uav_Programe_Cpu_Use 	 CpuUse;			/*CPU使用*/
}Uav_Programe_Status;

typedef struct
{
	/*= 1.飞行器本身的状态 =*/	               
	volatile UAV_LOCK_STATUS  		   	     LOCK_STATUS;				/*飞控锁定状态*/
	volatile Uav_Land_Status   		   	     UavLandStatus;				/*飞行器着陆状态*/
	volatile Uav_Fly_Type         	   	     UavFlyType;				/*飞行模式*/
	volatile UAV_HOME_SET_STATUS  		   	 HOME_SET_STATUS;		    /*GPS HOME 点设置状态*/
	volatile UAV_WIRELESS_CMC_STATUS   		 WIRELESS_CMC_STATUS;	    /*遥控和飞行器通信状态*/
	volatile UAV_HCI_SHOW_STATUS			 HCI_SHOW_STATUS;			/*HCI SHOW*/
										   
	/*= 2.飞行器工作方式状态 =*/	
	volatile Uav_Current_Fly_Mission		 UavCurrentFlyMission;		/*当前飞行任务*/
	volatile Uav_Control_Mode 				 UavControlMode;			/*飞行器控制模式*/
	volatile Uav_Senmod_Status				 UavSenmodStatus;			/*传感器/模块状态*/
	
	/*= 3.遥控操作状态 =*/
	volatile UAV_REMOT_OPERATE_STATUS		 REMOT_OPERATE_STATUS;		/*遥控操作状态*/
	
	/*= 4.程序运行状态 =*/
	volatile Uav_Programe_Status			 UavProgrameStatus;			/*程序运行状态*/
}Uav_Status;

extern Uav_Status g_sUav_Status;
extern Uav_Status *g_psUav_Status;

/*检测GPS是否可用(满足定位数据+数据融合成功)*/
UAV_SENMOD_USE_CONTROL_STATUS status_GPS_Fix_Ava_Check(Uav_Status *uavStatus);

/*=== 检测遥控和无人机通信情况 ===*/
#define CMC_DOG_FEED_FACTOR_10MS 	(1) /*10ms*/

/*遥控与飞机通讯状态*/
UAV_WIRELESS_CMC_STATUS status_check_aircraft_remot_communication(SimulateWatchDog *uavRemotCMCDog, Uav_Status *uavStatus);

/*遥控与飞机通讯正常喂狗*/
void security_Feed_CMC_Succ_Dog_10MS(u8 _10msFoc, SimulateWatchDog *uavRemotCMCDog);

/*安全*/
extern SimulateWatchDog g_sUavRemotCMCDog;
extern SimulateWatchDog *g_psUavRemotCMCDog;

#endif
