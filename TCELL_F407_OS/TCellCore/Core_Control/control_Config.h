#ifndef _CONTROL_CONFIG_H_
#define _CONTROL_CONFIG_H_

#include "sys_Platform.h"
#include "attitude_Aircraft.h"

/*======= 0.x,y,z方向速度/角度限制 =======*/
/*z方向速度限制*/
#define CTRL_HEIGHT_CLIMB_UP_MAX_SPEED     			   (150)         /*向上最大攀爬速度(150cm/s):600~1000*/
#define CTRL_HEIGHT_CLIMB_DOWN_MAX_SPEED    		   (80)          /*向下最大攀爬速度(150cm/s):0~400*/
/*z方向加速度限制*/
#define CTRL_HEIGHT_CLIMB_UP_MAX_ACCELERATION          (300)         /*向上最大攀爬加速度(300cm/s^2)*/
#define CTRL_HEIGHT_CLIMB_DOWN_MAX_ACCELERATION   	   (200)         /*向下最大攀爬加速度(150cm/s^2)*/

/*x,y方向,速度限制*/
#define CTRL_HORIZONTAL_MAX_MOVE_SPEED		           (6.0f) 		 /*水平方向最大期望速度30度*6/100=1.8m/s*/
/*x,y方向,角度限制*/
#define REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX              (45)			 /*遥控PITCH、ROLL最大期望行程角度*/
#define REMOT_YAW_ANGLE_EXPECT_MAX  	               (150)		 /*遥控YAW最大期望行程:+150~-150*/
#define GIMBAL_PITCH_ANGLE_EXPECT_MAX              	   (90)			 /*万向节云台PITCH、ROLL最大期望行程:+90~-90*/
#define GIMBAL_YAW_ANGLE_EXPECT_MAX  	               (150)		 /*万向节云台YAW最大期望行程:+150~-150*/
#define REMOT_ANGLE_DEADZONE_EXCEPT		               (450)		 /*1000~1450, 1550~2000,死区之外的有效范围:450*/

/*x,y方向,角度过大保护,自动关闭电机*/
#define CTRL_HORIZONTAL_ANGLE_OVER_CHECK			   (SYS_ENABLE)
#define CTRL_HORIZONTAL_ANGLE_SAFE_MAX				   (60)			 /*最大允许倾斜角度*/

/*======= 1.Altitude Hold(竖直定高) =======*/
#define CTRL_HEIGHT_YAW_SPEED_FEEDFORWARD			   (SYS_ENABLE)  /*竖直速度前馈控制器*/
#define CTRL_HEIGHT_POS_CONTROL_ACC_STATUS             (SYS_ENABLE)  /*开启三环定高模式:即竖直高度位置+竖直速度+竖直加速度:SYS_ENABLE(1)开启,SYS_DISABLE(0)关闭*/
#define CTRL_HEIGHT_ONEKEY_TAKEOFF_HEIGHT			   (150)		 /*一键起飞离地高度(cm),此值应在超声波范围(200cm)内*/


/*======= 2.Position Hold(水平定点) =======*/
#define CTRL_HORIZONTAL_SENSOR_MODE_REMOT_EXPECT_ANGLE (SYS_DISABLE) /*水平定点模式下，打杆直接给姿态期望角*/
#define CTRL_HORIZONTAL_SENSOR_MODE_REMOT_EXPECT_SPEED (SYS_ENABLE)  /*水平定点模式下，打杆直接给姿态期望速度*/

/*======= 3.怠速,起飞,着陆检测油门 =======*/
/*怠速*/
#define CTRL_IDEL_TRANSITION_CONTINUE_MAX_TICKS        (100)  		 /*怠速启动最大计数器,5ms*100=500ms*/
#define CTRL_IDEL_TRANSITION_INC_PERIOD_TICKS          (2)   		 /*怠速递增间隔时间,10*5=50ms*/
#define CTRL_IDEL_THROTTLE_VALUE			           (1150) 		 /*油门怠速,取接近起转油门值即可*/

/*起飞*/
#define CTRL_THROTTLE_START_TURN  				       (1100) 		 /*起转油门量，油门倾角补偿用，太大会导致过补偿*/
#define CTRL_THROTTLE_START_FLY_VALUE  			       (1250) 		 /*起飞油门量*/

/*悬停油门*/
#define CTRL_THROTTLE_HOVER_ENABLE_STATUS			   (SYS_ENABLE)  /*默认悬停油门*/
#define CTRL_THROTTLE_HOVER_DEFAULT_VALUE			   (1450) 		 /*默认悬停油门*/


/*着陆*/
#define CTRL_LAND_CHECK_THROTTLE_THRESHOLD_MIN		   (1150) 		 /*着陆检测油门最小值*/

/*======= 4.控制系统各环控制周期(ms) taskPeriod * cnt =======*/
/*控制周期时基: PLATFORM_TASK_SCHEDULER_MIN_FOC_MS*/
/*高度控制周期*/
#define CTRL_VERTICAL_POSITION_CONTROL_PERIOD	       (2)	/*竖直位置控制周期(10ms)*/
#define CTRL_VERTICAL_SPEED_CONTROL_PERIOD			   (1)	/*竖直速度控制周期(5ms)*/
/*水平控制周期_GPS*/
#define CTRL_GPS_HORIZONTAL_POSITION_CONTROL_PERIOD	   (4)	/*水平位置控制周期(20ms)*/
#define CTRL_GPS_HORIZONTAL_SPEED_CONTROL_PERIOD	   (2)	/*水平速度控制周期(10ms)*/
/*水平控制周期_OpticFlow*/
#define CTRL_OPFLOW_HORIZONTAL_POSITION_CONTROL_PERIOD (8)	/*水平位置控制周期(40ms)*/
#define CTRL_OPFLOW_HORIZONTAL_SPEED_CONTROL_PERIOD	   (4)	/*水平速度控制周期(20ms)*/


/*======= 其它 =======*/
#define CTRL_THROTTLE_TOP_VALUE						   (1000) /*最大油门值*/
#define CTRL_THROTTLE_BOTTOM_VALUE					   (0)    /*最小油门值*/ 
#define REMOT_THROTTLE_DOWN_CROSSOVER	               (500 - REMOT_THROTTLE_MID_DEADZONE * 0.5)	/*油门杆给定期望速度时,下行程临界值*/
#define REMOT_THROTTLE_UP_CROSSOVER	   	               (500 + REMOT_THROTTLE_MID_DEADZONE * 0.5)	/*油门杆给定期望速度时,上行程临界值*/
#define REMOT_ANGLE_DEADZONE_BUTTOM 	               (REMOT_ANGLE_MID - REMOT_ANGLE_MID_DEADZONE * 0.5) /*1450*/
#define REMOT_ANGLE_DEADZONE_TOP    	               (REMOT_ANGLE_MID + REMOT_ANGLE_MID_DEADZONE * 0.5) /*1550*/

#include "control_AltHold.h"	/*定高*/
#include "control_PosHold.h"    /*定点*/
#include "control_Landing.h"    /*降落/返航*/
#include "control_Mission.h" 	/*飞行任务*/
#include "sins_Strapdown.h"		/*捷联惯导数据融合*/

#include "sys_ControlMap.h"

#ifdef CONTROL_SYS__PID
	#include "pid_System.h"
	/*PID控制算法*/
/*config*/
	#define PID_SYSTEM_CONFIG_USE_AVERAGE_FILTER		(SYS_DISABLE) /*均值滤波*/
#endif

#ifdef CONTROL_SYS__ADRC
	#include "adrc_System.h"	
	/*ADRC控制算法*/
#endif

#endif
