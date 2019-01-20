#ifndef _SINS_STRAPDOWN_H_
#define _SINS_STRAPDOWN_H_

#include "sys_Platform.h"
#include "ahrs_Caculation.h"
#include "attitude_Aircraft.h"
#include "period_Execute.h"
#include "sys_BoardMap.h"
#include "host_Transceive.h"
#include "attitude_Aircraft.h"

#define SINS_ACC_GRAVITY		(9.8f)

/*导航系*/
typedef enum
{
	EARTH_FRAME_X = 0,	/*导航系 X-lon-E*/
	EARTH_FRAME_Y = 1,	/*导航系 Y-lat-N*/
	EARTH_FRAME_Z = 2,  /*导航系 Z-het-U*/
}EARTH_FRAME_AXIS;

/*载体系*/
typedef enum
{
	BODY_FRAME_X = 0,	/*导航系 x*/
	BODY_FRAME_Y = 1,	/*导航系 y*/
	BODY_FRAME_Z = 2,   /*导航系 z*/
}BODY_FRAME_AXIS;

/*融合结果*/
typedef enum
{
	SINS_FUSION_FAIL = 0,
	SINS_FUSION_SUCC = 1,
}SINS_FUSION_STATUS;

/*融合对象*/
typedef enum
{
	SINS_FUSION_VERTICAL   = 0,	/*竖直*/
	SINS_FUSION_HORIZONTAL = 1,	/*水平*/
}SINS_FUSION_TARGET;

#define SINS_HISTORY_DATA_DEEP	(50)

/*捷联惯导*/
typedef struct
{
	fp32 			   curPosition[3];	 							/*位置估计量*/
	fp32 			   curSpeed[3];		 						    /*速度估计量*/
	fp32 			   curAcc[3]; 		 	   					   	/*加速度估计量*/
	fp32 			   pos_History[3][SINS_HISTORY_DATA_DEEP];	    /*历史惯导位置*/
	fp32 			   speed_History[3][SINS_HISTORY_DATA_DEEP];	/*历史惯导速度*/
    fp32 			   accOffset[3];							    /*惯导加速度漂移估计量*/
	fp32 			   lastAcc[3];			    					/*上次惯导加速度*/
	fp32 			   lastSpeed[3];								/*上次惯导速度*/
	fp32 		       estimatePos[3];								/*本次观测位置*/
	SINS_FUSION_STATUS FUSION_STATUS[2];							/*惯导融合状态*/
	
	u16				   sensorDataSync5ms[3];					    /*传感器数据同步cnt*/
}SINS;

/*三阶互补*/
typedef struct
{
	fp32 acc;	/*加速度*/
	fp32 speed;	/*速度*/
	fp32 pos;	/*位移*/
}TOCBackCorrect;

typedef struct
{
	fp32           estimateDealt[3];	/*估计(算)值与观测(传感器)值的差*/
	TOCBackCorrect BackCorrect[3];	    /*三路积分反馈修正量*/
	fp32 		   speedDealt[3];		/*速度增量*/
}TOCSystem;


/*机体系->导航系 加速度*/
void sins_get_body_relative_earth_acc(Acc3f *sinsAcc);

/*竖直方向,气压计和超声波切换(非独立融合)*/
void sins_vertical_bero_ultr_auto_change(Uav_Status *uavStatus);

/*水平方向,GPS和光流切换(独立融合)*/
void sins_horizontal_gps_opticflow_auto_change(Uav_Status *uavStatus);

/*三阶互补求竖直方向上的加速度、速度、位置(Z竖直)*/
void sins_thirdorder_complement_vertical(void);

/*卡尔曼滤波求竖直方向上的加速度、速度、位置(Z竖直)*/
void sins_kalman_estimate_vertical(void);

/*三阶互补求水平方向上的加速度、速度、位置(X,Y水平)*/
void sins_thirdorder_complement_horizontal(void);

/*卡尔曼滤波求水平方向上的加速度、速度、位置(X,Y水平)*/
void sins_kalman_estimate_horizontal(void);

/*捷联惯导对象重置:轴,位置,速度*/
void strapdown_ins_reset(SINS *sins, TOCSystem *tocSystem, EARTH_FRAME_AXIS AXIS, fp32 posTarg, fp32 speedTarg);



 /*捷联惯导:滤波后的惯导状态*/
extern SINS g_sSinsAfterFilter;	/*pitch、Roll数据在GPS异常时有效*/
extern SINS *g_psSinsAfterFilter;

/*捷联惯导:滤波后的惯导反馈量*/
extern SINS g_sSinsFilterFeedback;
extern SINS *g_psSinsFilterFeedback;

/*惯导中的加速步长*/
extern fp32 g_SinsAccLenth;


/*=== 捷联惯导 ===*/
extern SINS g_sSinsReal;
extern SINS *g_psSinsReal;

/*捷联惯导:原始的惯导状态*/
extern SINS g_sSinsOrigion;	/*( kalman+Strapdown)z(高度)、(Strapdown+kalman(GPS正常))x,y(水平正东、正北)*/
extern SINS *g_psSinsOrigion;

/*捷联惯导:滤波后的惯导反馈量*/
extern SINS g_sSinsFilterFeedback;
extern SINS *g_psSinsFilterFeedback;

/*=== 三阶互补 ===*/
/*竖直方向上*/
extern TOCSystem g_sTOCSystem;
extern TOCSystem *g_psTOCSystem;

#endif
