#ifndef _CONTROL_LANDING_H_
#define _CONTROL_LANDING_H_

#include "sys_Platform.h"
#include "control_Config.h"
#include "status_Aircraft.h"

/*返航点距离HOME点水平距离分级*/
#define CTRL_AUTO_GO_HOME_FARAWAY_DISTANCE_CM				(600)	/*距离home很远,计为A区域 cm*/	
#define CTRL_AUTO_GO_HOME_NEAR_DISTANCE_CM					(150)	/*距离home较远,计为B区域 cm*/	
#define CTRL_AUTO_GO_HOME_ARRIVE_DISTANCE_CM				(0)		/*到达HOME点,计为C区域 cm*/
	
/*根据返航点距离HOME点水平距离,来决定返航速度*/	
#define CTRL_AUTO_GO_HOME_A2B_FIRST_NAV_SPEED_CM_S			(100)	/*A->B,一级巡航速度 cm/s*/
#define CTRL_AUTO_GO_HOME_B2C_SECOND_NAV_SPEED_CM_S			(80)	/*B->C,二级巡航速度 cm/s*/
#define CTRL_AUTO_GO_HOME_NEARC_THIRD_NAV_SPEED_CM_S		(60)	/*C附近,三级巡航速度 cm/s*/
#define CTRL_AUTO_GO_HOME_DEFAULT_NAV_SPEED_CM_S			(50)	/*默认巡航速度 cm/s*/

/*根据返航点距离HOME点水平距离,来决定返航安全高度和速度*/
#define CTRL_AUTO_GO_HOME_FARAWAY_DISTANCE_SAFE_HEIGHT_CM	(800)	/*返航点距离HOME较远时的安全高度 cm*/
#define CTRL_AUTO_GO_HOME_FARAWAY_DISTANCE_CLIMB_CM_S		(80)	/*返航点距离HOME较远时的且小于安全高度,原地上升速度 cm*/
#define CTRL_AUTO_GO_HOME_ARRIVE_DESCEND_CM_S				(-40)	/*返航在HOME点上方时,上降速度 cm*/
#define CTRL_AUTO_GO_HOME_RAPID_DESCEND_CM_S				(-80)	/*快速下降至地面时的速度 cm*/

/*上升至安全高度再水平靠近HOME点的过渡时间*/
#define CTRL_AUTO_GO_HOME_CLIMB_THEN_CLOSE_TRANSITION_TICKS	(400)  	/*400*5 = 2s*/

/*竖直速度变化周期*/
#define CTRL_AUTO_GO_HOME_VERTICAL_SPEED_CHANGE_MAX_PERIOD	(8)		/*8*5 = 40ms*/

/*返航流程*/
typedef enum
{
	CTRL_GO_HOME_NULL_PROCESS   			    = 0,	/*无效枚举*/	
	CTRL_LAND_FARAWAY_CLIMB_SAFE_HEIGHT_GO_HOME = 1,	/*远离HOME点,先攀升到安全高度,返航*/
	CTRL_LAND_NEAR_HOLD_HEIGHT_GO_HOME 			= 2,	/*靠近HOME点,保持安全高度,返航*/	
	CTRL_LAND_ARRIVE_DESCEND_GO_HOME 			= 3,	/*HOME点附近,降落*/
}CTRL_GO_HOME_PROCESS;

/*返航点距离HOME点距离标定*/
typedef enum
{
	CTRL_GO_HOME_NULL_DISTANCE    = 0,	/*无效枚举*/
	CTRL_GO_HOME_FARAWAY_DISTANCE = 1,	/*返航点在HOME点远距离*/
	CTRL_GO_HOME_NEAR_DISTANCE 	  = 2,	/*返航点在HOME点近距离*/	
	CTRL_GO_HOME_ARRIVE_DISTANCE  = 2,	/*返航点靠近HOME点*/
}CTRL_GO_HOME_DISTANCE;

/*返航时是否手动干预*/
typedef enum
{
	CTRL_GO_HOME_HAND_MEDDLE_TRUE  = 0,	/*有手动干预*/
	CTRL_GO_HOME_HAND_MEDDLE_FALSE = 1,	/*无手动干预*/	
}CTRL_GO_HOME_HAND_MEDDLE_STATUS;

/*返航时竖直高度和水平位移控制模式*/
typedef enum
{	
	CTRL_GO_HOME_NULL_CTRL = 0,	/*无效枚举*/
	CTRL_GO_HOME_AUTO_CTRL = 1,	/*自动控制模式*/
	CTRL_GO_HOME_HAND_CTRL = 2,	/*手动控制模式*/
}CTRL_GO_HOME_CTRL_MODE;

/*是否为第一次(无手动干涉/消除手动干涉后)切返航任务*/
typedef enum
{
	CTRL_GO_HOME_FIRST_SWITCH_TURE  = 1, /*第一次切返航*/
	CTRL_GO_HOME_FIRST_SWITCH_FALSE = 2, /*非第一次切返航*/
}CTRL_GO_HOME_FIRST_SWITCH_STATUS;

/*返航是否达到安全高度*/
typedef enum
{
	CTRL_GO_HOME_SAFE_HEIGHT_NULL	   = 0,	/*该值安全高度已达到,并已设置相关状态,防止重新设置*/
	CTRL_GO_HOME_SAFE_HEIGHT_DISARRIVE = 1,	/*未达安全高度*/	
	CTRL_GO_HOME_SAFE_HEIGHT_ARRIVE    = 2,	/*已达安全高度*/
}CTRL_GO_HOME_SAFE_HEIGHT_STATUS;

/*过渡状态设置状态*/
typedef enum
{
	CTRL_GO_GOME_TRANSITION_SET_NO = 1,	/*过渡状态未设置设置*/	
	CTRL_GO_GOME_TRANSITION_SET_OK = 2,	/*过渡状态已设置*/
}CTRL_GO_GOME_TRANSITION_SET_STATUS;

/*着陆检测是否使能*/
typedef enum
{
	CTRL_LAND_CHECK_ENABLE  = 0, /*着陆检测使能*/
	CTRL_LAND_CHECK_DISABLE = 1, /*着陆检测失能*/
}CTRL_LAND_CHECK_STATUS;

typedef struct
{
	vu16								  	    land_throttle_min_check;	    /*着陆油门检测值*/
	volatile CTRL_LAND_CHECK_STATUS             LAND_CHECK_STATUS;    	        /*是否允许着陆检测*/	
	
	volatile CTRL_GO_HOME_DISTANCE 			    HOME_DISTANCE;					/*离HOME点距离标定*/	
	volatile CTRL_GO_HOME_PROCESS  			    LAST_GO_HOME_PROCESS;			/*上次返航流程*/
	volatile CTRL_GO_HOME_PROCESS  			    CUR_GO_HOME_PROCESS;			/*本次返航流程*/	
	volatile CTRL_GO_HOME_HAND_MEDDLE_STATUS    VERTICAL_HAND_MEDDLE_STATUS;	/*高度手动干预状态*/
	volatile CTRL_GO_HOME_HAND_MEDDLE_STATUS    HORIZONTAL_HAND_MEDDLE_STATUS;	/*水平手动干预状态*/	
	volatile CTRL_GO_HOME_CTRL_MODE			    LAST_HORIZONTAL_CTRL_MODE;		/*上次水平控制模式*/
	volatile CTRL_GO_HOME_CTRL_MODE			    CUR_HORIZONTAL_CTRL_MODE;		/*本次水平控制模式*/
	volatile CTRL_GO_HOME_FIRST_SWITCH_STATUS   FIRST_SWITCH_STATUS;			/*是否为第一次切返航模式*/	
	volatile CTRL_GO_HOME_SAFE_HEIGHT_STATUS    SAFE_HEIGHT_STATUS;				/*安全高度状态*/
	volatile CTRL_GO_GOME_TRANSITION_SET_STATUS TRANSITION_SET_STATUS;			/*过渡状态设置状态*/
	Vector2s_Nav 							    LandHomePosition;				/*家位置水平坐标*/
	Vector2s_Nav							  	LandCurPosition;				/*当前水平坐标*/	
	Vector2s_Nav 							  	LandTargetPosition;				/*过渡水平座标*/
	fp32 									  	heightExpect;					/*目标竖直高度期望*/
	fp32 									  	verticalSpeedExpect;			/*目标竖直攀升/下降速度期望*/
	vu8											verticalCalimbSpeedRate;		/*竖直攀升速度变化量*/
	vs8											verticalDescendSpeedRate;		/*竖直下降速度变化量*/	
	vu8										    verticalSpeedChangePeriod;		/*竖直速度变化周期*/
	u8											horizontalSpeedExpect;			/*水平速度期望*/
	vu16									  	climb2CloseTransitionTicks;		/*从先爬升到水平靠拢过渡时间*/
	Vector2f_Nav								RelativeHomeDistance;			/*当前点距离HOME点水平距离*/
}ControlLand;


/*触地状态检测*/
UAV_LAND_STATUS ctrl_Landing_Status_Check(Uav_Status *uavStatus);

/*检测返航时手动干预高度*/
CTRL_GO_HOME_HAND_MEDDLE_STATUS ctrl_Go_Home_Vertical_Hand_Meddle(void);

/*检测返航时手动干预水平*/
CTRL_GO_HOME_HAND_MEDDLE_STATUS ctrl_Go_Home_Horizontal_Hand_Meddle(void);

/*返航点状态更新*/
void ctrl_Go_Home_Status_Update(void);

/*返航控制*/
void ctrl_Go_Home_Control(fp32 controlDeltaT);

/*返航竖直高度控制*/
void ctrl_Go_Home_Vertical_Control(fp32 verticalSpeedExpect, fp32 heightExpect, fp32 controlDeltaT);

/*返航水平位置控制*/
void ctrl_Go_Home_Horizontal_Control(Vector2s_Nav targPos, Vector2s_Nav curPos, CTRL_GO_HOME_DISTANCE HOME_DISTANCE);

/*返航控制GPS*/
void ctrl_Go_Home_GPS_Control(fp32 controlDeltaT);

/*普通原地着陆控制(NO GPS)*/
void ctrl_Land_Ground_Control(fp32 controlDeltaT, Uav_Status *uavStatus);

/*得到相对目标点机体Pit、Rol方向偏移*/
Vector2f_Nav land_Gps_Offset_Relative_To_Home(Vector2s_Nav targPos, Vector2s_Nav curPos);

/*着陆返航控制*/
extern ControlLand g_sControlLand;

extern ControlLand *g_psControlLand;

#endif
