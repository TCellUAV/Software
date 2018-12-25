#ifndef _STATUS_AIRCRAFT_H_
#define _STATUS_AIRCRAFT_H_

#include "sys_Platform.h"
#include "bsp_BoardLib.h"

#define AIRCRAFT_LOCK_CONTINU_TIME_MS		(100)	 /*动作持续100ms(0.1s)以上锁定*/
#define AIRCRAFT_UNLOCK_CONTINU_TIME_MS		(2000)	 /*动作持续2000ms(2s)以上解锁*/
#define AIRCRAFT_AUTOLOCK_CONTINU_TIME_MS	(15000)	 /*未飞前未操作持续15000ms(15s)自动上锁*/

/*=== 飞行器本身的状态 ===*/
/*飞行器锁定状态*/
typedef enum
{
	AIRCRAFT_LOCKING  = 0,	/*已锁定*/
	AIRCRAFT_UNLOCK   = 1,	/*未锁定(已解锁)*/		
}AIRCRAFT_LOCK_STATUS;

/*飞行器飞行状态*/
typedef enum
{
	AIRCRAFT_LANDING = 1,	/*飞行器着陆中*/
	AIRCRAFT_FLYING  = 2,	/*飞行器飞行中*/
}AIRCRAFT_FLY_STATUS;

/*飞行器飞行模式(匿名上位机)*/
typedef enum
{
	AIRCRAFT_FLY_TYPE_ATTITUDE     = 1,  /*纯姿态飞行*/
	AIRCRAFT_FLY_TYPE_FIX_HEIGHT   = 2,  /*定高(竖直)*/	
	AIRCRAFT_FLY_TYPE_FIX_POS      = 3,  /*定点(竖直+水平)*/
	AIRCRAFT_FLY_TYPE_PATROL_LINE  = 11, /*巡线(航线)*/
	AIRCRAFT_FLY_TYPE_FALL_ALIGHT  = 20, /*降落(≠返航)*/
	AIRCRAFT_FLY_TYPE_GO_HOME      = 21, /*返航*/
}AIRCRAFT_FLY_TYPE;

/*一键控制状态*/
typedef enum
{
	AIRCRAFT_ONEKEY_CTRL_DISABLE = 0,	/*一键控制无效*/	
	AIRCRAFT_ONEKEY_CTRL_ENABLE  = 1,	/*一键控制有效*/
}AIRCRAFT_ONEKEY_CTRL_STATUS;

/*一键控制目标点设置状态*/
typedef enum
{
	AIRCRAFT_ONEKEY_TARGET_SET_NO = 0, /*一键控制目标点未设置*/
	AIRCRAFT_ONEKEY_TARGET_SET_OK = 1, /*一键控制目标点已设置*/
}AIRCRAFT_ONEKEY_TARGET_SET_STATUS;	

/*一键控制目标点到达状态*/
typedef enum
{
	AIRCRAFT_ONEKEY_TARGET_REACH_NO = 0, /*一键控制目标点未到达*/
	AIRCRAFT_ONEKEY_TARGET_REACH_OK = 1, /*一键控制目标点已到达*/
}AIRCRAFT_ONEKEY_TARGET_REACH_STATUS;	

/*Home点设置状态*/
typedef enum
{
	AIRCRAFT_HOME_SET    = 1, /*飞行器Home点已设置*/
	AIRCRAFT_HOME_NOTSET = 2, /*飞行器Home点未设置*/
}AIRCRAFT_HOME_STATUS;

/*遥控和飞行器通信状态*/
typedef enum
{
	AIRCRAFT_CMC_SUCC = 0,	/*遥控和飞行器通信成功*/
	AIRCRAFT_CMC_FAIL = 1,	/*遥控和飞行器通信失败*/	
}AIRCRAFT_CMC_STATUS;

/*人机交互-OLED显示页面状态*/
typedef enum
{
	/*页面显示*/
	AIRCRAFT_HCI_SHOW_HOLD_PAGE = 0,	/*OLED锁定当前页面*/
	AIRCRAFT_HCI_SHOW_FREE_PAGE = 1,	/*解除当前页锁定*/
	AIRCRAFT_HCI_SHOW_LAST_PAGE = 2,	/*OLED显示上一个页面*/
	AIRCRAFT_HCI_SHOW_NEXT_PAGE = 3,	/*OLED显示下一个页面*/
	
	/*显示开关*/
	AIRCRAFT_HCI_SHOW_ENABLE    = 7,	/*允许显示*/
	AIRCRAFT_HCI_SHOW_DISABLE   = 8,	/*不允许显示*/
}AIRCRAFT_HCI_SHOW_STATUS;

/*=== 工作方式状态 ===*/
/*定高工作状态*/
typedef enum
{
	AIRCRAFT_HEIGHT_BERO = 1,	/*气压计定高状态*/
	AIRCRAFT_HEIGHT_ULTR = 2,	/*超声波定高状态*/
}AIRCRAFT_FIX_HEIGHT; 

/*定点工作状态*/
typedef enum
{
	AIRCRAFT_POINT_GPS    = 1,  /*GPS定点*/
	AIRCRAFT_POINT_OPFLOW = 2,	/*光流定点*/
}AIRCRAFT_FIX_POINT;

/*控制模式*/
typedef enum
{
	AIRCRAFT_CONTROL_AUTO             = 1,	/*纯姿态自稳*/	
	AIRCRAFT_CONTROL_SENSOR           = 2,	/*传感器数据参与控制*/	
	AIRCRAFT_CONTROL_GPS_SENSOR       = 5,	/*传感器 GPS 数据参与控制*/	
	AIRCRAFT_CONTROL_OPTICFLOW_SENSOR = 6,	/*传感器 光流 数据参与控制*/		
	
	AIRCRAFT_CONTROL_SENSOR_TO_AUTO   = 7,	/*传感器参与定高定水平切自稳*/
	AIRCRAFT_CONTROL_AUTO_TO_SENSOR   = 8,	/*自稳切传感器参与定高定水平*/
	AIRCRAFT_CONTROL_KEEP_CUR_MODE    = 9,	/*保持当前模式*/
}AIRCRAFT_CONTROL_MODE;

/*参考点设置状态*/
typedef enum
{
	AIRCRAFT_REFERENCE_SET_NO = 0,	/*参考点未设置*/		
	AIRCRAFT_REFERENCE_SET_OK = 1,	/*参考点设置OK*/
}AIRCRAFT_REFERENCE_SET_STATUS;

/*=== 姿态数据状态 ===*/
/*高度数据状态*/
typedef enum
{
	HEIGHT_DATA_STATUS_OK   = 0,		/*有效*/
	HEIGHT_DATA_STATUS_NO   = 1,		/*无效*/
}HEIGHT_DATA_STATUS;

/*水平数据状态*/
typedef enum
{
	HORIZONTAL_DATA_STATUS_OK   = 0,	/*有效*/
	HORIZONTAL_DATA_STATUS_NO   = 1,	/*无效*/
}HORIZONTAL_DATA_STATUS;

/*高度零参考点设置状态*/
typedef enum
{
	/*单个传感器0参考点设置状态*/
	HEIGHT_BERO_ZERO_AVA         = 0x01,	/*气压计0参考点有效(xx01)-> (| 0001)*/
	HEIGHT_BERO_ZERO_DISAVA      = 0x0E,	/*气压计0参考点无效(xx10)-> (& 1110)*/
	HEIGHT_ULTR_ZERO_AVA         = 0x04,	/*超声波0参考点有效(01xx)-> (| 0100)*/
	HEIGHT_ULTR_ZERO_DISAVA      = 0x0B,	/*超声波0参考点无效(10xx)-> (& 1011)*/
	
	/*综合有效状态*/
	HEIGHT_BERO_ULTR_ZERO_DISAVA = 0x00, 	/*气压计和超声波都无效(0000)*/
	HEIGHT_BERO_ONLY_ZERO_AVA    = 0x01,	/*气压计有效,超声波无效(0001)*/	
	HEIGHT_ULTR_ONLY_ZERO_AVA    = 0x04,	/*气压计无效,超声波有效(0100)*/
	HEIGHT_BERO_ULTR_ZERO_AVA    = 0x05,	/*气压计和超声波都有效(0101)*/	
}HEIGHT_ZERO_SET_STATUS;

/*水平零参考点设置状态*/
typedef enum
{
	/*单个传感器0参考点设置状态*/
	HORIZONTAL_GPS_ZERO_AVA           = 0x01,	/*GPS水平0参考点有效(xx01)-> (| 0001)*/
	HORIZONTAL_GPS_ZERO_DISAVA        = 0x0E,	/*GPS水平0参考点无效(xx10)-> (& 1110)*/
	HORIZONTAL_OPFLOW_ZERO_AVA        = 0x04,	/*光流水平0参考点有效(01xx)-> (| 0100)*/
	HORIZONTAL_OPFLOW_ZERO_DISAVA     = 0x0B,	/*光流水平0参考点无效(10xx)-> (& 1011)*/
	
	/*综合有效状态*/
	HORIZONTAL_GPS_OPFLOW_ZERO_DISAVA = 0x00,  	/*GPS和光流都无效(0000)*/
	HORIZONTAL_GPS_ONLY_ZERO_AVA      = 0x01,	/*GPS有效,光流无效(0001)*/	
	HORIZONTAL_OPFLOW_ONLY_ZERO_AVA   = 0x04,	/*GPS无效,光流有效(0100)*/
	HORIZONTAL_GPS_OPFLOW_ZERO_AVA    = 0x05,	/*GPS和光流都有效(0101)*/
}HORIZONTAL_ZERO_SET_STATUS;

/*=== 高度&水平传感器有效性状态 ===*/
/*高度*/
typedef enum
{
	/*单个传感器状态*/
	HEIGHT_BERO_SENSOR_AVA    = 0x01,	/*气压计有效(xx01)-> (| 0001)*/
	HEIGHT_BERO_SENSOR_DISAVA = 0x0E,	/*气压计无效(xx10)-> (& 1110)*/
	HEIGHT_ULTR_SENSOR_AVA    = 0x04,	/*超声波有效(01xx)-> (| 0100)*/
	HEIGHT_ULTR_SENSOR_DISAVA = 0x0B,	/*超声波无效(10xx)-> (& 1011)*/
	
	/*综合有效状态*/
	HEIGHT_BERO_ULTR_DISAVA   = 0x00,	/*气压计和超声波都无效(0000)*/
	HEIGHT_BERO_ONLY_AVA      = 0x01,	/*气压计有效,超声波无效(0001)*/	
	HEIGHT_ULTR_ONLY_AVA      = 0x04,	/*气压计无效,超声波有效(0100)*/
	HEIGHT_BERO_ULTR_AVA      = 0x05,	/*气压计和超声波都有效(0101)*/	
}HEIGHT_SENSOR_AVA_STATUS;

/*水平*/
typedef enum
{
	/*单个传感器状态*/
	HORIZONTAL_GPS_SENSOR_AVA       = 0x01,	/*GPS有效(xx01)-> (| 0001)*/
	HORIZONTAL_GPS_SENSOR_DISAVA    = 0x0E,	/*GPS无效(xx10)-> (& 1110)*/
	HORIZONTAL_OPFLOW_SENSOR_AVA    = 0x04,	/*光流有效(01xx)-> (| 0100)*/
	HORIZONTAL_OPFLOW_SENSOR_DISAVA = 0x0B,	/*光流无效(10xx)-> (& 1011)*/
	
	/*综合有效状态*/
	HORIZONTAL_GPS_OPFLOW_DISAVA    = 0x00,	/*GPS和光流都无效(0000)*/
	HORIZONTAL_GPS_ONLY_AVA         = 0x01,	/*GPS有效,光流无效(0001)*/	
	HORIZONTAL_OPFLOW_ONLY_AVA      = 0x04,	/*GPS无效,光流波有效(0100)*/
	HORIZONTAL_GPS_OPFLOW_AVA       = 0x05,	/*GPS和光流都有效(0101)*/
}HORIZONTAL_SENSOR_AVA_STATUS;


/*GPS数据第一次使用有效状态*/
typedef enum
{	
	GPS_DATA_FIRST_DISAVA = 0,	/*第一次使用时无效标记*/
	GPS_DATA_FIRST_AVA    = 1,	/*第一次使用时有效标记(用于HOME点设置)*/
}GPS_DATA_FIRST_AVA_STATUS;

/*GPS信号质量,依据:定位卫星个数,水平精度因子,定位模式*/
typedef enum
{
	GPS_DATA_QUALITY_BAD      = 0,  /*信号很差*/
	GPS_DATA_QUALITY_MEDIUM   = 1,	/*信号中等*/
	GPS_DATA_QUALITY_GOOD     = 2,	/*信号很好*/
	
	/*卫星数量单项*/
	GPS_DATA_SATENBR_BAD      = 3,  
	GPS_DATA_SATENBR_MEDIUM   = 4,
	GPS_DATA_SATENBR_GOOD     = 5,	

	/*位置精度单项*/
	GPS_DATA_PDOP_BAD         = 6,
	GPS_DATA_PDOP_MEDIUM      = 7,
	GPS_DATA_PDOP_GOOD        = 8,	

	/*定位模式单项*/
	GPS_DATA_FIXTYPE_BAD      = 9,
	GPS_DATA_FIXTYPE_MEDIUM   = 10,
	GPS_DATA_FIXTYPE_GOOD     = 11,	
}GPS_DATA_QUALITY_STATUS;

/*定位数据满足条件+数据融合成功*/
typedef enum
{
	GPS_POS_FIX_AVA_TRUE  = 0,
	GPS_POS_FIX_AVA_FALSE = 1,
}GPS_POS_FIX_AVA_STATUS;

/*竖直方向气压计和超声波切换*/
typedef enum
{
	VERTICAL_SENSOR_USE_BERO = 0,
	VERTICAL_SENSOR_USE_ULTR = 1,	
	VERTICAL_SENSOR_USE_NULL = 0xff,	
}VERTICAL_SENSOR_AUTO_CHANGE;

/*程序初始化运行状态*/
typedef enum
{
	PROGRAME_INIT_START  = 0, 	/*程序初始化开始运行*/
	PROGRAME_INIT_FINISH = 1,	/*程序初始化运行完毕*/
}PROGRAME_INIT_STATUS;

typedef struct
{
	/*= 1.飞行器本身的状态 =*/	               
	volatile AIRCRAFT_LOCK_STATUS  		   	     LOCK_STATUS;				/*飞控锁定状态*/
	volatile AIRCRAFT_FLY_STATUS   		   	     CUR_FLY_STATUS;				/*飞行器飞行状态*/
	volatile AIRCRAFT_FLY_STATUS   		   	     LAST_FLY_STATUS;			/*上次飞行器飞行状态*/	
	volatile AIRCRAFT_FLY_TYPE         	   	     LAST_FLY_TYPE;				/*上次飞行模式(匿名上位机)*/
	volatile AIRCRAFT_FLY_TYPE         	   	     CUR_FLY_TYPE;				/*本次飞行模式(匿名上位机)*/
	volatile AIRCRAFT_HOME_STATUS  		   	     HOME_STATUS;				/*GPS HOME 点设置状态*/
	volatile AIRCRAFT_CMC_STATUS   		   	     CMC_STATUS;			    /*遥控和飞行器通信状态*/
	volatile AIRCRAFT_HCI_SHOW_STATUS	   	     HCI_SHOW_PAGE;			    /*人机交互显示页面*/
										   
	/*= 2.飞行器工作方式状态 =*/		       
	volatile AIRCRAFT_CONTROL_MODE 		   	     CUR_HEIGHT_CONTROL_MODE;			   /*记录当前遥控期望高度控制模式*/
	volatile AIRCRAFT_CONTROL_MODE 		   	     LAST_HEIGHT_CONTROL_MODE;			   /*记录上次遥控期望高度控制模式*/	
	volatile AIRCRAFT_CONTROL_MODE 		   	     CUR_HORIZONTAL_CONTROL_MODE; 		   /*记录当前遥控期望水平控制模式*/
	volatile AIRCRAFT_CONTROL_MODE 		   	     LAST_HORIZONTAL_CONTROL_MODE;		   /*记录上次遥控期望水平控制模式*/	
	volatile AIRCRAFT_CONTROL_MODE		   	     HEIGHT_REMOT_EXPECT_CONTROL_MODE;	   /*遥控期望高度控制模式*/
	volatile AIRCRAFT_CONTROL_MODE		   	     HORIZONTAL_REMOT_EXPECT_CONTROL_MODE; /*遥控期望水平控制模式*/	
	volatile AIRCRAFT_CONTROL_MODE		   	     HEIGHT_CONTROL_MODE;				   /*实际高度控制模式*/
	volatile AIRCRAFT_CONTROL_MODE		   	     HORIZONTAL_CONTROL_MODE;			   /*实际水平控制模式*/
	volatile AIRCRAFT_REFERENCE_SET_STATUS 	     HEIGHT_REFERENCE_SET_STATUS;		   /*高度控制过程中参考点设置状态*/
	volatile AIRCRAFT_REFERENCE_SET_STATUS 	     HORIZONTAL_REFERENCE_SET_STATUS;	   /*水平控制过程中参考点设置状态*/
	volatile AIRCRAFT_ONEKEY_CTRL_STATUS   	     ONEKEY_TAKEOFF_CONTROL_STATUS;		   /*一键起飞控制状态*/
	volatile AIRCRAFT_ONEKEY_TARGET_SET_STATUS   ONEKEY_TAKEOFF_TARGET_SET_STATUS;	   /*一键起飞目标点设置状态*/
	volatile AIRCRAFT_ONEKEY_TARGET_REACH_STATUS ONEKEY_TAKEOFF_TARGET_REACH_STATUS;   /*一键起飞目标点到达状态*/	
	volatile AIRCRAFT_ONEKEY_CTRL_STATUS         ONEKEY_LAND_CONTROL_STATUS;		   /*一键降落控制状态*/	
	volatile AIRCRAFT_ONEKEY_CTRL_STATUS         ONEKEY_CRUISE_CONTROL_STATUS; 		   /*一键巡航控制状态*/
	
	/*= 3.姿态数据状态 =*/
	/*高度*/
	volatile HEIGHT_DATA_STATUS  		         BERO_ZERO_PRESSURE;		/*气压计零参考点气压值状态*/
	volatile HEIGHT_DATA_STATUS  		         BERO_PRESSURE;				/*气压计当前气压值状态*/
	volatile HEIGHT_DATA_STATUS  		         BERO_ESTIMATE_ALTITUDE;	/*气压计观测高度值状态*/	
	                                             
	volatile HEIGHT_DATA_STATUS  		         ULTR_ZERO_ALTITUDE;		/*超声波零参考点高度值状态*/
	volatile HEIGHT_DATA_STATUS                  ULTR_ESTIMATE_ALTITUDE;  	/*超声波观测高度值状态*/
	                                             
	volatile HEIGHT_ZERO_SET_STATUS              ZERO_ALTITUDE;				/*高度参考零点设置状态*/
	                                             
	/*水平*/                                     
	volatile HORIZONTAL_DATA_STATUS              GPS_ESTIMATE_HORIZONTAL;			/*GPS观测水平值状态*/	
	volatile HORIZONTAL_DATA_STATUS              OPFLOW_ESTIMATE_HORIZONTAL;		/*光流观测水平值状态*/		
	                                             
	volatile HORIZONTAL_ZERO_SET_STATUS          ZERO_HORIZONTAL;					/*水平参考零点设置状态*/
	                                             
	/*=== 4.高度&水平传感器有效性状态 ===*/      
	volatile HEIGHT_SENSOR_AVA_STATUS            ESTIMATE_ALTITUDE;					/*传感器观测高度值状态*/
	volatile HORIZONTAL_SENSOR_AVA_STATUS        ESTIMATE_HORIZONTAL; 				/*传感器观测水平值状态*/
	
	/*=== 5.传感器相关 ===*/
	volatile GPS_DATA_FIRST_AVA_STATUS 			 GPS_DATA_FIRST_AVA_STATUS;			/*第一次使用(HOME点设置)GPS数据有效性*/
	volatile GPS_DATA_QUALITY_STATUS 			 GPS_QUALITY_STATUS;				/*GPS信号质量状态*/
	volatile GPS_POS_FIX_AVA_STATUS				 GPS_AVA_STATUS;					/*GPS数据可用性:卫星数据+水平融合成功*/
	
	volatile VERTICAL_SENSOR_AUTO_CHANGE		 CUR_VERTICAL_SENSOR;				/*当前竖直方向传感器*/
	volatile VERTICAL_SENSOR_AUTO_CHANGE		 LAST_VERTICAL_SENSOR;				/*上次竖直方向传感器*/	
	
	/*=== 6.程序运行状态 ==*/
	vu8											 cpuUsageMajor;							/*CPU 使用率 整数*/
	vu8	 										 cpuUsageMinor;							/*CPU 使用率 小数*/
	volatile PROGRAME_INIT_STATUS				 PLATFORM_INIT_STATUS;					/*平台程序初始化状态*/
	
}AircraftStatus;

extern AircraftStatus g_sAircraftStatus;
extern AircraftStatus *g_psAircraftStatus;


/*检测GPS是否可用(满足定位数据+数据融合成功)*/
GPS_POS_FIX_AVA_STATUS status_GPS_Fix_Ava_Check(AircraftStatus *aircraftStatus);

/*模拟看门狗*/
typedef struct
{	
	vu32 curTicks; 			/*当前ticks*/
	vu32 nextProcessTicks;   /*下次判断ticks*/
}SimulateWatchDog;

/*=== 检测遥控和无人机通信情况 ===*/
#define CMC_DOG_FEED_FACTOR_10MS 	(1) /*10ms*/

/*遥控与飞机通讯状态*/
AIRCRAFT_CMC_STATUS status_check_aircraft_remot_communication(SimulateWatchDog *uavRemotCMCDog, AircraftStatus *aircraftStatus);

/*遥控与飞机通讯正常喂狗*/
void security_Feed_CMC_Succ_Dog_10MS(u8 _10msFoc, SimulateWatchDog *uavRemotCMCDog);

/*安全*/
extern SimulateWatchDog g_sUavRemotCMCDog;
extern SimulateWatchDog *g_psUavRemotCMCDog;

#endif
