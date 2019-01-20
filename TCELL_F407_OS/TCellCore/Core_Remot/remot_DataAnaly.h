#ifndef _REMOT_DATAANALY_H_
#define _REMOT_DATAANALY_H_

#include "sys_Platform.h"
#include "status_Aircraft.h"

/*数据值判断*/
#define REMOT_DATA_MAX_VALUE		(2000)	/*遥控数据最大值*/
#define REMOT_DATA_MID_VALUE		(1500)	/*遥控数据中值*/
#define REMOT_DATA_MIN_VALUE		(1000)	/*遥控数据最小值*/
#define REMOT_DATA_MID_ERROR		(50)	/*摇杆置中误差*/
#define REMOT_DATA_PRECISION		(30)	/*遥控数据精度*/
#define REMOT_DATA_MIN_COMPENSATE	(100)	/*遥控最小值补偿(摇杆拉到最小位置时,容易出现0,因此可不必将摇杆拉到最小位置)*/

typedef enum
{
	REMOT_DATA_ZERO  = 0,
	REMOT_DATA_MIN   = 1,
	REMOT_DATA_MID   = 2,
	REMOT_DATA_MAX   = 3,
	REMOT_DATA_ERROR = 0xFF,
}REMOT_DATA_STATUS;

/*数据更新状态*/
typedef enum
{
	WAVE_STATUS_NEW = 1,	/*PWM输入数据状态是新数据*/
	WAVE_STATUS_OLD = 0,	/*PWM输入数据状态是老数据*/
}WAVE_UPDATE_STATUS;

/*拨键边沿有效状态*/
typedef enum
{
	REMOT_SWITCH_EDGE_AVA_NO = 0, /*边沿无效*/
	REMOT_SWITCH_EDGE_AVA_OK = 1, /*边沿有效*/
}REMOT_SWITCH_EDGE_AVA_STATUS;

/*====== PWM IN ======*/
#if defined(REMOTE_DATA_RECV__PWM) 
/*捕获边沿状态*/
typedef enum
{
	CAP_EDGE_RISING  = 0,
	CAP_EDGE_FALLING = 1,
}CAP_EDGE_STATUS;

/*PWM输入波形*/
typedef struct
{
	u16 		    	        fallingEdgeTicks;  /*下降沿时期*/
	u16 			            risingEdgeTicks;   /*上升沿时期*/
	u16 			            avaTicks;		   /*有效时期*/
	volatile CAP_EDGE_STATUS    CAP_EDGE;     	   /*边沿捕获状态(上升沿/下降沿)*/
	volatile WAVE_UPDATE_STATUS UPDATE_STATUS;	   /*数据更新状态*/
}PwmInputWave;

/*姿态通道*/
typedef struct
{
	PwmInputWave Roll;       		/*第1通道Roll*/
	PwmInputWave Pitch;     		/*第2通道Pitch*/
	PwmInputWave Throttle;   		/*第3通道Throttle*/
	PwmInputWave Yaw;        		/*第4通道Yaw*/
}RemotAtt;

/*拨键通道*/
typedef struct
{
	PwmInputWave SWA;		 /*第5通道TwistLeft*/	
	PwmInputWave SWB;		 /*第6通道TwistRight*/	
	PwmInputWave SWC;		 /*三段:一键起飞/一键降落*/
	PwmInputWave SWD;		 /*失控保护*/
}RemotSwitch;

/*云台万向节通道*/
typedef struct
{
	PwmInputWave VRA;		        /*第5通道TwistLeft*/	
	PwmInputWave VRB;		        /*第6通道TwistRight*/	
}RemotGimbal;
#endif

/*====== PPM IN ======*/
#if defined(REMOTE_DATA_RECV__PPM)

#define REMOT_PPM_RECV_MAX_CHANNEL_NBR	(8) /*PPM最大接收通道数*/

/*PPM接收状态*/
typedef enum
{
	REMOT_PPM_RECV_START  = 1, /*PPM接收开始*/
	REMOT_PPM_RECV_FINISH = 0, /*PPM接收结束*/
}REMOT_PPM_RECV_STATUS;

/*电平采样通道*/
typedef enum
{
	REMOT_PPM_SAMPLE_1ST_CNL  = 0,
	REMOT_PPM_SAMPLE_2ND_CNL  = 1,
	REMOT_PPM_SAMPLE_3RD_CNL  = 2,
	REMOT_PPM_SAMPLE_4TH_CNL  = 3,
	REMOT_PPM_SAMPLE_5TH_CNL  = 4,
	REMOT_PPM_SAMPLE_6TH_CNL  = 5,
	REMOT_PPM_SAMPLE_7TH_CNL  = 6,
	REMOT_PPM_SAMPLE_8TH_CNL  = 7,	
}REMOT_PPM_SAMPLE_CHANNEL;

typedef struct
{
	vu32 lastTick;
	vu32 curTick;
	vu32 deltaTick;
}Remot_PPM_Recv_Tick;

typedef struct
{
	u16 						avaTicks;		   /*有效时期*/
	volatile WAVE_UPDATE_STATUS UPDATE_STATUS;	   /*数据更新状态*/	
}PpmInputWave;

/*姿态通道*/
typedef struct
{
	PpmInputWave Roll;       		/*第1通道Roll*/
	PpmInputWave Pitch;     		/*第2通道Pitch*/
	PpmInputWave Throttle;   		/*第3通道Throttle*/
	PpmInputWave Yaw;        		/*第4通道Yaw*/
}RemotAtt;

/*拨键通道*/
typedef struct
{
	PpmInputWave SWA;		 /*第5通道TwistLeft*/	
	PpmInputWave SWB;		 /*第6通道TwistRight*/	
	PpmInputWave SWC;		 /*三段:一键起飞/一键降落*/
	PpmInputWave SWD;		 /*失控保护*/
}RemotSwitch;

/*云台万向节通道*/
typedef struct
{
	PpmInputWave VRA;		        /*第5通道TwistLeft*/	
	PpmInputWave VRB;		        /*第6通道TwistRight*/	
}RemotGimbal;
#endif

/*====== SBUS IN ======*/
#if defined(REMOTE_DATA_RECV__SBUS)
typedef struct
{
	u16 AvaTicks;		  /*有效时期*/
}SbusInputWave

/*姿态通道*/
typedef struct
{
	SbusInputWave Roll;       		/*第1通道Roll*/
	SbusInputWave Pitch;     		/*第2通道Pitch*/
	SbusInputWave Throttle;   		/*第3通道Throttle*/
	SbusInputWave Yaw;        		/*第4通道Yaw*/

}RemotAtt;

/*拨键通道*/
typedef struct
{
	SbusInputWave SWA;		 /*第5通道TwistLeft*/	
	SbusInputWave SWB;		 /*第6通道TwistRight*/	
	SbusInputWave SWC;		 /*三段:一键起飞/一键降落*/
	SbusInputWave SWD;		 /*失控保护*/
}RemotSwitch;

/*云台万向节通道*/
typedef struct
{
	SbusInputWave VRA;		        /*第5通道TwistLeft*/	
	SbusInputWave VRB;		        /*第6通道TwistRight*/	
}RemotGimbal;
#endif

/*遥控数据*/
typedef struct
{
	/*1-4(姿态)*/
	u16 AttRoll;		 /*横滚角*/
	u16 AttPitch;		 /*俯仰角*/
	u16 AttThrottle;	 /*油门*/
	u16 AttYaw;			 /*航向角*/
	
	/*5-8(拨键)*/
	u16 SWA;		 	 /*二段:运动模式/普通模式*/		 	
	u16 SWB;		     /*保留*/	
	u16 SWC;		     /*三段:一键起飞/一键降落*/
	u16 SWD;		     /*失控保护*/	
	
	/*9-10(万向节)*/
	u16 GimPitch;		 /*舵机云台:俯仰*/
	u16 GimYaw;			 /*舵机云台:航向*/
}RemotData;

/*遥控通道转换为期望角*/
typedef struct
{
	s16 roll;
	s16 pitch;
	u16 throttle;
	s16 yaw;
}Remot_Expect_Angle;

/*遥控数据对应的功能*/
typedef struct
{
	RemotAtt 			  		      Attitude;	/*遥控姿态*/
	RemotSwitch 		  		      Switch;	/*拨键*/	
	RemotGimbal 		  		      Gimbal;	/*万向节云台*/

	
	#if defined(REMOTE_DATA_RECV__PPM) /*PPM*/
	u16		   		                  PPM_Buff[REMOT_PPM_RECV_MAX_CHANNEL_NBR];
	volatile REMOT_PPM_RECV_STATUS    PPM_RECV_STATUS;
	volatile REMOT_PPM_SAMPLE_CHANNEL PPM_SAMPLE_CHANNLE;
	#endif
}ReceiverAnaly;

/*=== 遥控数据解析及解析成期望 ===*/
/*获取遥控各通道数据*/
void remot_get_all_channel_data(RemotData *remotData, ReceiverAnaly *receiverAnaly);

/*=== 遥控特定操作判断 ===*/
/*飞控:解锁/上锁/自动上锁*/
UAV_LOCK_STATUS remot_aircraft_lock_and_unlock(void);

/*=== 原始数据 极大/极小/中值判断 ===*/
REMOT_DATA_STATUS remot_Data_Range(u16 ChannelData, REMOT_DATA_STATUS testStatus);


#if defined(REMOTE_DATA_RECV__PWM)
/*=== MCU Tim 输入捕获解析 ===*/
/*拨键通道(TIM4)捕获+解析*/
void remot_Attitude_Channel_Analy(void);

/*姿态通道(TIM1)捕获+解析*/
void remot_Switch_Channel_Analy(void);

/*云台万向节旋钮通道(TIM1)捕获+解析*/
void remot_Gimbal_Channel_Analy(void);	
#endif

#if defined(REMOTE_DATA_RECV__PPM)
/*=== MCU PPM解析 ===*/
void remot_PPM_AllChannel_Analy(void);
#endif


/*遥控接收机PWM数据*/
extern ReceiverAnaly g_sReceiverAnaly;
extern ReceiverAnaly *g_psReceiverAnaly;

/*遥控数据*/
extern RemotData g_sRemotData;
extern RemotData *g_psRemotData;

#endif
