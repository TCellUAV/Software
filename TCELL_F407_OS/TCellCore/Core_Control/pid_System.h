#ifndef _PID_SYSTEM_H_
#define _PID_SYSTEM_H_

#include "sys_Platform.h"
#include "filter_DataProcess.h"
#include "bsp_BoardLib.h"

#define PID_PARAMETER_SETTING_NBR	    (21)
#define PID_PARAMETER_STOR_BUFF_LENTH	(sizeof(Vector3s_PID) * PID_PARAMETER_SETTING_NBR / sizeof(StorS16Data))

/*PID参数的调试执行状态*/
typedef enum
{
	
	PID_PARAMETER_DO_SAVE  = 0,		/*保存参数到存储器*/
	PID_PARAMETER_DO_RESET = 1,		/*重置为默认参数*/		
	PID_PARAMETER_DO_NULL  = 0xFF,
}PID_PARAMETER_DO_STATUS;

/*PID参数的有效状态*/
typedef enum
{
	PID_PARAMETER_DISAVA = 0,		/*PID参数无效*/	
	PID_PARAMETER_AVA    = 1,		/*PID参数有效*/
}PID_PARAMETER_AVA_STATUS;

/*PID调参下载状态*/
typedef enum
{
	PID_PARA_DOWNLOAD_FINISH = 0, /*参数下载完成*/
	PID_PARA_DOWNLOAD_START  = 1, /*参数下载开始*/
}PID_PARA_DOWNLOAD_STATUS; 

/*PID参数整定系统*/
typedef struct
{
	StorS16Data 		      		  RWBuff[PID_PARAMETER_STOR_BUFF_LENTH]; /*用于参数的读写Buff*/
	volatile PID_PARAMETER_DO_STATUS  DO_STATUS;							 /*PID参数的调试执行状态*/
    volatile PID_PARAMETER_AVA_STATUS AVA_STATUS;						     /*PID参数的有效性状态*/
	volatile PID_PARA_DOWNLOAD_STATUS PARA_DOWNLOAD_STATUS; 			     /*PID调参下载状态*/
}PidParameterSettingSystem;

/*PID限制标志*/
typedef enum
{
	PID_LIMIT_DISABLE = 0,	/*禁止限制*/
	PID_LIMIT_ENABLE  = 1,	/*允许限制*/
}PID_LIMIT_FLAG;

/*PID控制环对象列表*/
typedef enum
{
	 /****** 角度和角速度控制 ******/	
     PID_CONTROLER_PITCH_GYRO           = 0,	
	 PID_CONTROLER_ROLL_GYRO            = 1,	
	 PID_CONTROLER_YAW_GYRO    		    = 2,
     PID_CONTROLER_PITCH_ANGLE          = 3,
     PID_CONTROLER_ROLL_ANGLE           = 4,
     PID_CONTROLER_YAW_ANGLE            = 5,
									    
	 /****** 位置和速度控制 ******/	    
     PID_CONTROLER_HIGH_SPEED           = 6,		
     PID_CONTROLER_HIGH_POSITION        = 7,
     PID_CONTROLER_LATITUDE_SPEED       = 8,	
     PID_CONTROLER_LATITUDE_POSITION    = 9,	
     PID_CONTROLER_LONGITUDE_SPEED      = 10,	
     PID_CONTROLER_LONGITUDE_POSITION   = 11,	
	 /****** 光流位置和速度控制 ******/
	 PID_CONTROLER_OPTICFLOW_X_SPEED    = 12,	
     PID_CONTROLER_OPTICFLOW_X_POSITION = 13,		
	 PID_CONTROLER_OPTICFLOW_Y_SPEED    = 14,	
     PID_CONTROLER_OPTICFLOW_Y_POSITION = 15,	

	 /****** 加速度控制 ******/	
     PID_CONTROLER_HIGH_ACC			    = 16,
     PID_CONTROLER_LONGITUDE_ACC 	    = 17,
     PID_CONTROLER_LATITUDE_ACC         = 18,
     PID_CONTROLER_OPTICFLOW_X_ACC 	    = 19,
     PID_CONTROLER_OPTICFLOW_Y_ACC      = 20, 	 
}PID_CONTROLER_LINK;

/*PID控制环*/
typedef struct
{
	PID_LIMIT_FLAG 			ERROR_LIMIT_FLAG;			 /*偏差限幅标志*/
	PID_LIMIT_FLAG 			INTEGRATE_LIMIT_FLAG;		 /*积分限幅标志*/
	PID_LIMIT_FLAG 			INTEGRATE_SEPARATION_FLAG;	 /*积分分离标志*/
	fp32		   			expect;						 /*期望*/
	fp32		   			feedback;					 /*反馈*/
	fp32 		   			error;						 /*偏差*/
    fp32 		   			lastError;					 /*上次偏差*/
    fp32 		   			errorMax;					 /*偏差限幅值*/
    fp32 		   			integrateSeparationError;	 /*积分分离偏差值*/
    fp32 		   			integrate;					 /*积分值*/
    fp32 		   			integrateMax;				 /*积分限幅值*/
    Vector3f_PID   			PID;					     /*控制参数kP,kI,kD*/
	Vector3f_PID			PidScale;					 /*用于临时调整PID的3个系数比例*/
    fp32 		   			controlOutput;				 /*控制器总输出*/
    fp32 		   			lastControlOutPut;			 /*上次控制器总输出*/
    fp32		   			controlOutPutLimit;			 /*输出限幅*/
    /***************************************/	
	fp32 					lastLastError;				 /*上上次偏差*/
	fp32 				    adaptableKd;				 /*自适应微分参数*/
    fp32 		  			lastFeedBack;				 /*上次反馈值*/
    fp32 		  		    disErr;				 		 /*误差微分量*/
    fp32 		   			disErrHistory[5];		 	 /*历史误差微分量*/
    /***************************************/		
    fp32 		   			errLPF;						 /*误差低通滤波量*/		 
    fp32 		   			lastErrLPF;					 /*上次误差低通滤波量*/
    fp32 		   			disErrLPF;					 /*误差微分低通滤波量*/
    fp32 		   			lastDisErrLPF;				 /*上次误差微分低通滤波量*/	
	fp32				    lastLastDisErrLPF;			 /*上上次误差微分低通滤波量*/
    PeriodExecuteTime	    PidControlDT;				 /*PID控制间隔时间*/
}PidLink;

/*PID控制系统*/
typedef struct
{	
	 /****** 角度和角速度控制 ******/
     PidLink 				   PitchGyro;			/*pitch角速度环*/	
     PidLink 				   RollGyro;            /*roll角速度环*/	
     PidLink 				   YawGyro;             /*yaw角速度环*/	
     PidLink 				   PitchAngle;			/*pitch角度环*/
     PidLink 				   RollAngle;           /*roll角度环*/
     PidLink 				   YawAngle;            /*yaw角度环*/
		
	 /****** 位置和速度控制 ******/	
     PidLink 				   HighSpeed;			/*high(竖直向上)速度环*/	
     PidLink 				   HighPosition;		/*high(竖直向上)位置环*/
     PidLink 				   LatitudeSpeed;		/*纬度(水平Y方向)速度环*/
     PidLink 				   LatitudePosition;	/*纬度(水平Y方向)位置环*/
     PidLink 				   LongitudeSpeed;		/*经度(水平X方向)速度环*/	
     PidLink 				   LongitudePosition;	/*经度(水平X方向)位置环*/
     PidLink 				   OpticFlowXSpeed;		/*光流(水平X方向)速度环*/	
     PidLink 				   OpticFlowXPosition;	/*光流(水平X方向)位置环*/
     PidLink 				   OpticFlowYSpeed;		/*光流(水平Y方向)速度环*/		
     PidLink 				   OpticFlowYPosition;	/*光流(水平Y方向)位置环*/
	
	 /****** 加速度控制 ******/	
     PidLink 				   HighAcc;				/*high(竖直向上)加速度环*/
     PidLink 				   LongitudeAcc;		/*经度(水平X方向)加速度环*/
     PidLink 				   LatitudeAcc;			/*纬度(水平Y方向)加速度环*/
     PidLink 				   OpticFlowXAcc;		/*光流(水平X方向)加速度环*/
     PidLink 				   OpticFlowYAcc;		/*光流(水平Y方向)加速度环*/	 
	
	 /****** PID整定参数存储 ******/
	 PidParameterSettingSystem PidSettingSystem;	/*PID参数整定系统*/
}PidSystem;

void pid_System_Init(PidSystem *pidSystem);			/*PID控制系统初始化*/
void pid_Link_Init(PidLink *pidLink, PID_CONTROLER_LINK CONTROLER_LINK); /*PID控制单环初始化*/
fp32 pid_Control_General_Dp(PidLink *pidLink);		/*PID通用控制计算*/
fp32 pid_Control_Yaw_Dp(PidLink *pidLink);  		/*PID Yaw角控制计算*/

fp32 pid_Control_Div_LPF(PidLink *pidLink, PID_CONTROLER_LINK CONTROLER_LINK); 		 /*PID DIV控制低通滤波*/
fp32 pid_Control_Err_LPF(PidLink *pidLink, PID_CONTROLER_LINK CONTROLER_LINK);  	 /*PID ERR控制低通滤波*/

fp32 pid_Control_Div_LPF_Gyro(PidLink *pidLink, PID_CONTROLER_LINK CONTROLER_LINK);  /*PID DIV控制低通滤波 Gyro*/
fp32 pid_Control_Div_LPF_Differential_Forward(PidLink *pidLink, PID_CONTROLER_LINK CONTROLER_LINK);  /*微分先行PID控制器*/

void pid_Link_Integrate_Reset(PidLink *pidLink);	/*PID单环积分复位*/
void pid_Horizontal_Takeoff_Integrate_Reset(void);	/*PID水平起飞前积分复位*/
void pid_Horizontal_GPS_Ctrl_Integrate_Reset(void);	/*PID水平控制积分复位*/
void pid_Vertical_Ctrl_Integrate_Reset(void);		/*PID竖直控制积分复位*/

SYS_RETSTATUS pid_parameter_save_or_reset(PidSystem *pidSystem);/*PID参数保存或重置(在线调参)*/
SYS_RETSTATUS pid_Parameter_Read_And_Init(PidSystem *pidSystem);/*读取存储器内部PID参数(初始化)*/

extern PidSystem g_sPidSystem; 
extern PidSystem *g_psPidSystem;

#endif
