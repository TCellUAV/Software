#ifndef _HOST_TRANSCEIVE_H_
#define _HOST_TRANSCEIVE_H_

#include "sys_Platform.h"

#include "ANO_DT.h" 	/*匿名上位机*/
#include "VCAN_DT.h"	/*山外上位机*/
#include "BSK_DT.h"		/*天穹(BlueSky)上位机*/

/********** 用户拓展 **********/
/*上位机发送至下位机指令 用户拓展*/
typedef enum
{
	USER_HOST_CMD_SW_RS = 0xAA, /*切换上位机及数据上传对象*/
}USER_HOST_CMD_ID;

/*上位机类型*/
typedef enum
{
	USER_HOST_CHOOSE_ANO  = 0x01, /*匿名上位机*/
	USER_HOST_CHOOSE_VCAN = 0x02, /*山外上位机*/
	USER_HOST_CHOOSE_BSK  = 0x03, /*BlueSky上位机*/	
}USER_HOST_CHOOSE_TARG;

/*数据类型*/
typedef enum
{
	USER_ANO_MSG_EXCHANGE				    = 0xFF, /*上位机功能(不显示波形)*/
	
	/*数据波形*/
	USER_HOST_MSG_SENSOR_RAW_FILTER   	    = 0,  /*传感器原始和滤波值*/
	USER_HOST_MSG_SINS_HEIGHT         	    = 1,  /*竖直方向融合数据*/
	USER_HOST_MSG_SINS_HORIZONTAL_X   	    = 2,  /*水平X方向融合数据*/
	USER_HOST_MSG_SINS_HORIZONTAL_Y   	    = 3,  /*水平Y方向融合数据*/
	USER_HOST_MSG_OPTICFLOW_CTRL_DATA 	    = 4,  /*光流控制数据*/
	
	/*控制系统波形*/
	USER_HOST_MSG_VER_LINK_CONTROL          = 10, /*竖直方向*/
	USER_HOST_MSG_GPS_HOR_X_LINK_CONTROL    = 11, /*GPS 水平X方向*/
	USER_HOST_MSG_GPS_HOR_Y_LINK_CONTROL    = 12, /*GPS 水平Y方向*/
	USER_HOST_MSG_OPFLOW_HOR_X_LINK_CONTROL = 13, /*OPTIC FLOW 水平X方向*/
	USER_HOST_MSG_OPFLOW_HOR_Y_LINK_CONTROL = 14, /*OPTIC FLOW 水平Y方向*/
	USER_HOST_MSG_HOR_Z_LINK_CONTROL  	    = 15, /*水平Z方向*/
}USER_HOST_MSG_ID;

/*上传切换状态*/
typedef enum
{
	USER_HOST_SWITCH_NO = 0xAA, /*未切换*/
	USER_HOST_SWITCH_OK = 0x55, /*已切换*/
}USER_HOST_SWITCH_STATUS;

/*上位机与下位机数据切换*/
typedef struct
{
	volatile USER_HOST_CHOOSE_TARG   HOST_TARG; 	/*上位机目标*/
	volatile USER_HOST_MSG_ID        MSG_ID;	    /*消息ID*/
	volatile USER_HOST_SWITCH_STATUS SWITCH_STATUS; /*切换状态*/
	volatile u16				     period_5MS;	/*上传间隔周期(5ms * n)*/
}User_Send_Host_System;

/*传感器原始数据&滤波数据*/
typedef struct 
{
	/*加速度*/
	s16 accX;             /*未滤波的当前加速度数据*/
    s16 accXFilter;       /*经过滤波后的加速度数据*/    
    s16 accY;				
    s16 accYFilter;        
    s16 accZ;
    s16 accZFilter;            
	
	/*角速度*/
    s16 gyroX;            /*未滤波的当前角速度数据*/
    s16 gyroXFilter;      /*经过滤波后的角速度数据*/    
    s16 gyroY;
    s16 gyroYFilter;            
    s16 gyroZ;
    s16 gyroZFilter;   
	
	/*磁力计*/
    s16 magX;             /*未滤波的地磁数据*/
    s16 magXFilter;       /*经过滤波后的地磁数据*/    
    s16 magY;
    s16 magYFilter;            
    s16 magZ;
    s16 magZFilter; 
	
	/*气压计*/
	s32 bero;
	s32 beroFilter;
}SendSensorData;


/*惯导融合数据*/
typedef struct
{
	s32 sinsPosition;	/*SINS估算高度*/
	s32 sinsSpeed;		/*SINS估算速度*/
	s32 sinsAcc;		/*SINS估算加速度*/
	s32 obPosition;		/*传感器观测高度*/
	s32 obAcc;			/*传感器观测加速度*/
	s32 corAcc;			/*加速度修正量*/
	s32 corSpd;			/*速度修正量*/
	s32 corPos;			/*位置修正量*/
}SendSINSData;

/*PID控制环参数(4个)*/
typedef struct
{
	s32 data1;
	s32 data2;
	s32 data3;
	s32 data4;
	s32 data5;
	s32 data6;
	s32 data7;
	s32 data8;	
	u8  waveChNbr; /*波形通道数*/
}SendPIDPara;

/*光流位移和速度*/
typedef struct
{
	/*原始位移量*/
	s32 intPosition;		/*积分位移*/
	s32 intPositionLPF;		/*积分位移低通滤波*/
	s32 angleCompensate;	/*姿态角补偿积分位移*/
	s32 curRawPosition;		/*本次处理后的积分位移*/
	
	/*原始速度量*/
	s32 diffSpeed;			/*微分速度*/
	s32 diffSpeedLPF;		/*微分速度低通滤波*/
	
	/*转化后的真实位移和速度*/
	s32 realPosition;		/*转化后的位移*/
	s32 realSpeed;			/*转化后的速度*/
}SendOpticFlowCtrlData;

/*====== 飞控设置指令接收及解析(上位机通用) ======*/
/*接收初步解析帕判断*/
void user_Host_Data_Receive_Prepare(u8 data);

/*飞控设置指令解析*/
void user_Host_Data_Receive_Anl(u8 *data_buf,u8 num);

/*====== ANO匿名上位机_示波器 ======*/
void user_ANO_Send_Host_Wave_Data(USER_HOST_MSG_ID USER_WAVE_TARG, u32 periodTaskMs);

/*1.发送传感器滤波前后数据到上位机*/
void user_ANO_Send_Sensor_RawFilter_Data(SendSensorData sensorData);

/*2.发送竖直惯导数据到上位机*/
void user_ANO_Send_Sins_Height_Data(SendSINSData sendSinsData);

/*发送水平X方向惯导数据到上位机*/
void user_ANO_Send_Sins_Horizontal_X_Data(SendSINSData sendSinsData);

/*发送水平Y方向惯导数据到上位机*/
void user_ANO_Send_Sins_Horizontal_Y_Data(SendSINSData sendSinsData);

/*3.发送PID控制系统数据到上位机*/
void user_ANO_Send_Pid_Link_Data(SendPIDPara sendPidPara);

/*4.发送光流控制数据到上位机*/
void user_ANO_Send_OpticFlow_Ctrl_Data(SendOpticFlowCtrlData sendOpticFlowCtrlData);


/*====== VCAN山外上位机_示波器 ======*/
void user_VCAN_Send_Host_Wave_Data(USER_HOST_MSG_ID USER_WAVE_TARG, u32 periodTaskMs);

/*1.发送传感器滤波前后数据到上位机*/
void user_VCAN_Send_Sensor_RawFilter_Data(SendSensorData sensorData);

/*2.发送竖直惯导数据到上位机*/
void user_VCAN_Send_Sins_Height_Data(SendSINSData sendSinsData);

/*发送竖直惯导数据到上位机*/
void user_VCAN_Send_Sins_Horizontal_X_Data(SendSINSData sendSinsData);

/*发送竖直惯导数据到上位机*/
void user_VCAN_Send_Sins_Horizontal_Y_Data(SendSINSData sendSinsData);

/*3.发送PID控制系统数据到上位机*/
void user_VCAN_Send_Pid_Link_Data(SendPIDPara sendPidPara);

/*4.发送光流控制数据到上位机*/
void user_VCAN_Send_OpticFlow_Ctrl_Data(SendOpticFlowCtrlData sendOpticFlowCtrlData);

/*数据上传上位机系统*/
extern User_Send_Host_System g_sUserSendHostSystem;

/*传感器数据原始值和滤波值*/
extern SendSensorData g_sSendSensorData; 
/*竖直+水平惯导(三阶互补)值*/
extern SendSINSData g_sSendSinsDataHeight;	/*Z轴竖直方向的惯导*/
extern SendSINSData g_sSendSinsDataHorizontalX;	/*X轴(正东)水平方向的惯导*/
extern SendSINSData g_sSendSinsDataHorizontalY;	/*Y轴(正北)水平方向的惯导*/
/*PID调试波形数据*/
extern SendPIDPara  g_sSendPidPara;
#endif
