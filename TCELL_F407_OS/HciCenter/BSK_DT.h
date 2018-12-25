#ifndef _BSK_DT_H_
#define _BSK_DT_H_

#include "sys_Platform.h"

typedef enum
{
	OFF_BSK_DT_SEND_BASE_FLY_DATA     = 0x01,	/*基本飞控数据*/
	OFF_BSK_DT_SEND_FLY_STATUS	      = 0x02,	/*飞行状态*/
	OFF_BSK_DT_SEND_SENSOR_DATA       = 0x03,	/*传感器数据*/
	OFF_BSK_DT_SEND_SENSOR_CALIB_DATA = 0x04, 	/*传感器校准数据*/
	OFF_BSK_DT_SEND_SENSOR_CALIB_CMD  =  0x05,  /*传感器校准命令*/
	OFF_BSK_DT_SEND_REMOT_DATA		  = 0x08,	/*遥控数据*/
	OFF_BSK_DT_SEND_MOTOR_PWM_OUTPUT  = 0x09,   /*电机PWM输出*/
	OFF_BSK_DT_SEND_BATTERY_INFO	  = 0x0A,   /*电池信息*/
	OFF_BSK_DT_SEND_ATTITUDE_PID_PARA = 0x10, 	/*姿态PID参数*/
	OFF_BSK_DT_SEND_POSITION_PID_PARA = 0x11, 	/*位置PID参数*/
	OFF_BSK_DT_SEND_PID_RDWR_RESPONSE = 0x12,  	/*PID读写响应*/
	OFF_BSK_DT_SEND_GPS_DATA		  = 0x20, 	/*GPS数据*/
	OFF_BSK_DT_SEND_ATT_EST_CTRL_DATA = 0x30, 	/*姿态估计与控制数据*/
	OFF_BSK_DT_SEND_SPD_EST_CTRL_DATA = 0x31, 	/*速度估计与控制数据*/
	OFF_BSK_DT_SEND_POS_EST_CTRL_DATA = 0x32, 	/*位置估计与控制数据*/
	OFF_BSK_DT_SEND_USER_DATA		  = 0x33, 	/*用户自定义数据*/
	OFF_BSK_DT_SEND_HEART_PACKAGE     = 0x05, 	/*心跳包*/
}OFF_BSK_DT_SEND_HOST_MSG_ID;

/**/

#endif
