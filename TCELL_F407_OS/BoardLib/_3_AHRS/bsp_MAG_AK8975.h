#ifndef _BSP_MAG_AK8975_H_
#define _BSP_MAG_AK8975_H_

#include "bsp_MAG_AK8975_CMD.h"
#include "sys_Platform.h"
#include "sys_McuInit.h"
#include "ssp_SimI2C.h"
#include "msp_I2C.h"

/*Sensitivity Adjustment values*/
typedef struct
{
	u8 x;
	u8 y;
	u8 z;
}Ak8975_ASA;

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
typedef struct
{
	SSP_SimI2C *SimI2cMaster;
	Mag3s      Mag;
	Ak8975_ASA Asa;
	fp32       TestMagYaw;
}BSP_AK8975;
#endif

#if defined(STD_PROTOCOL_HARDWARE_I2C)
typedef struct
{
	MSP_I2c    *I2cMaster;
	Mag3s      Mag;
	Ak8975_ASA Asa;
	fp32       TestMagYaw;
}BSP_AK8975;
#endif

/*初始化*/
SYS_RETSTATUS bsp_AK8975_Init(BSP_AK8975 *ak8975);

/*获取从机地址识别码*/
u8 bsp_AK8975_Get_Id(BSP_AK8975 *ak8975);

/*获取灵敏度调整值*/
SYS_RETSTATUS bsp_AK8975_Get_AsaValue(BSP_AK8975 *ak8975);

/*读取3个轴的数据*/
Mag3s *bsp_AK8975_Get_Mag_Data(BSP_AK8975 *ak8975);

/*对数据进行灵敏度调整*/
Mag3s *bsp_AK8975_Asa_Dp(BSP_AK8975 *ak8975);

/*测试:获取磁力计的Yaw(水平位置测试偏航角)*/
fp32 bsp_AK8975_Get_Mag_Yaw(BSP_AK8975 *ak8975);

extern BSP_AK8975 g_sAk8975;

#endif 
