#ifndef _BSP_MAG_HMC5883L_H_
#define _BSP_MAG_HMC5883L_H_

#include "bsp_MAG_HMC5883L_CMD.h"
#include "sys_Platform.h"
#include "sys_McuInit.h"
#include "ssp_SimI2C.h"
#include "msp_I2C.h"

/*HMC5883L识别*/
typedef enum
{
	HMC5883L_IDENT_OK   = 0,	
	HMC5883L_IDENT_FAIL = 1,
}HMC5883L_IDENT_STATUS;

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
typedef struct
{
	SSP_SimI2C *SimI2cMaster;
	Mag3s   	Mag;
	fp32    	testMagYaw;
}BSP_HMC5883L;
#endif

#if defined(STD_PROTOCOL_HARDWARE_I2C)
typedef struct
{
	MSP_I2c *I2cMaster;
	Mag3s   Mag;
	fp32    testMagYaw;
}BSP_HMC5883L;
#endif

/*初始化*/
SYS_RETSTATUS bsp_HMC5883L_Init(BSP_HMC5883L* hmc5883l);

/*读取3个轴的数据*/
Mag3s *bsp_HMC5883L_Get_Mag_Data(BSP_HMC5883L *hmc5883l);

/*测试:获取磁力计的Yaw(水平位置测试偏航角)*/
fp32 bsp_HMC5883L_Get_Mag_Yaw(BSP_HMC5883L *hmc5883l);

extern BSP_HMC5883L g_sHmc5883l;

#endif 
