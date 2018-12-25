#ifndef _BSP_BERO_MS5611_H_
#define _BSP_BERO_MS5611_H_

#include "bsp_BERO_MS5611_CMD.h"
#include "sys_Platform.h"
#include "sys_McuInit.h"
#include "ssp_SimI2C.h"

/**
  * @brief  气压计相关结构体
  */  
  
#if defined(STD_PROTOCOL_SOFTWARE_I2C)
typedef struct
{  
	SSP_SimI2C *SimI2cMaster;
	u16  	   PROM_C[8];
	u8 	 	   Buff[12];
	fp32 	   Temperature;
	fp32 	   Pressure;
	fp32 	   Altitude;
}BSP_MS5611;
#endif


#if defined(STD_PROTOCOL_HARDWARE_I2C)
typedef struct
{  
	MSP_I2c *I2cMaster;
	u16  	PROM_C[8];
	u8 	 	Buff[12];
	fp32 	Temperature;
	fp32 	Pressure;
	fp32 	Altitude;
}BSP_MS5611;
#endif


/*MS5611初始化*/
SYS_RETSTATUS bsp_MS5611_Init(BSP_MS5611 *ms5611);	

/*读MS5611的PROM*/
SYS_RETSTATUS bsp_MS5611_ReadPROM(BSP_MS5611 *ms5611);

/*获取处理后的温度值*/
s32 bsp_MS5611_GetTemperature(BSP_MS5611 *ms5611, u8 OSR);

/*获取处理后的气压值*/	
s32 bsp_MS5611_GetPressure(BSP_MS5611 *ms5611, u8 OSR);	

/*获取dT*/
s32 bsp_MS5611_GetDeltaTemperature(BSP_MS5611 *ms5611, u8 OSR);

/*获取原始温度数据*/
u32 bsp_MS5611_GetRawTemperature(BSP_MS5611 *ms5611, u8 OSR);	

/*获取原始气压数据*/	
u32 bsp_MS5611_GetRawPressure(BSP_MS5611 *ms5611, u8 OSR);	

/*读24bit ADC值*/
u32 bsp_MS5611_DoConversion(BSP_MS5611 *ms5611, u8 DevCmd);	

/*将气压值转换为海拔高度*/
fp32 bsp_MS5611_GetAltitude(BSP_MS5611 *ms5611); 	

extern BSP_MS5611 g_sMs5611;

#endif
