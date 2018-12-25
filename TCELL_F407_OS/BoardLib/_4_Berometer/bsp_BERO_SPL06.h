#ifndef _BSP_BERO_SPL06_H_
#define _BSP_BERO_SPL06_H_

#include "bsp_BERO_SPL06_CMD.h"
#include "sys_Platform.h"
#include "sys_McuInit.h"
#include "ssp_SimI2C.h"
#include "msp_I2C.h"
#include "msp_I2C.h"

typedef enum
{
	SPL06_SENSOR_PRESSURE    = 0,
	SPL06_SENSOR_TEMPERATURE = 1,	
}SPL06_SENSOR_TARG;

typedef enum
{
	SPL06_STANDBY_MODE    = 1,	/*备用模式*/
	SPL06_COMMAND_MODE    = 2,	/*命令模式*/	
	SPL06_BACKGROUND_MODE = 3,	/*后台模式*/
}SPL06_MEAS_MODE;

typedef enum
{
	/*Standby Mode*/
	SPL06_IDLE_OR_STOP_BACKGROUND_MEAS_TYPE = 0,

	/*Command Mode(命令发起一次测量)*/	
	SPL06_PRESSURE_MEAS_TYPE			    = 1,
	SPL06_TEMPERATURE_MEAS_TYPE			    = 2,
	SPL06_NA_1_TYPE 						= 3,
	SPL06_NA_2_TYPE 						= 4,
	
	/*Background Mode(后台自动连续测量)*/
	SPL06_CONTINUOUS_PRESSURE_TYPE    	    = 5,
	SPL06_CONTINUOUS_TEMPERATURE_TYPE       = 6,
	SPL06_CONTINUOUS_P_AND_T_TYPE           = 7,
}SPL06_MEAS_TYPE;

/*Calibration Coefficients data*/
typedef struct
{
	s16 c0;
	s16 c1;
	s32 c00;
	s32 c10;
	s16 c01;
	s16 c11;
	s16 c20;
	s16 c21;
	s16 c30;
}SPL06_Coef_Data;

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
typedef struct
{
	SSP_SimI2C 		*SimI2cMaster;
	SPL06_Coef_Data CoefData;
	u8 				ChipId;	/*产品ID | 版本ID*/
	s32 			PressureAdc;
	s32				TemperatureAdc;
	s32				kP;
	s32				kT;
	fp32            Pressure;
	fp32            Temperature;	
	s32 			rawAltitude;	/*原始海拔高度 cm*/
	s32 			filterAltitude;	/*滤波后海拔高度 cm*/	
}BSP_SPL06;
#endif

#if defined(STD_PROTOCOL_HARDWARE_I2C)
typedef struct
{
	MSP_I2c 		*I2cMaster;
	SPL06_Coef_Data CoefData;
	u8 				ChipId;	/*产品ID | 版本ID*/
	s32 			PressureAdc;
	s32				TemperatureAdc;
	s32				kP;
	s32				kT;
	fp32            Pressure;
	fp32            Temperature;	
	s32 			rawAltitude;	/*原始海拔高度 cm*/
	s32 			filterAltitude;	/*滤波后海拔高度 cm*/	
}BSP_SPL06;
#endif

/*SPL06-001初始化*/
SYS_RETSTATUS bsp_SPL06_Init(BSP_SPL06 *spl06);	

/*Get Calibration Coefficients*/
SYS_RETSTATUS bsp_SPL06_Get_Calib_Coef(BSP_SPL06 *spl06);

/*pressure rate set*/
SYS_RETSTATUS bsp_SPL06_Pressure_Rate_Set(BSP_SPL06 *spl06, u8 measureRate, u8 oversampleRate);

/*temperature  rate set*/
SYS_RETSTATUS bsp_SPL06_Temperature_Rate_Set(BSP_SPL06 *spl06, u8 tmpSensorSelect, u8 measureRate, u8 oversampleRate);

/*set measurement mode and type then start measure*/
SYS_RETSTATUS bsp_SPL06_Set_And_Start_Measure(BSP_SPL06 *spl06, SPL06_MEAS_MODE measMode, SPL06_MEAS_TYPE measType);

/*spl06 get raw temperature(adc)*/
s32 bsp_SPL06_Get_Temperature_Adc(BSP_SPL06 *spl06);

/*spl06 get raw pressure(adc)*/
s32 bsp_SPL06_Get_Pressure_Adc(BSP_SPL06 *spl06);

/*spl06 get temperature*/
fp32 bsp_SPL06_Get_Temperature(BSP_SPL06 *spl06);

/*spl06 get pressure*/
fp32 bsp_SPL06_Get_Pressure(BSP_SPL06 *spl06);

/*get spl06 altitude*/
s32 bsp_SPL06_Get_Altitude(BSP_SPL06 *spl06, fp32 referencePa);


extern BSP_SPL06 g_sSpl06;
#endif
