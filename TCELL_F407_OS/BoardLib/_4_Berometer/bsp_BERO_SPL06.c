#include "bsp_BERO_SPL06.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

BSP_SPL06 g_sSpl06 = {0};

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
/**
  * @brief  Write a byte to the specified device address through SIM_I2C bus.
  * @param  mcSimI2cMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  DevCmd: the command would be writen to the specified device address
  * @return  None
  */
static SYS_RETSTATUS Spl06_SimI2c_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif	
	
	FunctionalState state;
	
	ssp_SimI2C_Start(g_sSpl06.SimI2cMaster);
	
	state = ssp_SimI2C_WriteByte(g_sSpl06.SimI2cMaster, (DevAddr << 1) | 0);
	
	if (state == ENABLE)
	{
		state = ssp_SimI2C_WriteByte(g_sSpl06.SimI2cMaster, RegAddr);
		
		if (state == ENABLE)
		{
			state = ssp_SimI2C_WriteByte(g_sSpl06.SimI2cMaster, Data);
			
			if (state == ENABLE)
			{
				ssp_SimI2C_Stop(g_sSpl06.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}
	
	ssp_SimI2C_Stop(g_sSpl06.SimI2cMaster);
	
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁释放*/
//	rt_mutex_release(&i2c_mutex);
#endif	
	
	return SYS_RET_FAIL;
}

/**
  * @brief  Read an byte from the specified device address through SIM_I2C bus.
  * @param  mcSimI2cMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @return  the byte read from SIM_I2C bus
  */
static SYS_RETSTATUS  Spl06_SimI2c_ReadSomeByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t len, uint8_t *RdBuff)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif		
	
	FunctionalState state;
	
	ssp_SimI2C_Start(g_sSpl06.SimI2cMaster);
	
	state = ssp_SimI2C_WriteByte(g_sSpl06.SimI2cMaster, (DevAddr << 1) | 0);
	
	if (state == ENABLE)
	{
		state = ssp_SimI2C_WriteByte(g_sSpl06.SimI2cMaster, RegAddr);
		
		if (state == ENABLE)
		{
			ssp_SimI2C_Start(g_sSpl06.SimI2cMaster);	
			
			state = ssp_SimI2C_WriteByte(g_sSpl06.SimI2cMaster, (DevAddr << 1) | 1);
			
			if (state == ENABLE)
			{
				while(len)
				{
					if (len == 1)
					{
						*RdBuff = ssp_SimI2C_ReadByte(g_sSpl06.SimI2cMaster, RdBuff, DISABLE);
					}
					else
					{
						*RdBuff = ssp_SimI2C_ReadByte(g_sSpl06.SimI2cMaster, RdBuff, ENABLE);
					}
					
					len--;
					RdBuff++;
				}
				
				ssp_SimI2C_Stop(g_sSpl06.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}

	ssp_SimI2C_Stop(g_sSpl06.SimI2cMaster);
	
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁释放*/
//	rt_mutex_release(&i2c_mutex);
#endif	
	
	return SYS_RET_FAIL;	
}
#endif

#if defined(STD_PROTOCOL_HARDWARE_I2C)
/**
  * @brief  Write a byte to the specified device address through SIM_I2C bus.
  * @param  mcSimI2cMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  DevCmd: the command would be writen to the specified device address
  * @return  None
  */
static SYS_RETSTATUS Spl06_I2c_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	u8 wrData = Data;
	
	ret = I2C_Write_SomeByte(g_sSpl06.I2cMaster, RegAddr, &wrData, 1);
	
	return ret;
}

/**
  * @brief  Read an byte from the specified device address through SIM_I2C bus.
  * @param  mcSimI2cMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @return  the byte read from SIM_I2C bus
  */
static SYS_RETSTATUS  Spl06_I2c_ReadSomeByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t len, uint8_t *RdBuff)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	ret = I2C_Read_SomeByte(g_sSpl06.I2cMaster, RegAddr, RdBuff, len);
	
	return ret;	
}
#endif

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
#define SPL06_I2C_WRITE_ONEBYTE	 Spl06_SimI2c_WriteOneByte
#define SPL06_I2C_READ_SOMEBYTE	 Spl06_SimI2c_ReadSomeByte
#endif

#if defined(STD_PROTOCOL_HARDWARE_I2C)
#define SPL06_I2C_WRITE_ONEBYTE	 Spl06_I2c_WriteOneByte
#define SPL06_I2C_READ_SOMEBYTE  Spl06_I2c_ReadSomeByte
#endif


/*SPL06-001初始化*/
#if defined(HW_CUT__USE_MD_BERO)

SYS_RETSTATUS bsp_SPL06_Init(BSP_SPL06 *spl06)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	u8 readData = 0;
	
	/*spl06 i2c init*/
	#if defined(STD_PROTOCOL_HARDWARE_I2C)		
	spl06->I2cMaster 			= &g_sBeroI2C;
	spl06->I2cMaster->slaveAddr = (SPL06_SLAVEADDR << 1);
	#endif
	
	#if defined(STD_PROTOCOL_SOFTWARE_I2C)	
	spl06->SimI2cMaster         = &g_sBeroSimI2C;
	#endif

	sys_DelayMs(1);	
	
	/*读取SPL06的ID*/
	SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_ID, 1, &readData);
	
	if (readData == SPL06_PROD_REV_ID_R)
	{
		spl06->ChipId = readData;
	}
	else
	{
		statusRet = SYS_RET_FAIL;
	}
	
	/*get calibration coefficients*/
	statusRet |= bsp_SPL06_Get_Calib_Coef(spl06);
	
	/*Pressure Sensor Set(measurement rate : 128, resolution : 64)*/
	statusRet |= bsp_SPL06_Pressure_Rate_Set(spl06, 128, 64);
	
	/*Temperature Sensor Set(measurement rate : 32, resolution : 8)*/	
	statusRet |= bsp_SPL06_Temperature_Rate_Set(spl06, SPL06_TMP_EXT_SENSOR_W, 32, 8);
	
	/*set measurement mode and type then start measure*/
	statusRet |= bsp_SPL06_Set_And_Start_Measure(spl06, SPL06_BACKGROUND_MODE, SPL06_CONTINUOUS_P_AND_T_TYPE);
	
	return statusRet;
}
#endif

/*Get Calibration Coefficients*/
SYS_RETSTATUS bsp_SPL06_Get_Calib_Coef(BSP_SPL06 *spl06)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	u8 dataHigh, dataMid, dataLow;
	
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C0_H, 1, &dataHigh);
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C0_L_C1_H, 1, &dataMid);
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C1_L, 1, &dataLow);	
	
	spl06->CoefData.c0 = ((s16)dataHigh << 4) | (dataMid >> 4);		/*c0 : 12bit*/
	spl06->CoefData.c0 = (spl06->CoefData.c0 & 0x0800) ? (0xF000 | spl06->CoefData.c0) : spl06->CoefData.c0;
	spl06->CoefData.c1 = ((s16)(dataMid & 0x0F) << 8) | dataLow;	/*c1 : 12bit*/
	spl06->CoefData.c1 = (spl06->CoefData.c1 & 0x0800) ? (0xF000 | spl06->CoefData.c1) : spl06->CoefData.c1;
	
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C00_H, 1, &dataHigh);
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C00_M, 1, &dataMid);
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C00_L_C10_H, 1, &dataLow); 
	
	spl06->CoefData.c00 = ((s32)dataHigh << 12) | ((s16)dataMid << 4) | (dataLow >> 4);	/*c00 : 20bit*/
	spl06->CoefData.c00 = (spl06->CoefData.c00 & 0x080000) ? (0xFFF00000 | spl06->CoefData.c00) : spl06->CoefData.c00;
	
	dataHigh = dataLow;

	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C10_M, 1, &dataMid);		
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C10_L, 1, &dataLow);
	
	spl06->CoefData.c10 = (((s32)dataHigh & 0x0f) << 16) | ((s16)dataMid << 8) | dataLow; /*c10 : 20bit*/
	spl06->CoefData.c10 = (spl06->CoefData.c10 & 0x080000) ? (0xFFF00000 | spl06->CoefData.c10) : spl06->CoefData.c10;
	
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C01_H, 1, &dataHigh);		
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C01_L, 1, &dataLow);

	spl06->CoefData.c01 = ((s16)dataHigh << 8) | dataLow;	/*c01 : 16bit*/
	
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C11_H, 1, &dataHigh);		
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C11_L, 1, &dataLow);

	spl06->CoefData.c11 = ((s16)dataHigh << 8) | dataLow;	/*c11 : 16bit*/

	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C20_H, 1, &dataHigh);		
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C20_L, 1, &dataLow);

	spl06->CoefData.c20 = ((s16)dataHigh << 8) | dataLow;	/*c20 : 16bit*/
	
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C21_H, 1, &dataHigh);		
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C21_L, 1, &dataLow);

	spl06->CoefData.c21 = ((s16)dataHigh << 8) | dataLow;	/*c21 : 16bit*/
	
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C30_H, 1, &dataHigh);		
	statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_COEF_C30_L, 1, &dataLow);

	spl06->CoefData.c30 = ((s16)dataHigh << 8) | dataLow;	/*c30 : 16bit*/
	
	return statusRet;
}

/*pressure rate set*/
SYS_RETSTATUS bsp_SPL06_Pressure_Rate_Set(BSP_SPL06 *spl06, u8 measureRate, u8 oversampleRate)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	u8 prsCfgReg = 0;
	u32 kp = 0;
	
	/*测量频率设置*/
	switch(measureRate)
	{
		case 1:
		{
			prsCfgReg |= SPL06_PRS_1_MEAS_RATE_W;	/*000*/
		}break;	
		
		case 2:
		{
			prsCfgReg |= SPL06_PRS_2_MEAS_RATE_W;	/*001*/		
		}break;
		
		case 4:
		{
			prsCfgReg |= SPL06_PRS_4_MEAS_RATE_W;	/*010*/					
		}break;

		case 8:
		{
			prsCfgReg |= SPL06_PRS_8_MEAS_RATE_W;	/*011*/					
		}break;

		case 16:
		{
			prsCfgReg |= SPL06_PRS_16_MEAS_RATE_W;	/*100*/			
		}break;

		case 32:
		{
			prsCfgReg |= SPL06_PRS_32_MEAS_RATE_W;	/*101*/			
		}break;

		case 64:
		{
			prsCfgReg |= SPL06_PRS_64_MEAS_RATE_W;	/*110*/			
		}break;

		case 128:
		{
			prsCfgReg |= SPL06_PRS_128_MEAS_RATE_W;	/*111*/			
		}break;	

		default:break;
	}
	
	/*细分采样设置(精度precision)*/	
	switch(oversampleRate)
	{
		/*single(单次)*/
		case 1:
		{
			prsCfgReg |= SPL06_PRS_1_OVERSAMP_RATE_W; /*0000*/
			kp = 524288;
		}break;
		
		/*low power(低功耗)*/
		case 2:
		{
			prsCfgReg |= SPL06_PRS_2_OVERSAMP_RATE_W; /*0001*/
			kp = 1572864;		
		}break;
		
		case 4:
		{
			prsCfgReg |= SPL06_PRS_4_OVERSAMP_RATE_W; /*0010*/			
			kp = 3670016;
		}break;

		case 8:
		{
			prsCfgReg |= SPL06_PRS_8_OVERSAMP_RATE_W; /*0011*/	
			kp = 7864320;
		}break;

		/*standard(标准)*/
		case 16:
		{
			prsCfgReg |= SPL06_PRS_16_OVERSAMP_RATE_W; /*0100*/			
			kp = 253952;			
		}break;

		case 32:
		{
			prsCfgReg |= SPL06_PRS_32_OVERSAMP_RATE_W; /*0101*/			
			kp = 516096;			
		}break;

		/*high precision(高精度)*/
		case 64:
		{
			prsCfgReg |= SPL06_PRS_64_OVERSAMP_RATE_W; /*0110*/			
			kp = 1040384;			
		}break;

		case 128:
		{
			prsCfgReg |= SPL06_PRS_128_OVERSAMP_RATE_W; /*0111*/			
			kp = 2088960;			
		}break;	
		
		default:break;
	}
	
	/*气压采样设置*/
	spl06->kP = kp;
	statusRet |= SPL06_I2C_WRITE_ONEBYTE(SPL06_SLAVEADDR, SPL06_PRS_CFG, prsCfgReg);
		
	if (oversampleRate > 8) /*> 8 times*/
	{
		statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_CFG_REG, 1, &prsCfgReg);
		statusRet |= SPL06_I2C_WRITE_ONEBYTE(SPL06_SLAVEADDR, SPL06_CFG_REG, prsCfgReg | SPL06_CFG_P_SHIFT_WR);
	}
	
	return statusRet;
}

/*temperature  rate set*/
SYS_RETSTATUS bsp_SPL06_Temperature_Rate_Set(BSP_SPL06 *spl06, u8 tmpSensorSelect, u8 measureRate, u8 oversampleRate)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	u8 tmpCfgReg = 0;
	u32 kt = 0;
	
	/*测量频率设置*/
	switch(measureRate)
	{
		case 1:
		{
			tmpCfgReg |= SPL06_TMP_1_MEAS_RATE_W;	/*000*/
		}break;	
		
		case 2:
		{
			tmpCfgReg |= SPL06_TMP_2_MEAS_RATE_W;	/*001*/		
		}break;
		
		case 4:
		{
			tmpCfgReg |= SPL06_TMP_4_MEAS_RATE_W;	/*010*/					
		}break;

		case 8:
		{
			tmpCfgReg |= SPL06_TMP_8_MEAS_RATE_W;	/*011*/					
		}break;

		case 16:
		{
			tmpCfgReg |= SPL06_TMP_16_MEAS_RATE_W;	/*100*/			
		}break;

		case 32:
		{
			tmpCfgReg |= SPL06_TMP_32_MEAS_RATE_W;	/*101*/			
		}break;

		case 64:
		{
			tmpCfgReg |= SPL06_TMP_64_MEAS_RATE_W;	/*110*/			
		}break;

		case 128:
		{
			tmpCfgReg |= SPL06_TMP_128_MEAS_RATE_W;	/*111*/			
		}break;	

		default:break;
	}
	
	/*细分采样设置(精度precision)*/	
	switch(oversampleRate)
	{
		/*single(单次)*/
		case 1:
		{
			tmpCfgReg |= SPL06_TMP_1_OVERSAMP_RATE_W; /*0000*/
			kt = 524288;
		}break;
		
		/*low power(低功耗)*/
		case 2:
		{
			tmpCfgReg |= SPL06_TMP_2_OVERSAMP_RATE_W; /*0001*/
			kt = 1572864;		
		}break;
		
		case 4:
		{
			tmpCfgReg |= SPL06_TMP_4_OVERSAMP_RATE_W; /*0010*/			
			kt = 3670016;
		}break;

		case 8:
		{
			tmpCfgReg |= SPL06_TMP_8_OVERSAMP_RATE_W; /*0011*/	
			kt = 7864320;
		}break;

		/*standard(标准)*/
		case 16:
		{
			tmpCfgReg |= SPL06_TMP_16_OVERSAMP_RATE_W; /*0100*/			
			kt = 253952;			
		}break;

		case 32:
		{
			tmpCfgReg |= SPL06_TMP_32_OVERSAMP_RATE_W; /*0101*/			
			kt = 516096;			
		}break;

		/*high precision(高精度)*/
		case 64:
		{
			tmpCfgReg |= SPL06_TMP_64_OVERSAMP_RATE_W; /*0110*/			
			kt = 1040384;			
		}break;

		case 128:
		{
			tmpCfgReg |= SPL06_TMP_128_OVERSAMP_RATE_W; /*0111*/			
			kt = 2088960;			
		}break;	
		
		default:break;
	}
	
	/*温度传感器选择内部/外部*/
	tmpCfgReg |= tmpSensorSelect;
	
	/*温度采样设置*/
	spl06->kT = kt;
	statusRet |= SPL06_I2C_WRITE_ONEBYTE(SPL06_SLAVEADDR, SPL06_TMP_CFG, tmpCfgReg);
		
	if (oversampleRate > 8)  /*> 8 times*/
	{
		statusRet |= SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_CFG_REG, 1, &tmpCfgReg);
		statusRet |= SPL06_I2C_WRITE_ONEBYTE(SPL06_SLAVEADDR, SPL06_CFG_REG, tmpCfgReg | SPL06_CFG_T_SHIFT_WR);
	}
	
	return statusRet;	
}

/*set measurement mode and type then start measure*/
SYS_RETSTATUS bsp_SPL06_Set_And_Start_Measure(BSP_SPL06 *spl06, SPL06_MEAS_MODE measMode, SPL06_MEAS_TYPE measType)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	
	statusRet |= SPL06_I2C_WRITE_ONEBYTE(SPL06_SLAVEADDR, SPL06_MEAS_CFG, measType);
	
	return statusRet;
}

/*spl06 get raw temperature(adc)*/
s32 bsp_SPL06_Get_Temperature_Adc(BSP_SPL06 *spl06)
{
	u8 dataBuff[3] = {0};
	s32 tmpAdc;
	
	SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_TMP_B2, 3, dataBuff);		

	tmpAdc = ((s32)dataBuff[0] << 16) | ((s32)dataBuff[1] << 8) | (s32)dataBuff[2];
	tmpAdc = (tmpAdc & 0x800000) ? (0xFF000000 | tmpAdc) : tmpAdc;
	
	spl06->TemperatureAdc = tmpAdc;	

	return (spl06->TemperatureAdc);	
}

/*spl06 get raw pressure(adc)*/
s32 bsp_SPL06_Get_Pressure_Adc(BSP_SPL06 *spl06)
{
	u8 dataBuff[3] = {0};
	s32 prsAdc;
	
	SPL06_I2C_READ_SOMEBYTE(SPL06_SLAVEADDR, SPL06_PSR_B2, 3, dataBuff);		
	
	prsAdc = ((s32)dataBuff[0] << 16) | ((s32)dataBuff[1] << 8) | (s32)dataBuff[2];
	prsAdc = (prsAdc & 0x800000) ? (0xFF000000 | prsAdc) : prsAdc;
	
	spl06->PressureAdc = prsAdc;
	
	return (spl06->PressureAdc);
}

/*spl06 get temperature*/
fp32 bsp_SPL06_Get_Temperature(BSP_SPL06 *spl06)
{
	fp32 Tcomp, Traw_sc;
	s32 rawTemperature;
	
	/*获取气压计温度原始值*/
	rawTemperature = bsp_SPL06_Get_Temperature_Adc(spl06);
	
	/*Calculate scaled measurement results*/
	Traw_sc = (fp32)rawTemperature / (fp32)spl06->kT;
	
	/*Calculate compensated measurement results*/
	Tcomp   = (spl06->CoefData.c0 * 0.5f) + (spl06->CoefData.c1 * Traw_sc);
	
	spl06->Temperature = Tcomp;
	
	return (spl06->Temperature);
}

/*spl06 get pressure(Pa)*/
fp32 bsp_SPL06_Get_Pressure(BSP_SPL06 *spl06)
{
	s32 rawPressure;
	fp32 Traw_sc, Praw_sc, Pcomp;
	fp32 qua2, qua3;	
	
	/*Calculate scaled measurement results*/
	Traw_sc = (fp32)spl06->TemperatureAdc / (fp32)spl06->kT;
	
	/*get raw pressure*/	
	rawPressure = bsp_SPL06_Get_Pressure_Adc(spl06);
	
	/*Calculate scaled measurement results*/
	Praw_sc = (fp32)rawPressure / (fp32)spl06->kP;
	
	/*Calculate compensated measurement results*/
	qua2  = spl06->CoefData.c10 + Praw_sc * (spl06->CoefData.c20 + Praw_sc * spl06->CoefData.c30);
	qua3  = Traw_sc * Praw_sc * (spl06->CoefData.c11 + Praw_sc * spl06->CoefData.c21);
	Pcomp = spl06->CoefData.c00 + Praw_sc * qua2 + Traw_sc * spl06->CoefData.c01 + qua3;
	
	spl06->Pressure = Pcomp;
	
	return (spl06->Pressure);
}

/*get spl06 altitude*/
/*
当前气压求海拔高度
			 ┌                                     ┐5.256
			 │                   ┌				 ┐ │
			 │			     H   │      6357	 │ │
Pa = 101.3 X │1 - 0.0255 X ----- │ ------------- │ │
			 │			    1000 │           H   │ │
			 │					 │  6357 + ----- │ │
			 │					 └          1000 ┘ │
			 └									   ┘
	Pa -- Current atmospheric pressure(当前大气压), kPa;
	H  -- Current altitude(当前海拔高度), m;

	等式两边各换算成Pa求海拔
*/
s32 bsp_SPL06_Get_Altitude(BSP_SPL06 *spl06, fp32 referencePa)
{
	fp32 A = 0, altM = 0;
	
	if (referencePa != 0)	/*初始化为0,违背除法法则*/
	{
		A = powf((spl06->Pressure / referencePa), 1 / 5.257f);
		altM = ((1 - A) * 6357000.0f) / (A + 161.1035f);	
	}

	spl06->rawAltitude = (s32)(altM * 100.0f);	/*m -> cm*/
	
	return (spl06->rawAltitude);
}
