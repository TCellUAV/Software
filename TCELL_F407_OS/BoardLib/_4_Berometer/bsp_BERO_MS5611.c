#include "bsp_BERO_MS5611.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

BSP_MS5611 g_sMs5611;

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
/**
  * @brief  Write a byte to the specified device address through SIM_I2C bus.
  * @param  mcSimI2cMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  DevCmd: the command would be writen to the specified device address
  * @return  None
  */
static SYS_RETSTATUS Ms5611_SimI2c_WriteOneByte(u8 DevAddr, u8 RegAddr, u8 Data)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif		
	
	FunctionalState state;
	
	ssp_SimI2C_Start(g_sMs5611.SimI2cMaster);
	
	state = ssp_SimI2C_WriteByte(g_sMs5611.SimI2cMaster, (DevAddr << 1) | 0);
	
	if (state == ENABLE)
	{
		state = ssp_SimI2C_WriteByte(g_sMs5611.SimI2cMaster, RegAddr);
		
		if (state == ENABLE)
		{
			state = ssp_SimI2C_WriteByte(g_sMs5611.SimI2cMaster, Data);
			
			if (state == ENABLE)
			{
				ssp_SimI2C_Stop(g_sMs5611.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}
	
	ssp_SimI2C_Stop(g_sMs5611.SimI2cMaster);
	
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
static SYS_RETSTATUS  Ms5611_SimI2c_ReadSomeByte(u8 DevAddr, u8 DevCmd, u8 len, u8 *RdBuff)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif	
	
	FunctionalState state;
	
	ssp_SimI2C_Start(g_sMs5611.SimI2cMaster);
	
	state = ssp_SimI2C_WriteByte(g_sMs5611.SimI2cMaster, (DevAddr << 1) | 0);
	
	if (state == ENABLE)
	{
		state = ssp_SimI2C_WriteByte(g_sMs5611.SimI2cMaster, DevCmd);
		
		if (state == ENABLE)
		{
			ssp_SimI2C_Start(g_sMs5611.SimI2cMaster);	
			
			state = ssp_SimI2C_WriteByte(g_sMs5611.SimI2cMaster, (DevAddr << 1) | 1);
			
			if (state == ENABLE)
			{
				while(len)
				{
					if (len == 1)
					{
						*RdBuff = ssp_SimI2C_ReadByte(g_sMs5611.SimI2cMaster, RdBuff, DISABLE);
					}
					else
					{
						*RdBuff = ssp_SimI2C_ReadByte(g_sMs5611.SimI2cMaster, RdBuff, ENABLE);
					}
					
					len--;
					RdBuff++;
				}
				
				ssp_SimI2C_Stop(g_sMs5611.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}
	
	ssp_SimI2C_Stop(g_sMs5611.SimI2cMaster);
	
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
static SYS_RETSTATUS Ms5611_I2c_WriteOneByte(u8 DevAddr, u8 RegAddr, u8 Data)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	u8 wrData = Data;
	
	ret = I2C_Write_SomeByte(g_sMs5611.I2cMaster, RegAddr, &wrData, 1);
	
	return ret;
}

/**
  * @brief  Read an byte from the specified device address through SIM_I2C bus.
  * @param  mcSimI2cMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @return  the byte read from SIM_I2C bus
  */
static SYS_RETSTATUS  Ms5611_I2c_ReadSomeByte(u8 DevAddr, u8 DevCmd, u8 len, u8 *RdBuff)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	ret = I2C_Read_SomeByte(g_sMs5611.I2cMaster, DevCmd, RdBuff, len);
	
	return ret;	
}

#endif

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
#define MS5611_I2C_WRITE_ONEBYTE	 Ms5611_SimI2c_WriteOneByte
#define MS5611_I2C_READ_SOMEBYTE	 Ms5611_SimI2c_ReadSomeByte
#endif

#if defined(STD_PROTOCOL_HARDWARE_I2C)
#define MS5611_I2C_WRITE_ONEBYTE	 Ms5611_I2c_WriteOneByte
#define MS5611_I2C_READ_SOMEBYTE 	 Ms5611_I2c_ReadSomeByte
#endif


/*MS5611初始化*/
#if defined(HW_CUT__USE_MD_BERO)

SYS_RETSTATUS bsp_MS5611_Init(BSP_MS5611 *ms5611)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	
	/*ms5611 i2c init*/
	#if defined(STD_PROTOCOL_HARDWARE_I2C)	
	ms5611->I2cMaster            = &g_sBeroI2C;
	ms5611->I2cMaster->slaveAddr = (MS5611_ADDR << 1);
	#endif
	
	#if defined(STD_PROTOCOL_SOFTWARE_I2C)	
	ms5611->SimI2cMaster         = &g_sBeroSimI2C;
	#endif
	
	sys_DelayMs(1);
	
	/*reset*/
	MS5611_I2C_WRITE_ONEBYTE(MS5611_ADDR, MS5611_RESET, 0);
	sys_DelayMs(100);
	
	if (bsp_MS5611_ReadPROM(ms5611) != SYS_RET_SUCC)
	{
		statusRet = SYS_RET_FAIL;
	}
	
	return statusRet;
}
#endif

/*读MS5611的PROM(出厂校准参数)*/
SYS_RETSTATUS bsp_MS5611_ReadPROM(BSP_MS5611 *ms5611)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	u8 i;
	
	for (i = 0; i < MS5611_PROM_REG_COUNT; i++)
	{
		statusRet |= MS5611_I2C_READ_SOMEBYTE(MS5611_ADDR, MS5611_PROM_BASE_ADDR + i * MS5611_PROM_REG_SIZE, \
											  MS5611_PROM_REG_SIZE, ms5611->Buff + i * MS5611_PROM_REG_SIZE);
	}
	
	if (statusRet == SYS_RET_SUCC)
	{
		ms5611->PROM_C[1] = ((u16)(ms5611->Buff[0] << 8)) | ms5611->Buff[1];
		ms5611->PROM_C[2] = ((u16)(ms5611->Buff[2] << 8)) | ms5611->Buff[3];
		ms5611->PROM_C[3] = ((u16)(ms5611->Buff[4] << 8)) | ms5611->Buff[5];
		ms5611->PROM_C[4] = ((u16)(ms5611->Buff[6] << 8)) | ms5611->Buff[7];
		ms5611->PROM_C[5] = ((u16)(ms5611->Buff[8] << 8)) | ms5611->Buff[9];
		ms5611->PROM_C[6] = ((u16)(ms5611->Buff[10] << 8)) | ms5611->Buff[11];		
	}
	
	return statusRet;
}

/*获取处理后的温度值*/
s32 bsp_MS5611_GetTemperature(BSP_MS5611 *ms5611, u8 OSR)
{
	s32 dT, TEMP = 0;
	
	dT   = bsp_MS5611_GetDeltaTemperature(ms5611, OSR);
	TEMP = 2000l + dT * (ms5611->PROM_C[6] >> 23);
	
	ms5611->Temperature = TEMP;
	
	return (ms5611->Temperature);
}

/*获取处理后的气压值*/	
s32 bsp_MS5611_GetPressure(BSP_MS5611 *ms5611, u8 OSR)
{
	s64 OFF, SENS, OFF2, SENS2;
	s32 P, dT, T2, TEMP;
	u32 D1;
	
	/*get dT*/
	dT   = bsp_MS5611_GetDeltaTemperature(ms5611, OSR);
	
	/*get Actual temprature*/
	TEMP = 2000l + dT * (ms5611->PROM_C[6] >> 23); 
	ms5611->Temperature = TEMP;
	
	/*get raw pressure*/
	D1   = bsp_MS5611_GetRawPressure(ms5611, OSR);
	
	OFF  = (ms5611->PROM_C[2] << 16) + ((ms5611->PROM_C[4] * dT) >> 7);
	SENS = (ms5611->PROM_C[1] << 15) + ((ms5611->PROM_C[3] * dT) >> 8);
	
	if (TEMP < 2000) /*TEMP < 20st.C*/
	{
		T2    = (dT * dT) >> 31;
		OFF2  = 5 * (TEMP - 2000) * (TEMP - 2000) >> 1;
		SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) >> 2;
		
		if (TEMP < -1500) /*TEMP < -15st.C*/
		{
			OFF2  = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
			SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500) >> 1);
		}
	}
	else
	{
		T2    = 0;
		OFF2  = 0;
		SENS2 = 0;	
	}
	
	TEMP = TEMP - T2;
	OFF  = OFF - OFF2;
	SENS = SENS - SENS2;
	
	P = ((D1 * SENS >> 21) - OFF) >> 15;
	
	ms5611->Pressure = P;
	
	return P;
}

/*获取dT*/
s32 bsp_MS5611_GetDeltaTemperature(BSP_MS5611 *ms5611, u8 OSR)
{
	s32 dT;
	u32 D2;
	
	D2 = bsp_MS5611_GetRawTemperature(ms5611, OSR);
	dT = D2 - (ms5611->PROM_C[5] << 8);
	
	return dT;
}

/*获取原始温度数据*/
u32 bsp_MS5611_GetRawTemperature(BSP_MS5611 *ms5611, u8 OSR)
{
	u32 rawData = 0;
	
	rawData = bsp_MS5611_DoConversion(ms5611, MS5611_D2 + OSR);
	
	return rawData;
}

/*获取原始气压数据*/	
u32 bsp_MS5611_GetRawPressure(BSP_MS5611 *ms5611, u8 OSR)
{
	u32 rawData = 0;
	
	rawData = bsp_MS5611_DoConversion(ms5611, MS5611_D1 + OSR);
	
	return rawData;
}

/*读24bit ADC(未补偿的温度和气压)值*/
u32 bsp_MS5611_DoConversion(BSP_MS5611 *ms5611, u8 DevCmd)
{
	SYS_RETSTATUS staRet = SYS_RET_SUCC;
	u8 conver1 = 0, conver2 = 0, conver3 = 0;
	u32 converRet = 0;

	MS5611_I2C_WRITE_ONEBYTE(MS5611_ADDR, DevCmd, 0);
	
	staRet |= MS5611_I2C_READ_SOMEBYTE(MS5611_ADDR, MS5611_ADC_READ, MS5611_D1D2_SIZE, ms5611->Buff);
	
	if (staRet != SYS_RET_SUCC)
	{
		return 0xffffffff;
	}
	
	conver1 = ms5611->Buff[0];
	conver2 = ms5611->Buff[1];
	conver3 = ms5611->Buff[2];
		
	converRet = ((u32)(conver1 << 16) + (u16)(conver2 << 8) + conver3);  //24位ADC

	return converRet;
}

/*将气压值转换为海拔高度*/
fp32 bsp_MS5611_GetAltitude(BSP_MS5611 *ms5611)
{
	/*①formula*/
//	ms5611->Altitude = ((powf((MS5611_SEA_LEVEL_PRESS / ms5611->Pressure), 1.0f / 5.257f) - 1.0f) * (ms5611->Temperature + 273.15f)) / 0.0065f;
	
	/*②formula*/
	ms5611->Altitude = 4433000.0f * (1.0f - powf((SEA_LEVEL_PRESSURE / ms5611->Pressure), 1.0f / 5.257f));
	
	return (ms5611->Altitude);
}	
