#include "bsp_MAG_HMC5983.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

BSP_HMC5983 g_sHmc5983;

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
/**
  * @brief  Write a byte to the specified device address through SIM_I2C bus.
  * @param  SimI2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @param  Data: the data would be writen to the specified device address
  * @return  None
  */
static SYS_RETSTATUS Hmc5983_SimI2c_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
#ifdef HW_CUT__USE_MD_MAG
#ifdef HW_CUT__USE_GPS_MAG
//	rt_mutex_take(&gps_mag_i2c_mutex, 5);
#else
//	rt_mutex_take(&i2c_mutex, 5);	
#endif
#endif
#endif
	
	FunctionalState state;
	
	ssp_SimI2C_Start(g_sHmc5983.SimI2cMaster);
	
	state = ssp_SimI2C_WriteByte(g_sHmc5983.SimI2cMaster, (DevAddr << 1) | 0);
	
	if (state == ENABLE)
	{
		state = ssp_SimI2C_WriteByte(g_sHmc5983.SimI2cMaster, RegAddr);
		
		if (state == ENABLE)
		{
			state = ssp_SimI2C_WriteByte(g_sHmc5983.SimI2cMaster, Data);
			
			if (state == ENABLE)
			{
				ssp_SimI2C_Stop(g_sHmc5983.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}
	
	ssp_SimI2C_Stop(g_sHmc5983.SimI2cMaster);
	
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁释放*/
#ifdef HW_CUT__USE_MD_MAG
#ifdef HW_CUT__USE_GPS_MAG
//	rt_mutex_release(&gps_mag_i2c_mutex);
#else
//	rt_mutex_release(&i2c_mutex);
#endif
#endif
#endif	
	
	return SYS_RET_FAIL;
}

/**
  * @brief  Read an byte from the specified device address through SIM_I2C bus.
  * @param  SimI2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @return  the byte read from SIM_I2C bus
  */
static SYS_RETSTATUS Hmc5983_SimI2c_ReadSomeByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t len, uint8_t *RdBuff)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
#ifdef HW_CUT__USE_MD_MAG
#ifdef HW_CUT__USE_GPS_MAG
//	rt_mutex_take(&gps_mag_i2c_mutex, 5);
#else
//	rt_mutex_take(&i2c_mutex, 5);	
#endif
#endif
#endif	
	
	FunctionalState state;
	
	ssp_SimI2C_Start(g_sHmc5983.SimI2cMaster);
	
	state = ssp_SimI2C_WriteByte(g_sHmc5983.SimI2cMaster, (DevAddr << 1) | 0);
	
	if (state == ENABLE)
	{
		state = ssp_SimI2C_WriteByte(g_sHmc5983.SimI2cMaster, RegAddr);
		
		if (state == ENABLE)
		{
			ssp_SimI2C_Start(g_sHmc5983.SimI2cMaster);	
			
			state = ssp_SimI2C_WriteByte(g_sHmc5983.SimI2cMaster, (DevAddr << 1) | 1);
			
			if (state == ENABLE)
			{
				while(len)
				{
					if (len == 1)
					{
						*RdBuff = ssp_SimI2C_ReadByte(g_sHmc5983.SimI2cMaster, RdBuff, DISABLE);
					}
					else
					{
						*RdBuff = ssp_SimI2C_ReadByte(g_sHmc5983.SimI2cMaster, RdBuff, ENABLE);
					}
					
					len--;
					RdBuff++;
				}
				
				ssp_SimI2C_Stop(g_sHmc5983.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}

	ssp_SimI2C_Stop(g_sHmc5983.SimI2cMaster);
	
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁释放*/
#ifdef HW_CUT__USE_MD_MAG
#ifdef HW_CUT__USE_GPS_MAG
//	rt_mutex_release(&gps_mag_i2c_mutex);
#else
//	rt_mutex_release(&i2c_mutex);
#endif
#endif
#endif	
	
	return SYS_RET_FAIL;	
}
#endif

#if defined(STD_PROTOCOL_HARDWARE_I2C)
/**
  * @brief  Write a byte to the specified device address through SIM_I2C bus.
  * @param  SimI2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @param  Data: the data would be writen to the specified device address
  * @return  None
  */
static SYS_RETSTATUS Hmc5983_I2c_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	u8 wrData = Data;
	
	ret = I2C_Write_SomeByte(g_sHmc5983.I2cMaster, RegAddr, &wrData, 1);
	
	return ret;
}

/**
  * @brief  Read an byte from the specified device address through SIM_I2C bus.
  * @param  SimI2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @return  the byte read from SIM_I2C bus
  */
static SYS_RETSTATUS Hmc5983_I2c_ReadSomeByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t len, uint8_t *RdBuff)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	ret = I2C_Read_SomeByte(g_sHmc5983.I2cMaster, RegAddr, RdBuff, len);
	
	return ret;	
}
#endif

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
#define HMC5983_I2C_WRITE_ONEBYTE	 Hmc5983_SimI2c_WriteOneByte
#define HMC5983_I2C_READ_SOMEBYTE	 Hmc5983_SimI2c_ReadSomeByte
#endif

#if defined(STD_PROTOCOL_HARDWARE_I2C)
#define HMC5983_I2C_WRITE_ONEBYTE	 Hmc5983_I2c_WriteOneByte
#define HMC5983_I2C_READ_SOMEBYTE 	 Hmc5983_I2c_ReadSomeByte
#endif


/*HMC5983 初始化*/	
#if defined(HW_CUT__USE_MD_MAG)

SYS_RETSTATUS bsp_HMC5983_Init(BSP_HMC5983* hmc5983)
{
	u8 identBuff[3] = {0};
	SYS_RETSTATUS statusRet          = SYS_RET_SUCC;
	HMC5983_IDENT_STATUS identStatus = HMC5983_IDENT_OK;
	
	/*hmc5983 i2c init*/
#ifdef HW_CUT__USE_MD_MAG
#ifdef HW_CUT__USE_GPS_MAG
	#if defined(STD_PROTOCOL_HARDWARE_I2C)
	hmc5983->I2cMaster            = &g_sGpsMagI2C;		/*挂在GPS从机I2C下*/
	hmc5983->I2cMaster->slaveAddr = (HMC5983_SLAVEADDR << 1);
	#endif
	
	#if defined(STD_PROTOCOL_SOFTWARE_I2C)
	hmc5983->SimI2cMaster         = &g_sGpsMagSimI2C;
	#endif
#else
	#if defined(STD_PROTOCOL_HARDWARE_I2C)	
	hmc5983->I2cMaster            = &g_sMagI2C;			/*挂在板载I2C下*/
	hmc5983->I2cMaster->slaveAddr = (HMC5983_SLAVEADDR << 1);
	#endif
	
	#if defined(STD_PROTOCOL_SOFTWARE_I2C)
	hmc5983->SimI2cMaster         = &g_sMagSimI2C;
	#endif	
#endif
#endif	
	
	sys_DelayMs(1);
	
	/*识别A*/
	statusRet = HMC5983_I2C_READ_SOMEBYTE(HMC5983_SLAVEADDR, HMC5983_IDENT_REG_A, 1, identBuff);
	
	if (identBuff[0] != HMC5983_IDENT_A_CONST_VAL)
	{
		identStatus |= HMC5983_IDENT_FAIL;
	}
	
	/*识别B*/
	statusRet = HMC5983_I2C_READ_SOMEBYTE(HMC5983_SLAVEADDR, HMC5983_IDENT_REG_B, 1, identBuff + 1);	

	if (identBuff[1] != HMC5983_IDENT_B_CONST_VAL)
	{
		identStatus |= HMC5983_IDENT_FAIL;
	}	
	
	/*识别C*/
	statusRet = HMC5983_I2C_READ_SOMEBYTE(HMC5983_SLAVEADDR, HMC5983_IDENT_REG_C, 1, identBuff + 2);
	
	if (identBuff[2] != HMC5983_IDENT_C_CONST_VAL)
	{
		identStatus |= HMC5983_IDENT_FAIL;
	}
	
	/*判断HMC5983是否识别成功*/
	if (identStatus == HMC5983_IDENT_OK)
	{
		statusRet = SYS_RET_SUCC;
	}
	
	/*采样平均数, 输出频率75hz*/
	statusRet |= HMC5983_I2C_WRITE_ONEBYTE(HMC5983_SLAVEADDR, HMC5983_CONF_REG_A, 0x78);
	
	/*增益,适用磁场范围:±8.10Ga, 高斯增益:230*/
	statusRet |= HMC5983_I2C_WRITE_ONEBYTE(HMC5983_SLAVEADDR, HMC5983_CONF_REG_B, 0xE0);

	/*连续测量模式*/
	statusRet |= HMC5983_I2C_WRITE_ONEBYTE(HMC5983_SLAVEADDR, HMC5983_MODE_REG, 0x00);	
	
	return statusRet;
}
#endif

/*读取3个轴的数据*/
Mag3s *bsp_HMC5983_Get_Mag_Data(BSP_HMC5983 *hmc5983)
{
	u8 magBuff[6] = {0};
	
	if (HMC5983_I2C_READ_SOMEBYTE(HMC5983_SLAVEADDR, HMC5983_DATA_X_MSB_REG, 6, magBuff) == SYS_RET_SUCC)
	{
		hmc5983->Mag.x = ((s16)(magBuff[0] << 8) | magBuff[1]);
		hmc5983->Mag.z = ((s16)(magBuff[2] << 8) | magBuff[3]);
		hmc5983->Mag.y = ((s16)(magBuff[4] << 8) | magBuff[5]);		
	}
	
	/*座标系调整,为了和IMU座标系一致*/
	hmc5983->Mag.x =  hmc5983->Mag.x;
	hmc5983->Mag.y = -hmc5983->Mag.y;
	hmc5983->Mag.z = -hmc5983->Mag.z;
	
	return &(hmc5983->Mag);
}

/*读取芯片内部温度数据*/
fp32 bsp_HMC5983_Get_Temperature(BSP_HMC5983 *hmc5983)
{
	u8 tempBuff[2] = {0};
	
	if (HMC5983_I2C_READ_SOMEBYTE(HMC5983_SLAVEADDR, HMC5983_TEMP_MSB_REG, 2, tempBuff) == SYS_RET_SUCC)
	{
		hmc5983->temperature = (tempBuff[0] * 2^8 + tempBuff[1]) / (2^4 * 8) + 25.0f;
	}
	else
	{
		hmc5983->temperature = 255.0f;
	}
	
	return (hmc5983->temperature);
}

/*测试:获取磁力计的Yaw(水平位置测试偏航角)*/
fp32 bsp_HMC5983_Get_Mag_Yaw(BSP_HMC5983 *hmc5983)
{
	fp32 curentYaw;
	
	/*读取磁力计3个轴的数据*/
	bsp_HMC5983_Get_Mag_Data(hmc5983);
	
	/*3轴数据处理*/
	
	/*计算水平Yaw角*/
	curentYaw = (atan2(hmc5983->Mag.y, hmc5983->Mag.x) * (180 / PI) + 180);  

	hmc5983->testMagYaw = curentYaw;	
	
	return curentYaw;
}
