#include "bsp_MAG_HMC5883L.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

BSP_HMC5883L g_sHmc5883l;

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
/**
  * @brief  Write a byte to the specified device address through SIM_I2C bus.
  * @param  SimI2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @param  Data: the data would be writen to the specified device address
  * @return  None
  */
static SYS_RETSTATUS Hmc5883l_SimI2c_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif	
	
	FunctionalState state;
	
	ssp_SimI2C_Start(g_sHmc5883l.SimI2cMaster);
	
	state = ssp_SimI2C_WriteByte(g_sHmc5883l.SimI2cMaster, (DevAddr << 1) | 0);
	
	if (state == ENABLE)
	{
		state = ssp_SimI2C_WriteByte(g_sHmc5883l.SimI2cMaster, RegAddr);
		
		if (state == ENABLE)
		{
			state = ssp_SimI2C_WriteByte(g_sHmc5883l.SimI2cMaster, Data);
			
			if (state == ENABLE)
			{
				ssp_SimI2C_Stop(g_sHmc5883l.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}
	
	ssp_SimI2C_Stop(g_sHmc5883l.SimI2cMaster);
	
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁释放*/
//	rt_mutex_release(&i2c_mutex);
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
static SYS_RETSTATUS Hmc5883l_SimI2c_ReadSomeByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t len, uint8_t *RdBuff)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif	
	
	FunctionalState state;
	
	ssp_SimI2C_Start(g_sHmc5883l.SimI2cMaster);
	
	state = ssp_SimI2C_WriteByte(g_sHmc5883l.SimI2cMaster, (DevAddr << 1) | 0);
	
	if (state == ENABLE)
	{
		state = ssp_SimI2C_WriteByte(g_sHmc5883l.SimI2cMaster, RegAddr);
		
		if (state == ENABLE)
		{
			ssp_SimI2C_Start(g_sHmc5883l.SimI2cMaster);	
			
			state = ssp_SimI2C_WriteByte(g_sHmc5883l.SimI2cMaster, (DevAddr << 1) | 1);
			
			if (state == ENABLE)
			{
				while(len)
				{
					if (len == 1)
					{
						*RdBuff = ssp_SimI2C_ReadByte(g_sHmc5883l.SimI2cMaster, RdBuff, DISABLE);
					}
					else
					{
						*RdBuff = ssp_SimI2C_ReadByte(g_sHmc5883l.SimI2cMaster, RdBuff, ENABLE);
					}
					
					len--;
					RdBuff++;
				}
				
				ssp_SimI2C_Stop(g_sHmc5883l.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}

	ssp_SimI2C_Stop(g_sHmc5883l.SimI2cMaster);
	
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
  * @param  SimI2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @param  Data: the data would be writen to the specified device address
  * @return  None
  */
static SYS_RETSTATUS Hmc5883l_I2c_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	u8 wrData = Data;
	
	ret = I2C_Write_SomeByte(g_sHmc5883l.I2cMaster, RegAddr, &wrData, 1);
	
	return ret;
}

/**
  * @brief  Read an byte from the specified device address through SIM_I2C bus.
  * @param  SimI2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @return  the byte read from SIM_I2C bus
  */
static SYS_RETSTATUS Hmc5883l_I2c_ReadSomeByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t len, uint8_t *RdBuff)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	ret = I2C_Read_SomeByte(g_sHmc5883l.I2cMaster, RegAddr, RdBuff, len);
	
	return ret;	
}
#endif

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
#define HMC5883L_I2C_WRITE_ONEBYTE	 Hmc5883l_SimI2c_WriteOneByte
#define HMC5883L_I2C_READ_SOMEBYTE	 Hmc5883l_SimI2c_ReadSomeByte
#endif

#if defined(STD_PROTOCOL_HARDWARE_I2C)
#define HMC5883L_I2C_WRITE_ONEBYTE	 Hmc5883l_I2c_WriteOneByte
#define HMC5883L_I2C_READ_SOMEBYTE 	 Hmc5883l_I2c_ReadSomeByte
#endif


/*HMC5883L 初始化*/	
#if defined(HW_CUT__USE_MD_MAG)

SYS_RETSTATUS bsp_HMC5883L_Init(BSP_HMC5883L* hmc5883l)
{
	u8 identBuff[3] = {0};
	SYS_RETSTATUS statusRet           = SYS_RET_SUCC;
	HMC5883L_IDENT_STATUS identStatus = HMC5883L_IDENT_OK;
	
	/*hmc5883l i2c init*/
#ifdef HW_CUT__USE_MD_MAG
#ifdef HW_CUT__USE_GPS_MAG
	#if defined(STD_PROTOCOL_HARDWARE_I2C)
	hmc5883l->I2cMaster            = &g_sGpsMagI2C;		/*挂在GPS从机I2C下*/
	hmc5883l->I2cMaster->slaveAddr = (HMC5883L_SLAVEADDR << 1);
	#endif
	
	#if defined(STD_PROTOCOL_SOFTWARE_I2C)
	hmc5883l->SimI2cMaster         = &g_sGpsMagSimI2C;
	#endif
#else
	#if defined(STD_PROTOCOL_HARDWARE_I2C)	
	hmc5883l->I2cMaster            = &g_sMagI2C;			/*挂在板载I2C下*/
	hmc5883l->I2cMaster->slaveAddr = (HMC5883L_SLAVEADDR << 1);
	#endif
	
	#if defined(STD_PROTOCOL_SOFTWARE_I2C)
	hmc5883l->SimI2cMaster         = &g_sMagSimI2C;
	#endif	
#endif
#endif
	
	sys_DelayMs(1);
	
	/*识别A*/
	statusRet = HMC5883L_I2C_READ_SOMEBYTE(HMC5883L_SLAVEADDR, HMC5883L_IDENT_REG_A, 1, identBuff);
	
	if (identBuff[0] != HMC5883L_IDENT_A_CONST_VAL)
	{
		identStatus |= HMC5883L_IDENT_FAIL;
	}
	
	/*识别B*/
	statusRet = HMC5883L_I2C_READ_SOMEBYTE(HMC5883L_SLAVEADDR, HMC5883L_IDENT_REG_B, 1, identBuff + 1);	

	if (identBuff[1] != HMC5883L_IDENT_B_CONST_VAL)
	{
		identStatus |= HMC5883L_IDENT_FAIL;
	}	
	
	/*识别C*/
	statusRet = HMC5883L_I2C_READ_SOMEBYTE(HMC5883L_SLAVEADDR, HMC5883L_IDENT_REG_C, 1, identBuff + 2);
	
	if (identBuff[2] != HMC5883L_IDENT_C_CONST_VAL)
	{
		identStatus |= HMC5883L_IDENT_FAIL;
	}
	
	/*判断HMC5883L是否识别成功*/
	if (identStatus == HMC5883L_IDENT_OK)
	{
		statusRet = SYS_RET_SUCC;
	}
	
	/*采样平均数, 输出频率75hz*/
	statusRet |= HMC5883L_I2C_WRITE_ONEBYTE(HMC5883L_SLAVEADDR, HMC5883L_CONF_REG_A, 0x78);
	
	/*增益,适用磁场范围:±8.10Ga, 高斯增益:230*/
	statusRet |= HMC5883L_I2C_WRITE_ONEBYTE(HMC5883L_SLAVEADDR, HMC5883L_CONF_REG_B, 0xE0);

	/*连续测量模式*/
	statusRet |= HMC5883L_I2C_WRITE_ONEBYTE(HMC5883L_SLAVEADDR, HMC5883L_MODE_REG, 0x00);	
	
	return statusRet;
}
#endif

/*读取3个轴的数据*/
Mag3s *bsp_HMC5883L_Get_Mag_Data(BSP_HMC5883L *hmc5883l)
{
	u8 magBuff[6] = {0};
	
	if (HMC5883L_I2C_READ_SOMEBYTE(HMC5883L_SLAVEADDR, HMC5883L_DATA_X_MSB_REG, 6, magBuff) == SYS_RET_SUCC)
	{
		hmc5883l->Mag.x = ((s16)(magBuff[0] << 8) | magBuff[1]);
		hmc5883l->Mag.z = ((s16)(magBuff[2] << 8) | magBuff[3]);
		hmc5883l->Mag.y = ((s16)(magBuff[4] << 8) | magBuff[5]);		
	}
	
	/*座标系调整,为了和IMU座标系一致*/
	hmc5883l->Mag.x =  hmc5883l->Mag.x;
	hmc5883l->Mag.y = -hmc5883l->Mag.y;
	hmc5883l->Mag.z = -hmc5883l->Mag.z;
	
	return &(hmc5883l->Mag);
}

/*测试:获取磁力计的Yaw(水平位置测试偏航角)*/
fp32 bsp_HMC5883L_Get_Mag_Yaw(BSP_HMC5883L *hmc5883l)
{
	fp32 curentYaw;
	
	/*读取磁力计3个轴的数据*/
	bsp_HMC5883L_Get_Mag_Data(hmc5883l);
	
	/*3轴数据处理*/
	
	/*计算水平Yaw角*/
	curentYaw = (atan2(hmc5883l->Mag.y, hmc5883l->Mag.x) * (180 / PI) + 180);  

	hmc5883l->testMagYaw = curentYaw;	
	
	return curentYaw;
}
