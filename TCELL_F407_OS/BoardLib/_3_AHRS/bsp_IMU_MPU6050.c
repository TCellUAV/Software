#include "bsp_IMU_MPU6050.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

BSP_MPU6050 g_sMpu6050 = 
{
	.GyroZero.x = 0,
	.GyroZero.y = 0,
	.GyroZero.z = 0,	
};

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
/**
  * @brief  Write a byte to the specified device address through SIM_I2C bus.
  * @param  SimI2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @param  Data: the data would be writen to the specified device address
  * @return  None
  */
static SYS_RETSTATUS Mpu6050_SimI2c_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif	
	
	FunctionalState state;
	
	ssp_SimI2C_Start(g_sMpu6050.SimI2cMaster);
	
	state = ssp_SimI2C_WriteByte(g_sMpu6050.SimI2cMaster, (DevAddr << 1) | 0);
	
	if (state == ENABLE)
	{
		state = ssp_SimI2C_WriteByte(g_sMpu6050.SimI2cMaster, RegAddr);
		
		if (state == ENABLE)
		{
			state = ssp_SimI2C_WriteByte(g_sMpu6050.SimI2cMaster, Data);
			
			if (state == ENABLE)
			{
				ssp_SimI2C_Stop(g_sMpu6050.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}
	
	ssp_SimI2C_Stop(g_sMpu6050.SimI2cMaster);
	
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
static SYS_RETSTATUS Mpu6050_SimI2c_ReadSomeByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t len, uint8_t *RdBuff)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif	
	
	FunctionalState state;
	
	ssp_SimI2C_Start(g_sMpu6050.SimI2cMaster);
	
	state = ssp_SimI2C_WriteByte(g_sMpu6050.SimI2cMaster, (DevAddr << 1) | 0);
	
	if (state == ENABLE)
	{
		state = ssp_SimI2C_WriteByte(g_sMpu6050.SimI2cMaster, RegAddr);
		
		if (state == ENABLE)
		{
			ssp_SimI2C_Start(g_sMpu6050.SimI2cMaster);	
			
			state = ssp_SimI2C_WriteByte(g_sMpu6050.SimI2cMaster, (DevAddr << 1) | 1);
			
			if (state == ENABLE)
			{
				while(len)
				{
					if (len == 1)
					{
						*RdBuff = ssp_SimI2C_ReadByte(g_sMpu6050.SimI2cMaster, RdBuff, DISABLE);
					}
					else
					{
						*RdBuff = ssp_SimI2C_ReadByte(g_sMpu6050.SimI2cMaster, RdBuff, ENABLE);
					}
					
					len--;
					RdBuff++;
				}
				
				ssp_SimI2C_Stop(g_sMpu6050.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}

	ssp_SimI2C_Stop(g_sMpu6050.SimI2cMaster);
	
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
static SYS_RETSTATUS Mpu6050_I2c_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	u8 wrData = Data;
	
	ret = I2C_Write_SomeByte(g_sMpu6050.I2cMaster, RegAddr, &wrData, 1);
	
	return ret;	
}

/**
  * @brief  Read an byte from the specified device address through SIM_I2C bus.
  * @param  SimI2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @return  the byte read from SIM_I2C bus
  */
static SYS_RETSTATUS Mpu6050_I2c_ReadSomeByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t len, uint8_t *RdBuff)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	ret = I2C_Read_SomeByte(g_sMpu6050.I2cMaster, RegAddr, RdBuff, len);
	
	return ret;	
}
#endif

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
#define MPU6050_I2C_WRITE_ONEBYTE	 Mpu6050_SimI2c_WriteOneByte
#define MPU6050_I2C_READ_SOMEBYTE	 Mpu6050_SimI2c_ReadSomeByte
#endif

#if defined(STD_PROTOCOL_HARDWARE_I2C)
#define MPU6050_I2C_WRITE_ONEBYTE	 Mpu6050_I2c_WriteOneByte
#define MPU6050_I2C_READ_SOMEBYTE 	 Mpu6050_I2c_ReadSomeByte
#endif

/*MPU6050 初始化*/
#if defined(HW_CUT__USE_MD_IMU)

SYS_RETSTATUS bsp_MPU6050_Init(BSP_MPU6050* mpu6050)
{	
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	u8 i;
	Gyro3s ImuGyroData;
	Gyro3s *pImuGyroData = &ImuGyroData;

	/*mpu6050 i2c init*/
	#if defined(STD_PROTOCOL_HARDWARE_I2C)
	mpu6050->I2cMaster            = &g_sImuI2C;
	mpu6050->I2cMaster->slaveAddr = (MPU_SLAVEADDR << 1);
	#endif
	
	#if defined(STD_PROTOCOL_SOFTWARE_I2C)
	mpu6050->SimI2cMaster         = &g_sImuSimI2C;
	#endif
	
	sys_DelayMs(1);
	
	/*mpu6050 init*/
	MPU6050_I2C_WRITE_ONEBYTE(MPU_SLAVEADDR, MPU_PWR_MGMT_1, 0x80);  	/*复位,禁止进入低功耗模式,使能温度传感器,内部8Mhz*/
	sys_DelayMs(100);
	
	MPU6050_I2C_WRITE_ONEBYTE(MPU_SLAVEADDR, MPU_PWR_MGMT_1, 0x00);		/*唤醒MPU6050*/	
	MPU6050_I2C_WRITE_ONEBYTE(MPU_SLAVEADDR, MPU_CONFIG, 0x02);			/*低通滤波器,滤波带宽A:94, G:98, FS:1Khz*/	
	MPU6050_I2C_WRITE_ONEBYTE(MPU_SLAVEADDR, MPU_SMPRT_DIV, 0x00);		/*陀螺仪采样频率1khz,Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)*/	
	MPU6050_I2C_WRITE_ONEBYTE(MPU_SLAVEADDR, MPU_GYRO_CONFIG, 0x10);	/*陀螺仪量程范围:±1000°/s*/	
	MPU6050_I2C_WRITE_ONEBYTE(MPU_SLAVEADDR, MPU_ACCEL_CONFIG, 0x08);	/*加速度计量程范围:±4g*/	
	
	if (bsp_MPU6050_GetId(mpu6050) != MPU_SLAVEADDR)
	{
		statusRet = SYS_RET_FAIL;
	}	
	
	sys_DelayMs(500);
	
	/*陀螺仪零偏值*/
	for (i = 0; i < 100; i++)
	{
		pImuGyroData = bsp_MPU6050_GetGyro(mpu6050);
		
		mpu6050->GyroZero.x += pImuGyroData->x;
		mpu6050->GyroZero.y += pImuGyroData->y;
		mpu6050->GyroZero.z += pImuGyroData->z;
		sys_DelayMs(5);
	}
	
	mpu6050->GyroZero.x /= 100.0f;
	mpu6050->GyroZero.y /= 100.0f;
	mpu6050->GyroZero.z /= 100.0f;
	
	return statusRet;
}
#endif


/*获取从机地址*/
u8 bsp_MPU6050_GetId(BSP_MPU6050* mpu6050)
{
	u8 slaveId = 0;
	
	if (MPU6050_I2C_READ_SOMEBYTE(MPU_SLAVEADDR, MPU_WHO_AM_I, 1, &slaveId) != SYS_RET_SUCC)
	{
		slaveId = 0xff;
	}
	
	return slaveId;	
}

/*获取温度值*/
fp32 bsp_MPU6050_GetTemp(BSP_MPU6050* mpu6050)
{
	u8 tempBuff[2] = {0};
	s16 uwValue  = 0;
	fp32 tempRet  = 0;
	
	if (MPU6050_I2C_READ_SOMEBYTE(MPU_SLAVEADDR, MPU_TEMP_OUT_H, 2, tempBuff) == SYS_RET_SUCC)
	{
		uwValue = (((tempBuff[0]) & 0xff) << 8) | tempBuff[1];
		tempRet = 36.53f + (uwValue/ 340.0f);
	}
	else
	{
		tempRet = 255.0f;
	}
	
	return tempRet;
}

 /*获取加速度*/
Acc3s* bsp_MPU6050_GetAcc(BSP_MPU6050* mpu6050)
{
	u8 accBuff[6] = {0};	
	
	if (MPU6050_I2C_READ_SOMEBYTE(MPU_SLAVEADDR, MPU_ACCEL_XOUT_H, 6, accBuff) == SYS_RET_SUCC)
	{
		mpu6050->Acc.x = ((s16)(accBuff[0] << 8) | accBuff[1]);
		mpu6050->Acc.y = ((s16)(accBuff[2] << 8) | accBuff[3]);
		mpu6050->Acc.z = ((s16)(accBuff[4] << 8) | accBuff[5]);		
	}
	
	return &(mpu6050->Acc);
}

/*获取角速度*/
Gyro3s* bsp_MPU6050_GetGyro(BSP_MPU6050* mpu6050)
{
	u8 gyroBuff[6] = {0};
	
	if (MPU6050_I2C_READ_SOMEBYTE(MPU_SLAVEADDR, MPU_GYRO_XOUT_H, 6, gyroBuff) == SYS_RET_SUCC)
	{
		mpu6050->Gyro.x = ((s16)(gyroBuff[0] << 8) | gyroBuff[1]);
		mpu6050->Gyro.y = ((s16)(gyroBuff[2] << 8) | gyroBuff[3]);
		mpu6050->Gyro.z = ((s16)(gyroBuff[4] << 8) | gyroBuff[5]);		
	}	
	
	return &(mpu6050->Gyro);	
}
