#include "bsp_EEPROM_AT24CXX.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

/*AT24CXX set*/
BSP_AT24CXX g_sAt24cxx = 
{
	.At24CxxTarg = BSP_FAM_AT24C02,
	.MaxSize     = BSP_AT24C02_MAX_SIZE,
};

#ifdef STD_PROTOCOL_SOFTWARE_I2C
/**
  * @brief  Write a byte to the specified device address through SIM_I2C bus.
  * @param  SimI2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @param  Data: the data would be writen to the specified device address
  * @return  None
  */
static SYS_RETSTATUS At24cxx_SimI2c_WriteOneByte(u8 DevAddr, u8 WriteAddr, u8 Data)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif	
	
	FunctionalState statusRet;
	
	ssp_SimI2C_Start(g_sAt24cxx.SimI2cMaster);
	
	if (g_sAt24cxx.At24CxxTarg > BSP_FAM_AT24C16)
	{
		statusRet = ssp_SimI2C_WriteByte(g_sAt24cxx.SimI2cMaster, DevAddr); /*发送写命令*/
		
		if (statusRet == ENABLE)
		{
			statusRet = ssp_SimI2C_WriteByte(g_sAt24cxx.SimI2cMaster, WriteAddr >> 8);	 /*发送高地址*/	
		}
	}
	else
	{
		statusRet = ssp_SimI2C_WriteByte(g_sAt24cxx.SimI2cMaster, DevAddr + ((WriteAddr / 256) << 1)); 	/*送器件地址0XA0,写数据*/
	}
		
	if (statusRet == ENABLE)
	{
		statusRet = ssp_SimI2C_WriteByte(g_sAt24cxx.SimI2cMaster, WriteAddr % 256); 	/*发送低地址*/	

		if (statusRet == ENABLE)
		{				
			statusRet = ssp_SimI2C_WriteByte(g_sAt24cxx.SimI2cMaster, Data); 	/*发送字节*/

			if (statusRet == ENABLE)
			{
				ssp_SimI2C_Stop(g_sAt24cxx.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}
	
	ssp_SimI2C_Stop(g_sAt24cxx.SimI2cMaster);
	
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
static SYS_RETSTATUS At24cxx_SimI2c_ReadOneByte(u8 DevAddr, u8 ReadAddr, u8 *pData)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif		
	
	FunctionalState statusRet;
	
	ssp_SimI2C_Start(g_sAt24cxx.SimI2cMaster);
	
	if (g_sAt24cxx.At24CxxTarg > BSP_FAM_AT24C16)
	{
		statusRet = ssp_SimI2C_WriteByte(g_sAt24cxx.SimI2cMaster, DevAddr); /*发送写命令*/		
		
		if (statusRet == ENABLE)
		{
			statusRet = ssp_SimI2C_WriteByte(g_sAt24cxx.SimI2cMaster, ReadAddr >> 8);	 /*发送高地址*/		
		}
	}
	else
	{
		statusRet = ssp_SimI2C_WriteByte(g_sAt24cxx.SimI2cMaster, DevAddr + ((ReadAddr / 256) << 1)); /*发送器件地址0XA0,写数据 */	
	}		
		
	if (statusRet == ENABLE)
	{
		statusRet = ssp_SimI2C_WriteByte(g_sAt24cxx.SimI2cMaster, ReadAddr % 256); /*发送低地址*/

		if (statusRet == ENABLE)
		{
			ssp_SimI2C_Start(g_sAt24cxx.SimI2cMaster);

			statusRet = ssp_SimI2C_WriteByte(g_sAt24cxx.SimI2cMaster, DevAddr + 1); /*进入接收模式*/			

			if (statusRet == ENABLE)
			{
				*pData = ssp_SimI2C_ReadByte(g_sAt24cxx.SimI2cMaster, pData, DISABLE);
					
				ssp_SimI2C_Stop(g_sAt24cxx.SimI2cMaster);
				return SYS_RET_SUCC;
			}
		}
	}	

	ssp_SimI2C_Stop(g_sAt24cxx.SimI2cMaster);
	
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁释放*/
//	rt_mutex_release(&i2c_mutex);
#endif	
	
	return SYS_RET_FAIL;	
}
#endif

#ifdef STD_PROTOCOL_HARDWARE_I2C
/**
  * @brief  Write a byte to the specified device address through SIM_I2C bus.
  * @param  I2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @param  Data: the data would be writen to the specified device address
  * @return  None
  */
static SYS_RETSTATUS At24cxx_I2c_WriteOneByte(u8 DevAddr, u8 WriteAddr, u8 Data)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	u8 wrData = Data;

	ret = I2C_Write_SomeByte(g_sAt24cxx.I2cMaster, WriteAddr, &wrData, 1);
	
	return ret;
}

/**
  * @brief  Read an byte from the specified device address through SIM_I2C bus.
  * @param  I2CMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @return  the byte read from SIM_I2C bus
  */
static SYS_RETSTATUS At24cxx_I2c_ReadOneByte(u8 DevAddr, u8 ReadAddr, u8 *pData)
{
	SYS_RETSTATUS ret = SYS_RET_FAIL;
	
	ret = I2C_Read_SomeByte(g_sAt24cxx.I2cMaster, ReadAddr, pData, 1);
	
	return ret;
}
#endif

#ifdef  STD_PROTOCOL_SOFTWARE_I2C
#define AT24CXX_I2C_WRITE_ONEBYTE	 At24cxx_SimI2c_WriteOneByte
#define AT24CXX_I2C_READ_ONEBYTE	 At24cxx_SimI2c_ReadOneByte
#endif

#ifdef  STD_PROTOCOL_HARDWARE_I2C
#define AT24CXX_I2C_WRITE_ONEBYTE	 At24cxx_I2c_WriteOneByte
#define AT24CXX_I2C_READ_ONEBYTE 	 At24cxx_I2c_ReadOneByte
#endif

/*=== AT24CXX基本操作函数 ===*/
/*初始化AT24Cxx*/
#if defined(HW_CUT__USE_EEPROM_STOR)

SYS_RETSTATUS bsp_AT24CXX_Init(BSP_AT24CXX *AT24CXX)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	
	/*AT24CXX i2c init*/
	#if defined(STD_PROTOCOL_HARDWARE_I2C)
	AT24CXX->I2cMaster            = &g_sEePromI2C;
	AT24CXX->I2cMaster->slaveAddr = BSP_AT24CXX_SLAVEADDR;
	#endif
	
	#if defined(STD_PROTOCOL_SOFTWARE_I2C)
	AT24CXX->SimI2cMaster         = &g_sEePromSimI2C;
	#endif	
	
	sys_DelayMs(1);
	
	/*check if AT24CXX is exist*/
	if (bsp_AT24CXX_Check(AT24CXX) == SYS_RET_SUCC)
	{
		statusRet |= SYS_RET_SUCC; 
	}
	else 
	{
		statusRet |= SYS_RET_FAIL;
	}
	
	return statusRet;	
}
#endif

/*检测AT24CXX是否存在*/
u8 bsp_AT24CXX_Check(BSP_AT24CXX *AT24CXX)
{
	u8 existMark;
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
		
	/*read the mark*/
	existMark = bsp_AT24CXX_ReadOneByte(AT24CXX, AT24CXX->MaxSize, &existMark);	
	
	if (existMark == BSP_AT24CXX_EXISTMARK)
	{
		statusRet = SYS_RET_SUCC;
	}		   
	else	/*first init*/
	{
		bsp_AT24CXX_WriteOneByte(AT24CXX, AT24CXX->MaxSize, BSP_AT24CXX_EXISTMARK);
		
	    existMark = bsp_AT24CXX_ReadOneByte(AT24CXX, AT24CXX->MaxSize, &existMark);	  
		
		if (existMark == BSP_AT24CXX_EXISTMARK)
		{
			statusRet |= SYS_RET_SUCC;
		}
		else
		{
			statusRet |= SYS_RET_FAIL;
		}
	}
	
	return statusRet;			
}

/*主机向AT24CXX 写一个字节*/
void bsp_AT24CXX_WriteOneByte(BSP_AT24CXX *AT24CXX, u16 WriteAddr, u8 Data)
{
	AT24CXX_I2C_WRITE_ONEBYTE(BSP_AT24CXX_SLAVEADDR, WriteAddr, Data);
}

/*主机从AT24CXX 读一个字节*/
u8 bsp_AT24CXX_ReadOneByte(BSP_AT24CXX *AT24CXX, u16 ReadAddr, u8* pData)
{
	AT24CXX_I2C_READ_ONEBYTE(BSP_AT24CXX_SLAVEADDR, ReadAddr, pData);					   

	return *pData;
}

/*主机向AT24CXX 连续写入一些字节*/
void bsp_AT24CXX_WriteSomeByte(BSP_AT24CXX *AT24CXX, u16 WriteAddr, u8 *pBuff, u16 WriteNbr)
{
	while(WriteNbr)
	{
		bsp_AT24CXX_WriteOneByte(AT24CXX, WriteAddr, *pBuff);	
		WriteNbr--;
		WriteAddr++;
		pBuff++;
	}
}

/*主机从AT24CXX 连续读出一些字节*/
u8* bsp_AT24CXX_ReadSomeByte(BSP_AT24CXX *AT24CXX, u16 ReadAddr, u8 *pBuff, u16 ReadNbr)
{
	u8 *pTempBuff = pBuff;;
	
	while(ReadNbr)
	{
		*pBuff = bsp_AT24CXX_ReadOneByte(AT24CXX, ReadAddr, pBuff);	
		ReadNbr--;
		ReadAddr++;
		pBuff++;
	}
	
	return pTempBuff;
}

/*主机向AT24CXX 写入x字的数据(8/16/32)*/
void bsp_AT24CXX_WriteSizeByte(BSP_AT24CXX *AT24CXX, u16 WriteAddr, u32 Data, u8 Size)
{
	u8 i;
	
	for (i = 0; i < Size; i++)
	{
		bsp_AT24CXX_WriteOneByte(AT24CXX, WriteAddr + i, (Data << (8 * i)) & 0xff);
	}			
}

/*主机从AT24CXX 读出x字的数据(8/16/32)*/
u32 bsp_AT24CXX_ReadSizeByte(BSP_AT24CXX *AT24CXX, u16 ReadAddr, u32* pData, u8 Size)
{
	u8 i;
	
	for (i = 0; i < Size; i++) 
	{
		*pData <<= 8;
		*pData += bsp_AT24CXX_ReadOneByte(AT24CXX, ReadAddr + Size - i - 1, NULL);
	}

	return *pData;
}

/*=== AT24CXX读/写数 ===*/
/*写有符号32bit整型数*/
void bsp_AT24CXX_Write_1_S32Data(BSP_AT24CXX *AT24CXX, u16 WriteAddr, s32 s32Data)
{
	u8 i;
	
	/*申明一个联合体变量*/
	StorS32Data storS32Data;
	
	storS32Data.value = s32Data;

	for (i = 0; i < S32_BYTE_NUM; i++)
	{
		bsp_AT24CXX_WriteOneByte(AT24CXX, WriteAddr + i, storS32Data.byte[i]);
		
		/*连续写入数据时需要加1ms左右延时,否则写入失败*/
		sys_DelayMs(5);
	}
}

/*读有符号32bit整型数*/
SYS_RETSTATUS bsp_AT24CXX_Read_1_S32Data(BSP_AT24CXX *AT24CXX, u16 ReadAddr, s32 *s32Data)
{
	SYS_RETSTATUS readRet = SYS_RET_SUCC;
	u8 i;

	/*申明一个联合体变量*/
	StorS32Data storS32Data;

	for (i = 0; i < S32_BYTE_NUM; i++)
	{
		storS32Data.byte[i] = bsp_AT24CXX_ReadOneByte(AT24CXX, ReadAddr + i, (storS32Data.byte + i));
		
		/*连续写入数据时需要加1ms左右延时,否则写入失败*/
		sys_DelayMs(5);		
	}
	
	/*该位置没有有效数据,EEPROM默认0xff*/
	if ((storS32Data.byte[0] == 0xff) && \
		(storS32Data.byte[1] == 0xff) && \
		(storS32Data.byte[2] == 0xff) && \
		(storS32Data.byte[3] == 0xff))
	{
		readRet = SYS_RET_FAIL;
	}
	else
	{
		*s32Data = storS32Data.value;
	}

	return readRet;	
}

/*写无符号32bit整型数*/
void bsp_AT24CXX_Write_1_U32Data(BSP_AT24CXX *AT24CXX, u16 WriteAddr, u32 u32Data)
{
	u8 i;
	
	/*申明一个联合体变量*/
	StorU32Data storU32Data;
	
	storU32Data.value = u32Data;

	for (i = 0; i < U32_BYTE_NUM; i++)
	{
		bsp_AT24CXX_WriteOneByte(AT24CXX, WriteAddr + i, storU32Data.byte[i]);
		
		/*连续写入数据时需要加1ms左右延时,否则写入失败*/
		sys_DelayMs(5);
	}
}

/*读无符号32bit整型数*/
SYS_RETSTATUS bsp_AT24CXX_Read_1_U32Data(BSP_AT24CXX *AT24CXX, u16 ReadAddr, u32 *u32Data)
{
	SYS_RETSTATUS readRet = SYS_RET_SUCC;
	u8 i;

	/*申明一个联合体变量*/
	StorU32Data storU32Data;

	for (i = 0; i < U32_BYTE_NUM; i++)
	{
		storU32Data.byte[i] = bsp_AT24CXX_ReadOneByte(AT24CXX, ReadAddr + i, (storU32Data.byte + i));
		
		/*连续写入数据时需要加1ms左右延时,否则写入失败*/
		sys_DelayMs(5);		
	}
	
	/*该位置没有有效数据,EEPROM默认0xff*/
	if ((storU32Data.byte[0] == 0xff) && \
		(storU32Data.byte[1] == 0xff) && \
		(storU32Data.byte[2] == 0xff) && \
		(storU32Data.byte[3] == 0xff))
	{
		readRet = SYS_RET_FAIL;
	}
	else
	{
		*u32Data = storU32Data.value;
	}

	return readRet;	
}

/*写有符号16bit整型数*/
void bsp_AT24CXX_Write_1_S16Data(BSP_AT24CXX *AT24CXX, u16 WriteAddr, s16 s16Data)
{
	u8 i;
	
	/*申明一个联合体变量*/
	StorS16Data storS16Data;
	
	storS16Data.value = s16Data;

	for (i = 0; i < S16_BYTE_NUM; i++)
	{
		bsp_AT24CXX_WriteOneByte(AT24CXX, WriteAddr + i, storS16Data.byte[i]);
		
		/*连续写入数据时需要加1ms左右延时,否则写入失败*/
		sys_DelayMs(5);
	}
}

/*读有符号16bit整型数*/
SYS_RETSTATUS bsp_AT24CXX_Read_1_S16Data(BSP_AT24CXX *AT24CXX, u16 ReadAddr, s16 *s16Data)
{
	SYS_RETSTATUS readRet = SYS_RET_SUCC;
	u8 i;

	/*申明一个联合体变量*/
	StorS16Data storS16Data;

	for (i = 0; i < S16_BYTE_NUM; i++)
	{
		storS16Data.byte[i] = bsp_AT24CXX_ReadOneByte(AT24CXX, ReadAddr + i, (storS16Data.byte + i));
		
		/*连续写入数据时需要加1ms左右延时,否则写入失败*/
		sys_DelayMs(5);		
	}
	
	/*该位置没有有效数据,EEPROM默认0xff*/
	if ((storS16Data.byte[0] == 0xff) && \
		(storS16Data.byte[1] == 0xff))
	{
		readRet = SYS_RET_FAIL;
	}
	else
	{
		*s16Data = storS16Data.value;
	}

	return readRet;	
}

/*写无符号16bit整型数*/
void bsp_AT24CXX_Write_1_U16Data(BSP_AT24CXX *AT24CXX, u16 WriteAddr, u16 u16Data)
{
	u8 i;
	
	/*申明一个联合体变量*/
	StorU16Data storU16Data;
	
	storU16Data.value = u16Data;

	for (i = 0; i < U16_BYTE_NUM; i++)
	{
		bsp_AT24CXX_WriteOneByte(AT24CXX, WriteAddr + i, storU16Data.byte[i]);
		
		/*连续写入数据时需要加1ms左右延时,否则写入失败*/
		sys_DelayMs(5);
	}
}

/*读无符号16bit整型数*/
SYS_RETSTATUS bsp_AT24CXX_Read_1_U16Data(BSP_AT24CXX *AT24CXX, u16 ReadAddr, u16 *u16Data)
{
	SYS_RETSTATUS readRet = SYS_RET_SUCC;
	u8 i;

	/*申明一个联合体变量*/
	StorU16Data storU16Data;

	for (i = 0; i < U16_BYTE_NUM; i++)
	{
		storU16Data.byte[i] = bsp_AT24CXX_ReadOneByte(AT24CXX, ReadAddr + i, (storU16Data.byte + i));
		
		/*连续写入数据时需要加1ms左右延时,否则写入失败*/
		sys_DelayMs(5);		
	}
	
	/*该位置没有有效数据,EEPROM默认0xff*/
	if ((storU16Data.byte[0] == 0xff) && \
		(storU16Data.byte[1] == 0xff))
	{
		readRet = SYS_RET_FAIL;
	}
	else
	{
		*u16Data = storU16Data.value;
	}

	return readRet;	
}

/*写浮点数*/
void bsp_AT24CXX_Write_1_FloatData(BSP_AT24CXX *AT24CXX, u16 WriteAddr, fp32 fpData)
{
	u8 i;
	
	/*申明一个联合体变量*/
	StorFloatData floatData;
	
	floatData.value = fpData;

	for (i = 0; i < FLOAT_BYTE_NUM; i++)
	{
		bsp_AT24CXX_WriteOneByte(AT24CXX, WriteAddr + i, floatData.byte[i]);
		
		/*连续写入数据时需要加1ms左右延时,否则写入失败*/
		sys_DelayMs(5);
	}
}
/*读浮点数*/
SYS_RETSTATUS bsp_AT24CXX_Read_1_FloatData(BSP_AT24CXX *AT24CXX, u16 ReadAddr, fp32 *fpData)
{
	SYS_RETSTATUS readRet = SYS_RET_SUCC;
	u8 i;

	/*申明一个联合体变量*/
	StorFloatData floatData;

	for (i = 0; i < FLOAT_BYTE_NUM; i++)
	{
		floatData.byte[i] = bsp_AT24CXX_ReadOneByte(AT24CXX, ReadAddr + i, (floatData.byte + i));
		
		/*连续写入数据时需要加1ms左右延时,否则写入失败*/
		sys_DelayMs(5);		
	}
	
	/*该位置没有有效数据,EEPROM默认0xff*/
	if ((floatData.byte[0] == 0xff) && \
		(floatData.byte[1] == 0xff) && \
		(floatData.byte[2] == 0xff) && \
		(floatData.byte[3] == 0xff))
	{
		readRet = SYS_RET_FAIL;
	}
	else
	{
		*fpData = floatData.value;
	}

	return readRet;	
}

/*=== AT24CXX功能操作函数 ===*/
/*向AT24CXX写入3个有符号整数*/
void bsp_AT24CXX_Write_3_S32Data(BSP_AT24CXX *AT24CXX, u16 WriteAddr, s32 s32Data1, s32 s32Data2, s32 s32Data3)
{
	/*写入第一个有符号整数*/
	bsp_AT24CXX_Write_1_S32Data(AT24CXX, WriteAddr, s32Data1);

	WriteAddr += S32_BYTE_NUM; 	/*写入目标地址偏移*/
	
	/*写入第二个有符号整数*/
	bsp_AT24CXX_Write_1_S32Data(AT24CXX, WriteAddr, s32Data2);
	
	WriteAddr += S32_BYTE_NUM; 	/*写入目标地址偏移*/
	
	/*写入第三个有符号整数*/
	bsp_AT24CXX_Write_1_S32Data(AT24CXX, WriteAddr, s32Data3);
}

/*从AT24CXX读出3个有符号整数*/
SYS_RETSTATUS bsp_AT24CXX_Read_3_S32Data(BSP_AT24CXX *AT24CXX, u16 ReadAddr, s32 *s32Data1, s32 *s32Data2, s32 *s32Data3)
{
	SYS_RETSTATUS readRet = SYS_RET_SUCC;
	
	/*读出第一个有符号整数*/
	readRet |= bsp_AT24CXX_Read_1_S32Data(AT24CXX, ReadAddr, s32Data1);
	
	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}

	ReadAddr += S32_BYTE_NUM; 	/*读出目标地址偏移*/
	
	/*读出第二个有符号整数*/
	readRet |= bsp_AT24CXX_Read_1_S32Data(AT24CXX, ReadAddr, s32Data2);

	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}	
	
	ReadAddr += S32_BYTE_NUM; 	/*读出目标地址偏移*/
	
	/*读出第三个有符号整数*/
	readRet |= bsp_AT24CXX_Read_1_S32Data(AT24CXX, ReadAddr, s32Data3);

	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}	
	
	/*readRet != 0, Read Error!*/	
	
	return readRet;
}

/*向AT24CXX写入3个无符号整数*/
void bsp_AT24CXX_Write_3_U32Data(BSP_AT24CXX *AT24CXX, u16 WriteAddr, u32 u32Data1, u32 u32Data2, u32 u32Data3)
{
	/*写入第一个无符号整数*/
	bsp_AT24CXX_Write_1_U32Data(AT24CXX, WriteAddr, u32Data1);

	WriteAddr += U32_BYTE_NUM; 	/*写入目标地址偏移*/
	
	/*写入第二个无符号整数*/
	bsp_AT24CXX_Write_1_U32Data(AT24CXX, WriteAddr, u32Data2);
	
	WriteAddr += U32_BYTE_NUM; 	/*写入目标地址偏移*/
	
	/*写入第三个无符号整数*/
	bsp_AT24CXX_Write_1_U32Data(AT24CXX, WriteAddr, u32Data3);
}

/*从AT24CXX读出3个无符号整数*/
SYS_RETSTATUS bsp_AT24CXX_Read_3_U32Data(BSP_AT24CXX *AT24CXX, u16 ReadAddr, u32 *u32Data1, u32 *u32Data2, u32 *u32Data3)
{
	SYS_RETSTATUS readRet = SYS_RET_SUCC;
	
	/*读出第一个无符号整数*/
	readRet |= bsp_AT24CXX_Read_1_U32Data(AT24CXX, ReadAddr, u32Data1);
	
	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}

	ReadAddr += U32_BYTE_NUM; 	/*读出目标地址偏移*/
	
	/*读出第二个无符号整数*/
	readRet |= bsp_AT24CXX_Read_1_U32Data(AT24CXX, ReadAddr, u32Data2);

	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}	
	
	ReadAddr += U32_BYTE_NUM; 	/*读出目标地址偏移*/
	
	/*读出第三个无符号整数*/
	readRet |= bsp_AT24CXX_Read_1_U32Data(AT24CXX, ReadAddr, u32Data3);

	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}	
	
	/*readRet != 0, Read Error!*/	
	
	return readRet;
}

/*向AT24CXX写入3个有符号16bit整数*/
void bsp_AT24CXX_Write_3_S16Data(BSP_AT24CXX *AT24CXX, u16 WriteAddr, s16 s16Data1, s16 s16Data2, s16 s16Data3)
{
	/*写入第一个无符号整数*/
	bsp_AT24CXX_Write_1_S16Data(AT24CXX, WriteAddr, s16Data1);

	WriteAddr += S16_BYTE_NUM; 	/*写入目标地址偏移*/
	
	/*写入第二个无符号整数*/
	bsp_AT24CXX_Write_1_S16Data(AT24CXX, WriteAddr, s16Data2);
	
	WriteAddr += S16_BYTE_NUM; 	/*写入目标地址偏移*/
	
	/*写入第三个无符号整数*/
	bsp_AT24CXX_Write_1_S16Data(AT24CXX, WriteAddr, s16Data3);
}

/*从AT24CXX读出3个有符号16bit整数*/
SYS_RETSTATUS bsp_AT24CXX_Read_3_S16Data(BSP_AT24CXX *AT24CXX, u16 ReadAddr, s16 *s16Data1, s16 *s16Data2, s16 *s16Data3)
{
	SYS_RETSTATUS readRet = SYS_RET_SUCC;
	
	/*读出第一个无符号整数*/
	readRet |= bsp_AT24CXX_Read_1_S16Data(AT24CXX, ReadAddr, s16Data1);
	
	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}

	ReadAddr += S16_BYTE_NUM; 	/*读出目标地址偏移*/
	
	/*读出第二个无符号整数*/
	readRet |= bsp_AT24CXX_Read_1_S16Data(AT24CXX, ReadAddr, s16Data2);

	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}	
	
	ReadAddr += S16_BYTE_NUM; 	/*读出目标地址偏移*/
	
	/*读出第三个无符号整数*/
	readRet |= bsp_AT24CXX_Read_1_S16Data(AT24CXX, ReadAddr, s16Data3);

	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}	
	
	/*readRet != 0, Read Error!*/	
	
	return readRet;
}

/*向AT24CXX写入3个无符号16bit整数*/
void bsp_AT24CXX_Write_3_U16Data(BSP_AT24CXX *AT24CXX, u16 WriteAddr, u16 u16Data1, u16 u16Data2, u16 u16Data3)
{
	/*写入第一个无符号整数*/
	bsp_AT24CXX_Write_1_U16Data(AT24CXX, WriteAddr, u16Data1);

	WriteAddr += U16_BYTE_NUM; 	/*写入目标地址偏移*/
	
	/*写入第二个无符号整数*/
	bsp_AT24CXX_Write_1_U16Data(AT24CXX, WriteAddr, u16Data2);
	
	WriteAddr += U16_BYTE_NUM; 	/*写入目标地址偏移*/
	
	/*写入第三个无符号整数*/
	bsp_AT24CXX_Write_1_U16Data(AT24CXX, WriteAddr, u16Data3);
}

/*从AT24CXX读出3个无符号16bit整数*/
SYS_RETSTATUS bsp_AT24CXX_Read_3_U16Data(BSP_AT24CXX *AT24CXX, u16 ReadAddr, u16 *u16Data1, u16 *u16Data2, u16 *u16Data3)
{
	SYS_RETSTATUS readRet = SYS_RET_SUCC;
	
	/*读出第一个无符号整数*/
	readRet |= bsp_AT24CXX_Read_1_U16Data(AT24CXX, ReadAddr, u16Data1);
	
	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}

	ReadAddr += U16_BYTE_NUM; 	/*读出目标地址偏移*/
	
	/*读出第二个无符号整数*/
	readRet |= bsp_AT24CXX_Read_1_U16Data(AT24CXX, ReadAddr, u16Data2);

	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}	
	
	ReadAddr += U16_BYTE_NUM; 	/*读出目标地址偏移*/
	
	/*读出第三个无符号整数*/
	readRet |= bsp_AT24CXX_Read_1_U16Data(AT24CXX, ReadAddr, u16Data3);

	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}	
	
	/*readRet != 0, Read Error!*/	
	
	return readRet;
}


/*向AT24CXX写入3个浮点数*/
void bsp_AT24CXX_Write_3_FloatData(BSP_AT24CXX *AT24CXX, u16 WriteAddr, fp32 fpData1, fp32 fpData2, fp32 fpData3)
{
	/*写入第一个浮点数*/
	bsp_AT24CXX_Write_1_FloatData(AT24CXX, WriteAddr, fpData1);

	WriteAddr += FLOAT_BYTE_NUM; 	/*写入目标地址偏移*/
	
	/*写入第二个浮点数*/
	bsp_AT24CXX_Write_1_FloatData(AT24CXX, WriteAddr, fpData2);
	
	WriteAddr += FLOAT_BYTE_NUM; 	/*写入目标地址偏移*/
	
	/*写入第三个浮点数*/
	bsp_AT24CXX_Write_1_FloatData(AT24CXX, WriteAddr, fpData3);
}

/*从AT24CXX读出3个浮点数*/
SYS_RETSTATUS bsp_AT24CXX_Read_3_FloatData(BSP_AT24CXX *AT24CXX, u16 ReadAddr, fp32 *fpData1, fp32 *fpData2, fp32 *fpData3)
{
	SYS_RETSTATUS readRet = SYS_RET_SUCC;
	
	/*读出第一个浮点数*/
	readRet |= bsp_AT24CXX_Read_1_FloatData(AT24CXX, ReadAddr, fpData1);
	
	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}

	ReadAddr += FLOAT_BYTE_NUM; 	/*读出目标地址偏移*/
	
	/*读出第二个浮点数*/
	readRet |= bsp_AT24CXX_Read_1_FloatData(AT24CXX, ReadAddr, fpData2);

	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}	
	
	ReadAddr += FLOAT_BYTE_NUM; 	/*读出目标地址偏移*/
	
	/*读出第三个浮点数*/
	readRet |= bsp_AT24CXX_Read_1_FloatData(AT24CXX, ReadAddr, fpData3);

	if (readRet != SYS_RET_SUCC)
	{
		readRet = SYS_RET_FAIL;
	}	
	
	/*readRet != 0, Read Error!*/	
	
	return readRet;
}
