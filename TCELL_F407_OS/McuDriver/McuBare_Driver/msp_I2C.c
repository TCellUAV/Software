#include "msp_I2C.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

/*I2C初始化*/
void msp_I2c_Init(MSP_I2c *i2c)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	I2C_InitTypeDef   I2C_InitStruct;
	RCC_ClocksTypeDef RCC_Clocks;
	
	/*SCL GPIO*/
	GPIO_InitStruct.GPIO_Pin   = i2c->SCL_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(i2c->SCL_GPIO, &GPIO_InitStruct);
	
	/*SDA GPIO*/
	GPIO_InitStruct.GPIO_Pin   = i2c->SDA_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(i2c->SDA_GPIO, &GPIO_InitStruct);	
	
	/*Tx/Rx PinAF*/
	GPIO_PinAFConfig(i2c->SCL_GPIO, i2c->SCL_PinSource, i2c->GPIO_AF);
	GPIO_PinAFConfig(i2c->SDA_GPIO, i2c->SDA_PinSource, i2c->GPIO_AF);
	
	/*Uart Init*/
	I2C_InitStruct.I2C_Mode 			   = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle 		   = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 		   = 0x00;	/*STM32 的自身地址,不与从器件相同即可*/
	I2C_InitStruct.I2C_Ack 				   = I2C_Ack_Enable;
	I2C_InitStruct.I2C_ClockSpeed 		   = i2c->I2C_CLOCK_SPEED;	/*指定时钟总线速率,100/400kHz */
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(i2c->I2c, &I2C_InitStruct);
	
	/* I2C Initialize */
    I2C_Cmd(i2c->I2c, ENABLE);
	
    /*超时设置*/
    RCC_GetClocksFreq(&RCC_Clocks);
    i2c->timeOut = (RCC_Clocks.SYSCLK_Frequency /10000);	
}


/*I2C读若干字节*/
SYS_RETSTATUS I2C_Read_SomeByte(MSP_I2c *i2c, u8 rdAddr, u8 *rdBuff, u8 rdNbr)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif
	
	SYS_RETSTATUS ret = SYS_RET_SUCC;
	u32 timeCnt;
	
	timeCnt = i2c->timeOut;
	while((--timeCnt) && (I2C_GetFlagStatus(i2c->I2c, I2C_FLAG_BUSY))){};
	if (timeCnt == 0)
	{
		ret = SYS_RET_FAIL;
	}
	
	/*发送I2C的START信号,接口自动从从设备编程主设备*/
    I2C_GenerateSTART(i2c->I2c, ENABLE);

	timeCnt = i2c->timeOut;
    while((--timeCnt) && (!I2C_CheckEvent(i2c->I2c, I2C_EVENT_MASTER_MODE_SELECT))){};
	if (timeCnt == 0)
	{
		ret = SYS_RET_FAIL;
	}

    I2C_Send7bitAddress(i2c->I2c, i2c->slaveAddr, I2C_Direction_Transmitter);
	timeCnt = i2c->timeOut;
    while((--timeCnt) && (!I2C_CheckEvent(i2c->I2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))){};
	if (timeCnt == 0)
	{
		ret = SYS_RET_FAIL;
	}
	
	I2C_Cmd(i2c->I2c,ENABLE);	/*使能外设*/

    I2C_SendData(i2c->I2c, rdAddr);
	timeCnt = i2c->timeOut;
    while((--timeCnt) && (!I2C_CheckEvent(i2c->I2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED))){};
	if (timeCnt == 0)
	{
		ret = SYS_RET_FAIL;
	}

    I2C_GenerateSTART(i2c->I2c, ENABLE);
	timeCnt = i2c->timeOut;
    while((--timeCnt) && (!I2C_CheckEvent(i2c->I2c, I2C_EVENT_MASTER_MODE_SELECT))){};
	if (timeCnt == 0)
	{
		ret = SYS_RET_FAIL;
	}

    I2C_Send7bitAddress(i2c->I2c, i2c->slaveAddr, I2C_Direction_Receiver);
	timeCnt = i2c->timeOut;
    while((--timeCnt) && (!I2C_CheckEvent(i2c->I2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))){};
	if (timeCnt == 0)
	{
		ret = SYS_RET_FAIL;
	}
	
	/*多字节读取*/
	while(rdNbr)
	{	
		if(rdNbr == 1)
		{
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(i2c->I2c, DISABLE);

			/* Send STOP Condition */
			I2C_GenerateSTOP(i2c->I2c, ENABLE);
		}

		/* Test on EV7 and clear it */
		timeCnt = i2c->timeOut;
		while((--timeCnt) && (!I2C_CheckEvent(i2c->I2c, I2C_EVENT_MASTER_BYTE_RECEIVED))){};
		if (timeCnt == 0)
		{
			ret = SYS_RET_FAIL;		
			
			break;
		}
	
		/* Read a byte from the MPU6050 */
		*rdBuff = I2C_ReceiveData(i2c->I2c);

		/* Point to the next location where the byte read will be saved */
		rdBuff++;

		/* Decrement the read bytes counter */
		rdNbr--;
	}

    I2C_AcknowledgeConfig(i2c->I2c, ENABLE);
	
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁释放*/
//	rt_mutex_release(&i2c_mutex);
#endif	

    return ret;		
}

/*I2C写若干字节*/
SYS_RETSTATUS I2C_Write_SomeByte(MSP_I2c *i2c, u8 wrAddr, u8 *wrBuff, u8 wrNbr)
{
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁获取*/
//	rt_mutex_take(&i2c_mutex, 5);
#endif	
	
	SYS_RETSTATUS ret = SYS_RET_SUCC;
	u32 timeCnt;
	
    timeCnt = i2c->timeOut;
    while((--timeCnt) && I2C_GetFlagStatus(i2c->I2c, I2C_FLAG_BUSY)){};
	if (timeCnt == 0)
	{
		ret = SYS_RET_FAIL;		
	} 

    I2C_GenerateSTART(i2c->I2c, ENABLE);
    timeCnt = i2c->timeOut;
    while((--timeCnt) && (!I2C_CheckEvent(i2c->I2c, I2C_EVENT_MASTER_MODE_SELECT))){};
	if (timeCnt == 0)
	{
		ret = SYS_RET_FAIL;		
	} 

    I2C_Send7bitAddress(i2c->I2c, i2c->slaveAddr, I2C_Direction_Transmitter);
    timeCnt = i2c->timeOut;
    while((--timeCnt) && (!I2C_CheckEvent(i2c->I2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))){};
	if (timeCnt == 0)
	{
		ret = SYS_RET_FAIL;		
	}

    I2C_SendData(i2c->I2c, wrAddr);
    timeCnt = i2c->timeOut;
    while((--timeCnt) && (!I2C_CheckEvent(i2c->I2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED))){};
	if (timeCnt == 0)
	{
		ret = SYS_RET_FAIL;		
	} 
	
	/*写入若干字节*/
	while((wrNbr--) && (ret == SYS_RET_SUCC))
	{
		I2C_SendData(i2c->I2c, *wrBuff);
		
		wrBuff++;
		
		timeCnt = i2c->timeOut;
		while((--timeCnt) && (!I2C_CheckEvent(i2c->I2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED))){};
		if (timeCnt == 0)
		{
			ret = SYS_RET_FAIL;		
		}
	}

    I2C_GenerateSTOP(i2c->I2c, ENABLE);
	
#ifdef PLATFORM_RTOS__RT_THREAD
	/*I2C互斥锁释放*/
//	rt_mutex_release(&i2c_mutex);
#endif	
	
	return ret;
}

