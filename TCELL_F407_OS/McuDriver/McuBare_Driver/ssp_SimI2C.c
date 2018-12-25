#include "ssp_SimI2C.h"

void ssp_SimI2C_Init(SSP_SimI2C *SimI2C)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/*SIM I2C SCL GPIO Init*/
	GPIO_InitStruct.GPIO_Pin   = SimI2C->SCL_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;	
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(SimI2C->SCL_GPIO, &GPIO_InitStruct);
	
	/*SIM I2C SDA GPIO Init*/	
	GPIO_InitStruct.GPIO_Pin   = SimI2C->SDA_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;	
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(SimI2C->SDA_GPIO, &GPIO_InitStruct);	
	
	/*I2C GPIO Default*/
	SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);
	SYS_GPIO_SET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
	SIM_I2C_DELAY(1);
}

void ssp_SimI2C_IsBusy(SSP_SimI2C *SimI2C)
{
	/*读取SDA线上的电平状态,若为低电平则说明总线被从机控制，若为高电平则说明总线空闲，可以准备发送开始条件*/
	while(SYS_GPIO_READ(SimI2C->SDA_GPIO, SimI2C->SDA_Pin) == Bit_RESET)
	{
		SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
		SYS_GPIO_RESET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
		SIM_I2C_DELAY(3);
		
		SYS_GPIO_SET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
		SYS_GPIO_SET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
		SIM_I2C_DELAY(3);
	}
}

/*产生开始信号*/
void ssp_SimI2C_Start(SSP_SimI2C *SimI2C)
{
	/*判断下总线是否处于空闲状态*/
	ssp_SimI2C_IsBusy(SimI2C);
	
	/*先让SCL中的电平为低，防止因为SCL处于高电平而使后面将SDA拉高时，可能会触发一个stop信号*/
	SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);
	
	SYS_GPIO_SET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
	SIM_I2C_DELAY(1);
	
	SYS_GPIO_SET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);
	
	SYS_GPIO_RESET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
	SIM_I2C_DELAY(1);
	
	/*将SCL拉低，钳住SCL线，准备发送地址数据*/
	SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);	
}

/*产生结束信号*/
void ssp_SimI2C_Stop(SSP_SimI2C *SimI2C)
{
	SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);
	
	SYS_GPIO_RESET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
	SIM_I2C_DELAY(1);	
	
	SYS_GPIO_SET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);
	
	SYS_GPIO_SET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
	SIM_I2C_DELAY(1);	
}

/*产生应答信号或者非应答信号,ackState->为ENABLE时，则产生应答信号*/
void ssp_SimI2C_SetAck(SSP_SimI2C *SimI2C, FunctionalState ackState)
{
	SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);
	
	if (ackState == ENABLE)
	{
		SYS_GPIO_RESET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
		SIM_I2C_DELAY(1);
	}
	else if (ackState == DISABLE)
	{
		SYS_GPIO_SET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
		SIM_I2C_DELAY(1);	
	}
	
	SYS_GPIO_SET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(2);	
	
	/*拉低SCL线，钳住SCL，准备下一个操作*/
	SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);
	
	SYS_GPIO_SET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
	SIM_I2C_DELAY(1);
}

/*获得应答信号（ENABLE）或者非应答信号（DISABLE）*/
FunctionalState ssp_SimI2C_GetAck(SSP_SimI2C *SimI2C)
{
	FunctionalState ask;
	
	SYS_GPIO_SET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);

	/*读取SDA线上的电平状态*/
	if (SYS_GPIO_READ(SimI2C->SDA_GPIO, SimI2C->SDA_Pin) == Bit_RESET)
	{
		ask = ENABLE;
	}
	else
	{
		ask = DISABLE;
	}
	
	/*主机取走响应信息，并钳住SCL，准备下一步操作*/
	SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);
	
	return ask;
}

/*写出数据给从机，并返回应答或者非应答信号*/
FunctionalState ssp_SimI2C_WriteByte(SSP_SimI2C *SimI2C, uint8_t data)
{
	uint8_t i;
	
	/*类似移位寄存器的功能，将数据通过I/O口发送出去*/
	for (i = 0; i < 8; i++)
	{
		SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
		SIM_I2C_DELAY(1);
		
		if ((data & 0x80) == 0x80)
		{
			SYS_GPIO_SET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);	
		}
		else if ((data & 0x80) == 0)
		{
			SYS_GPIO_RESET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
		}
		
		data = data << 1;
		SIM_I2C_DELAY(1);	
		
		SYS_GPIO_SET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
		SIM_I2C_DELAY(2);		
	}
	
	/*主机释放SDA线，使得总线空闲，以便从机能发出响应信息,并钳住SCL线*/
	SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
	SIM_I2C_DELAY(1);	
	
	SYS_GPIO_SET(SimI2C->SDA_GPIO, SimI2C->SDA_Pin);
	SIM_I2C_DELAY(1);

	return ssp_SimI2C_GetAck(SimI2C);
}

/*读取从机发送的数据,并决定是应答还是非应答*/
uint8_t ssp_SimI2C_ReadByte(SSP_SimI2C *SimI2C, uint8_t *data, FunctionalState ackState)
{
	uint8_t i;
	
	*data = 0x00;
	
	/*类似移位寄存器的功能，将数据从I/O口中读取进来*/
	for (i = 0; i < 8; i++)
	{
		SYS_GPIO_SET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
		SIM_I2C_DELAY(1);

		/*先移位，后赋值*/
		(*data) = (*data) << 1;
		
		if (SYS_GPIO_READ(SimI2C->SDA_GPIO, SimI2C->SDA_Pin) == 1)
		{
			(*data) = (*data) | 0x01;
		}
		else if (SYS_GPIO_READ(SimI2C->SDA_GPIO, SimI2C->SDA_Pin) == 0)
		{
			(*data) = (*data) | 0x00;
		}
		
		/*拉低SCL,以便从机准备好下一个数据*/
		SYS_GPIO_RESET(SimI2C->SCL_GPIO, SimI2C->SCL_Pin);
		SIM_I2C_DELAY(2);
	}
	
	ssp_SimI2C_SetAck(SimI2C, ackState);
	
	return *data;
}
