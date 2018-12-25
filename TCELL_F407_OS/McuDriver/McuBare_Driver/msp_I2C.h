#ifndef _MSP_I2C_H_
#define _MSP_I2C_H_

#include "sys_Platform.h"

typedef enum
{
	MSP_I2C_CLOCK_SPEED_100KHZ  = 100000,	/*I2C 慢速模式*/
	MSP_I2C_CLOCK_SPEED_400KHZ  = 400000,	/*I2C 快速模式*/
	MSP_I2C_CLOCK_SPEED_3400KHZ = 3400000,	/*I2C 高快速模式*/		
}MSP_I2C_CLOCK_SPEED;

typedef struct
{
	I2C_TypeDef  		*I2c;
	u32 	 	 		RCC_Periph_I2C;
	MSP_I2C_CLOCK_SPEED I2C_CLOCK_SPEED;
	
	GPIO_TypeDef 		*SCL_GPIO;
	u16 		 		SCL_Pin;
	u32 		 		RCC_Periph_SCL_GPIO;
	u16 		 		SCL_PinSource;
	
	GPIO_TypeDef 		*SDA_GPIO;
	u16 		 		SDA_Pin;	
	u32 		 		RCC_Periph_SDA_GPIO;	
	u16 		 		SDA_PinSource;	
	
	u8 			 		GPIO_AF;
	
	u8  				slaveAddr; /*从机地址*/
	
	u32			 		timeOut;
}MSP_I2c;

/*I2C初始化*/
void msp_I2c_Init(MSP_I2c *i2c);

/*I2C读若干字节*/
SYS_RETSTATUS I2C_Read_SomeByte(MSP_I2c *i2c, u8 rdAddr, u8 *rdBuff, u8 rdNbr);

/*I2C写若干字节*/
SYS_RETSTATUS I2C_Write_SomeByte(MSP_I2c *i2c, u8 wrAddr, u8 *wrBuff, u8 wrNbr);

#endif
