#ifndef _SSP_I2C_H_
#define _SSP_I2C_H_

#include "sys_Platform.h"
#include "msp_GPIO.h"

/*模式I2C延时*/
#define SIM_I2C_DELAY(foc)				nop_delay_us(foc)

/*SDA管脚配置输入输出方向*/
/*F4*/
#define SIM_I2C_SDA_IN(GPIOx, PINx)		{GPIOx->MODER &= ~(3 << (PINx * 2)); GPIOx->MODER |= (0 << (PINx * 2));	SIM_I2C_DELAY(1);}	/*输入模式*/
#define SIM_I2C_SDA_OUT(GPIOx, PINx)	{GPIOx->MODER &= ~(3 << (PINx * 2)); GPIOx->MODER |= (1 << (PINx * 2)); SIM_I2C_DELAY(1);}    /*输出模式*/

typedef struct
{
	GPIO_TypeDef *SCL_GPIO;
	u16 		 SCL_Pin;
	u32 		 RCC_Periph_SCL_GPIO;
	
	GPIO_TypeDef *SDA_GPIO;
	u16 		 SDA_Pin;	
	u32 		 RCC_Periph_SDA_GPIO;	
}SSP_SimI2C;

void ssp_SimI2C_Init(SSP_SimI2C *SimI2C);
void ssp_SimI2C_IsBusy(SSP_SimI2C *SimI2C);
void ssp_SimI2C_Start(SSP_SimI2C *SimI2C);
void ssp_SimI2C_Stop(SSP_SimI2C *SimI2C);
void ssp_SimI2C_SetAck(SSP_SimI2C *SimI2C, FunctionalState ackState);
FunctionalState ssp_SimI2C_GetAck(SSP_SimI2C *SimI2C);
FunctionalState ssp_SimI2C_WriteByte(SSP_SimI2C *SimI2C, uint8_t data);
uint8_t ssp_SimI2C_ReadByte(SSP_SimI2C *SimI2C, uint8_t *data, FunctionalState ackState);	

#endif
