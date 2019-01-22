#ifndef _MSP_GPIO_H_
#define _MSP_GPIO_H_

#include "sys_Platform.h"

#define SYS_GPIO_SET(GPIO, PIN)		GPIO_SetBits(GPIO, PIN)
#define SYS_GPIO_RESET(GPIO, PIN)	GPIO_ResetBits(GPIO, PIN)
#define SYS_GPIO_READ(GPIO, PIN)	GPIO_ReadInputDataBit(GPIO, PIN)

typedef enum
{
	MSP_GPIO_NORMAL = 0,
	MSP_GPIO_OUT    = 1, 
	MSP_GPIO_INP_D  = 2,
	MSP_GPIO_INP_U  = 3,
	MSP_GPIO_EXTI   = 4,
	MSP_GPIO_PWM    = 5,
	MSP_GPIO_UART   = 6,	
	MSP_GPIO_I2C    = 7,
	MSP_GPIO_SPI    = 8,
}MSP_GPIO_PURPOSE;

typedef struct
{
	GPIO_TypeDef      *GPIO;
	u16 		      GPIO_Pin;
	u32 		      RCC_Periph_GPIO;
	GPIOMode_TypeDef  Mode;
	GPIOOType_TypeDef OType;
	GPIOPuPd_TypeDef  PuPd;
	GPIOSpeed_TypeDef Speed;
}MSP_General_Gpio;

void msp_GPIO_Init(MSP_General_Gpio *gpio);

#endif
