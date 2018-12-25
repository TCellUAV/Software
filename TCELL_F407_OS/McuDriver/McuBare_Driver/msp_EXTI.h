#ifndef _MSP_EXTI_H_
#define _MSP_EXTI_H_

#include "sys_Platform.h"
#include "msp_GPIO.h"

typedef struct
{
	MSP_General_Gpio    Gpio;
	u32 				Periph_SYSCFG;
	u8 					NVIC_IRQChannel;
	u8				    EXTI_PortSource;
	u8	 			    EXTI_PinSource;
	u32				    EXTI_Line;
	EXTIMode_TypeDef    EXTI_Mode;
	EXTITrigger_TypeDef EXTI_Trigger;
}MSP_Exti;

/*外部中断初始化*/
void msp_EXTI_Init(MSP_Exti *exti);

#endif
