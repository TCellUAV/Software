#include "msp_EXTI.h"

/*外部中断初始化*/
void msp_EXTI_Init(MSP_Exti *exti)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	
	/*GPIO初始化*/
	GPIO_InitStruct.GPIO_Pin = exti->Gpio.GPIO_Pin;
	/*上升沿触发*/
	if (exti->EXTI_Trigger == EXTI_Trigger_Rising)
	{
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN; /*下拉*/
	}
	/*下降沿触发*/
	else if (exti->EXTI_Trigger == EXTI_Trigger_Falling)
	{
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;   /*上拉*/	
	}
	/*上下边沿触发*/
	else if (exti->EXTI_Trigger == EXTI_Trigger_Rising_Falling)
	{
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	}
	/*输入模式*/
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 
	GPIO_Init(exti->Gpio.GPIO, &GPIO_InitStruct);
	
	/*IO的复用功能必须使能*/
	SYSCFG_EXTILineConfig(exti->EXTI_PortSource, exti->EXTI_PinSource);
	
	/*外部中断初始化*/
	EXTI_InitStruct.EXTI_Line    = exti->EXTI_Line;
	EXTI_InitStruct.EXTI_Mode    = exti->EXTI_Mode;
	EXTI_InitStruct.EXTI_Trigger = exti->EXTI_Trigger;	
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
}
