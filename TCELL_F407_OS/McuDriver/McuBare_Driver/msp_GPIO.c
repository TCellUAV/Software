#include "msp_GPIO.h"

void msp_GPIO_Init(MSP_General_Gpio *gpio)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*GPIO³õÊ¼»¯*/
	GPIO_InitStruct.GPIO_Pin   = gpio->GPIO_Pin;
	GPIO_InitStruct.GPIO_Mode  = gpio->Mode;
	GPIO_InitStruct.GPIO_PuPd  = gpio->PuPd;	
	GPIO_InitStruct.GPIO_OType = gpio->OType;
	GPIO_InitStruct.GPIO_Speed = gpio->Speed;	

	GPIO_Init(gpio->GPIO, &GPIO_InitStruct);	
}
