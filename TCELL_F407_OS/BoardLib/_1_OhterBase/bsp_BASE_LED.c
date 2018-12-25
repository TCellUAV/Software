#include "bsp_BASE_LED.h"

SYS_RETSTATUS bsp_Led_Init(void)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*LED1 clock enable*/
	RCC_AHB1PeriphClockCmd(BSP_LED1_PERIPH_RCC_CLOCK, ENABLE);	 
	
	/*LED2 clock enable*/	
	RCC_AHB1PeriphClockCmd(BSP_LED2_PERIPH_RCC_CLOCK, ENABLE);	 	
      
	/*LED1 GPIO Init*/
	GPIO_InitStruct.GPIO_Pin   = BSP_LED1_GPIO_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(BSP_LED1_GPIO_Port, &GPIO_InitStruct);
	
	/*LED2 GPIO Init*/
	GPIO_InitStruct.GPIO_Pin   = BSP_LED2_GPIO_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(BSP_LED2_GPIO_Port, &GPIO_InitStruct);	
	
	/*gpio default status*/
	SYS_GPIO_SET(BSP_LED1_GPIO_Port, BSP_LED1_GPIO_Pin);
	SYS_GPIO_SET(BSP_LED2_GPIO_Port, BSP_LED2_GPIO_Pin);	
	
	return statusRet;
}

void bsp_Led_On(BSP_LED_RESOURCE bspLedResource)
{
	uint16_t onLed = 0;
	
	switch(bspLedResource)
	{
		case BSP_LR_LED1:
		{
			onLed = BSP_LED1_GPIO_Pin;
		}break;
		
		case BSP_LR_LED2:
		{
			onLed = BSP_LED2_GPIO_Pin;			
		}break;		

		case BSP_LR_LED3:
		{
//			onLed = BSP_LED3_GPIO_Pin;
		}break;	
		
		case BSP_LR_LEDALL:
		{
			onLed = BSP_LED1_GPIO_Pin | BSP_LED2_GPIO_Pin;// | BSP_LED3_GPIO_Pin;			
		}break;
		
		default:break;
	}
	
	/*turn on*/
	SYS_GPIO_RESET(GPIOB, onLed);	
}

void bsp_Led_Off(BSP_LED_RESOURCE bspLedResource)
{
	uint16_t gpioPin = 0;
	
	switch(bspLedResource)
	{
		case BSP_LR_LED1:
		{
			gpioPin = BSP_LED1_GPIO_Pin;
		}break;
		
		case BSP_LR_LED2:
		{
			gpioPin = BSP_LED2_GPIO_Pin;			
		}break;		

		case BSP_LR_LED3:
		{
//			gpioPin = BSP_LED3_GPIO_Pin;
		}break;	
		
		case BSP_LR_LEDALL:
		{
			gpioPin = BSP_LED1_GPIO_Pin | BSP_LED2_GPIO_Pin;// | BSP_LED3_GPIO_Pin;			
		}break;
		
		default:break;
	}
	
	SYS_GPIO_SET(GPIOB, gpioPin);	
}
