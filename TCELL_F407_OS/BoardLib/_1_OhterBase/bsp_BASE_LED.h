#ifndef _BSP_BASE_LED_H_
#define _BSP_BASE_LED_H_

#include "sys_Platform.h"
#include "msp_GPIO.h"

#define BSP_LED1_GPIO_Port 			    GPIOB
#define BSP_LED1_GPIO_Pin 				GPIO_Pin_1
#define BSP_LED1_PERIPH_RCC_CLOCK		RCC_AHB1Periph_GPIOB

#define BSP_LED2_GPIO_Port 			    GPIOB
#define BSP_LED2_GPIO_Pin 				GPIO_Pin_0
#define BSP_LED2_PERIPH_RCC_CLOCK		RCC_AHB1Periph_GPIOB

typedef enum
{
	BSP_LED_SET_OFF = 0,
	BSP_LED_SET_ON  = 1,
}BSP_LED_SET_STA;

typedef enum
{
	BSP_LR_LED1   = 1,
	BSP_LR_LED2   = 2,
	BSP_LR_LED3   = 4,
	BSP_LR_LEDALL = 7,
}BSP_LED_RESOURCE;

SYS_RETSTATUS bsp_Led_Init(void);
void bsp_Led_On(BSP_LED_RESOURCE bspLedResource);
void bsp_Led_Off(BSP_LED_RESOURCE bspLedResource);

#endif
