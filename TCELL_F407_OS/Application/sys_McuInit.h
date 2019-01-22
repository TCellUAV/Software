#ifndef _SYS_MCUINIT_H_
#define _SYS_MCUINIT_H_

/*
中文:
sys_McuInit是为了将GPIO、RCC的初始化与其功能、对应模块分离;
提高初始化代码的通用性,繁杂的初始化都放到这里做了,后面直接使用。

English:
sys_McuInit is to separate the initialization of GPIO and RCC from 
its functions and corresponding modules.
Improve the versatility of the initialization code, 
the complicated initialization is put here, and used directly later.
*/

#include "sys_Platform.h"
#include "bsp_BoardLib.h"	/*针对板载资源来初始化*/
#include "sys_Debug.h"

#include "msp_GPIO.h"
#include "msp_MutGPIO.h"
#include "msp_EXTI.h"
#include "msp_UART.h"
#include "msp_DMA.h"
#include "msp_SPI.h"
#include "msp_TIM.h"
#include "msp_I2C.h"

#include "ssp_SimI2C.h"
#include "ssp_SimSPI.h"

/*RCC初始化类型*/
typedef enum
{
	SYS_RCC_AHB1 = 1,
	SYS_RCC_AHB2 = 2,
	SYS_RCC_AHB3 = 3,
	SYS_RCC_APB1 = 4,	
	SYS_RCC_APB2 = 5,		
}SYS_RCC_TREE;

/*GPIO初始化类型*/
typedef enum
{
	SYS_GPIO_GENERAL = 1, 
	SYS_GPIO_EXTI    = 4,
	SYS_GPIO_PWM_IN  = 5,
	SYS_GPIO_PWM_OUT = 6,	
	SYS_GPIO_UART    = 7,	
	SYS_GPIO_I2C	 = 8,
	SYS_GPIO_SIMI2C  = 9,
	SYS_GPIO_SIMSPI  = 10,
}SYS_GPIO_PURPOSE;

/*MCU(除TIM之外)初始化*/
void sys_Mcu_Peripheral_Init(void);

/*MCU_TIM初始化*/
void sys_mcu_tim_init(void);

/*MCU_EXTI初始化*/
void sys_mcu_exti_init(void);

/*外设时钟初始化*/
void sys_Peripheral_RCC_Init(SYS_RCC_TREE rccTree, u32 rccPeriph, FunctionalState NewState);

/*外设&GPIO初始化*/
void msp_Peripheral_GPIO_Init(SYS_GPIO_PURPOSE gpioPurpose, void *gpioPeriphStruct);

/*=== 0.基础模块 ===*/
/*RGB*/
#if defined(HW_CUT__USE_RGB)
/*level(电平型)*/
#if defined(RGB_LEVEL_TYPE)
extern MSP_TricolorGpio g_sLevelRgbGpio;
#endif

/*clock(时序型)*/
#if defined(RGB_CLOCK_TYPE)
extern  g_sClockRgbGpio;
#endif
#endif

/*BEEP*/
#if defined(HW_CUT__USE_BEEP)
extern TimSinglePwmOut g_sBeepPwmOut;
#endif

/*=== 1.数据存储 ===*/
/*MCU FLASH*/
#if defined(HW_CUT__USE_FLASH_STOR)

#endif

/*EEPROM*/
#if defined(HW_CUT__USE_EEPROM_STOR)
#if defined(STD_PROTOCOL_HARDWARE_I2C)
extern MSP_I2c g_sEePromI2C;
#endif

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
extern SSP_SimI2C g_sEePromSimI2C;
#endif
#endif

/*SD CARD*/
#if defined(HW_CUT__USE_TFSD_STOR)

#endif

/*=== 2.IMU + 磁力计 ===*/
/*IMU I2C*/
#if defined(HW_CUT__USE_MD_IMU)
#if defined(STD_PROTOCOL_HARDWARE_I2C)
extern MSP_I2c g_sImuI2C;
#endif

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
extern SSP_SimI2C g_sImuSimI2C;
#endif
#endif

/*MAG I2C*/
#if defined(HW_CUT__USE_MD_MAG)
#if defined(STD_PROTOCOL_HARDWARE_I2C)
extern MSP_I2c g_sMagI2C;
#endif

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
extern SSP_SimI2C g_sMagSimI2C;
#endif
#endif

/*GPS_MAG I2C*/
#if defined(HW_CUT__USE_GPS_MAG)
#if defined(STD_PROTOCOL_HARDWARE_I2C)
extern MSP_I2c g_sGpsMagI2C;
#endif

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
extern SSP_SimI2C g_sGpsMagSimI2C;
#endif
#endif

/*=== 3.气压计 ===*/
/*BERO I2C*/
#if defined(HW_CUT__USE_MD_BERO)
#if defined(STD_PROTOCOL_HARDWARE_I2C)
extern MSP_I2c g_sBeroI2C;
#endif

#if defined(STD_PROTOCOL_SOFTWARE_I2C)
extern SSP_SimI2C g_sBeroSimI2C;
#endif
#endif

/*=== 4.超声波 ===*/
#if defined(HW_CUT__USE_ULTR)
/*ULTR UART*/
extern MSP_Uart g_sUltrUart;
#endif

/*=== 5.GPS ===*/
#if defined(HW_CUT__USE_GPS)
/*GPS UART*/
extern MSP_Uart g_sGpsUart;
#endif

/*=== 6.光流 ===*/
#if defined(HW_CUT__USE_OPTICFLOW)
/*OPTICALFLOW UART*/
extern MSP_Uart g_sOpticalFlowUart;
#endif

/*=== 7.人机交互 ===*/
#if defined(HW_CUT__USE_HCI_OLED)
/*OLED SIMSPI*/
extern SSP_SimSPI g_sOledSimSPI;
#endif


/*=== TIM_Alarm ===*/
/*任务调度定时器_Main(Alarm)*/
extern TimAlarm g_sTimAlarmTaskMain;
/*任务调度定时器_Assist(Alarm)*/
extern TimAlarm g_sTimAlarmTaskAssist;
/*执行间隔计数器(10ms)*/
extern TimAlarm g_sTimExecutePeriod;

/*=== TIM_PWM_IN ===*/
#if defined(REMOTE_DATA_RECV__PWM)
/*PWM_In通道中的Gimbal部分*/
extern TimPwmIn g_sTimPwmIn_Gimbal;
/*PWM_In通道中的Attitude部分*/
extern TimPwmIn g_sTimPwmIn_Attitude;
#endif

/*=== TIM_PWM_OUT ===*/
/*PWM_Out通道中的Motor部分*/
extern TimMultiPwmOut g_sTimMultiPwmOut;

/*EXTI Init*/
#if defined(REMOTE_DATA_RECV__PPM)
extern MSP_Exti g_sPPMRecvExti;
#endif

#endif
