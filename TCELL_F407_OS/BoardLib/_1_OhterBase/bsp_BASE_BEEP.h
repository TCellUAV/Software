#ifndef _BSP_BASE_BEEP_H_
#define _BSP_BASE_BEEP_H_

#include "sys_Platform.h"
#include "sys_BoardMap.h"
#include "sys_McuInit.h"
#include "msp_TIM.h"

#define BEEP_LOUDNESS_LOW_DUTY_CYCLE		(1/4)
#define BEEP_LOUDNESS_MID_DUTY_CYCLE		(2/4)
#define BEEP_LOUDNESS_HIGH_DUTY_CYCLE		(3/4)

/*声音强度*/
typedef enum
{
	BEEP_LOUDNESS_NULL = 0, /*无*/
	BEEP_LOUDNESS_LOW  = 1, /*低音*/
	BEEP_LOUDNESS_MID  = 2, /*中音*/
	BEEP_LOUDNESS_HIGH = 3, /*高音*/
}BEEP_LOUDNESS_TYPE;

/*单次显示样式*/
typedef enum
{
	BEEP_CIRCLE_NULL    = 0, /*无*/
	BEEP_CIRCLE_ONCE    = 1, /*一次*/
	BEEP_CIRCLE_FOREVER = 2,  /*永远*/
}BEEP_CIRCLE_TYPE;	

typedef struct
{
	/*单个显示*/
	BEEP_LOUDNESS_TYPE    LOUDNESS_TYPE; 	/*声音强度*/
	BEEP_CIRCLE_TYPE 	  CIRCLE_TYPE;  	/*循环样式*/
	u32			     	  conTick5MS;       /*持续时长_5ms*/
}Beep_Work;

typedef struct
{
	TimSinglePwmOut *SinglePWM;
	Beep_Work		work;
}BSP_BEEP;

/*初始化*/
SYS_RETSTATUS bsp_BEEP_Init(BSP_BEEP *beep);

extern BSP_BEEP g_sBeep;

#endif
