#ifndef _BSP_BASE_RGB_H_
#define _BSP_BASE_RGB_H_

#include "sys_Platform.h"
#include "sys_BoardMap.h"
#include "sys_McuInit.h"
#include "msp_MutGPIO.h"

#define RGB_SHOW_CONTINUE_TICK_FOC_5MS	(5)  /*rgb显示持续时间5ms时基*/
#define RGB_WORK_LIST_MAX_NBR			(10) /*RGB工作链最大长度(一个就是一次显示任务)*/
#define RGB_DELAY_MS(ms)				sys_DelayMs(ms)			

/*显示颜色*/
typedef enum
{
	RGB_COLOR_NULL   = 0, /*无*/
	RGB_COLOR_RED    = 1, /*红*/
	RGB_COLOR_GREEN  = 2, /*绿*/
	RGB_COLOR_BLUE   = 3, /*蓝*/
	RGB_COLOR_CYAN   = 4, /*青*/
	RGB_COLOR_PINK   = 5, /*粉*/
	RGB_COLOR_YELLOW = 6, /*黄*/
	RGB_COLOR_WHITE  = 7, /*白*/
}RGB_COLOR_TYPE;

/*单次显示样式*/
typedef enum
{
	RGB_SIG_SHOW_NULL   = 0, /*无*/
	RGB_SIG_SHOW_DARK   = 1, /*暗*/
	RGB_SIG_SHOW_LIGHT  = 2, /*亮*/
	RGB_SIG_SHOW_TOGGLE = 3, /*闪*/
}RGB_SIG_SHOW_TYPE;

/*工作链显示样式*/
typedef enum
{
	RGB_LST_SHOW_NULL   = 0, /*无*/
	RGB_LST_SHOW_ONCE   = 1, /*一次*/
	RGB_LST_SHOW_CIRCLE = 2, /*循环*/
	RGB_LST_SHOW_RANDOM = 3, /*随机*/	
}RGB_LST_SHOW_TYPE;

typedef struct
{
	/*单个显示*/
	RGB_COLOR_TYPE    COLOR_TYPE; 	  /*颜色*/
	RGB_SIG_SHOW_TYPE SHOW_TYPE;  /*样式*/
	u32			      conTick5MS;     /*持续时长_5ms*/
}Rgb_Work;

/*电平型RGB*/
#ifdef RGB_LEVEL_TYPE 
typedef struct
{
	MSP_TricolorGpio *TricolorGpio;
	Rgb_Work 		  WorkList[RGB_WORK_LIST_MAX_NBR];   /*RGB工作链*/
	Rgb_Work 		  UserList[RGB_WORK_LIST_MAX_NBR];   /*用户操作空间*/
	RGB_LST_SHOW_TYPE LIST_SHOW_TYPE;			/*工作链显示样式*/
}BSP_RGB;
#endif

/*时序型RGB*/
#ifdef RGB_CLOCK_TYPE 
typedef struct
{
	
}BSP_RGB;
#endif

/*初始化*/
SYS_RETSTATUS bsp_RGB_Init(BSP_RGB *rgb);

/*工作链设置*/
void bsp_rgb_work_list_set(BSP_RGB *rgb, const Rgb_Work *userList, RGB_LST_SHOW_TYPE LIST_SHOW_TYPE);

/*工作链清除*/
void bsp_rgb_work_list_clear(BSP_RGB *rgb);

/*工作链显示*/
void bsp_rgb_work_list_show(BSP_RGB *rgb);

/*颜色显示*/
void bsp_rgb_show_color(BSP_RGB *rgb, RGB_COLOR_TYPE COLOR_TYPE, RGB_SIG_SHOW_TYPE SHOW_TYPE);

extern BSP_RGB g_sRgb;

#endif
