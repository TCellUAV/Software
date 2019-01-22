#include "bsp_BASE_RGB.h"

BSP_RGB g_sRgb = {0};

/*初始化*/
SYS_RETSTATUS bsp_RGB_Init(BSP_RGB *rgb)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	
	/*rgb gpio init*/
	#if defined(RGB_LEVEL_TYPE) /*电平型RGB*/
	rgb->TricolorGpio = &g_sLevelRgbGpio;
	#endif
	
	#if defined(RGB_CLOCK_TYPE)	/*时序型RGB*/
	rgb->Gpio = (Rgb_Gpio*)&g_sClockRgbGpio;
	#endif	
	
	RGB_DELAY_MS(1);
	
	return statusRet;	
}

/*工作链设置*/
void bsp_rgb_work_list_set(BSP_RGB *rgb, const Rgb_Work *userList, RGB_LST_SHOW_TYPE LIST_SHOW_TYPE)
{
	u32 i;
	
	/*先清除显示链*/
	bsp_rgb_work_list_clear(rgb);
	
	/*设置显示链单个显示任务*/
	for (i = 0; i < RGB_WORK_LIST_MAX_NBR; i++)
	{
		if (userList[i].COLOR_TYPE == RGB_COLOR_NULL)
		{
			break;
		}
		
		rgb->WorkList[i].COLOR_TYPE = userList[i].COLOR_TYPE; 									 /*颜色设置*/
		rgb->WorkList[i].SHOW_TYPE  = userList[i].SHOW_TYPE; 								     /*样式设置*/
		rgb->WorkList[i].conTick5MS = userList[i].conTick5MS * RGB_SHOW_CONTINUE_TICK_FOC_5MS;    /*持续时间设置*/
	}
	
	/*显示链显示样式*/
	rgb->LIST_SHOW_TYPE = LIST_SHOW_TYPE;
}

/*工作链清除*/
void bsp_rgb_work_list_clear(BSP_RGB *rgb)
{
	u32 i;
	
	/*设置显示链单个显示任务*/
	for (i = 0; i < RGB_WORK_LIST_MAX_NBR; i++)
	{
		rgb->WorkList[i].COLOR_TYPE = RGB_COLOR_NULL; 	 /*颜色设置*/ 
		rgb->WorkList[i].SHOW_TYPE  = RGB_SIG_SHOW_NULL; /*样式设置*/		
		rgb->WorkList[i].conTick5MS = 0;    			 /*持续时间设置*/		
	}	
	
	/*清除显示链任务*/
	rgb->LIST_SHOW_TYPE = RGB_LST_SHOW_NULL;	
}

/*工作链显示*/
void bsp_rgb_work_list_show(BSP_RGB *rgb)
{
	u32 i;
	
	/*扫描显示链*/
	for (i = 0; i < RGB_WORK_LIST_MAX_NBR; i++)
	{
		/*判断为null,表示单个任务到此结束*/
		if (rgb->WorkList[i].COLOR_TYPE == RGB_COLOR_NULL)
		{
			break;
		}		
		
		/*显示*/
		bsp_rgb_show_color(rgb, rgb->WorkList[i].COLOR_TYPE, rgb->WorkList[i].SHOW_TYPE);
		
		/*延时*/
		RGB_DELAY_MS(rgb->WorkList[i].conTick5MS);
	}
	
	/*显示链显示样式为单次,则直接清除显示链*/
	if (rgb->LIST_SHOW_TYPE == RGB_LST_SHOW_ONCE)
	{
		bsp_rgb_work_list_clear(rgb);
	}
}

/*颜色显示*/
void bsp_rgb_show_color(BSP_RGB *rgb, RGB_COLOR_TYPE COLOR_TYPE, RGB_SIG_SHOW_TYPE SHOW_TYPE)
{	
	/*电平型RGB*/
	#ifdef RGB_LEVEL_TYPE 
	static MSP_General_Gpio OpGpio;
	
	/*判断显示颜色*/
	switch(COLOR_TYPE)
	{
		case RGB_COLOR_RED:    /*红*/
		{
			OpGpio.GPIO     = rgb->TricolorGpio->Red.GPIO;
			OpGpio.GPIO_Pin = rgb->TricolorGpio->Red.GPIO_Pin;			
		}break;
		
		case RGB_COLOR_GREEN:  /*绿*/
		{
			OpGpio.GPIO     = rgb->TricolorGpio->Green.GPIO;
			OpGpio.GPIO_Pin = rgb->TricolorGpio->Green.GPIO_Pin;			
		}break;

		case RGB_COLOR_BLUE:   /*蓝*/
		{
			OpGpio.GPIO     = rgb->TricolorGpio->Blue.GPIO;
			OpGpio.GPIO_Pin = rgb->TricolorGpio->Blue.GPIO_Pin;		
		}break;

		case RGB_COLOR_CYAN:   /*青*/
		{
		
		}break;

		case RGB_COLOR_PINK:   /*粉*/
		{
		
		}break;

		case RGB_COLOR_YELLOW: /*黄*/
		{
		
		}break;

		case RGB_COLOR_WHITE:  /*白*/
		{
		
		}break;		
		
		default:break;
	}
	
	/*判断显示样式*/
	switch(SHOW_TYPE)
	{
		case RGB_SIG_SHOW_DARK:   /*暗*/
		{
			SYS_GPIO_RESET(OpGpio.GPIO, OpGpio.GPIO_Pin);
		}break;

		case RGB_SIG_SHOW_LIGHT:  /*亮*/
		{
			SYS_GPIO_SET(OpGpio.GPIO, OpGpio.GPIO_Pin);		
		}break;

		case RGB_SIG_SHOW_TOGGLE: /*闪*/
		{
		
		}break;	

		default:break;		
	}	
	#endif

	/*时序型RGB*/
	#ifdef RGB_CLOCK_TYPE 

	#endif	
}
