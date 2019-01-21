#include "msp_NVIC.h"
#include "sys_McuInit.h"

void msp_NVIC_Init(MSP_NVIC_GROUP mspNvicGroup)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(mspNvicGroup);		/*优先组别2*/
		
	/*Systick定时器*/
#ifndef PLATFORM_RTOS__RT_THREAD
	NVIC_SetPriority(SysTick_IRQn, 0);
#endif

	/*ExecutePeriod周期执行的时间系统定时器*/
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sTimExecutePeriod.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   /*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;   /*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 

	/*TIM3定时器PWM输出*/
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sTimMultiPwmOut.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   /*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;   /*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		
	
	#if defined(REMOTE_DATA_RECV__PWM)	
	/*TIM4定时器PWM输入捕获(遥控数据接收)*/
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sTimPwmIn_Attitude.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   /*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;   /*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	/*TIM1定时器PWM输入捕获(遥控数据接收)*/
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sTimPwmIn_Switch.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   /*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;   /*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	#endif
	
	#if defined(REMOTE_DATA_RECV__PPM)
	/*外部中断(遥控数据节后PPM)*/
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sPPMRecvExti.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   /*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;   /*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		
	#endif
	
	/*USER 串口空闲 中断(RX)*/
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sDebugUart.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;    /*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;    /*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		
	
	/*GPS 串口空闲 中断(RX)*/
#if defined(HW_CUT__USE_GPS)
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sGpsUart.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;    /*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;    /*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
#endif	
	NVIC_Init(&NVIC_InitStructure);		

	/*ULTR 串口空闲 中断(RX)*/
#if defined(HW_CUT__USE_ULTR)	
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sUltrUart.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;    /*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;    /*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
#endif	

	/*OPTICALFLOW 串口空闲 中断(RX)*/
#if defined(HW_CUT__USE_OPTICFLOW)	
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sOpticalFlowUart.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;    /*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;    /*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
#endif	
	
	/*Task_Main调度定时器*/
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sTimAlarmTaskMain.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	/*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;	/*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/*Task_Assist调度定时器*/
	NVIC_InitStructure.NVIC_IRQChannel                   = g_sTimAlarmTaskAssist.NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	/*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;	/*响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}
