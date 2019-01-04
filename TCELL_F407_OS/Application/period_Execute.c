#include "period_Execute.h"
#include "sys_McuInit.h"

/*=== 周期执行时间信息 ===*/
/*用于某个定时器定时加1,来计算系统上电后的时间*/
vu32 g_vu32_Time_Period_Cnt_Foc_Ms = 0;

/*系统执行周期周期*/
SystemPeriodExecuteTime g_sSystemPeriodExecuteTime = {0};
SystemPeriodExecuteTime *g_psSystemPeriodExecuteTime = &g_sSystemPeriodExecuteTime;

/*获取周期执行时间信息*/
void get_Period_Execute_Time_Info(PeriodExecuteTime *periodExecuteTime)
{
	/*上次的nowtime赋值给本次的lasttime*/
	periodExecuteTime->LastTime  = periodExecuteTime->NowTime;
	
	/*根据定时器和全局g_Time_Period_Cnt_Foc_Us计算本次时刻*/
	periodExecuteTime->NowTime   = (g_sTimExecutePeriod.Period * g_vu32_Time_Period_Cnt_Foc_Ms + \
								    g_sTimExecutePeriod.Tim->CNT) / 1000.0f;   /*g_psTimExecutePeriod->Period中断一次过了多少us(xUs / 1000 = xMs)*/
	
	/*计算两次执行间隔时间差*/
	periodExecuteTime->DeltaTime = periodExecuteTime->NowTime - periodExecuteTime->LastTime;
}

/*=== 系统tick(禁止放到systick) ===*/
#ifndef PLATFORM_RTOS__RT_THREAD
void sys_Systick_Init(void)
{
	/*设置重载值*/
	SysTick->LOAD  = (uint32_t)(SystemCoreClock/1000000 - 1UL);

	/*清空计数器中的值*/
	SysTick->VAL   = 0UL;

	/*设置systick的时钟源和开启systick的中断*/
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |SysTick_CTRL_TICKINT_Msk;
	
	/*先将systick关闭，有需要计时的时候再打开*/
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; 
}

vu32 g_vu32DelayCounter = 0;

void sys_DelayMs(u16 nms)
{
	if (nms <= 0)
	{
		return;
	}

	g_vu32DelayCounter = nms * 1000;
	
	SysTick->VAL  = 0;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	
	while(g_vu32DelayCounter != 0);
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void sys_DelayUs(u32 nus)
{
	if (nus <= 0)
	{
		return;
	}

	g_vu32DelayCounter = nus;
	
	SysTick->VAL  = 0;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	
	while(g_vu32DelayCounter != 0);
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	
}
#endif

/*用__NOP()做延时*/
vu32 nop_time_test = 100000000;
void nop_delay_init(void)
{	
	__NOP();
	
	while(--nop_time_test)
	{
		__NOP();
	}
		
	__NOP();
}

#define THIS_CHIP_ONE_WHILE_NOP_NS (68)	/*由 nop_delay_init 测得,并需要手动更新数值*/
vs32 nop_delay_nbr;

void nop_delay_us(u32 us) 
{	
	nop_delay_nbr = (us * 1000) / THIS_CHIP_ONE_WHILE_NOP_NS;
	
	while(nop_delay_nbr > 0)
	{
		__NOP();		
		nop_delay_nbr--;
	}
}

/*获取g_vu32Tick*/
vu32 g_vu32TickCounter = 0;

uint32_t my_GetTick(void)
{
	return g_vu32TickCounter;
}

void my_Tick_Ins(void)
{
	g_vu32TickCounter++;
}

