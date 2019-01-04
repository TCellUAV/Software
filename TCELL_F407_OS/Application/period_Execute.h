#ifndef _PERIOD_EXECUTE_H_
#define _PERIOD_EXECUTE_H_

#include "sys_Platform.h"

/*=== 周期执行时间信息 ===*/
typedef struct
{
	volatile float LastTime;		/*上次时间*/
	volatile float NowTime;			/*本次时间*/
	volatile float DeltaTime;		/*上本次时间间隔*/
}PeriodExecuteTime;	/*单位us*/

typedef struct
{
/*RTOS*/
#ifndef PLATFORM_RTOS__RT_THREAD
	PeriodExecuteTime Task;					/*任务执行间隔时间*/
#endif	
	
	PeriodExecuteTime AhrsAttitude;		    /*AHRS姿态解算间隔时间*/
	PeriodExecuteTime BeroAboveAltitude;	/*气压计(上方)获取海拔高度间隔时间*/
	PeriodExecuteTime BeroBeneathAltitude;	/*气压计(下方)获取海拔高度间隔时间*/
	PeriodExecuteTime UltrAltitude;			/*超声波获取海拔高度间隔时间*/	
	PeriodExecuteTime GpsCtrlData;			/*GPS控制数据获取间隔时间*/
	PeriodExecuteTime OpflowCtrlData;		/*OPTICFLOW控制数据获取间隔时间*/	
	PeriodExecuteTime SINS_High;			/*竖直方向上惯导融合间隔时间*/
	PeriodExecuteTime SINS_Horizontal;		/*水平方向上惯导融合间隔时间*/
	PeriodExecuteTime CTRL_MainLeading;		/*主导控制器执行间隔时间*/
	
	/*OS任务执行执行周期测试*/
	PeriodExecuteTime BaseModule;
	PeriodExecuteTime EulerAngle;
	PeriodExecuteTime VerFusion;
	PeriodExecuteTime GpsDataGet;
	PeriodExecuteTime OpFlowDataGet;	
	PeriodExecuteTime GpsHorFusion;
	PeriodExecuteTime OpflowHorFusion;
	PeriodExecuteTime UavCtrl;
	PeriodExecuteTime UavCalib;
	PeriodExecuteTime FlyLog;	
	PeriodExecuteTime HciOledShow;
	PeriodExecuteTime HciHostSlave;
	PeriodExecuteTime TaskStatusCheck;
	
	/*Test everywhere*/
	PeriodExecuteTime EveryWhere;
}SystemPeriodExecuteTime;

/*获取周期执行时间信息*/
void get_Period_Execute_Time_Info(PeriodExecuteTime *periodExecuteTime);

/*用于某个定时器定时加1,来计算系统上电后的时间*/
extern vu32 g_vu32_Time_Period_Cnt_Foc_Ms;

/*系统执行周期周期*/
extern SystemPeriodExecuteTime g_sSystemPeriodExecuteTime;
extern SystemPeriodExecuteTime *g_psSystemPeriodExecuteTime;


/*=== 系统tick(禁止放到systick) ===*/
#ifndef PLATFORM_RTOS__RT_THREAD
void sys_Systick_Init(void);
void sys_DelayMs(u16 nms);
void sys_DelayUs(u32 nus);
extern vu32 g_vu32DelayCounter;
#endif

/*用__NOP()做延时*/
void nop_delay_init(void);
void nop_delay_us(u32 us);
void nop_delay_ms(u16 ms);

uint32_t my_GetTick(void);
void my_Tick_Ins(void);

#endif
