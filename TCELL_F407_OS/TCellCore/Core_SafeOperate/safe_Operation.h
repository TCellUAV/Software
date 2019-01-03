#ifndef _SAFE_OPERATION_H_
#define _SAFE_OPERATION_H_

#include "sys_Platform.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

typedef enum
{
	SAFE_TASK_CHECK_BREAKDOWN = 0,
	SAFE_TASK_CHECK_NORMAL    = 1,
}SAFE_TASK_CHECK_STATUS;

typedef struct
{
	volatile SYS_RETERR 			RETERR[30];
	volatile SAFE_TASK_CHECK_STATUS TASK_CHECK_STATUS;
}Safe_Task_Check_Status;

typedef struct
{
	volatile UAV_FLY_MISSION 	    SAFE_FORCE_MISSION;	/*出于安全性而做的强制任务*/
	volatile Safe_Task_Check_Status Task_Check_Status;   /*任务调度检测状态*/
}Safe_Operation;

/*任务调度状态检测*/
SAFE_TASK_CHECK_STATUS safe_task_status_check(Safe_Operation *safe_Operation);

/*获取安全强制飞行任务*/
UAV_FLY_MISSION safe_force_mission_get(Safe_Operation *safe_Operation);

extern Safe_Operation gs_SafeOperation;
extern Safe_Operation *gps_SafeOperation;

#endif
