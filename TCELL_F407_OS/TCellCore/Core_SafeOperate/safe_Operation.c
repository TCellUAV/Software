#include "safe_Operation.h"

Safe_Operation gs_SafeOperation = 
{
	/*出于安全性而做的强制任务*/	
	.SAFE_FORCE_MISSION = UAV_FLY_MISSION_NULL,		   
	
	/*任务调度检测*/
	.Task_Check_Status  =
	{
		.RETERR 		   = {SYS_RETERR_0ZR},			  /*默认全无错*/
		.TASK_CHECK_STATUS = SAFE_TASK_CHECK_BREAKDOWN,   /*任务调度检测状态*/
	},		
};

Safe_Operation *gps_SafeOperation = &gs_SafeOperation;

/*任务调度状态检测*/
SAFE_TASK_CHECK_STATUS safe_task_status_check(Safe_Operation *safe_Operation)
{
	static vu8 i;
	
	/*判断任务调度得信号量是否正常*/
	
	/*=== 基础外设 ===*/
	if (base_module_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[0] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[0] = SYS_RETERR_1ST;	
	}

	/*=== 传感器数据获取及姿态融合 ===*/
	if (euler_angle_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[1] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[1] = SYS_RETERR_2ND;	
	}
	
	if (ver_fusion_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[2] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[2] = SYS_RETERR_3RD;	
	}
	
	if (gps_update_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[3] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[3] = SYS_RETERR_4TH;	
	}
	
	if (opflow_update_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[4] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[4] = SYS_RETERR_5TH;	
	}
	
	if (gps_hor_fusion_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[5] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[5] = SYS_RETERR_6TH;	
	}
	
	if (opflow_hor_fusion_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[6] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[6] = SYS_RETERR_7TH;	
	}
	
	/*=== 控制系统 ===*/
	if (uav_ctrl_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[7] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[7] = SYS_RETERR_8TH;		
	}	

	/*=== 校准系统 ===*/
	if (uav_calib_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[8] = SYS_RETERR_0ZR;
	}	
	else
	{
		safe_Operation->Task_Check_Status.RETERR[8] = SYS_RETERR_9TH;
	}	
	
	/*=== 飞行相关 ===*/
	if (tfsd_fly_log_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[9] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[9] = SYS_RETERR_10TH;
	}	
	
	/*=== 人机交互 ===*/
	if (hci_oled_show.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[10] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[10] = SYS_RETERR_11TH;
	}
	
	if (hci_host_slave_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[11] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[11] = SYS_RETERR_12TH;
	}	
	
	/*=== 任务调度状态 ===*/
	if (task_status_check_sem.value == 0)
	{
		safe_Operation->Task_Check_Status.RETERR[12] = SYS_RETERR_0ZR;
	}
	else
	{
		safe_Operation->Task_Check_Status.RETERR[12] = SYS_RETERR_13TH;
	}	
	
	/*判断各线程任务调度是否正常*/
	for (i = 0; i < 30; i++)
	{
		/*某一个线程调度有问题*/
		if (safe_Operation->Task_Check_Status.RETERR[i] != SYS_RETERR_0ZR)
		{
			/*标记系统崩溃*/
			safe_Operation->Task_Check_Status.TASK_CHECK_STATUS = SAFE_TASK_CHECK_BREAKDOWN;
			
			break;
		}
		else
		{
			/*标记系统正常*/
			safe_Operation->Task_Check_Status.TASK_CHECK_STATUS = SAFE_TASK_CHECK_NORMAL;			
		}
	}
	
	/*任务调度异常*/
	if (safe_Operation->Task_Check_Status.TASK_CHECK_STATUS == SAFE_TASK_CHECK_BREAKDOWN)
	{
		/*设置安全强制任务: 自动一键返航*/
		safe_Operation->SAFE_FORCE_MISSION = UAV_FLY_MISSION_ONEKEY_LAND_HOME;
	}
	/*任务调度正常*/
	else if (safe_Operation->Task_Check_Status.TASK_CHECK_STATUS == SAFE_TASK_CHECK_NORMAL)
	{
		/*安全强制任务清除*/
		safe_Operation->SAFE_FORCE_MISSION = UAV_FLY_MISSION_NULL;
	}
	
	return (safe_Operation->Task_Check_Status.TASK_CHECK_STATUS);
}


/*获取安全强制飞行任务*/
UAV_FLY_MISSION safe_force_mission_get(Safe_Operation *safe_Operation)
{
	return (safe_Operation->SAFE_FORCE_MISSION);
}

