#include "control_Aircraft.h"
#include "safe_Operation.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

CTRL_SysStatus g_sCtrlSysStatus = 
{
	.pid  = CTRL_SYSTEM_PARA_INIT_FAIL,
	.adrc = CTRL_SYSTEM_PARA_INIT_FAIL,
};

CTRL_SysStatus *g_psCtrlSysStatus = &g_sCtrlSysStatus;

ControlAircraft g_sControlAircraft = 
{
	.heightHoldThrottle = 0,       	/*高度保持油门(用于定高)*/
	.RemotExpectAngle   = {0},      /*遥控值转化为期望角度*/
	.CurMotorPwmOutput  = {0},		/*PWM输出给电机*/
	.LastMotorPwmOutput = {0},
	.GO_HOME_STATUS     = CTRL_AIRCRAFT_GO_HOME_DISABLE,	/*上电后(着陆状态)不允许失联自动返航*/
	.GO_HOME_SET        = CTRL_AIRCRAFT_GO_HOME_NOTSET,     /*自动返航设置状态*/
};

ControlAircraft *g_psControlAircraft = &g_sControlAircraft;

/*控制算法参数初始化*/
SYS_RETERR ctrl_autocontrol_para_init(CTRL_SysStatus *sysStatus)
{
	SYS_RETERR errRet = SYS_RETERR_FF;
	
/*PID*/
#if (CONTROL_SYS__ONLY_PID == SYS_ENABLE)
	/*PID Init*/
	sysStatus->pid = (CTRL_SYSTEM_PARA_INIT_STATUS)pid_Parameter_Read_And_Init(&g_sPidSystem);
#endif	

/*ADRC*/
#if (CONTROL_SYS__ONLY_ADRC == SYS_ENABLE)
	/*ADRC Init*/
	sysStatus->adrc = (CTRL_SYSTEM_PARA_INIT_STATUS)adrc_Parameter_Read_And_Init(&g_sPidSystem);
#endif	
	
/*PID + ADRC*/
#if (CONTROL_SYS__PID_ADRC == SYS_ENABLE)
	/*PID Init*/
	sysStatus->pid = (CTRL_SYSTEM_PARA_INIT_STATUS)pid_Parameter_Read_And_Init(&g_sPidSystem);	
	
	/*ADRC Init*/	
	sysStatus->adrc = (CTRL_SYSTEM_PARA_INIT_STATUS)adrc_Parameter_Read_And_Init(&g_sPidSystem);
#endif
	
	if ((sysStatus->pid == CTRL_SYSTEM_PARA_INIT_SUCC) && (sysStatus->adrc == CTRL_SYSTEM_PARA_INIT_SUCC))
	{
		errRet = SYS_RETERR_0ZR;	/*控制算法:PID+ADRC*/
	}
	else if ((sysStatus->pid == CTRL_SYSTEM_PARA_INIT_FAIL) && (sysStatus->adrc == CTRL_SYSTEM_PARA_INIT_SUCC))
	{
		errRet = SYS_RETERR_1ST;		
	}
	else if ((sysStatus->pid == CTRL_SYSTEM_PARA_INIT_SUCC) && (sysStatus->adrc == CTRL_SYSTEM_PARA_INIT_FAIL))
	{
		errRet = SYS_RETERR_2ND;	/*控制算法:PID*/	
	}	
	else if ((sysStatus->pid == CTRL_SYSTEM_PARA_INIT_FAIL) && (sysStatus->adrc == CTRL_SYSTEM_PARA_INIT_FAIL))
	{
		errRet = SYS_RETERR_3RD;
	}	
	
	return errRet;
}

/*检测控制模式更改频率*/
vu8 g_CtrlCheckControlModeTicks = 0;

/*飞行器自动控制系统*/
void ctrl_auto_control_system_dp(void)
{	
	/*控制模式选择*/
	g_CtrlCheckControlModeTicks++;
	
	if (g_CtrlCheckControlModeTicks >= (20 / PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)) /*每20ms检测一次*/
	{
		g_CtrlCheckControlModeTicks = 0;		
		
		/*飞行模式及飞行任务选择*/
		ctrl_Control_Mode_Select(g_psUav_Status);
	}
	
	/************* 1.主导控制器: 定高+输出水平期望角 *************/
	ctrl_MainLeading_Control_Dp();
	
	/************* 2.姿态控制器: 角度外环+角速度内环 *************/
	ctrl_Attitude_Control_Dp();
}

/*飞行模式及飞行任务选择*/
void ctrl_Control_Mode_Select(Uav_Status *uavStatus)
{	
	/*遥控期望竖直控制模式 更迭*/
	uavStatus->UavControlMode.Vertical.Mode_Switch.LastTime = uavStatus->UavControlMode.Vertical.Mode_Switch.ThisTime;
	
	/*遥控期望水平控制模式 更迭*/
	uavStatus->UavControlMode.Horizontal.Mode_Switch.LastTime = uavStatus->UavControlMode.Horizontal.Mode_Switch.ThisTime;
	
	/*飞行任务 更迭*/
	uavStatus->UavCurrentFlyMission.LAST_FLY_MISSION = uavStatus->UavCurrentFlyMission.CURRENT_FLY_MISSION;
	
	/*============ 判断遥控器模式选择对应拨键的值,来进行模式切换 ============*/
	/*SWITCH A 默认上拨为纯姿态自稳,下拨为定高*/
	if (remot_Data_Range(g_sRemotData.SWA, REMOT_DATA_MIN) == REMOT_DATA_MIN)
	{
		uavStatus->UavControlMode.Vertical.Mode_Switch.ThisTime = UAV_VERTICAL_CONTROL_SELFAUTO; /*竖直姿态自稳模式模式*/
	}
	else if (remot_Data_Range(g_sRemotData.SWA, REMOT_DATA_MAX) == REMOT_DATA_MAX)
	{
		uavStatus->UavControlMode.Vertical.Mode_Switch.ThisTime = UAV_VERTICAL_CONTROL_FIX_HEIGHT; /*传感器定高模式*/		
	}

	/*SWITCH B 默认上拨为纯姿态自稳,下拨为水平定点*/
	if (remot_Data_Range(g_sRemotData.SWB, REMOT_DATA_MIN) == REMOT_DATA_MIN)
	{
		uavStatus->UavControlMode.Horizontal.Mode_Switch.ThisTime = UAV_HORIZONTAL_CONTROL_SELFAUTO;	/*水平姿态自稳模式模式*/	
	}
	else if (remot_Data_Range(g_sRemotData.SWB, REMOT_DATA_MAX) == REMOT_DATA_MAX)
	{
		uavStatus->UavControlMode.Horizontal.Mode_Switch.ThisTime = UAV_HORIZONTAL_CONTROL_FIX_POS;     /*传感器水平定点模式*/
	}
	
	/*SWITCH C 上拨为清除当前任务进入姿态控制模式,中拨为一键起飞,下拨为一键返航*/
	if (remot_Data_Range(g_sRemotData.SWC, REMOT_DATA_MIN) == REMOT_DATA_MIN)
	{
		/*有任务时,清空任务,自动切换到执行任务前的模式*/
		if (uavStatus->UavCurrentFlyMission.CURRENT_FLY_MISSION != UAV_FLY_MISSION_NULL)
		{
			/*清除上一个任务,并设置当前任务为NULL*/
			control_fly_mission_set(uavStatus, UAV_FLY_MISSION_NULL, UAV_MISSION_ATTITUDE_CTRL_ENABLE);
			
			/*重置竖直高度零参考点设置状态*/
			uavStatus->UavControlMode.Vertical.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_NO;

			/*重置水平位置零参考点设置状态*/
			uavStatus->UavControlMode.Horizontal.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_NO;
		}
	}
	else if (remot_Data_Range(g_sRemotData.SWC, REMOT_DATA_MID) == REMOT_DATA_MID)
	{
		/*检测是否满足一键起飞条件: 使能 & 还未执行 & 定高状态*/
		if (/*(uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.ENABLE_STATUS == UAV_MISSION_ENABLE) && \*/
			(uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.EXECUTE_STATUS == UAV_MISSION_EXECUTE_NOT) && \
			(uavStatus->UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_FIX_HEIGHT))
		{
			/*一键起飞不稳定,取消*/
//			/*清除上一个任务,任务切换成一键起飞,使能姿态控制*/
//			control_fly_mission_set(uavStatus, UAV_FLY_MISSION_ONEKEY_FLY, UAV_MISSION_ATTITUDE_CTRL_ENABLE);
//			
//			/*标记任务已执行,防止重复设置任务状态*/
//			uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.EXECUTE_STATUS = UAV_MISSION_EXECUTE_YES;
		}
	}
	else if (remot_Data_Range(g_sRemotData.SWC, REMOT_DATA_MAX) == REMOT_DATA_MAX)
	{
		/*检测是否满足一键降落条件:使能 & 还未执行 & 定高状态*/		
		if (/*(uavStatus->UavCurrentFlyMission.Onekey_Mission.LandHome.ENABLE_STATUS == UAV_MISSION_ENABLE) && \*/
			(uavStatus->UavCurrentFlyMission.Onekey_Mission.LandHome.EXECUTE_STATUS == UAV_MISSION_EXECUTE_NOT) && \
			(uavStatus->UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_FIX_HEIGHT))
		{
			/*清除上一个任务,任务切换成一键返航,禁止任务外的姿态控制*/
			control_fly_mission_set(uavStatus, UAV_FLY_MISSION_ONEKEY_LAND_HOME, UAV_MISSION_ATTITUDE_CTRL_DISABLE);

			/*标记任务已执行,防止重复设置任务状态*/
			uavStatus->UavCurrentFlyMission.Onekey_Mission.LandHome.EXECUTE_STATUS = UAV_MISSION_EXECUTE_YES;
		}
	}
	
	/*============ 定高&定点遥控切换期望 ============*/
	/*高度:本次期望控制模式和上次期望控制模式不同*/
	if (uavStatus->UavControlMode.Vertical.Mode_Switch.ThisTime != uavStatus->UavControlMode.Vertical.Mode_Switch.LastTime)
	{
		/*自稳切定高  时刻*/
		if (uavStatus->UavControlMode.Vertical.Mode_Switch.ThisTime == UAV_VERTICAL_CONTROL_FIX_HEIGHT)
		{
			/*判断SWITCH D拨键状态来选定竖直方向传感器工作状态:不切换/切换*/
			if (remot_Data_Range(g_sRemotData.SWD, REMOT_DATA_MIN) == REMOT_DATA_MIN)
			{
				/*无限制传感器 参与定高*/	
				uavStatus->UavSenmodStatus.Vertical.WORK_STATUS = UAV_SENMOD_WORK_LIMITLESS;
			}
			else if (remot_Data_Range(g_sRemotData.SWD, REMOT_DATA_MAX) == REMOT_DATA_MAX)
			{
				/*有限制传感器和无限制传感器自动切换 参与定高*/			
				uavStatus->UavSenmodStatus.Vertical.WORK_STATUS = UAV_SENMOD_WORK_AUTO_SWITCH;
			}
			
			/*竖直方向 标记本次竖直方向 遥控期望 为 定高模式*/
			uavStatus->UavControlMode.Vertical.EXPECT_CONTROL_MODE = UAV_VERTICAL_CONTROL_FIX_HEIGHT;
			
			/*竖直方向 标记零参考点未设置(切换后需要重新记录当前高度)*/
			uavStatus->UavControlMode.Vertical.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_NO;
		}
		/*定高切自稳  时刻*/
		else if (uavStatus->UavControlMode.Vertical.Mode_Switch.ThisTime == UAV_VERTICAL_CONTROL_SELFAUTO)
		{
			/*竖直方向 默认恢复无限制传感器*/
			uavStatus->UavSenmodStatus.Vertical.WORK_STATUS = UAV_SENMOD_WORK_LIMITLESS; 
			
			/*竖直方向 标记本次遥控期望为自稳*/
			uavStatus->UavControlMode.Vertical.EXPECT_CONTROL_MODE = UAV_VERTICAL_CONTROL_SELFAUTO;
		}
	}

	/*水平:本次期望控制模式和上次期望控制模式不同*/
	if (uavStatus->UavControlMode.Horizontal.Mode_Switch.ThisTime != uavStatus->UavControlMode.Horizontal.Mode_Switch.LastTime) 
	{
		/*自稳切水平定点  时刻*/
		if (uavStatus->UavControlMode.Horizontal.Mode_Switch.ThisTime == UAV_HORIZONTAL_CONTROL_FIX_POS)
		{
			/*判断SWITCH D拨键状态来选定竖直方向传感器工作状态:不切换/切换*/
			if (remot_Data_Range(g_sRemotData.SWD, REMOT_DATA_MIN) == REMOT_DATA_MIN)
			{
				/*无限制传感器参与定点*/
				uavStatus->UavSenmodStatus.Horizontal.WORK_STATUS = UAV_SENMOD_WORK_LIMITLESS;		
			}
			else if (remot_Data_Range(g_sRemotData.SWD, REMOT_DATA_MAX) == REMOT_DATA_MAX)
			{
				/*有限制传感器和无限制传感器自动切换*/				
				uavStatus->UavSenmodStatus.Horizontal.WORK_STATUS = UAV_SENMOD_WORK_AUTO_SWITCH;
			}
			
			/*水平方向 标记本次为自稳切水平定点*/
			uavStatus->UavControlMode.Horizontal.EXPECT_CONTROL_MODE = UAV_HORIZONTAL_CONTROL_FIX_POS;
			
			/*水平方向 标记零参考点未设置(切换后需要重新记录当前水平位置)*/
			uavStatus->UavControlMode.Horizontal.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_NO;			
		}
		/*水平定点切自稳  时刻*/
		else if (uavStatus->UavControlMode.Horizontal.Mode_Switch.ThisTime == UAV_HORIZONTAL_CONTROL_SELFAUTO)
		{
			/*水平方向 默认恢复无限制传感器*/
			uavStatus->UavSenmodStatus.Horizontal.WORK_STATUS = UAV_SENMOD_WORK_LIMITLESS;
			
			/*水平方向 标记本次遥控期望为自稳*/
			uavStatus->UavControlMode.Horizontal.EXPECT_CONTROL_MODE = UAV_HORIZONTAL_CONTROL_SELFAUTO;			
		}
	}

	/*水平方向,GPS和光流切换(独立融合和控制)*/
	sins_horizontal_gps_opticflow_auto_change(uavStatus);
	
	/*飞行器未失联,才接受遥控改变当前飞行状态*/
	if (uavStatus->WIRELESS_CMC_STATUS == UAV_WIRELESS_CMC_SUCC)
	{
		/*============ 定高条件判断及控制模式切换 ============*/
		/*竖直自稳切定高*/
		if (uavStatus->UavControlMode.Vertical.EXPECT_CONTROL_MODE == UAV_VERTICAL_CONTROL_FIX_HEIGHT)
		{
			/*竖直定高传感器有效*/
			if (uavStatus->UavSenmodStatus.Vertical.CURRENT_USE != UAV_VERTICAL_SENMOD_CURRENT_NULL)
			{
				/*高度参考点未设置*/
				if (uavStatus->UavControlMode.Vertical.REFERENCE_SET_STATUS == UAV_SWITCH_REFERENCE_SET_NO)
				{
					/*每次切换只保留一次当前油门值*/
					g_psControlAircraft->heightHoldThrottle = g_psControlAircraft->RemotExpectAngle.throttle;
		
					/*将Switch_SWA开关向下拨动瞬间的惯导竖直位置估计作为目标高度*/
					g_psPidSystem->HighPosition.expect = g_psSinsReal->curPosition[EARTH_FRAME_Z];
		
					/*设置成功后 标记零参考点已设置(防止定高后,重新更新期望高度)*/
					uavStatus->UavControlMode.Vertical.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_OK;

					/*飞行模式标记为定高*/
					uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_FIX_HEIGHT; /*= 2->定高*/
		
					/*竖直控制模式为:定高控制模式*/
					uavStatus->UavControlMode.Vertical.CONTROL_MODE = UAV_VERTICAL_CONTROL_FIX_HEIGHT;
				}
			}
			/*竖直定高传感器无效*/
			else if (uavStatus->UavSenmodStatus.Vertical.CURRENT_USE == UAV_VERTICAL_SENMOD_CURRENT_NULL)
			{
				/*飞行模式标记为姿态模式*/
				uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/				
			
				/*竖直控制模式为:自稳控制模式*/
				uavStatus->UavControlMode.Vertical.CONTROL_MODE = UAV_VERTICAL_CONTROL_SELFAUTO;
			}
		}
		else if (uavStatus->UavControlMode.Vertical.EXPECT_CONTROL_MODE == UAV_VERTICAL_CONTROL_SELFAUTO)	/*定高切自稳*/
		{
			/*飞行模式标记为姿态*/
			uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/	
		
			/*竖直控制模式为:自稳控制模式*/
			uavStatus->UavControlMode.Vertical.CONTROL_MODE = UAV_VERTICAL_CONTROL_SELFAUTO;			
		}
		
		/*============ 定点条件判断及控制模式切换 ============*/
		/*水平自稳切定点,水平数据来源: 光流 / GPS*/
		if (uavStatus->UavControlMode.Horizontal.EXPECT_CONTROL_MODE == UAV_HORIZONTAL_CONTROL_FIX_POS)
		{
			/*水平定点传感器是GPS*/
			if (uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE == UAV_HORIZONTAL_SENMOD_CURRENT_GPS)
			{
				/*参考点未设置*/
				if(uavStatus->UavControlMode.Horizontal.REFERENCE_SET_STATUS == UAV_SWITCH_REFERENCE_SET_NO)
				{
					/*将当前惯导水平位置估计作为目标悬停点*/
					g_psPidSystem->LatitudePosition.expect  = g_psSinsReal->curPosition[EARTH_FRAME_Y]; /*N*/
					g_psPidSystem->LongitudePosition.expect = g_psSinsReal->curPosition[EARTH_FRAME_X];	/*E*/
						
					/*GPS: PID水平控制积分复位*/
					pid_Horizontal_GPS_Ctrl_Integrate_Reset();
			
					/*设置成功后 标记零参考点已设置(防止定点后,重新更新期望位置)*/
					uavStatus->UavControlMode.Horizontal.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_OK;	
			
					/*飞行模式标记为定点*/
					uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_FIX_POS;	/*= 3->定点*/		
			
					/*水平控制模式为:定点控制模式*/
					uavStatus->UavControlMode.Horizontal.CONTROL_MODE = UAV_HORIZONTAL_CONTROL_FIX_POS;
				}
			}
			/*水平定点传感器是OPTICFLOW*/
			else if (uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE == UAV_HORIZONTAL_SENMOD_CURRENT_OPTICFLOW)
			{
				/*参考点未设置*/
				if(uavStatus->UavControlMode.Horizontal.REFERENCE_SET_STATUS == UAV_SWITCH_REFERENCE_SET_NO)
				{
					/*将当前惯导水平位置估计作为目标悬停点*/
						
					/*光流PID水平控制积分复位*/
			
					/*设置成功后 标记零参考点已设置(防止定点后,重新更新期望位置)*/
					uavStatus->UavControlMode.Horizontal.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_OK;	
			
					/*飞行模式标记为定点*/
					uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_FIX_POS;	/*= 3->定点*/		
			
					/*水平控制模式为:定点控制模式*/
					uavStatus->UavControlMode.Horizontal.CONTROL_MODE = UAV_HORIZONTAL_CONTROL_FIX_POS;
				}			
			}
			/*水平定点传感器无效*/
			else if (uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE == UAV_HORIZONTAL_SENMOD_CURRENT_NULL)
			{
				/*判断当前是定高还是姿态*/
				if (uavStatus->UavFlyType.CURRENT == UAV_FLY_TYPE_ATTITUDE)
				{
					/*飞行模式标记为姿态模式*/
					uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/				
				}
				else if (uavStatus->UavFlyType.CURRENT == UAV_FLY_TYPE_FIX_HEIGHT)
				{
					/*飞行模式标记为定高模式*/
					uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_FIX_HEIGHT; /*= 2->定高*/
				}

				/*退出点后,标记参考点未设置,这样会一直check*/
				uavStatus->UavControlMode.Horizontal.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_NO;
				
				/*GPS: PID水平控制积分复位*/
				pid_Horizontal_GPS_Ctrl_Integrate_Reset();
				
				/*光流PID水平控制积分复位*/	
				
			
				/*水平控制模式为:自稳控制模式*/
				uavStatus->UavControlMode.Horizontal.CONTROL_MODE = UAV_HORIZONTAL_CONTROL_SELFAUTO;				
			}
		}
		else if(uavStatus->UavControlMode.Horizontal.EXPECT_CONTROL_MODE == UAV_HORIZONTAL_CONTROL_SELFAUTO) /*水平定点切自稳*/
		{
			/*飞行模式标记为非定点模式*/
			if (uavStatus->UavFlyType.CURRENT == UAV_FLY_TYPE_FIX_HEIGHT) /*如果仍处在定高模式,则为定高模式*/
			{
				uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_FIX_HEIGHT;
			}
			else /*如果没处在定高,也没处在定点,则为纯姿态模式*/
			{
				uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/			
			}		
			
			/*GPS: PID水平控制积分复位*/
			pid_Horizontal_GPS_Ctrl_Integrate_Reset();
				
			/*光流PID水平控制积分复位*/			
			
			
			/*水平控制模式为:自稳控制模式*/
			uavStatus->UavControlMode.Horizontal.CONTROL_MODE = UAV_HORIZONTAL_CONTROL_SELFAUTO;			
		}
		
		/*============ SWD拨键根据飞行器当前状态设置飞行任务 ============*/		
		/*判断当前是否在竖直定高+水平定点模式*/
		if ((uavStatus->UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_FIX_HEIGHT) && \
			(uavStatus->UavControlMode.Horizontal.CONTROL_MODE == UAV_HORIZONTAL_CONTROL_FIX_POS))
		{
			/*在光流+超声波参与定点的控制下,获取当前飞行任务*/
			if ((uavStatus->UavSenmodStatus.Vertical.CURRENT_USE == UAV_VERTICAL_SENMOD_CURRENT_ULTR) && \
				(uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE == UAV_HORIZONTAL_SENMOD_CURRENT_OPTICFLOW))
			{
				/*5000ms内:1次光流追点/2次光流循黑线任务*/		
				uavStatus->UavCurrentFlyMission.CURRENT_FLY_MISSION = control_fly_mission_check_ultr_opflow_pos(uavStatus, 5000);
			}
			/*在GPS+气压计参与定点的控制下,获取当前飞行任务*/
			else if ((uavStatus->UavSenmodStatus.Vertical.CURRENT_USE == UAV_VERTICAL_SENMOD_CURRENT_BERO) && \
					 (uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE == UAV_HORIZONTAL_SENMOD_CURRENT_GPS))
			{
				/*5000ms内:1次记点/2次按记点开始巡天*/		
				uavStatus->UavCurrentFlyMission.CURRENT_FLY_MISSION = control_fly_mission_check_bero_gps_pos(uavStatus, 5000);
			}
		}
	}
	
	/*=== 判断是否需要执行安全强制任务 ===*/
	if (safe_force_mission_get(&gs_SafeOperation) != UAV_FLY_MISSION_NULL)
	{
		/*如果是解锁状态*/
		if (uavStatus->LOCK_STATUS == UAV_LOCK_NOT)
		{
			/*着陆状态*/
			if (uavStatus->UavLandStatus.ThisTime == UAV_LAND_YES)
			{
				/*,强制上锁解锁*/
				uavStatus->LOCK_STATUS = UAV_LOCK_YES;
			}
			/*飞行状态*/
			else if (uavStatus->UavLandStatus.ThisTime == UAV_LAND_NOT)
			{				
				/*清除上一个任务,执行强制性的安全任务*/
				control_fly_mission_set(uavStatus, gs_SafeOperation.SAFE_FORCE_MISSION, UAV_MISSION_ATTITUDE_CTRL_DISABLE);
			}
		}	
	}
}

/*姿态控制器:角度外环+角速度内环*/
void ctrl_Attitude_Control_Dp(void)
{
	/*角度环控制*/
	ctrl_Attitude_Angle_Control_Dp();

	/*角速度环控制*/	
	ctrl_Attitude_Gyro_Control_Dp();
}


vu16 g_WaitMagGyroFuseTicks = 0; /*等待磁力计和陀螺仪融合时间*/

/*角度外环控制*/
void ctrl_Attitude_Angle_Control_Dp(void)
{
	/*==== ROLL(横滚角),PITCH(俯仰角) 外环角度反馈与控制 ====*/
	/***************外环 角度 反馈****************/
	g_psPidSystem->PitchAngle.feedback = g_psAttitudeAll->Ahrs.pitch;
	g_psPidSystem->RollAngle.feedback  = g_psAttitudeAll->Ahrs.roll;
	
	/***************外环 角度 控制****************/
	g_psPidSystem->PitchAngle.controlOutput = pid_Control_General_Dp(&g_psPidSystem->PitchAngle); 	  /*PITCH角PID运算及输出*/	
	g_psPidSystem->RollAngle.controlOutput  = pid_Control_General_Dp(&g_psPidSystem->RollAngle);	  /*ROLL角PID运算及输出*/	
	
	/*=========== YAW(偏航角) 外环角度反馈与控制 =========*/	
	/*YAW摇杆置于中间位置*/
	if (g_psControlAircraft->RemotExpectAngle.yaw == 0)
	{
		/*无头模式、飞机上电后一段时间锁定偏航角,磁力计、陀螺仪融合需要一段时间,这里取500次(2.5S)*/
		if (g_WaitMagGyroFuseTicks <= 500)
		{
			g_WaitMagGyroFuseTicks++;
		}
		
		/*YAW摇杆置于中间位值期望角为0、满足落地条件,偏航角期望重置为当前偏航角*/
		if ((g_psPidSystem->YawAngle.expect == 0) || \
			(g_WaitMagGyroFuseTicks <= 500) || \
			(g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_YES))
		{
			/*偏航角期望重置为当前偏航角*/
			g_psPidSystem->YawAngle.expect = g_psAttitudeAll->Ahrs.yaw;
		}
		
		/*偏航角反馈更新*/
		g_psPidSystem->YawAngle.feedback = g_psAttitudeAll->Ahrs.yaw;
		
		/*PID Yaw角控制及输出*/
		g_psPidSystem->YawAngle.controlOutput = pid_Control_Yaw_Dp(&(g_psPidSystem->YawAngle));  
		
		/*更新偏航角速度环期望,来源于偏航角度控制器输出*/
		g_psPidSystem->YawGyro.expect = g_psPidSystem->YawAngle.controlOutput;	
	}
	else /*波动偏航方向杆后,只进行内环角速度控制(遥控期望直接给内环角速度)*/
	{
		/*偏航角期望给0,不进行角度控制*/
		g_psPidSystem->YawAngle.expect = 0;		
		
		/*偏航角速度环期望,直接来源于遥控器打杆量*/
		g_psPidSystem->YawGyro.expect = g_psControlAircraft->RemotExpectAngle.yaw;
	}
}

#define CTRL_YAW_FEEDFORWARD 	(0.25f) /*偏航角前馈控制*/
vu16 g_YawControlFaultHandleTicks = 0; /*等待磁力计和陀螺仪融合时间*/

/*角速度内环控制*/
void ctrl_Attitude_Gyro_Control_Dp(void)
{
	/*==== ROLL(横滚角),PITCH(俯仰角) 内环角速度反馈与控制 ====*/
	/***************内环 角速度 期望****************/
	g_psPidSystem->PitchGyro.expect = g_psPidSystem->PitchAngle.controlOutput; /*等于角度外环输出*/
	g_psPidSystem->RollGyro.expect  = g_psPidSystem->RollAngle.controlOutput;  /*等于角度外环输出*/
	
	/***************内环 角速度 反馈****************/
	g_psPidSystem->PitchGyro.feedback = g_psGyroFeedback->Pitch;
	g_psPidSystem->RollGyro.feedback  = g_psGyroFeedback->Roll;	
	
	/***************内环 角速度 控制****************/
	g_psPidSystem->PitchGyro.controlOutput = pid_Control_Div_LPF(&(g_psPidSystem->PitchGyro), PID_CONTROLER_PITCH_GYRO);  /*PID DIV控制低通滤波*/
	g_psPidSystem->RollGyro.controlOutput  = pid_Control_Div_LPF(&(g_psPidSystem->RollGyro), PID_CONTROLER_ROLL_GYRO);    /*PID DIV控制低通滤波*/
	
	/*============ YAW(偏航角) 内环角速度反馈与控制 ===========*/
	/***************内环 角速度 反馈****************/
	g_psPidSystem->YawGyro.feedback = g_psGyroFeedback->Yaw;

	/***************内环 角速度 控制****************/	
	g_psPidSystem->YawGyro.controlOutput = pid_Control_Div_LPF(&(g_psPidSystem->YawGyro), PID_CONTROLER_YAW_GYRO);    /*PID DIV控制低通滤波*/	
	
	/***************偏航角前馈控制****************/
	g_psPidSystem->YawGyro.controlOutput += CTRL_YAW_FEEDFORWARD * g_psPidSystem->YawGyro.expect;
	
	/*内环 角速度 输出限幅*/
	if (g_psPidSystem->YawGyro.controlOutput >= g_psPidSystem->YawAngle.controlOutPutLimit)
	{
		g_psPidSystem->YawGyro.controlOutput = g_psPidSystem->YawAngle.controlOutPutLimit;
	}

	if (g_psPidSystem->YawGyro.controlOutput <= -g_psPidSystem->YawAngle.controlOutPutLimit)
	{
		g_psPidSystem->YawGyro.controlOutput = -g_psPidSystem->YawAngle.controlOutPutLimit;
	}

    /*=== 偏航控制异常情况判断,即偏航控制量很大时,偏航角速度很小,如此时为强外力干扰、已着地等 ===*/	
	if ((math_Abs(g_psPidSystem->YawGyro.controlOutput) > ( g_psPidSystem->YawGyro.controlOutPutLimit / 2)) && \
	    (math_Abs(g_psGyroFeedback->Yaw) <= 30.0f)) /*偏航控制输出相对较大,但偏航角速度相对很小*/
	{
		g_YawControlFaultHandleTicks++;	/*5ms执行一次*/
		
		if (g_YawControlFaultHandleTicks >= 500)
		{
			g_YawControlFaultHandleTicks = 500; /*限幅*/
		}
	}
	else
	{
		g_YawControlFaultHandleTicks /= 2; /*不满足，快速削减至0*/
	}
	
	if (g_YawControlFaultHandleTicks >= 400) /*2S执行一次, 特殊处理*/
	{
		/*Tick清0*/
		g_YawControlFaultHandleTicks = 0;		
		
		/*清空偏航角度控制的积分*/
		pid_Link_Integrate_Reset(&(g_psPidSystem->YawAngle));		
		
		/*清空偏航角速度控制的积分*/
		pid_Link_Integrate_Reset(&(g_psPidSystem->YawGyro)); 
		
		/*偏航角期待更新为当前偏航角*/
		g_psPidSystem->YawAngle.expect = g_psAttitudeAll->Ahrs.yaw;
	}
}

/*主导控制器: 自稳+定高&定点*/
void ctrl_MainLeading_Control_Dp(void)
{
	fp32 mainControlDeltaT;
	
	/*获取主导控制器执行间隔时间*/
	get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->CTRL_MainLeading));
	
	/*间隔时间换算成秒*/
	mainControlDeltaT = g_psSystemPeriodExecuteTime->CTRL_MainLeading.DeltaTime / 1000.0f;
	
	/*飞行任务：无 || 执行任务同时执行姿态控制*/
	if ((g_sUav_Status.UavCurrentFlyMission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_NULL) || \
		(g_sUav_Status.UavCurrentFlyMission.ATTITUDE_CTRL_STATUS == UAV_MISSION_ATTITUDE_CTRL_ENABLE))
	{
		/*根据遥控器切换档位,飞控进入不同模式*/
		/*================== 1.高度自稳,水平自稳 模式 ==================*/
		if ((g_sUav_Status.UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_SELFAUTO) && \
			(g_sUav_Status.UavControlMode.Horizontal.CONTROL_MODE == UAV_HORIZONTAL_CONTROL_SELFAUTO))
		{
			/*更新自稳控制下的ROLL(横滚)和PITCH(俯仰)期望角 (遥控期望角已限制在最大值范围)*/
			g_psPidSystem->PitchAngle.expect = g_psControlAircraft->RemotExpectAutoAngle.pitch; /*遥控期待自稳角pitch*/
			g_psPidSystem->RollAngle.expect  = g_psControlAircraft->RemotExpectAutoAngle.roll;	/*遥控期待自稳角roll*/
		
			/*对于油门范围是0~1000,对于PWM周期范围是1000~2000,因此最低为1000*/
			if (g_psControlAircraft->RemotExpectAngle.throttle <= 1000)
			{
				g_psControlAircraft->ctrlThrottle = 1000;
			}
			else /*油门直接来源于遥控器油门给定*/
			{
				g_psControlAircraft->ctrlThrottle = g_psControlAircraft->RemotExpectAngle.throttle;
			}
		
			/*油门值更迭*/
			g_psControlAircraft->lastCtrlThrottle = g_psControlAircraft->ctrlThrottle;
		}
		/*================== 2.高度定高,水平自稳 模式 ==================*/
		else if ((g_sUav_Status.UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_FIX_HEIGHT) && \
			     (g_sUav_Status.UavControlMode.Horizontal.CONTROL_MODE == UAV_HORIZONTAL_CONTROL_SELFAUTO))
		{
			/*更新水平自稳控制下的ROLL(横滚)和PITCH(俯仰)期望角 (遥控期望角已限制在最大值范围)*/
			g_psPidSystem->PitchAngle.expect = g_psControlAircraft->RemotExpectAutoAngle.pitch; /*遥控期待自稳角pitch*/
			g_psPidSystem->RollAngle.expect  = g_psControlAircraft->RemotExpectAutoAngle.roll;	/*遥控期待自稳角roll*/
			
			/*竖直高度(定高)控制器*/
			vertical_Control_AltHold(mainControlDeltaT);	
		}
		/*================== 3.高度定高,水平定点(GPS/光流) 模式 ==================*/		
		else if ((g_sUav_Status.UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_FIX_HEIGHT) && \
				 (g_sUav_Status.UavControlMode.Horizontal.CONTROL_MODE != UAV_HORIZONTAL_CONTROL_FIX_POS))
		{
			/*竖直高度控制器*/
			vertical_Control_AltHold(mainControlDeltaT);

			/*水平位置控制器*/		
			horizontal_Control_PosHold(mainControlDeltaT);	
		}
		/*================== 4.高度其它,水平其它 ==================*/		
		else
		{	
			/*更新水平角度期望,更新控制油门*/
			/*更新水平自稳控制下的ROLL(横滚)和PITCH(俯仰)期望角 (遥控期望角已限制在最大值范围)*/
			g_psPidSystem->PitchAngle.expect = g_psControlAircraft->RemotExpectAutoAngle.pitch;
			g_psPidSystem->RollAngle.expect  = g_psControlAircraft->RemotExpectAutoAngle.roll;		
		
			/*油门范围0~1000*/
			if (g_psControlAircraft->RemotExpectAngle.throttle <= 1000)
			{
				g_psControlAircraft->ctrlThrottle = 1000;
			}
			else
			{
				/*油门直接来源于遥控器油门给定*/
				g_psControlAircraft->ctrlThrottle = g_psControlAircraft->RemotExpectAngle.throttle;
			}
		
			/*控制油门推迭*/
			g_psControlAircraft->lastCtrlThrottle = g_psControlAircraft->ctrlThrottle;
		}
	}
	
	/*飞行任务：一键起飞*/
	if (g_sUav_Status.UavCurrentFlyMission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_ONEKEY_FLY)
	{
		control_fly_mission_onekey_fly(g_psUav_Status);
	}
	/*飞行任务：失联或遥控人为遥控一键返航*/
	else if (g_sUav_Status.UavCurrentFlyMission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_ONEKEY_LAND_HOME) 
	{
		/*飞行器自动返航控制*/
		ctrl_Go_Home_Control(mainControlDeltaT);
	}
	/*飞行任务：光流追点*/
	else if (g_sUav_Status.UavCurrentFlyMission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_OPFLOW_FOLLOW_POINT) 
	{
		/*清除上一个任务,未做默认无任务*/		
		control_fly_mission_set(g_psUav_Status, UAV_FLY_MISSION_NULL, UAV_MISSION_ATTITUDE_CTRL_ENABLE);
	}
	/*飞行任务：光流巡线*/
	else if (g_sUav_Status.UavCurrentFlyMission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_OPFLOW_FOLLOW_LINE) 
	{
		/*清除上一个任务,未做默认无任务*/		
		control_fly_mission_set(g_psUav_Status, UAV_FLY_MISSION_NULL, UAV_MISSION_ATTITUDE_CTRL_ENABLE);
	}
	/*飞行任务：GPS记录座标*/
	else if (g_sUav_Status.UavCurrentFlyMission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_GPS_WRITE_POS)
	{
		control_fly_mission_bero_gps_write_pos(g_psUav_Status);
	}
	/*飞行任务：GPS巡天*/
	else if (g_sUav_Status.UavCurrentFlyMission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_GPS_PATROL_SKY) 
	{
		/*清除上一个任务,未做默认无任务*/
		control_fly_mission_set(g_psUav_Status, UAV_FLY_MISSION_NULL, UAV_MISSION_ATTITUDE_CTRL_ENABLE);
	}
}

/*油门倾角补偿*/
void ctrl_Throttle_DipAngle_Compensate(void)
{
	fp32 CosPitch_CosRoll = math_Abs(COS_PITCH * COS_ROLL);
	fp32 throttleMakeup = 0;
	fp32 temp = 0;
	
	if (CosPitch_CosRoll >= 0.999999f)
	{
		CosPitch_CosRoll = 0.999999f;
	}
	
	if (CosPitch_CosRoll <= 0.000001f)
	{
		CosPitch_CosRoll = 0.000001f;
	}
	
	if (CosPitch_CosRoll <= 0.50f)
	{
		CosPitch_CosRoll = 0.50f; /*Pitch,Roll约等于30度*/
	}
	
	/*大于起转油门量*/
	if (g_psControlAircraft->ctrlThrottle >= CTRL_THROTTLE_START_TURN)
	{
		temp = (u16)MATH_MAX(MATH_ABS(100 * g_psAttitudeAll->Ahrs.pitch), MATH_ABS(100 * g_psAttitudeAll->Ahrs.roll));
		temp = math_Constrain(9000 - temp, 3000, 0) / (3000 * CosPitch_CosRoll);
		throttleMakeup = (g_psControlAircraft->ctrlThrottle - CTRL_THROTTLE_START_TURN) * temp; /*油门倾角补偿*/
		
		/*油门输出*/
		g_psControlAircraft->throttleOutput = (u16)(CTRL_THROTTLE_START_TURN + throttleMakeup);
		g_psControlAircraft->throttleOutput = (u16)(math_Constrain(g_psControlAircraft->throttleOutput, 2000, CTRL_THROTTLE_START_TURN));
	}
	else /*低于起转油门量*/
	{
		g_psControlAircraft->throttleOutput = g_psControlAircraft->ctrlThrottle;
	}
}

vu16 g_vu16ThrottleIdleTransitionsTicks = 0;  		  /*怠速过渡ticks*/
vu16 g_vu16ThrottleIdleContinueTicks    = 0;  	      /*怠速持续ticks*/

/*自控系统控制器输出*/
void ctrl_auto_control_system_output(void)
{		
	/*油门倾角补偿*/
	ctrl_Throttle_DipAngle_Compensate();
	
	/*着陆检测*/
	g_sUav_Status.UavLandStatus.ThisTime = ctrl_Landing_Status_Check(g_psUav_Status);
	
	/*=== 检测飞行器处于解锁状态 ===*/
	if (g_sUav_Status.LOCK_STATUS == UAV_LOCK_NOT)
	{
		/*检测到飞行状态为着陆状态,输出怠速*/
		if (g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_YES)
		{
			/*如果上次油门输出值为最低位,进入怠速时,安排过渡过程*/
			if ((g_psControlAircraft->LastMotorPwmOutput.channle1 <= REMOT_THROTTLE_BASE_VALUE) && \
				(g_psControlAircraft->LastMotorPwmOutput.channle2 <= REMOT_THROTTLE_BASE_VALUE) && \
				(g_psControlAircraft->LastMotorPwmOutput.channle3 <= REMOT_THROTTLE_BASE_VALUE) && \
				(g_psControlAircraft->LastMotorPwmOutput.channle4 <= REMOT_THROTTLE_BASE_VALUE))
			{
				g_vu16ThrottleIdleTransitionsTicks = CTRL_IDEL_TRANSITION_CONTINUE_MAX_TICKS;
			}
			/*上次为起飞状态,本次为着陆状态,即完成一次升降,落地后,飞行器自动上锁*/
			else if (g_sUav_Status.UavLandStatus.LastTime == UAV_LAND_NOT)
			{
				g_sUav_Status.LOCK_STATUS = UAV_LOCK_YES;
			}
			
			g_vu16ThrottleIdleContinueTicks++;
			
			if (g_vu16ThrottleIdleContinueTicks >= CTRL_IDEL_TRANSITION_INC_PERIOD_TICKS)
			{
				if (g_vu16ThrottleIdleTransitionsTicks >= 1)
				{
					g_vu16ThrottleIdleTransitionsTicks--;
				}

				g_vu16ThrottleIdleContinueTicks = 0;				
			}
			
			/*油门怠速*/
			g_psControlAircraft->CurMotorPwmOutput.channle1 = REMOT_THROTTLE_BASE_VALUE + \
															  (CTRL_IDEL_TRANSITION_CONTINUE_MAX_TICKS - g_vu16ThrottleIdleTransitionsTicks) * \
															  (CTRL_IDEL_THROTTLE_VALUE - REMOT_THROTTLE_BASE_VALUE) / \
															   CTRL_IDEL_TRANSITION_CONTINUE_MAX_TICKS;   
			
			g_psControlAircraft->CurMotorPwmOutput.channle2 = REMOT_THROTTLE_BASE_VALUE + \
															  (CTRL_IDEL_TRANSITION_CONTINUE_MAX_TICKS - g_vu16ThrottleIdleTransitionsTicks) * \
															  (CTRL_IDEL_THROTTLE_VALUE - REMOT_THROTTLE_BASE_VALUE) / \
															   CTRL_IDEL_TRANSITION_CONTINUE_MAX_TICKS;   
			
			g_psControlAircraft->CurMotorPwmOutput.channle3 = REMOT_THROTTLE_BASE_VALUE + \
															  (CTRL_IDEL_TRANSITION_CONTINUE_MAX_TICKS - g_vu16ThrottleIdleTransitionsTicks) * \
															  (CTRL_IDEL_THROTTLE_VALUE - REMOT_THROTTLE_BASE_VALUE) / \
															   CTRL_IDEL_TRANSITION_CONTINUE_MAX_TICKS;   
			
			g_psControlAircraft->CurMotorPwmOutput.channle4 = REMOT_THROTTLE_BASE_VALUE + \
															  (CTRL_IDEL_TRANSITION_CONTINUE_MAX_TICKS - g_vu16ThrottleIdleTransitionsTicks) * \
															  (CTRL_IDEL_THROTTLE_VALUE - REMOT_THROTTLE_BASE_VALUE) / \
														       CTRL_IDEL_TRANSITION_CONTINUE_MAX_TICKS; 

			/*飞行器处于着陆状态,清除PID控制系统积分项*/
			pid_Horizontal_Takeoff_Integrate_Reset();
		}
		/*解锁后不满足着陆条件,即检测到飞行状态为飞行状态*/
		else if (g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_NOT)
		{
			/*=== 1.竖直方向,姿态自稳模式 ===*/
			if (g_sUav_Status.UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_SELFAUTO)				
			{
				/*大于起飞油门量*/
				if (g_psControlAircraft->ctrlThrottle >= CTRL_THROTTLE_START_FLY_VALUE)
				{
					/*默认水平姿态环角速度读控制器来源于PID*/
					g_psControlAircraft->CurMotorPwmOutput.channle1 = (u16)(g_psControlAircraft->throttleOutput    - \
																		    g_psPidSystem->RollGyro.controlOutput  + \
																			g_psPidSystem->PitchGyro.controlOutput - \
																			g_psPidSystem->YawGyro.controlOutput);

					g_psControlAircraft->CurMotorPwmOutput.channle2 = (u16)(g_psControlAircraft->throttleOutput    + \
																			g_psPidSystem->RollGyro.controlOutput  - \
																			g_psPidSystem->PitchGyro.controlOutput - \
																			g_psPidSystem->YawGyro.controlOutput);

					g_psControlAircraft->CurMotorPwmOutput.channle3 = (u16)(g_psControlAircraft->throttleOutput    + \
																			g_psPidSystem->RollGyro.controlOutput  + \
																			g_psPidSystem->PitchGyro.controlOutput + \
																			g_psPidSystem->YawGyro.controlOutput);

					g_psControlAircraft->CurMotorPwmOutput.channle4 = (u16)(g_psControlAircraft->throttleOutput    - \
																			g_psPidSystem->RollGyro.controlOutput  - \
																			g_psPidSystem->PitchGyro.controlOutput + \
																			g_psPidSystem->YawGyro.controlOutput);
				}
				else /*小于起飞油门量*/
				{
					g_psControlAircraft->CurMotorPwmOutput.channle1 = (u16)(g_psControlAircraft->throttleOutput);
					g_psControlAircraft->CurMotorPwmOutput.channle2 = (u16)(g_psControlAircraft->throttleOutput);
					g_psControlAircraft->CurMotorPwmOutput.channle3 = (u16)(g_psControlAircraft->throttleOutput);
					g_psControlAircraft->CurMotorPwmOutput.channle4 = (u16)(g_psControlAircraft->throttleOutput);				
				
					/*飞行器未起飞,处于着陆状态,清除PID控制系统积分项*/
					pid_Horizontal_Takeoff_Integrate_Reset();
				}

				/*四路PWM总输出限幅*/
				g_psControlAircraft->CurMotorPwmOutput.channle1 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle1, \
																					  2000, CTRL_IDEL_THROTTLE_VALUE);

			    g_psControlAircraft->CurMotorPwmOutput.channle2 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle2, \
																					  2000, CTRL_IDEL_THROTTLE_VALUE);

				g_psControlAircraft->CurMotorPwmOutput.channle3 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle3, \
																					  2000, CTRL_IDEL_THROTTLE_VALUE);

				g_psControlAircraft->CurMotorPwmOutput.channle4 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle4, \
																					  2000, CTRL_IDEL_THROTTLE_VALUE);	
				
			}
			/*=== 2.竖直方向,传感器定高模式,油门托管 ===*/
			else if (g_sUav_Status.UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_FIX_HEIGHT)	
			{
				/*默认水平姿态环角速度读控制器来源于PID*/
				g_psControlAircraft->CurMotorPwmOutput.channle1 = (u16)(g_psControlAircraft->throttleOutput    - \
																		g_psPidSystem->RollGyro.controlOutput  + \
																		g_psPidSystem->PitchGyro.controlOutput - \
																		g_psPidSystem->YawGyro.controlOutput);

				g_psControlAircraft->CurMotorPwmOutput.channle2 = (u16)(g_psControlAircraft->throttleOutput    + \
																		g_psPidSystem->RollGyro.controlOutput  - \
																		g_psPidSystem->PitchGyro.controlOutput - \
																		g_psPidSystem->YawGyro.controlOutput);

				g_psControlAircraft->CurMotorPwmOutput.channle3 = (u16)(g_psControlAircraft->throttleOutput    + \
																		g_psPidSystem->RollGyro.controlOutput  + \
																		g_psPidSystem->PitchGyro.controlOutput + \
																		g_psPidSystem->YawGyro.controlOutput);
																		
				g_psControlAircraft->CurMotorPwmOutput.channle4 = (u16)(g_psControlAircraft->throttleOutput    - \
																		g_psPidSystem->RollGyro.controlOutput  - \
																		g_psPidSystem->PitchGyro.controlOutput + \
																		g_psPidSystem->YawGyro.controlOutput);				
			}	

			/*四路PWM总输出限幅*/
			g_psControlAircraft->CurMotorPwmOutput.channle1 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle1, \
																				  2000, CTRL_IDEL_THROTTLE_VALUE);

			g_psControlAircraft->CurMotorPwmOutput.channle2 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle2, \
																				  2000, CTRL_IDEL_THROTTLE_VALUE);

		    g_psControlAircraft->CurMotorPwmOutput.channle3 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle3, \
																				  2000, CTRL_IDEL_THROTTLE_VALUE);

			g_psControlAircraft->CurMotorPwmOutput.channle4 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle4, \
																				  2000, CTRL_IDEL_THROTTLE_VALUE);				
		}
	}
	/*=== 检测飞行器处于锁定状态 ===*/
	else if (g_sUav_Status.LOCK_STATUS == UAV_LOCK_YES)
	{
		/*四个电机PWM值为停转值*/
		g_psControlAircraft->CurMotorPwmOutput.channle1 = REMOT_THROTTLE_BASE_VALUE;
		g_psControlAircraft->CurMotorPwmOutput.channle2 = REMOT_THROTTLE_BASE_VALUE;
		g_psControlAircraft->CurMotorPwmOutput.channle3 = REMOT_THROTTLE_BASE_VALUE;
		g_psControlAircraft->CurMotorPwmOutput.channle4 = REMOT_THROTTLE_BASE_VALUE;
		
		/*飞行器处于着陆状态,清除PID控制系统积分项*/
		pid_Horizontal_Takeoff_Integrate_Reset();
		pid_Vertical_Ctrl_Integrate_Reset();
	}
	
	/*电机PWM输出更迭*/
	g_psControlAircraft->LastMotorPwmOutput.channle1 = g_psControlAircraft->CurMotorPwmOutput.channle1;
	g_psControlAircraft->LastMotorPwmOutput.channle2 = g_psControlAircraft->CurMotorPwmOutput.channle2;
	g_psControlAircraft->LastMotorPwmOutput.channle3 = g_psControlAircraft->CurMotorPwmOutput.channle3;
	g_psControlAircraft->LastMotorPwmOutput.channle4 = g_psControlAircraft->CurMotorPwmOutput.channle4;	
	
	/*四路PWM总输出限幅*/
	g_psControlAircraft->CurMotorPwmOutput.channle1 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle1, \
																		  2000, ESC_MIN_PULSE_ZERO_SPEED_VALUE);

	g_psControlAircraft->CurMotorPwmOutput.channle2 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle2, \
																		  2000, ESC_MIN_PULSE_ZERO_SPEED_VALUE);

	g_psControlAircraft->CurMotorPwmOutput.channle3 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle3, \
																		  2000, ESC_MIN_PULSE_ZERO_SPEED_VALUE);

	g_psControlAircraft->CurMotorPwmOutput.channle4 = (u16)math_Constrain(g_psControlAircraft->CurMotorPwmOutput.channle4, \
																		  2000, ESC_MIN_PULSE_ZERO_SPEED_VALUE);	
	
	/*控制参数必须读取成功,才允许电机转动,固定机架调试时不用*/
	#if (CONTROL_SYS__ONLY_PID == SYS_ENABLE)		/*PID算法*/
	if (g_psCtrlSysStatus->pid == CTRL_SYSTEM_PARA_INIT_SUCC)
	{
	#elif (CONTROL_SYS__ONLY_ADRC == SYS_ENABLE)	/*ADRC算法*/
	if (g_psCtrlSysStatus->adrc == CTRL_SYSTEM_PARA_INIT_SUCC)
	{
	#elif (CONTROL_SYS__PID_ADRC == SYS_ENABLE)		/*PID + ADRC算法*/	
	if ((g_psCtrlSysStatus->pid == CTRL_SYSTEM_PARA_INIT_SUCC) && \
		(g_psCtrlSysStatus->adrc == CTRL_SYSTEM_PARA_INIT_SUCC))
	{		
	#endif
	
		/*翻机（炸机）保护*/
		if ((math_Abs(g_psAttitudeAll->Ahrs.pitch) <= CTRL_HORIZONTAL_ANGLE_SAFE_MAX) && \
			(math_Abs(g_psAttitudeAll->Ahrs.roll) <= CTRL_HORIZONTAL_ANGLE_SAFE_MAX)) /*安全角度范围内,正常控制*/
		{
			msp_TimPwmOut_SetPluse(&g_sTimPwmOut_Motor, MSP_TIM_CH4, \
								   g_psControlAircraft->CurMotorPwmOutput.channle1);
		
			msp_TimPwmOut_SetPluse(&g_sTimPwmOut_Motor, MSP_TIM_CH3, \
								   g_psControlAircraft->CurMotorPwmOutput.channle2);
		
			msp_TimPwmOut_SetPluse(&g_sTimPwmOut_Motor, MSP_TIM_CH2, \
								   g_psControlAircraft->CurMotorPwmOutput.channle3);
		
			msp_TimPwmOut_SetPluse(&g_sTimPwmOut_Motor, MSP_TIM_CH1, \
								   g_psControlAircraft->CurMotorPwmOutput.channle4);
		}
		else /*非安全角度范围内,关闭电机,并锁定*/
		{
			/*关闭电机*/
			msp_TimPwmOut_SetPluse(&g_sTimPwmOut_Motor, MSP_TIM_CH4, \
		   	    			       ESC_MIN_PULSE_ZERO_SPEED_VALUE);
		
			msp_TimPwmOut_SetPluse(&g_sTimPwmOut_Motor, MSP_TIM_CH3, \
			  				       ESC_MIN_PULSE_ZERO_SPEED_VALUE);
		
		    msp_TimPwmOut_SetPluse(&g_sTimPwmOut_Motor, MSP_TIM_CH2, \
							       ESC_MIN_PULSE_ZERO_SPEED_VALUE);
		
		    msp_TimPwmOut_SetPluse(&g_sTimPwmOut_Motor, MSP_TIM_CH1, \
							       ESC_MIN_PULSE_ZERO_SPEED_VALUE);
		
		   /*锁定状态*/
		   g_sUav_Status.LOCK_STATUS = UAV_LOCK_YES;
		}		
	}
}
