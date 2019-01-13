#include "control_Mission.h"

/*喂任务检测狗*/
void feed_control_fly_mission_check_dog(u8 _10msFoc, SimulateWatchDog *mission_check_dog)
{
	u32 feedCheckDogTimeX10Ms = 0;
	
	/*遥控最大允许操作间隔:5000ms*/
	if (_10msFoc > 250)
	{
		_10msFoc = 250;
	}
	
	/*x X 10ms*/
	feedCheckDogTimeX10Ms = _10msFoc * 10;
	mission_check_dog->nextProcessTicks = my_GetTick();	     	  /*获取当前tick, sys_GetTick 10ms获取一次*/
	mission_check_dog->nextProcessTicks += feedCheckDogTimeX10Ms; /*延时*/	
}

/*获取任务检测狗存活状态*/
SIM_WATCH_DOG_STATUS get_control_fly_mission_dog_status(SimulateWatchDog *mission_check_dog)
{
	/*获取当前tick _10ms*/
	mission_check_dog->curTicks = my_GetTick();
	
	/*如果curTicks < nextProcessTicks,说明时间未到*/
	if (mission_check_dog->curTicks < mission_check_dog->nextProcessTicks)
	{
		return SIM_WATCH_DOG_ALIVE;
	}
	else
	{
		return SIM_WATCH_DOG_DIED;
	}
}

/*=== 设置和获取任务 ===*/
/*任务状态机初始化*/
void control_fly_mission_machine_reset(Uav_Status *uavStatus, UAV_FLY_MISSION TARG_MISSION)
{
	switch(TARG_MISSION)
	{
		/*一键起飞*/
		case UAV_FLY_MISSION_ONEKEY_FLY:
		{
			/*目标未设置*/
			uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.TARG_SET_STATUS   = UAV_MISSION_TARG_SET_NO;
			
			/*到达状态:否*/
			uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.TARG_REACH_STATUS = UAV_MISSION_TARG_REACH_NO;
			
			/*标记任务未执行*/
			uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.EXECUTE_STATUS = UAV_MISSION_EXECUTE_NOT;
		}break;
		
		/*一键返航/降落*/
		case UAV_FLY_MISSION_ONEKEY_LAND_HOME:
		{
			/*目标未设置*/
			uavStatus->UavCurrentFlyMission.Onekey_Mission.LandHome.TARG_SET_STATUS   = UAV_MISSION_TARG_SET_NO;
			
			/*到达状态:否*/
			uavStatus->UavCurrentFlyMission.Onekey_Mission.LandHome.TARG_REACH_STATUS = UAV_MISSION_TARG_REACH_NO;	

			/*标记任务未执行*/
			uavStatus->UavCurrentFlyMission.Onekey_Mission.LandHome.EXECUTE_STATUS = UAV_MISSION_EXECUTE_NOT;			
		}break;		
		
		/*光流追点*/
		case UAV_FLY_MISSION_OPFLOW_FOLLOW_POINT:
		{
		
		}break;

		/*光流巡线*/
		case UAV_FLY_MISSION_OPFLOW_FOLLOW_LINE:
		{
		
		}break;

		/*GPS写入当前点位*/
		case UAV_FLY_MISSION_GPS_WRITE_POS:
		{
		
		}break;	

		/*GPS巡天*/
		case UAV_FLY_MISSION_GPS_PATROL_SKY:
		{
		
		}break;		

		default:break;
	}
}

/*清除任务*/
void control_fly_mission_clear(Uav_Status *uavStatus, UAV_FLY_MISSION TARG_MISSION)
{
	/*复位当前任务的状态机*/
	control_fly_mission_machine_reset(uavStatus, TARG_MISSION);
	
	/*恢复任务对系统的改变*/
	switch(TARG_MISSION)
	{
		/*一键起飞*/
		case UAV_FLY_MISSION_ONEKEY_FLY:
		{
			g_psPidSystem->HighAcc.PidScale.kP      = 1.0f;
			g_psPidSystem->HighAcc.PidScale.kI      = 1.0f;		
			g_psPidSystem->HighSpeed.PidScale.kP    = 1.0f;
			g_psPidSystem->HighSpeed.PidScale.kI    = 1.0f;
			g_psPidSystem->HighPosition.PidScale.kP = 1.0f;
			g_psPidSystem->HighPosition.PidScale.kI = 1.0f;
		}break;
		
		/*一键返航/降落*/
		case UAV_FLY_MISSION_ONEKEY_LAND_HOME:
		{
			
		}break;		
		
		/*光流追点*/
		case UAV_FLY_MISSION_OPFLOW_FOLLOW_POINT:
		{
		
		}break;

		/*光流巡线*/
		case UAV_FLY_MISSION_OPFLOW_FOLLOW_LINE:
		{
		
		}break;

		/*GPS写入当前点位*/
		case UAV_FLY_MISSION_GPS_WRITE_POS:
		{
		
		}break;	

		/*GPS巡天*/
		case UAV_FLY_MISSION_GPS_PATROL_SKY:
		{
		
		}break;		

		default:break;
	}	
	
	/*标记当前任务为NULL*/
	uavStatus->UavCurrentFlyMission.CURRENT_FLY_MISSION = UAV_FLY_MISSION_NULL;
}

/*清除上一个任务,设置当前任务*/
void control_fly_mission_set(Uav_Status *uavStatus, UAV_FLY_MISSION SET_MISSION, UAV_MISSION_ATTITUDE_CTRL_STATUS ATTITUDE_CTRL_STATUS)
{
	/*清除上个任务的状态*/
	control_fly_mission_clear(uavStatus, uavStatus->UavCurrentFlyMission.LAST_FLY_MISSION);	
	
	/*设置当前任务*/
	uavStatus->UavCurrentFlyMission.CURRENT_FLY_MISSION = SET_MISSION;
	
	/*设置是否允许同时执行姿态控制*/
	uavStatus->UavCurrentFlyMission.ATTITUDE_CTRL_STATUS = ATTITUDE_CTRL_STATUS;
	
	/*该任务状态初始化*/
	control_fly_mission_machine_reset(uavStatus, SET_MISSION);
}

/*获取当前任务*/
UAV_FLY_MISSION control_fly_mission_get(void)
{
	return (g_psUav_Status->UavCurrentFlyMission.CURRENT_FLY_MISSION);
}

/*=== 常规任务 ===*/
/*一键起飞*/
void control_fly_mission_onekey_fly(Uav_Status *uavStatus)
{
	/*竖直定高传感器有效,且高度目标点未设置*/
	if ((uavStatus->UavSenmodStatus.Vertical.CURRENT_USE != UAV_VERTICAL_SENMOD_CURRENT_NULL) && \
	    (uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.TARG_SET_STATUS == UAV_MISSION_TARG_SET_NO))
	{
		/*设置一键起飞期望悬停高度*/
		g_psPidSystem->HighPosition.expect = g_psSinsReal->curPosition[EARTH_FRAME_Z] + CTRL_HEIGHT_ONEKEY_TAKEOFF_HEIGHT;
		
		/*标记一键任务参考点已设置*/
		uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.TARG_SET_STATUS = UAV_MISSION_TARG_SET_OK;
		
		/*飞行模式标记为定高*/
		uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_FIX_HEIGHT; /*= 2->定高*/
		
		/*竖直控制模式为:定高控制模式*/
		uavStatus->UavControlMode.Vertical.CONTROL_MODE = UAV_VERTICAL_CONTROL_FIX_HEIGHT;
	}
	/*切定高起飞时:竖直定高传感器无效,且高度目标点未设置*/
	else if ((uavStatus->UavSenmodStatus.Vertical.CURRENT_USE == UAV_VERTICAL_SENMOD_CURRENT_NULL) && \
			 (uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.TARG_SET_STATUS == UAV_MISSION_TARG_SET_NO))
	{
		/*飞行模式标记为姿态模式*/
		uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/			
			
		/*竖直控制模式为:自稳控制模式*/
		uavStatus->UavControlMode.Vertical.CONTROL_MODE = UAV_VERTICAL_CONTROL_SELFAUTO;	

		/*一键起飞任务失败,清除任务*/
		control_fly_mission_clear(uavStatus, UAV_FLY_MISSION_ONEKEY_FLY);
	}
	/*切定高起飞状态下:竖直定高传感器无效,且已设置高度目标点(传感器故障)*/
	else if ((uavStatus->UavSenmodStatus.Vertical.CURRENT_USE == UAV_VERTICAL_SENMOD_CURRENT_NULL) && \
			 (uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.TARG_SET_STATUS == UAV_MISSION_TARG_SET_OK))
	{
		/*飞行模式标记为姿态模式*/
		uavStatus->UavFlyType.CURRENT = UAV_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/				
			
		/*竖直控制模式为:自稳控制模式*/
		uavStatus->UavControlMode.Vertical.CONTROL_MODE = UAV_VERTICAL_CONTROL_SELFAUTO;
		
		/*一键起飞任务失败,清除任务*/
		control_fly_mission_clear(uavStatus, UAV_FLY_MISSION_ONEKEY_FLY);
	}
	
	/*控制模式为定高控制(处于定高起飞任务) & 目标点已设置 & 且未到达目标点*/
	if ((uavStatus->UavFlyType.CURRENT == UAV_FLY_TYPE_FIX_HEIGHT) && \
		(uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.TARG_SET_STATUS == UAV_MISSION_TARG_SET_OK) && \
		(uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.TARG_REACH_STATUS == UAV_MISSION_TARG_REACH_NO))
	{
		/*临时设置竖直控制的PID参数,实现快速起飞*/
		g_psPidSystem->HighAcc.PidScale.kP      = 1.0f;
		g_psPidSystem->HighAcc.PidScale.kI      = 1.5f;		
		g_psPidSystem->HighSpeed.PidScale.kP    = 1.2f;
		g_psPidSystem->HighSpeed.PidScale.kI    = 1.2f;
		g_psPidSystem->HighPosition.PidScale.kP = 1.5f;
		g_psPidSystem->HighPosition.PidScale.kI = 1.0f;		
		
		/*当前高度大于期望高度,将参数恢复正常*/
		if (g_psPidSystem->HighPosition.expect <= g_psSinsReal->curPosition[EARTH_FRAME_Z])
		{
			/*标记已达目标点*/
			uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.TARG_REACH_STATUS = UAV_MISSION_TARG_REACH_OK;
			
			/*恢复正常控制比例参数*/
			g_psPidSystem->HighAcc.PidScale.kP      = 1.0f;
			g_psPidSystem->HighAcc.PidScale.kI      = 1.0f;		
			g_psPidSystem->HighSpeed.PidScale.kP    = 1.0f;
			g_psPidSystem->HighSpeed.PidScale.kI    = 1.0f;
			g_psPidSystem->HighPosition.PidScale.kP = 1.0f;
			g_psPidSystem->HighPosition.PidScale.kI = 1.0f;				
			
			/*清除该任务*/
			control_fly_mission_clear(uavStatus, UAV_FLY_MISSION_ONEKEY_FLY);
		}
	}
}

/*一键/失联返航/降落*/
void control_fly_mission_gohome(void)
{

}

/*=== 需要超声波+光流定点的任务 ===*/
Control_Fly_Mission_Switch_Motion gs_mission_check_ultr_opflow = 
{
	.HIGH_EDGE_AVA_STATUS = REMOT_SWITCH_EDGE_AVA_NO,
	.LOW_EDGE_AVA_STATUS  = REMOT_SWITCH_EDGE_AVA_NO,
	.COUNTDOWN_STATUS     = CONTROL_MISSION_COUNTDOWN_FINISH,
	.CHECK_STATUS         = CONTROL_MISSION_START_CHECK_DISABLE, /*默认失能检测*/
	.edge_cnt     	 	  = 0,
	.count_down      	  = {0},
};

/*任务检测*/
UAV_FLY_MISSION control_fly_mission_check_ultr_opflow_pos(Uav_Status *uavStatus, u32 checkPeriodMS)
{
	/*若任务切换异常,默认返回当前正在执行的任务*/
	UAV_FLY_MISSION FLY_MISSION_RET = uavStatus->UavCurrentFlyMission.LAST_FLY_MISSION;
	
	/*检测SWD拨键位置*/
	if (remot_Data_Range(g_sRemotData.SWD, REMOT_DATA_MIN) == REMOT_DATA_MIN)
	{
		if (gs_mission_check_ultr_opflow.CHECK_STATUS == CONTROL_MISSION_START_CHECK_ENABLE)
		{
			gs_mission_check_ultr_opflow.LOW_EDGE_AVA_STATUS = REMOT_SWITCH_EDGE_AVA_OK;
		}
	}
	else if (remot_Data_Range(g_sRemotData.SWD, REMOT_DATA_MAX) == REMOT_DATA_MAX)
	{	
		/*判断是否是新一轮检测*/
		if (gs_mission_check_ultr_opflow.COUNTDOWN_STATUS == CONTROL_MISSION_COUNTDOWN_FINISH)
		{
			/*使能检测*/
			gs_mission_check_ultr_opflow.CHECK_STATUS = CONTROL_MISSION_START_CHECK_ENABLE;
			
			/*开始倒计时250 * 20ms = 5000ms*/
			feed_control_fly_mission_check_dog(checkPeriodMS / 20 / 10, &gs_mission_check_ultr_opflow.count_down);
			
			/*标记已开始倒计时*/
			gs_mission_check_ultr_opflow.COUNTDOWN_STATUS = CONTROL_MISSION_COUNTDOWN_START;
		}
		
		if (gs_mission_check_ultr_opflow.CHECK_STATUS == CONTROL_MISSION_START_CHECK_ENABLE)
		{
			gs_mission_check_ultr_opflow.HIGH_EDGE_AVA_STATUS = REMOT_SWITCH_EDGE_AVA_OK;
		}
	}
	
	if ((gs_mission_check_ultr_opflow.LOW_EDGE_AVA_STATUS == REMOT_SWITCH_EDGE_AVA_OK) && \
		(gs_mission_check_ultr_opflow.HIGH_EDGE_AVA_STATUS == REMOT_SWITCH_EDGE_AVA_OK))
	{
		/*拨动次数++*/
		gs_mission_check_ultr_opflow.edge_cnt++;
		
		gs_mission_check_ultr_opflow.LOW_EDGE_AVA_STATUS  = REMOT_SWITCH_EDGE_AVA_NO;
		gs_mission_check_ultr_opflow.HIGH_EDGE_AVA_STATUS = REMOT_SWITCH_EDGE_AVA_NO;		
	}
	
	/*处于倒计时状态,且倒计时已到时间*/
	if ((get_control_fly_mission_dog_status(&gs_mission_check_ultr_opflow.count_down) == SIM_WATCH_DOG_DIED) && \
		(gs_mission_check_ultr_opflow.COUNTDOWN_STATUS == CONTROL_MISSION_COUNTDOWN_START))
	{
		/*根据拨键次数来决定飞行任务*/
		if (gs_mission_check_ultr_opflow.edge_cnt == MISSION_OPFLOW_FOLLOW_POINT)
		{
			FLY_MISSION_RET = UAV_FLY_MISSION_OPFLOW_FOLLOW_POINT;	/*光流追点*/
		}
		else if (gs_mission_check_ultr_opflow.edge_cnt == MISSION_OPFLOW_FOLLOW_LINE)
		{
			FLY_MISSION_RET = UAV_FLY_MISSION_OPFLOW_FOLLOW_LINE;	/*光流循线*/
		}
		
		/*本轮倒计时结束*/
		gs_mission_check_ultr_opflow.COUNTDOWN_STATUS = CONTROL_MISSION_COUNTDOWN_FINISH;
		
		/*失能检测*/
		gs_mission_check_ultr_opflow.CHECK_STATUS = CONTROL_MISSION_START_CHECK_DISABLE;
		
		/*cnt归0*/
		gs_mission_check_ultr_opflow.edge_cnt = 0;
	}
	
	return FLY_MISSION_RET;
}

/*光流追点*/
void control_fly_mission_ultr_opflow_follow_point(void)
{
	
}

/*光流循线*/
void control_fly_mission_ultr_opflow_follow_line(void)
{

}

/*=== 需要GPS+气压计定点的任务 ===*/
Control_Fly_Mission_Switch_Motion gs_mission_check_bero_gps = 
{
	.HIGH_EDGE_AVA_STATUS = REMOT_SWITCH_EDGE_AVA_NO,
	.LOW_EDGE_AVA_STATUS  = REMOT_SWITCH_EDGE_AVA_NO,
	.COUNTDOWN_STATUS     = CONTROL_MISSION_COUNTDOWN_FINISH,
	.CHECK_STATUS         = CONTROL_MISSION_START_CHECK_DISABLE, /*默认失能检测*/
	.edge_cnt     	 	  = 0,
	.count_down      	  = {0},
};

/*任务检测*/
UAV_FLY_MISSION control_fly_mission_check_bero_gps_pos(Uav_Status *uavStatus, u32 checkPeriodMS)
{
	/*若任务切换异常,默认返回当前正在执行的任务*/	
	UAV_FLY_MISSION FLY_MISSION_RET = uavStatus->UavCurrentFlyMission.LAST_FLY_MISSION;
	
	/*检测SWD拨键位置*/
	if (remot_Data_Range(g_sRemotData.SWD, REMOT_DATA_MIN) == REMOT_DATA_MIN)
	{
		if (gs_mission_check_bero_gps.CHECK_STATUS == CONTROL_MISSION_START_CHECK_ENABLE)
		{
			gs_mission_check_bero_gps.LOW_EDGE_AVA_STATUS = REMOT_SWITCH_EDGE_AVA_OK;
		}
	}
	else if (remot_Data_Range(g_sRemotData.SWD, REMOT_DATA_MAX) == REMOT_DATA_MAX)
	{	
		/*判断是否是新一轮检测*/
		if (gs_mission_check_bero_gps.COUNTDOWN_STATUS == CONTROL_MISSION_COUNTDOWN_FINISH)
		{
			/*使能检测*/
			gs_mission_check_bero_gps.CHECK_STATUS = CONTROL_MISSION_START_CHECK_ENABLE;
			
			/*开始倒计时250 * 20ms = 5000ms*/
			feed_control_fly_mission_check_dog(checkPeriodMS / 20 / 10, &gs_mission_check_ultr_opflow.count_down);
			
			/*标记已开始倒计时*/
			gs_mission_check_bero_gps.COUNTDOWN_STATUS = CONTROL_MISSION_COUNTDOWN_START;
		}
		
		if (gs_mission_check_bero_gps.CHECK_STATUS == CONTROL_MISSION_START_CHECK_ENABLE)
		{
			gs_mission_check_bero_gps.HIGH_EDGE_AVA_STATUS = REMOT_SWITCH_EDGE_AVA_OK;
		}
	}
	
	if ((gs_mission_check_bero_gps.LOW_EDGE_AVA_STATUS == REMOT_SWITCH_EDGE_AVA_OK) && \
		(gs_mission_check_bero_gps.HIGH_EDGE_AVA_STATUS == REMOT_SWITCH_EDGE_AVA_OK))
	{
		/*拨动次数++*/
		gs_mission_check_bero_gps.edge_cnt++;
		
		gs_mission_check_bero_gps.LOW_EDGE_AVA_STATUS  = REMOT_SWITCH_EDGE_AVA_NO;
		gs_mission_check_bero_gps.HIGH_EDGE_AVA_STATUS = REMOT_SWITCH_EDGE_AVA_NO;		
	}
	
	/*处于倒计时状态,且倒计时已到时间*/
	if ((get_control_fly_mission_dog_status(&gs_mission_check_bero_gps.count_down) == SIM_WATCH_DOG_DIED) && \
		(gs_mission_check_bero_gps.COUNTDOWN_STATUS == CONTROL_MISSION_COUNTDOWN_START))
	{
		/*根据拨键次数来决定飞行任务*/
		if (gs_mission_check_bero_gps.edge_cnt == MISSION_OPFLOW_FOLLOW_POINT)
		{
			FLY_MISSION_RET = UAV_FLY_MISSION_GPS_WRITE_POS;	/*GPS写入当前座标*/
		}
		else if (gs_mission_check_bero_gps.edge_cnt == MISSION_OPFLOW_FOLLOW_LINE)
		{
			FLY_MISSION_RET = UAV_FLY_MISSION_GPS_PATROL_SKY;	/*按写入的GPS座标进行巡天*/
		}
		
		/*本轮倒计时结束*/
		gs_mission_check_bero_gps.COUNTDOWN_STATUS = CONTROL_MISSION_COUNTDOWN_FINISH;
		
		/*失能检测*/
		gs_mission_check_bero_gps.CHECK_STATUS = CONTROL_MISSION_START_CHECK_DISABLE;
		
		/*cnt归0*/
		gs_mission_check_bero_gps.edge_cnt = 0;
	}
	
	return FLY_MISSION_RET;
}

/*GPS记录当前座标*/
void control_fly_mission_bero_gps_write_pos(Uav_Status *uavStatus)
{
	/*判断是否完成写入*/
	
	/*若GPS点位写入完成,则清除上一个任务,清除任务*/
	control_fly_mission_set(uavStatus, UAV_FLY_MISSION_NULL, UAV_MISSION_ATTITUDE_CTRL_ENABLE);
}

/*GPS巡天*/
void control_fly_mission_bero_gps_patrol_sky(Uav_Status *uavStatus)
{
	/**/
}

/*=== 需要OPENMV的任务 ===*/
