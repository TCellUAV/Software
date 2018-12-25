#include "control_Aircraft.h"

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
	.GO_HOME_STATUS     = CTRL_AIRCRAFT_GO_HOME_DISABLE,	/*上电后不允许自动返航*/
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
		
		ctrl_Control_Mode_Select(&g_sAircraftStatus);
	}
	
	/************* 1.主导控制器: 定高+输出水平期望角 *************/
	ctrl_MainLeading_Control_Dp();
	
	/************* 2.姿态控制器: 角度外环+角速度内环 *************/
	ctrl_Attitude_Control_Dp();
}

/*高度,水平控制模式选择*/
void ctrl_Control_Mode_Select(AircraftStatus *aircraftStatus)
{
	/*上次遥控期望高度控制模式*/
	aircraftStatus->LAST_HEIGHT_CONTROL_MODE = aircraftStatus->CUR_HEIGHT_CONTROL_MODE;

	/*上次遥控期望水平位置控制模式*/
	aircraftStatus->LAST_HORIZONTAL_CONTROL_MODE = aircraftStatus->CUR_HORIZONTAL_CONTROL_MODE;
	
	/*============ 判断遥控器模式选择对应拨键的值,来进行模式切换 ============*/
	/*SWITCH A 默认上拨为纯姿态自稳,下拨为定高*/
	if (remot_Data_Range(g_sRemotData.SWA, REMOT_DATA_MIN) == REMOT_DATA_MIN)
	{
		aircraftStatus->CUR_HEIGHT_CONTROL_MODE = AIRCRAFT_CONTROL_AUTO;	/*竖直姿态自稳模式模式*/		
	}
	else if (remot_Data_Range(g_sRemotData.SWA, REMOT_DATA_MAX) == REMOT_DATA_MAX)
	{
		aircraftStatus->CUR_HEIGHT_CONTROL_MODE = AIRCRAFT_CONTROL_SENSOR;	/*传感器定高模式*/
	}

	/*SWITCH B 默认上拨为纯姿态自稳,下拨为水平定点*/
	if (remot_Data_Range(g_sRemotData.SWB, REMOT_DATA_MIN) == REMOT_DATA_MIN)
	{
		aircraftStatus->CUR_HORIZONTAL_CONTROL_MODE = AIRCRAFT_CONTROL_AUTO;	/*水平姿态自稳模式模式*/	
	}
	else if (remot_Data_Range(g_sRemotData.SWB, REMOT_DATA_MAX) == REMOT_DATA_MAX)
	{
		aircraftStatus->CUR_HORIZONTAL_CONTROL_MODE = AIRCRAFT_CONTROL_SENSOR;  /*传感器水平定点模式*/
	}
	
	/*============ 定高&定点遥控切换期望 ============*/
	/*判断高度和水平控制模式是否切换*/
	/*高度:本次控制模式和上次控制模式不同*/
	if (aircraftStatus->CUR_HEIGHT_CONTROL_MODE != aircraftStatus->LAST_HEIGHT_CONTROL_MODE)
	{
		/*自稳切定高*/
		if (aircraftStatus->CUR_HEIGHT_CONTROL_MODE == AIRCRAFT_CONTROL_SENSOR)
		{
			/*标记本次遥控期望为自稳切定高*/
			aircraftStatus->HEIGHT_REMOT_EXPECT_CONTROL_MODE = AIRCRAFT_CONTROL_AUTO_TO_SENSOR;
			
			/*标记零参考点未设置(切换后需要重新记录当前高度)*/
			aircraftStatus->HEIGHT_REFERENCE_SET_STATUS = AIRCRAFT_REFERENCE_SET_NO;
		}
		/*定高切自稳*/
		else if (aircraftStatus->CUR_HEIGHT_CONTROL_MODE == AIRCRAFT_CONTROL_AUTO)
		{
			/*标记本次遥控期望为定高切自稳*/
			aircraftStatus->HEIGHT_REMOT_EXPECT_CONTROL_MODE = AIRCRAFT_CONTROL_SENSOR_TO_AUTO;
		}
	}

	/*水平:本次控制模式和上次控制模式不同*/
	if (aircraftStatus->CUR_HORIZONTAL_CONTROL_MODE != aircraftStatus->LAST_HORIZONTAL_CONTROL_MODE) 
	{
		/*自稳切水平定点*/
		if (aircraftStatus->CUR_HORIZONTAL_CONTROL_MODE == AIRCRAFT_CONTROL_SENSOR)
		{
			/*标记本次为自稳切水平定点*/
			aircraftStatus->HORIZONTAL_REMOT_EXPECT_CONTROL_MODE = AIRCRAFT_CONTROL_AUTO_TO_SENSOR;
			
			/*标记零参考点未设置(切换后需要重新记录当前水平位置)*/
			aircraftStatus->HORIZONTAL_REFERENCE_SET_STATUS = AIRCRAFT_REFERENCE_SET_NO;
		}
		/*水平定点切自稳*/
		else if (aircraftStatus->CUR_HORIZONTAL_CONTROL_MODE == AIRCRAFT_CONTROL_AUTO)
		{
			/*标记本次为定高切自稳*/
			aircraftStatus->HORIZONTAL_REMOT_EXPECT_CONTROL_MODE = AIRCRAFT_CONTROL_SENSOR_TO_AUTO;						
		}
	}

	/*飞行器未失联,才接受遥控改变当前飞行状态*/
	if (aircraftStatus->CMC_STATUS == AIRCRAFT_CMC_SUCC)
	{
		/*============ 定高条件判断及控制模式切换 ============*/
		/*竖直自稳切定高*/
		if (aircraftStatus->HEIGHT_REMOT_EXPECT_CONTROL_MODE == AIRCRAFT_CONTROL_AUTO_TO_SENSOR)
		{
			/*只要气压计有效就可定高(超声波有效时,低高度数据来源超声波,否则数据来源气压计)*/
			if ((aircraftStatus->BERO_ESTIMATE_ALTITUDE == HEIGHT_DATA_STATUS_OK) && \
				(aircraftStatus->HEIGHT_REFERENCE_SET_STATUS == AIRCRAFT_REFERENCE_SET_NO))
			{
				/*每次切换只保留一次当前油门值*/
				g_psControlAircraft->heightHoldThrottle = g_psControlAircraft->RemotExpectAngle.throttle;
		
				/*将Switch_SWA开关向下拨动瞬间的惯导竖直位置估计作为目标高度*/
				g_psPidSystem->HighPosition.expect = g_psSinsReal->curPosition[EARTH_FRAME_Z];
		
				/*设置成功后 标记零参考点已设置(防止定高后,重新更新期望高度)*/
				aircraftStatus->HEIGHT_REFERENCE_SET_STATUS = AIRCRAFT_REFERENCE_SET_OK;

				/*飞行模式标记为定高*/
				aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_FIX_HEIGHT;	/*= 2->定高*/
		
				/*竖直控制模式为:定高控制模式*/
				aircraftStatus->HEIGHT_CONTROL_MODE = AIRCRAFT_CONTROL_SENSOR;
			}
			/*切定高时:气压计无效,且未设置点位*/
			else if ((aircraftStatus->BERO_ESTIMATE_ALTITUDE == HEIGHT_DATA_STATUS_NO) && \
					 (aircraftStatus->HEIGHT_REFERENCE_SET_STATUS == AIRCRAFT_REFERENCE_SET_NO))
			{
				/*飞行模式标记为姿态模式*/
				aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/			
			
				/*竖直控制模式为:自稳控制模式*/
				aircraftStatus->HEIGHT_CONTROL_MODE = AIRCRAFT_CONTROL_AUTO;		
			}
			/*定高状态下:气压计无效,且已设置点位(传感器故障)*/
			else if ((aircraftStatus->BERO_ESTIMATE_ALTITUDE == HEIGHT_DATA_STATUS_NO) && \
					 (aircraftStatus->HEIGHT_REFERENCE_SET_STATUS == AIRCRAFT_REFERENCE_SET_OK))
			{
				/*飞行模式标记为姿态模式*/
				aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/

				/*退出定高后,标记参考点未设置,这样会一直check*/
				aircraftStatus->HEIGHT_REFERENCE_SET_STATUS = AIRCRAFT_REFERENCE_SET_NO;				
			
				/*竖直控制模式为:自稳控制模式*/
				aircraftStatus->HEIGHT_CONTROL_MODE = AIRCRAFT_CONTROL_AUTO;				
			}
		}
		else if (aircraftStatus->HEIGHT_REMOT_EXPECT_CONTROL_MODE == AIRCRAFT_CONTROL_SENSOR_TO_AUTO)	/*定高切自稳*/
		{
			/*飞行模式标记为姿态*/
			aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/	
		
			/*竖直控制模式为:自稳控制模式*/
			aircraftStatus->HEIGHT_CONTROL_MODE = AIRCRAFT_CONTROL_AUTO;			
		}
		
		/*============ 定点条件判断及控制模式切换 ============*/
		/*水平自稳切定点,水平数据来源: 光流 / GPS*/
		if (aircraftStatus->HORIZONTAL_REMOT_EXPECT_CONTROL_MODE == AIRCRAFT_CONTROL_AUTO_TO_SENSOR)
		{
			/*(GPS数据OK && HOME已设置 && 融合成功) && 参考点未设置*/
			if((status_GPS_Fix_Ava_Check(&g_sAircraftStatus) == GPS_POS_FIX_AVA_TRUE) && \
			   (aircraftStatus->HORIZONTAL_REFERENCE_SET_STATUS == AIRCRAFT_REFERENCE_SET_NO))
			{
				/*将当前惯导水平位置估计作为目标悬停点*/
				g_psPidSystem->LatitudePosition.expect  = g_psSinsReal->curPosition[EARTH_FRAME_Y]; /*N*/
				g_psPidSystem->LongitudePosition.expect = g_psSinsReal->curPosition[EARTH_FRAME_X];	/*E*/
						
				/*PID水平控制积分复位*/
				pid_Horizontal_GPS_Ctrl_Integrate_Reset();
			
				/*设置成功后 标记零参考点已设置(防止定点后,重新更新期望位置)*/
				aircraftStatus->HORIZONTAL_REFERENCE_SET_STATUS = AIRCRAFT_REFERENCE_SET_OK;	
			
				/*飞行模式标记为定点*/
				aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_FIX_POS;	/*= 3->定点*/		
			
				/*水平控制模式为:定点控制模式*/
				aircraftStatus->HORIZONTAL_CONTROL_MODE = AIRCRAFT_CONTROL_GPS_SENSOR;
			}
			/*切定点时刻,不满足条件,自动切回水平自稳模式,并不断check,直到切换回自稳模式或者满足定点条件*/
			else if ((status_GPS_Fix_Ava_Check(&g_sAircraftStatus) == GPS_POS_FIX_AVA_FALSE) && \
					(aircraftStatus->HORIZONTAL_REFERENCE_SET_STATUS == AIRCRAFT_REFERENCE_SET_NO))
			{
				/*飞行模式标记为非定点模式*/
				if (aircraftStatus->CUR_FLY_TYPE == AIRCRAFT_FLY_TYPE_FIX_HEIGHT) /*如果仍处在定高模式,则为定高模式*/
				{
					aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_FIX_HEIGHT;
				}
				else /*如果没处在定高,也没处在定点,则为纯姿态模式*/
				{
					aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/			
				}	
				
				/*PID水平控制积分复位*/
				pid_Horizontal_GPS_Ctrl_Integrate_Reset();				
			
				/*水平控制模式为:自稳控制模式*/
				aircraftStatus->HORIZONTAL_CONTROL_MODE = AIRCRAFT_CONTROL_AUTO;
			}
			/*进入定点模式一段后,卫星信号不满足定点条件,切换回自稳模式(并不断check,直到切换回自稳模式或者满足定点条件),返航功能OK后,应切换成自动返航模式*/
			else if ((status_GPS_Fix_Ava_Check(&g_sAircraftStatus) == GPS_POS_FIX_AVA_FALSE) && \
					(aircraftStatus->HORIZONTAL_REFERENCE_SET_STATUS == AIRCRAFT_REFERENCE_SET_OK))
			{	
				/*飞行模式标记为非定点模式*/
				if (aircraftStatus->CUR_FLY_TYPE == AIRCRAFT_FLY_TYPE_FIX_HEIGHT) /*如果仍处在定高模式,则为定高模式*/
				{
					aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_FIX_HEIGHT;
				}
				else /*如果没处在定高,也没处在定点,则为纯姿态模式*/
				{
					aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/
				}
	
				/*PID水平控制积分复位*/
				pid_Horizontal_GPS_Ctrl_Integrate_Reset();
				
				/*自动退出定点模式 标记零参考点未设置(一直CHECK,直到满足定点模式/手动切自稳模式)*/
				aircraftStatus->HORIZONTAL_REFERENCE_SET_STATUS = AIRCRAFT_REFERENCE_SET_NO;
			
				/*水平控制模式为:自稳控制模式*/
				aircraftStatus->HORIZONTAL_CONTROL_MODE = AIRCRAFT_CONTROL_AUTO;
			}			
		}
		else if(aircraftStatus->HORIZONTAL_REMOT_EXPECT_CONTROL_MODE == AIRCRAFT_CONTROL_SENSOR_TO_AUTO) /*水平定点切自稳*/
		{
			/*飞行模式标记为非定点模式*/
			if (aircraftStatus->CUR_FLY_TYPE == AIRCRAFT_FLY_TYPE_FIX_HEIGHT) /*如果仍处在定高模式,则为定高模式*/
			{
				aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_FIX_HEIGHT;
			}
			else /*如果没处在定高,也没处在定点,则为纯姿态模式*/
			{
				aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_ATTITUDE;	/*= 1->姿态*/			
			}		
			
			/*PID水平控制积分复位*/
			pid_Horizontal_GPS_Ctrl_Integrate_Reset();
			
			/*水平控制模式为:自稳控制模式*/
			aircraftStatus->HORIZONTAL_CONTROL_MODE = AIRCRAFT_CONTROL_AUTO;			
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
			(g_psAircraftStatus->CUR_FLY_STATUS == AIRCRAFT_LANDING))
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
	g_psPidSystem->PitchGyro.feedback = g_psAngleGyro->Pitch;
	g_psPidSystem->RollGyro.feedback  = g_psAngleGyro->Roll;	
	
	/***************内环 角速度 控制****************/
	g_psPidSystem->PitchGyro.controlOutput = pid_Control_Div_LPF(&(g_psPidSystem->PitchGyro), PID_CONTROLER_PITCH_GYRO);  /*PID DIV控制低通滤波*/
	g_psPidSystem->RollGyro.controlOutput  = pid_Control_Div_LPF(&(g_psPidSystem->RollGyro), PID_CONTROLER_ROLL_GYRO);    /*PID DIV控制低通滤波*/
	
	/*============ YAW(偏航角) 内环角速度反馈与控制 ===========*/
	/***************内环 角速度 反馈****************/
	g_psPidSystem->YawGyro.feedback = g_psAngleGyro->Yaw;

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
	    (math_Abs(g_psAngleGyro->Yaw) <= 15.0f)) /*偏航控制输出相对较大,但偏航角速度相对很小*/
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
	
	if (g_YawControlFaultHandleTicks == 400) /*2S执行一次, 特殊处理*/
	{
		/*清空偏航角度控制的积分*/
		pid_Link_Integrate_Reset(&(g_psPidSystem->YawAngle));		
		
		/*清空偏航角速度控制的积分*/
		pid_Link_Integrate_Reset(&(g_psPidSystem->YawGyro)); 
		
		/*偏航角期待更新为当前偏航角*/
		g_psPidSystem->YawAngle.expect = g_psAttitudeAll->Ahrs.yaw;
		
		/*Tick清0*/
		g_YawControlFaultHandleTicks = 0;
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
	
	/*判断是否为失联或遥控一键 返航*/
	if (g_psAircraftStatus->CUR_FLY_TYPE != AIRCRAFT_FLY_TYPE_GO_HOME)
	{
		/*根据遥控器切换档位,飞控进入不同模式*/
		/*================== 1.高度自稳,水平自稳 模式 ==================*/
		if ((g_psAircraftStatus->HEIGHT_CONTROL_MODE == AIRCRAFT_CONTROL_AUTO) && \
			(g_psAircraftStatus->HORIZONTAL_CONTROL_MODE == AIRCRAFT_CONTROL_AUTO))
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
		else if ((g_psAircraftStatus->HEIGHT_CONTROL_MODE == AIRCRAFT_CONTROL_SENSOR) && \
			     (g_psAircraftStatus->HORIZONTAL_CONTROL_MODE == AIRCRAFT_CONTROL_AUTO))
		{
			/*更新水平自稳控制下的ROLL(横滚)和PITCH(俯仰)期望角 (遥控期望角已限制在最大值范围)*/
			g_psPidSystem->PitchAngle.expect = g_psControlAircraft->RemotExpectAutoAngle.pitch; /*遥控期待自稳角pitch*/
			g_psPidSystem->RollAngle.expect  = g_psControlAircraft->RemotExpectAutoAngle.roll;	/*遥控期待自稳角roll*/
			
			/*竖直高度(定高)控制器*/
			vertical_Control_AltHold(mainControlDeltaT);	
		}
		/*================== 3.高度定高,水平定点(GPS/光流) 模式 ==================*/		
		else if ((g_psAircraftStatus->HEIGHT_CONTROL_MODE == AIRCRAFT_CONTROL_SENSOR) && \
				(g_psAircraftStatus->HORIZONTAL_CONTROL_MODE != AIRCRAFT_CONTROL_AUTO))
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
	/*失联或遥控人为遥控一键返航*/
	else if (g_psAircraftStatus->CUR_FLY_TYPE == AIRCRAFT_FLY_TYPE_GO_HOME) 
	{
		/*飞行器自动返航控制*/
		ctrl_Go_Home_Control(mainControlDeltaT);
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
	
	/*着陆条件检测*/
	g_psAircraftStatus->CUR_FLY_STATUS = ctrl_Landing_Check_And_Idling();
	
	/*=== 检测飞行器处于解锁状态 ===*/
	if (g_psAircraftStatus->LOCK_STATUS == AIRCRAFT_UNLOCK)
	{
		/*检测到飞行状态为着陆状态,输出怠速*/
		if (g_psAircraftStatus->CUR_FLY_STATUS == AIRCRAFT_LANDING)
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
			else if (g_psAircraftStatus->LAST_FLY_STATUS == AIRCRAFT_FLYING)
			{
				g_psAircraftStatus->LOCK_STATUS = AIRCRAFT_LOCKING;
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
		else if (g_psAircraftStatus->CUR_FLY_STATUS == AIRCRAFT_FLYING)
		{
			/*=== 1.竖直方向,姿态自稳模式 ===*/
			if (g_psAircraftStatus->HEIGHT_CONTROL_MODE == AIRCRAFT_CONTROL_AUTO)
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
			else if (g_psAircraftStatus->HEIGHT_CONTROL_MODE == AIRCRAFT_CONTROL_SENSOR)	
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
	else if (g_psAircraftStatus->LOCK_STATUS == AIRCRAFT_LOCKING)
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
	#if (CTRL_MOTOR_DRIVER_ON_FIXED_DEBUGGING == SYS_DISABLE)	
	if (g_psCtrlSysStatus->pid == CTRL_SYSTEM_PARA_INIT_SUCC)
	{
	#endif
	#elif (CONTROL_SYS__ONLY_ADRC == SYS_ENABLE)	/*ADRC算法*/
	#if (CTRL_MOTOR_DRIVER_ON_FIXED_DEBUGGING == SYS_DISABLE)		
	if (g_psCtrlSysStatus->adrc == CTRL_SYSTEM_PARA_INIT_SUCC)
	{		
	#endif
	#elif (CONTROL_SYS__PID_ADRC == SYS_ENABLE)		/*PID + ADRC算法*/
	#if (CTRL_MOTOR_DRIVER_ON_FIXED_DEBUGGING == SYS_DISABLE)		
	if ((g_psCtrlSysStatus->pid == CTRL_SYSTEM_PARA_INIT_SUCC) && \
		(g_psCtrlSysStatus->adrc == CTRL_SYSTEM_PARA_INIT_SUCC))
	{		
	#endif
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
		g_sAircraftStatus.LOCK_STATUS = AIRCRAFT_LOCKING;
	}
			
	#if (CTRL_MOTOR_DRIVER_ON_FIXED_DEBUGGING == SYS_DISABLE)			
	}
	#endif
}
