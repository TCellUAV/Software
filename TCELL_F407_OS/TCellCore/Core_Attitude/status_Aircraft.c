#include "status_Aircraft.h"
#include "remot_DataAnaly.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

Uav_Status g_sUav_Status =
{
	/*= 1.飞行器本身的状态 =*/	               
	.LOCK_STATUS = UAV_LOCK_YES,			/*飞控锁定状态*/
	
	.UavLandStatus = 
	{
		.ThisTime = UAV_LAND_YES,
		.LastTime = UAV_LAND_YES,
	},										/*飞行器着陆状态*/
	
	.UavFlyType =
	{
		.CURRENT 			  = UAV_FLY_TYPE_ATTITUDE,
		.BEFORE_WIRELESS_MISS = UAV_FLY_TYPE_ATTITUDE,
	},									    /*飞行模式*/
	
	.AIRSTOP_TYPE = UAV_AIRSTOP_NOT, 		/*空中悬停状态*/
	
	.HOME_SET_STATUS     = UAV_HOME_SET_NOT,		    /*GPS HOME 点设置状态*/
	.WIRELESS_CMC_STATUS = UAV_WIRELESS_CMC_FAIL,	    /*遥控和飞行器通信状态*/
	.HCI_SHOW_STATUS     = UAV_HCI_SHOW_DISABLE,		/*HCI SHOW*/
										   
	/*= 2.飞行器工作方式状态 =*/													  /*当前飞行任务*/
	.UavControlMode = 
	{
		.Vertical = 
		{
			.Mode_Switch = 
			{
				.ThisTime = UAV_VERTICAL_CONTROL_SELFAUTO,
				.LastTime = UAV_VERTICAL_CONTROL_SELFAUTO,
			},		   /*模式切换*/		
			
			.EXPECT_CONTROL_MODE  = UAV_VERTICAL_CONTROL_SELFAUTO,  /*期望控制模式*/			
			.CONTROL_MODE         = UAV_VERTICAL_CONTROL_SELFAUTO,  /*实际控制模式*/			
			.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_NO, 	/*参考点设置状态*/			
		},
		
		.Horizontal = 
		{
			.Mode_Switch = 
			{
				.ThisTime = UAV_HORIZONTAL_CONTROL_SELFAUTO,
				.LastTime = UAV_HORIZONTAL_CONTROL_SELFAUTO,
			},		   /*模式切换*/		
			
			.EXPECT_CONTROL_MODE  = UAV_HORIZONTAL_CONTROL_SELFAUTO,  /*期望控制模式*/			
			.CONTROL_MODE         = UAV_HORIZONTAL_CONTROL_SELFAUTO,  /*实际控制模式*/			
			.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_NO, 	/*参考点设置状态*/			
		},
	},			/*飞行器控制模式*/
	
	.UavSenmodStatus = 
	{
		.Vertical = 
		{
			.WORK_STATUS = UAV_SENMOD_WORK_LIMITLESS, 		 /*默认使用无限制传感器*/
			
			.CURRENT_USE = UAV_VERTICAL_SENMOD_CURRENT_BERO, /*默认气压计*/
			
			.Bero = 
			{
				.EXIST_STATUS              = UAV_SENMOD_EXIST_NO,
				.DATA_STATUS               = UAV_SENMOD_DATA_NO,
				.ZERO_REFERENCE_SET_STATUS = UAV_SENMOD_ZERO_REFERENCE_SET_NO,
				.FIRST_USE_AVA_STATUS      = UAV_SENMOD_FIRST_USE_AVA_NO,
				.USE_CONTROL_STATUS        = UAV_SENMOD_USE_CONTROL_DISALLOW,
				.FUSION_STATUS 			   = UAV_SENMOD_FUSION_FINISH,					
			},
			
			.Ultr = 
			{
				.EXIST_STATUS              = UAV_SENMOD_EXIST_NO,
				.DATA_STATUS               = UAV_SENMOD_DATA_NO,
				.ZERO_REFERENCE_SET_STATUS = UAV_SENMOD_ZERO_REFERENCE_SET_NO,
				.FIRST_USE_AVA_STATUS      = UAV_SENMOD_FIRST_USE_AVA_NO,	
				.USE_CONTROL_STATUS        = UAV_SENMOD_USE_CONTROL_DISALLOW,	
				.FUSION_STATUS 			   = UAV_SENMOD_FUSION_FINISH,					
			},
		},
		
		.Horizontal = 
		{
			.WORK_STATUS = UAV_SENMOD_WORK_LIMITLESS, 		 /*默认使用无限制传感器*/
			
			.CURRENT_USE = UAV_HORIZONTAL_SENMOD_CURRENT_GPS, /*默认GPS*/
			
			.Opticflow = 
			{
				.EXIST_STATUS              = UAV_SENMOD_EXIST_NO,
				.DATA_STATUS               = UAV_SENMOD_DATA_NO,
				.ZERO_REFERENCE_SET_STATUS = UAV_SENMOD_ZERO_REFERENCE_SET_NO,
				.FIRST_USE_AVA_STATUS      = UAV_SENMOD_FIRST_USE_AVA_NO,	
				.USE_CONTROL_STATUS        = UAV_SENMOD_USE_CONTROL_DISALLOW,				
			},
			
			.Gps = 
			{
				.EXIST_STATUS              = UAV_SENMOD_EXIST_NO,
				.DATA_STATUS               = UAV_SENMOD_DATA_NO,
				.ZERO_REFERENCE_SET_STATUS = UAV_SENMOD_ZERO_REFERENCE_SET_NO,
				.FIRST_USE_AVA_STATUS      = UAV_SENMOD_FIRST_USE_AVA_NO,			
				.USE_CONTROL_STATUS        = UAV_SENMOD_USE_CONTROL_DISALLOW,				
				.FUSION_STATUS 			   = UAV_SENMOD_FUSION_FINISH,				
			},
		},
	},	/*传感器/模块状态*/
	
	/*= 3.遥控操作状态 =*/
	.REMOT_OPERATE_STATUS = UAV_REMOT_OPERATE_NOT_UNLOCK,		/*遥控操作状态*/	
	
	/*= 4.程序运行状态 =*/
	.UavProgrameStatus =
	{
		.INIT_STATUS = UAV_PROGRAME_INIT_START,
		.CpuUse = 
		{
			.major = 0,
			.minor = 0,
		},
	},			/*程序运行状态*/	
};

Uav_Status *g_psUav_Status = &g_sUav_Status;

/*检测GPS是否可用(满足定位数据+数据融合成功)*/
UAV_SENMOD_USE_CONTROL_STATUS status_GPS_Fix_Ava_Check(Uav_Status *uavStatus)
{
	/*1.HOME点已设置 
	  2.GPS数据满足定位条件
	  3.数据融合成功
	*/
	if ((uavStatus->HOME_SET_STATUS == UAV_HOME_SET_YES) && \
		(uavStatus->UavSenmodStatus.Horizontal.Gps.DATA_STATUS == UAV_SENMOD_DATA_OK) && \
		(g_psSinsReal->FUSION_STATUS[SINS_FUSION_HORIZONTAL] == SINS_FUSION_SUCC))
	{
		uavStatus->UavSenmodStatus.Horizontal.Gps.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_ALLOW; /*允许用于控制*/
	}
	else
	{
		uavStatus->UavSenmodStatus.Horizontal.Gps.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_DISALLOW; /*禁止用于控制*/		
	}
	
	return (uavStatus->UavSenmodStatus.Horizontal.Gps.USE_CONTROL_STATUS);
}

/*获取当前无人机的姿态控制模式和所用传感器*/
SYS_BOOLSTATUS check_uav_ctrl_and_sensor_use(Uav_Status *uavStatus, UAV_FLY_TYPE FLY_TYPE, UAV_VERTICAL_SENMOD_CURRENT_USE VER_SENMOD_USE, UAV_HORIZONTAL_SENMOD_CURRENT_USE HOR_SENMOD_USE)
{
	/*判断和指定check是否一致*/
	if ((uavStatus->UavFlyType.CURRENT == FLY_TYPE) && \
		(uavStatus->UavSenmodStatus.Vertical.CURRENT_USE == VER_SENMOD_USE) && \
		(uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE == HOR_SENMOD_USE))
	{
		/*返回正确*/
		return SYS_BOOL_TRUE;
	}
	
	/*返回错误*/
	return SYS_BOOL_FALSE;
}

SimulateWatchDog g_sUavRemotCMCDog = 
{
	.curTicks         = 0, /*初始值必须清0*/
	.nextProcessTicks = 0, /*初始值必须清0*/
};

SimulateWatchDog *g_psUavRemotCMCDog = &g_sUavRemotCMCDog;

/*遥控与飞机通讯状态*/
UAV_WIRELESS_CMC_STATUS status_check_uav_wireless(SimulateWatchDog *uavRemotCMCDog, Uav_Status *uavStatus)
{		
	uavRemotCMCDog->curTicks = my_GetTick();	/*获取当前tick foc = 10ms*/
	 
	/*如果curTicks < nextProcessTicks,说明"喂狗"流程正常,否则不正常*/
	if (uavRemotCMCDog->curTicks <= uavRemotCMCDog->nextProcessTicks)
	{
		/*标记通信成功*/
		uavStatus->WIRELESS_CMC_STATUS = UAV_WIRELESS_CMC_SUCC;
		
		/*通信成功后,重新标记自动返航未设置 且 恢复成自动返航前飞行模式*/
		if (g_psControlAircraft->GO_HOME_SET == CTRL_AIRCRAFT_GO_HOME_SET)
		{
			/*本次飞行模式为失联前模式(用于从失联到恢复通讯)*/
			uavStatus->UavFlyType.CURRENT = uavStatus->UavFlyType.BEFORE_WIRELESS_MISS;
			
			/*清空当前飞行任务*/
			control_fly_mission_clear(&gs_Uav_Fly_Mission, gs_Uav_Fly_Mission.CURRENT_FLY_MISSION);
			
			/*标记自动返航未设定*/
			g_psControlAircraft->GO_HOME_SET = CTRL_AIRCRAFT_GO_HOME_NOTSET;
		}
	}
	/*飞行途中(允许自动返航)失控返航,且锁定模式下,禁止切换到自动返航模式*/
	else
	{
		/*标记通信失败*/
		uavStatus->WIRELESS_CMC_STATUS = UAV_WIRELESS_CMC_FAIL;		
	
		/*判断是否满足失联返航:失联 / 接收机掉电无输入给飞控板*/
		if ((g_psControlAircraft->GO_HOME_STATUS == CTRL_AIRCRAFT_GO_HOME_ENABLE) && \
			(uavStatus->LOCK_STATUS == UAV_LOCK_NOT))
		{	
			/*自动返航 上次飞行状态保存 和 当前飞行状态设置 只设置一次*/
			if (g_psControlAircraft->GO_HOME_SET == CTRL_AIRCRAFT_GO_HOME_NOTSET)
			{	
				/*记录失联前的飞行模式,恢复通信后,可再切回之前模式*/
				uavStatus->UavFlyType.BEFORE_WIRELESS_MISS = uavStatus->UavFlyType.CURRENT;
		
				/*失联后,清除上一个任务,自动进入自动返航任务*/
				control_fly_mission_set(&gs_Uav_Fly_Mission, UAV_FLY_MISSION_ONEKEY_LAND_HOME, UAV_MISSION_ATTITUDE_CTRL_DISABLE);				
			
				/*标记自动返航已设定*/
				g_psControlAircraft->GO_HOME_SET = CTRL_AIRCRAFT_GO_HOME_SET;
			}
		}
	}
	
	return (uavStatus->WIRELESS_CMC_STATUS);
}

/*遥控与飞机通讯正常喂狗*/
void security_Feed_CMC_Succ_Dog_10MS(u8 _10msFoc, SimulateWatchDog *uavRemotCMCDog)
{
	u32 feedCmcDogTimeX10Ms = 0;
	
	/*遥控最大允许失联时间:500ms*/
	if (_10msFoc > 50)
	{
		_10msFoc = 50;
	}
	
	/*x X 10ms*/
	feedCmcDogTimeX10Ms = _10msFoc * CMC_DOG_FEED_FACTOR_10MS;
	
	uavRemotCMCDog->nextProcessTicks = my_GetTick();	     /*获取当前tick, sys_GetTick 10ms获取一次*/
	
	uavRemotCMCDog->nextProcessTicks += feedCmcDogTimeX10Ms; /*延时*/
}
