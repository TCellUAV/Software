#include "status_Aircraft.h"
#include "remot_DataAnaly.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

/*因为某些状态值初始化无意义是0xff,
  因此选用0xf0和0xf1初始化下列状态值*/

AircraftStatus g_sAircraftStatus = 
{
	/*= 1.飞行器本身的状态 =*/
	/*飞控锁定状态*/
	.LOCK_STATUS   				        = AIRCRAFT_LOCKING,    /*默认锁定*/
	                                    
	/*飞行器飞行状态*/                  
	.CUR_FLY_STATUS   				        = AIRCRAFT_LANDING,	/*初始化默认着陆中*/
	.LAST_FLY_STATUS                    = AIRCRAFT_LANDING,	/*上次飞行器飞行状态*/	
	                                    
	/*飞行器飞行模式*/                  
	.CUR_FLY_TYPE                       = AIRCRAFT_FLY_TYPE_ATTITUDE, /*默认纯姿态飞行模式*/
	.LAST_FLY_TYPE					    = AIRCRAFT_FLY_TYPE_ATTITUDE, /*默认纯姿态飞行模式*/
		                                
	/*GPS HOME 点设置状态*/             
	.HOME_STATUS			            = AIRCRAFT_HOME_NOTSET,	/*默认未设置GPS HOME点*/
	                                    
	/*遥控和飞行器通信状态*/            
	.CMC_STATUS 				        = AIRCRAFT_CMC_FAIL,   /*默认通信失败*/
	                                    
	/*人机交互显示页面*/                
	.HCI_SHOW_PAGE                      = AIRCRAFT_HCI_SHOW_DISABLE, /*默认不允许OLED显示*/	
	                                    
	/*= 2.飞行器工作方式状态 =*/	    
	.CUR_HEIGHT_CONTROL_MODE              = AIRCRAFT_CONTROL_AUTO,     	    /*纯姿态自稳*/
	.LAST_HEIGHT_CONTROL_MODE	          = AIRCRAFT_CONTROL_AUTO,     	    /*纯姿态自稳*/	
	.CUR_HORIZONTAL_CONTROL_MODE          = AIRCRAFT_CONTROL_AUTO,     	    /*纯姿态自稳*/
	.LAST_HORIZONTAL_CONTROL_MODE         = AIRCRAFT_CONTROL_AUTO,     	    /*纯姿态自稳*/
	.HEIGHT_REMOT_EXPECT_CONTROL_MODE     = AIRCRAFT_CONTROL_SENSOR_TO_AUTO, /*遥控期望高度控制模式*/
	.HORIZONTAL_REMOT_EXPECT_CONTROL_MODE = AIRCRAFT_CONTROL_SENSOR_TO_AUTO, /*遥控期望水平控制模式*/		
    .HEIGHT_CONTROL_MODE                  = AIRCRAFT_CONTROL_AUTO,			 /*默认实际高度控制模式为自稳*/
	.HORIZONTAL_CONTROL_MODE              = AIRCRAFT_CONTROL_AUTO,			 /*默认实际水平控制模式为自稳*/	
	.HEIGHT_REFERENCE_SET_STATUS          = AIRCRAFT_REFERENCE_SET_NO,		/*高度控制过程中参考点设置状态*/
	.HORIZONTAL_REFERENCE_SET_STATUS      = AIRCRAFT_REFERENCE_SET_NO,		/*水平控制过程中参考点设置状态*/	
	.ONEKEY_TAKEOFF_CONTROL_STATUS        = AIRCRAFT_ONEKEY_CTRL_DISABLE,		/*一键起飞控制状态*/
	.ONEKEY_TAKEOFF_TARGET_SET_STATUS     = AIRCRAFT_ONEKEY_TARGET_SET_NO,	/*一键起飞目标点设置状态*/
	.ONEKEY_TAKEOFF_TARGET_REACH_STATUS   = AIRCRAFT_ONEKEY_TARGET_REACH_NO,	/*一键起飞目标点到达状态*/
	.ONEKEY_LAND_CONTROL_STATUS           = AIRCRAFT_ONEKEY_CTRL_DISABLE,		/*一键降落控制状态*/	
	.ONEKEY_CRUISE_CONTROL_STATUS         = AIRCRAFT_ONEKEY_CTRL_DISABLE, 	/*一键巡航控制状态*/
								        
	/*= 3.姿态数据状态 =*/              
	/*高度*/                            
	/*气压计零参考点气压值状态*/        
	.BERO_ZERO_PRESSURE 		        = HEIGHT_DATA_STATUS_NO,  /*默认无效*/
								        
	/*气压计当前气压值状态*/            
	.BERO_PRESSURE 				        = HEIGHT_DATA_STATUS_NO,  /*默认无效*/
								        
	/*气压计观测高度值状态*/	        
	.BERO_ESTIMATE_ALTITUDE 	        = HEIGHT_DATA_STATUS_NO,  /*默认无效*/
								        
	/*超声波零参考点高度值状态*/        
	.ULTR_ZERO_ALTITUDE 		        = HEIGHT_DATA_STATUS_NO,  /*默认无效*/
								        
	/*超声波观测高度值状态*/            
	.ULTR_ESTIMATE_ALTITUDE 	        = HEIGHT_DATA_STATUS_NO,  /*默认无效*/
	
	/*高度(综合所有高度传感)参考零点设置状态*/
	.ZERO_ALTITUDE				        = HEIGHT_BERO_ULTR_ZERO_DISAVA,   /*默认为0,便于状态或运算*/
								        
	/*水平*/                            
	/*GPS观测水平值状态*/	            
	.GPS_ESTIMATE_HORIZONTAL 	        = HORIZONTAL_DATA_STATUS_NO, /*默认无效*/
								        
	/*光流观测水平值状态*/	            
	.OPFLOW_ESTIMATE_HORIZONTAL         = HORIZONTAL_DATA_STATUS_NO, /*默认无效*/
	
	/*水平(综合所有水平传感)参考零点设置状态*/	
	.ZERO_HORIZONTAL                    = HORIZONTAL_GPS_OPFLOW_ZERO_DISAVA, /*默认为0,便于状态或运算*/

	/*=== 4.高度&水平传感器有效性状态 ===*/
	/*传感器观测高度值状态*/	
	.ESTIMATE_ALTITUDE     		        = HEIGHT_BERO_ULTR_DISAVA,  /*默认为0,便于状态或运算*/

	/*传感器观测水平值状态*/
	.ESTIMATE_HORIZONTAL 		        = HORIZONTAL_GPS_OPFLOW_DISAVA,  /*默认为0,便于状态或运算*/	
	
	/*=== 5.传感器相关 ===*/
	.GPS_DATA_FIRST_AVA_STATUS          = GPS_DATA_FIRST_DISAVA,		 /*第一次使用(HOME点设置)GPS数据有效性*/	
	.GPS_QUALITY_STATUS					= GPS_DATA_QUALITY_BAD, 		 /*默认GPS信号质量差*/
	.CUR_VERTICAL_SENSOR                = VERTICAL_SENSOR_USE_BERO,		 /*本次竖直方向传感器*/
	.LAST_VERTICAL_SENSOR				= VERTICAL_SENSOR_USE_BERO,		 /*上次竖直方向传感器*/
	
	/*=== 6.程序运行状态 ==*/
	.PLATFORM_INIT_STATUS			    = PROGRAME_INIT_START,			 /*默认程序初始化开始*/	
};

AircraftStatus *g_psAircraftStatus = &g_sAircraftStatus;


/*检测GPS是否可用(满足定位数据+数据融合成功)*/
GPS_POS_FIX_AVA_STATUS status_GPS_Fix_Ava_Check(AircraftStatus *aircraftStatus)
{
	/*1.HOME点已设置 
	  2.GPS数据满足定位条件
	  3.数据融合成功
	*/
	if ((g_psAircraftStatus->HOME_STATUS == AIRCRAFT_HOME_SET) && \
		(g_psAircraftStatus->GPS_ESTIMATE_HORIZONTAL == HORIZONTAL_DATA_STATUS_OK) && \
		(g_psSinsReal->FUSION_STATUS[SINS_FUSION_HORIZONTAL] == SINS_FUSION_SUCC))
	{
		aircraftStatus->GPS_AVA_STATUS = GPS_POS_FIX_AVA_TRUE;
	}
	else
	{
		aircraftStatus->GPS_AVA_STATUS = GPS_POS_FIX_AVA_FALSE;
	}
	
	return (aircraftStatus->GPS_AVA_STATUS);
}

SimulateWatchDog g_sUavRemotCMCDog = 
{
	.curTicks         = 0, /*初始值必须清0*/
	.nextProcessTicks = 0, /*初始值必须清0*/
};

SimulateWatchDog *g_psUavRemotCMCDog = &g_sUavRemotCMCDog;

/*遥控与飞机通讯状态*/
AIRCRAFT_CMC_STATUS status_check_aircraft_remot_communication(SimulateWatchDog *uavRemotCMCDog, AircraftStatus *aircraftStatus)
{		
	uavRemotCMCDog->curTicks = my_GetTick();	/*获取当前tick foc = 10ms*/
	 
	/*如果curTicks < nextProcessTicks,说明"喂狗"流程正常,否则不正常*/
	if (uavRemotCMCDog->curTicks <= uavRemotCMCDog->nextProcessTicks)
	{
		/*标记通信成功*/
		aircraftStatus->CMC_STATUS = AIRCRAFT_CMC_SUCC;
		
		/*通信成功后,重新标记自动返航未设置 且 恢复成自动返航前飞行模式*/
		if (g_psControlAircraft->GO_HOME_SET == CTRL_AIRCRAFT_GO_HOME_SET)
		{
			/*本次飞行模式为上次飞行模式(用于从失联到恢复通讯)*/
			aircraftStatus->CUR_FLY_TYPE = aircraftStatus->LAST_FLY_TYPE;
		
			/*标记自动返航未设定*/
			g_psControlAircraft->GO_HOME_SET = CTRL_AIRCRAFT_GO_HOME_NOTSET;
		}
	}
	/*飞行途中(允许自动返航)失控返航,且锁定模式下,禁止切换到自动返航模式*/
	else
	{
		/*标记通信失败*/
		aircraftStatus->CMC_STATUS = AIRCRAFT_CMC_FAIL;
	
		/*判断是否满足失联返航*/
		if (((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_ZERO)     != REMOT_DATA_ZERO) || \
			  (remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_ZERO)    != REMOT_DATA_ZERO) || \
			  (remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_ZERO) != REMOT_DATA_ZERO) || \
			  (remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_ZERO)      != REMOT_DATA_ZERO)) && \
			  (g_psControlAircraft->GO_HOME_STATUS == CTRL_AIRCRAFT_GO_HOME_ENABLE) && \
			  (aircraftStatus->LOCK_STATUS == AIRCRAFT_UNLOCK))
		{	

		
			/*自动返航 上次飞行状态保存 和 当前飞行状态设置 只设置一次*/
			if (g_psControlAircraft->GO_HOME_SET == CTRL_AIRCRAFT_GO_HOME_NOTSET)
			{	
				/*记录失联前的飞行模式,恢复通信后,可再切回之前模式*/
				aircraftStatus->LAST_FLY_TYPE = aircraftStatus->CUR_FLY_TYPE;
		
				/*失联后标记飞行模式为自动返航*/
				aircraftStatus->CUR_FLY_TYPE = AIRCRAFT_FLY_TYPE_GO_HOME;
			
				/*标记自动返航已设定*/
				g_psControlAircraft->GO_HOME_SET = CTRL_AIRCRAFT_GO_HOME_SET;
			}
		}
	}
	
	return (aircraftStatus->CMC_STATUS);
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
