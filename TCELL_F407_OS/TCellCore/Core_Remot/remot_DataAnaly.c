#include "remot_DataAnaly.h"
#include "ahrs_Caculation.h"
#include "attitude_Aircraft.h"
#include "sys_McuInit.h"

#include "control_Config.h" /*遥控行程期望设置*/
#include "control_Aircraft.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

/*遥控接收机PWM数据*/
ReceiverAnaly g_sReceiverAnaly = 
{
	#if defined(REMOTE_DATA_RECV__PPM) /*PPM*/
	.PPM_Buff           = {0},
	.PPM_RECV_STATUS    = REMOT_PPM_RECV_FINISH,
	.PPM_SAMPLE_CHANNLE = REMOT_PPM_SAMPLE_1ST_CNL,
	#endif
};
ReceiverAnaly *g_psReceiverAnaly = &g_sReceiverAnaly;

/*遥控数据*/
RemotData g_sRemotData   = {0};
RemotData *g_psRemotData = &g_sRemotData;

/*=== 遥控数据解析及解析成期望 ===*/
/*获取遥控各通道数据*/
void remot_get_all_channel_data(RemotData *remotData, ReceiverAnaly *receiverAnaly)
{	
	/*判断姿态遥控数据是否更新正常,正常则飞行器处在正常遥控*/
	/*iA10B 参考网友设置失控时油门通道输出值不在范围内,(1000/109%)左右 */
	/*iA6 实测关闭遥控后,接收机即无输出*/	
	if ((receiverAnaly->Attitude.Throttle.UPDATE_STATUS == WAVE_STATUS_NEW) && \
		(receiverAnaly->Attitude.Throttle.avaTicks >= 980)) /*此处设置980,实测955(等于0的情况更多),连接正常时,值不会低于1000*/
	{
		/*在当前时刻喂狗100ms*/
		security_Feed_CMC_Succ_Dog_10MS(10, &g_sUavRemotCMCDog);
	}
	
	/*遥控数据赋值*/
/*=== Attitude: CH1 - CH4 ===*/	
	/*Attitude CH1 Roll Data*/
	if (receiverAnaly->Attitude.Roll.UPDATE_STATUS == WAVE_STATUS_NEW)	/*判断是否为新数据*/
	{
		receiverAnaly->Attitude.Roll.UPDATE_STATUS = WAVE_STATUS_OLD; /*用过的数据标记为老数据*/
		
		/*有效赋值*/
		if ((1000 <= receiverAnaly->Attitude.Roll.avaTicks) && \
			(receiverAnaly->Attitude.Roll.avaTicks <= 2000))
		{
			remotData->AttRoll = receiverAnaly->Attitude.Roll.avaTicks;
		}
		else
		{
			remotData->AttRoll = 0;			
		}
		
		/*将遥控数据转化成遥控ROLL期望角*/
		if (remotData->AttRoll <= REMOT_ANGLE_DEADZONE_BUTTOM)
		{
			g_psControlAircraft->RemotExpectAngle.roll = (remotData->AttRoll - REMOT_ANGLE_DEADZONE_BUTTOM) * \
														  REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX / REMOT_ANGLE_DEADZONE_EXCEPT; 
		}
		else if (remotData->AttRoll >= REMOT_ANGLE_DEADZONE_TOP)
		{
			g_psControlAircraft->RemotExpectAngle.roll = (remotData->AttRoll - REMOT_ANGLE_DEADZONE_TOP) * \
														  REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX / REMOT_ANGLE_DEADZONE_EXCEPT; 			
		}
		else
		{
			g_psControlAircraft->RemotExpectAngle.roll = 0;
		}
		
		/*遥控ROLL期望角范围限制*/
		if (g_psControlAircraft->RemotExpectAngle.roll >= REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX)
		{
			g_psControlAircraft->RemotExpectAngle.roll = REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX;
		}
		else if (g_psControlAircraft->RemotExpectAngle.roll <= -REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX)
		{
			g_psControlAircraft->RemotExpectAngle.roll = -REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX;		
		}		
		
		/*遥控期望自稳横滚角*/
		g_psControlAircraft->RemotExpectAutoAngle.roll = g_psControlAircraft->RemotExpectAngle.roll;
	}
	
	/*Attitude CH2 Pitch Data*/	
	if (receiverAnaly->Attitude.Pitch.UPDATE_STATUS == WAVE_STATUS_NEW)	/*判断是否为新数据*/
	{
		receiverAnaly->Attitude.Pitch.UPDATE_STATUS = WAVE_STATUS_OLD; /*用过的数据标记为老数据*/
		
		/*有效赋值*/
		if ((1000 <= receiverAnaly->Attitude.Pitch.avaTicks) && \
			(receiverAnaly->Attitude.Pitch.avaTicks <= 2000))
		{
			remotData->AttPitch = receiverAnaly->Attitude.Pitch.avaTicks;
		}
		else
		{
			remotData->AttPitch = 0;			
		}		

		/*将遥控数据转化成遥控PITCH期望角*/
		if (remotData->AttPitch <= REMOT_ANGLE_DEADZONE_BUTTOM)
		{
			g_psControlAircraft->RemotExpectAngle.pitch = (REMOT_ANGLE_DEADZONE_BUTTOM - remotData->AttPitch) * \
														   REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX / REMOT_ANGLE_DEADZONE_EXCEPT; 
		}
		else if (remotData->AttPitch >= REMOT_ANGLE_DEADZONE_TOP)
		{
			g_psControlAircraft->RemotExpectAngle.pitch = (REMOT_ANGLE_DEADZONE_TOP - remotData->AttPitch) * \
														   REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX / REMOT_ANGLE_DEADZONE_EXCEPT; 			
		}
		else
		{
			g_psControlAircraft->RemotExpectAngle.pitch = 0;
		}
		
		/*遥控PITCH期望角范围限制*/
		if (g_psControlAircraft->RemotExpectAngle.pitch >= REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX)
		{
			g_psControlAircraft->RemotExpectAngle.pitch = REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX;
		}
		else if (g_psControlAircraft->RemotExpectAngle.pitch <= -REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX)
		{
			g_psControlAircraft->RemotExpectAngle.pitch = -REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX;		
		}		

		/*遥控期望遥控期望自稳俯仰角*/
		g_psControlAircraft->RemotExpectAutoAngle.pitch = g_psControlAircraft->RemotExpectAngle.pitch;		
	}

	/*Attitude CH3 Throttle Data*/	
	if (receiverAnaly->Attitude.Throttle.UPDATE_STATUS == WAVE_STATUS_NEW)	/*判断是否为新数据*/
	{
		receiverAnaly->Attitude.Throttle.UPDATE_STATUS = WAVE_STATUS_OLD; /*用过的数据标记为老数据*/
		
		/*有效赋值*/
		if ((1000 <= receiverAnaly->Attitude.Throttle.avaTicks) && \
			(receiverAnaly->Attitude.Throttle.avaTicks <= 2000))
		{
			remotData->AttThrottle = receiverAnaly->Attitude.Throttle.avaTicks;
		}
		else
		{
			remotData->AttThrottle = 0;			
		}		
		
		/*为了安全,油门杆低位死区为50*/
		s16 tempRc = remotData->AttThrottle - 1050;
		
		/*0 ~ 1000*/
		if (tempRc <= 0) /*1105*/
		{
 			g_psControlAircraft->RemotExpectAngle.throttle = 0;
		}
		else if (tempRc >= 1000) /*2100*/
		{
			g_psControlAircraft->RemotExpectAngle.throttle = 1000;		
		}
		else
		{
			g_psControlAircraft->RemotExpectAngle.throttle = tempRc;		
		}	
		
		/*油门杆上推或者下推判断量(遥感油门原始行程量)*/
		g_psControlAircraft->throttleUpDownJudgeValue = g_psControlAircraft->RemotExpectAngle.throttle;
		
		/*油门值+油门基础值*/
		g_psControlAircraft->RemotExpectAngle.throttle += REMOT_THROTTLE_BASE_VALUE;
	}

	/*Attitude CH4 Yaw Data*/		
	if (receiverAnaly->Attitude.Yaw.UPDATE_STATUS == WAVE_STATUS_NEW)	/*判断是否为新数据*/
	{
		receiverAnaly->Attitude.Yaw.UPDATE_STATUS = WAVE_STATUS_OLD; /*用过的数据标记为老数据*/

		/*有效赋值*/
		if ((1000 <= receiverAnaly->Attitude.Yaw.avaTicks) && \
			(receiverAnaly->Attitude.Yaw.avaTicks <= 2000))
		{
			remotData->AttYaw = receiverAnaly->Attitude.Yaw.avaTicks;
		}
		else
		{
			remotData->AttYaw = 0;			
		}		
		
		/*将遥控数据转化成遥控YAW期望角*/
		if (remotData->AttYaw <= REMOT_ANGLE_DEADZONE_BUTTOM)
		{
			g_psControlAircraft->RemotExpectAngle.yaw = (REMOT_ANGLE_DEADZONE_BUTTOM - remotData->AttYaw) * \
														 REMOT_YAW_ANGLE_EXPECT_MAX / REMOT_ANGLE_DEADZONE_EXCEPT; 
		}
		else if (remotData->AttYaw >= REMOT_ANGLE_DEADZONE_TOP)
		{
			g_psControlAircraft->RemotExpectAngle.yaw = (REMOT_ANGLE_DEADZONE_TOP - remotData->AttYaw) * \
														 REMOT_YAW_ANGLE_EXPECT_MAX / REMOT_ANGLE_DEADZONE_EXCEPT; 			
		}
		else
		{
			g_psControlAircraft->RemotExpectAngle.yaw = 0;
		}

		/*遥控YAW期望角范围限制*/
		if (g_psControlAircraft->RemotExpectAngle.yaw >= REMOT_YAW_ANGLE_EXPECT_MAX)
		{
			g_psControlAircraft->RemotExpectAngle.yaw = REMOT_YAW_ANGLE_EXPECT_MAX;
		}
		else if (g_psControlAircraft->RemotExpectAngle.yaw <= -REMOT_YAW_ANGLE_EXPECT_MAX)
		{
			g_psControlAircraft->RemotExpectAngle.yaw = -REMOT_YAW_ANGLE_EXPECT_MAX;		
		}
	}		
	
/*=== Switch: CH5 - CH8 ===*/
	/*Switch CH1 SWA Data*/
	if (receiverAnaly->Switch.SWA.UPDATE_STATUS == WAVE_STATUS_NEW)	/*判断是否为新数据*/
	{
		receiverAnaly->Switch.SWA.UPDATE_STATUS = WAVE_STATUS_OLD; /*用过的数据标记为老数据*/
	
		/*有效赋值*/
		if ((1000 <= receiverAnaly->Switch.SWA.avaTicks) && \
			(receiverAnaly->Switch.SWA.avaTicks <= 2000))
		{
			remotData->SWA = receiverAnaly->Switch.SWA.avaTicks;
		}
		else
		{
			remotData->SWA = 0;
		}		
	}
	
	/*Switch CH2 SWB Data*/
	if (receiverAnaly->Switch.SWB.UPDATE_STATUS == WAVE_STATUS_NEW)	/*判断是否为新数据*/
	{
		receiverAnaly->Switch.SWB.UPDATE_STATUS = WAVE_STATUS_OLD; /*用过的数据标记为老数据*/

		/*有效赋值*/
		if ((1000 <= receiverAnaly->Switch.SWB.avaTicks) && \
			(receiverAnaly->Switch.SWB.avaTicks <= 2000))
		{
			remotData->SWB = receiverAnaly->Switch.SWB.avaTicks;
		}
		else
		{
			remotData->SWB = 0;
		}		
	}

	/*Switch CH3 SWC Data*/
	if (receiverAnaly->Switch.SWC.UPDATE_STATUS == WAVE_STATUS_NEW)	/*判断是否为新数据*/
	{
		receiverAnaly->Switch.SWC.UPDATE_STATUS = WAVE_STATUS_OLD; /*用过的数据标记为老数据*/

		/*有效赋值*/
		if ((1000 <= receiverAnaly->Switch.SWC.avaTicks) && \
			(receiverAnaly->Switch.SWC.avaTicks <= 2000))
		{
			remotData->SWC = receiverAnaly->Switch.SWC.avaTicks;
		}
		else
		{
			remotData->SWC = 0;
		}		
	}

	/*Switch CH4 SWD Data*/		
	if (receiverAnaly->Switch.SWD.UPDATE_STATUS == WAVE_STATUS_NEW)	/*判断是否为新数据*/
	{
		receiverAnaly->Switch.SWD.UPDATE_STATUS = WAVE_STATUS_OLD; /*用过的数据标记为老数据*/
		
		/*有效赋值*/
		if ((1000 <= receiverAnaly->Switch.SWD.avaTicks) && \
			(receiverAnaly->Switch.SWD.avaTicks <= 2000))
		{
			remotData->SWD = receiverAnaly->Switch.SWD.avaTicks;
		}
		else
		{
			remotData->SWD = 0;
		}	
	}	
	
/*=== Switch: CH9 - CH10 ===*/
	/*Switch CH1 Pitch Data*/	
	if (receiverAnaly->Gimbal.VRA.UPDATE_STATUS == WAVE_STATUS_NEW)	/*判断是否为新数据*/
	{
		receiverAnaly->Gimbal.VRA.UPDATE_STATUS = WAVE_STATUS_OLD; /*用过的数据标记为老数据*/

		/*有效赋值*/
		if ((1000 <= receiverAnaly->Gimbal.VRA.avaTicks) && \
			(receiverAnaly->Gimbal.VRA.avaTicks <= 2000))
		{
			remotData->GimPitch = receiverAnaly->Gimbal.VRA.avaTicks;
		}
		else
		{
			remotData->GimPitch = 0;			
		}		
		
		/*将遥控数据转化成万向节Pitch期望角*/
		if (remotData->GimPitch <= REMOT_ANGLE_DEADZONE_BUTTOM)
		{
			g_psControlAircraft->GimbalExpectAngle.pitch = (REMOT_ANGLE_DEADZONE_BUTTOM - remotData->GimPitch) * \
															GIMBAL_PITCH_ANGLE_EXPECT_MAX / REMOT_ANGLE_DEADZONE_EXCEPT; 
		}
		else if (remotData->GimPitch >= REMOT_ANGLE_DEADZONE_TOP)
		{
			g_psControlAircraft->GimbalExpectAngle.pitch = (REMOT_ANGLE_DEADZONE_TOP - remotData->GimPitch) * \
															GIMBAL_PITCH_ANGLE_EXPECT_MAX / REMOT_ANGLE_DEADZONE_EXCEPT; 			
		}
		else
		{
			g_psControlAircraft->GimbalExpectAngle.pitch = 0;
		}		
	}

	/*Switch CH2 Yaw Data*/		
	if (receiverAnaly->Gimbal.VRB.UPDATE_STATUS == WAVE_STATUS_NEW)	/*判断是否为新数据*/
	{
		receiverAnaly->Gimbal.VRB.UPDATE_STATUS = WAVE_STATUS_OLD; /*用过的数据标记为老数据*/

		/*有效赋值*/
		if ((1000 <= receiverAnaly->Gimbal.VRB.avaTicks) && \
			(receiverAnaly->Gimbal.VRB.avaTicks <= 2000))
		{
			remotData->GimYaw = receiverAnaly->Gimbal.VRB.avaTicks;
		}
		else
		{
			remotData->GimYaw = 0;			
		}		

		/*将遥控数据转化成万向节Yaw期望角*/
		if (remotData->GimYaw <= REMOT_ANGLE_DEADZONE_BUTTOM)
		{
			g_psControlAircraft->GimbalExpectAngle.yaw = (REMOT_ANGLE_DEADZONE_BUTTOM - remotData->GimYaw) * \
														  GIMBAL_YAW_ANGLE_EXPECT_MAX / REMOT_ANGLE_DEADZONE_EXCEPT; 
		}
		else if (remotData->GimPitch >= REMOT_ANGLE_DEADZONE_TOP)
		{
			g_psControlAircraft->GimbalExpectAngle.yaw = (REMOT_ANGLE_DEADZONE_TOP - remotData->GimYaw) * \
														  GIMBAL_YAW_ANGLE_EXPECT_MAX / REMOT_ANGLE_DEADZONE_EXCEPT; 			
		}
		else
		{
			g_psControlAircraft->GimbalExpectAngle.yaw = 0;
		}			
	}	
}

vu16 g_sUnlockContinueTicks   = 0;	/*解锁动作持续时长*/
vu16 g_sLockContinueTicks     = 0;	/*锁定动作持续时长*/
vu16 g_sAutoLockContinueTicks = 0;	/*自动上锁持续时长*/

/*飞控:解锁/上锁/自动上锁*/
UAV_LOCK_STATUS remot_aircraft_lock_and_unlock(void)
{		
	/*1.左摇杆内下角解锁*/
	/*
		*******       *******
		*     *       *     *
		*  *  *       *  *  * 
		*   * *       *     *
		*******       *******
			  *          
	*/
	if ((remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MIN) == REMOT_DATA_MIN) && \
		(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MAX)      == REMOT_DATA_MAX))
	{
		/*满足条件目标Ticks累加*/
		g_sUnlockContinueTicks++;
		
		/*非目标Ticks清0*/
		g_sLockContinueTicks     = 0;
		g_sAutoLockContinueTicks = 0;
		
		/*动作持续Ticks,解锁*/
		if (g_sUnlockContinueTicks >= (AIRCRAFT_UNLOCK_CONTINU_TIME_MS / RTOS_WAKE_UP_UAV_CTRL_FOC_MS))
		{
			/*完成一次检测,目标Ticks清0*/
			g_sUnlockContinueTicks = 0;			
			
			/*返回飞机状态: 未锁定*/
			g_psUav_Status->LOCK_STATUS = UAV_LOCK_NOT;
		}
	}
	
	/*2.左摇杆外下角锁定*/
	/*
		*******       *******
		*     *       *     *
		*  *  *       *  *  * 
		* *   *       *     *
		*******       *******
		*
	*/	
	if ((remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MIN) == REMOT_DATA_MIN) && \
		(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MIN)      == REMOT_DATA_MIN))
	{
		/*满足条件目标Ticks累加*/
		g_sLockContinueTicks++;
		
		/*非目标Ticks清0*/
		g_sUnlockContinueTicks   = 0;
		g_sAutoLockContinueTicks = 0;
		
		/*动作持续Ticks,锁定*/
		if (g_sLockContinueTicks >= (AIRCRAFT_LOCK_CONTINU_TIME_MS / RTOS_WAKE_UP_UAV_CTRL_FOC_MS))
		{
			/*完成一次检测,目标Ticks清0*/
			g_sLockContinueTicks = 0;

			g_psUav_Status->LOCK_STATUS = UAV_LOCK_YES;			
		}
	}	
	
	/*解锁状态&&着陆状态,一定时间不操作遥控(除油门外,其余3个摇杆都在中位),就自动锁定*/
	if ((g_psUav_Status->LOCK_STATUS == UAV_LOCK_NOT) && \
	    (g_psUav_Status->UavLandStatus.ThisTime == UAV_LAND_YES) && \
		(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MIN)  == REMOT_DATA_MIN) && \
		(remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MID)  == REMOT_DATA_MID) && \
		(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MID) == REMOT_DATA_MID) && \
		(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MID) == REMOT_DATA_MID))
	{
		/*满足条件目标Ticks累加*/
		g_sAutoLockContinueTicks++;
		
		/*非目标Ticks清0*/
		g_sUnlockContinueTicks = 0;
		g_sLockContinueTicks   = 0;
		
		/*满足条件持续Ticks,自动锁定*/
		if (g_sAutoLockContinueTicks >= (AIRCRAFT_AUTOLOCK_CONTINU_TIME_MS / RTOS_WAKE_UP_UAV_CTRL_FOC_MS))
		{
			/*完成一次检测,目标Ticks清0*/
			g_sAutoLockContinueTicks = 0;
			
			/*标记为上锁*/
			g_psUav_Status->LOCK_STATUS = UAV_LOCK_YES;
		}
	}
	
	/*解锁状态&&着陆状态&&遥控和飞行器断开通信*/
	if ((g_psUav_Status->LOCK_STATUS == UAV_LOCK_NOT) && \
	    (g_psUav_Status->UavLandStatus.ThisTime == UAV_LAND_YES) && \
	    (g_psUav_Status->WIRELESS_CMC_STATUS == UAV_WIRELESS_CMC_FAIL))
	{
		/*标记为上锁*/
		g_psUav_Status->LOCK_STATUS = UAV_LOCK_YES;	
	}
	
	return (g_psUav_Status->LOCK_STATUS);
}

/*=== 原始数据极大/极小/中值判断 ===*/
REMOT_DATA_STATUS remot_Data_Range(u16 ChannelData, REMOT_DATA_STATUS testStatus)
{
	REMOT_DATA_STATUS retStatus;
	
	switch(testStatus)
	{
		case REMOT_DATA_ZERO:	/*判断数据是不是空的,即有没有遥控操作*/
		{
			if (ChannelData == 0)
			{
				retStatus = REMOT_DATA_ZERO;	  /*数据是空的,即没有遥控操作*/
			}
			else
			{
				retStatus = REMOT_DATA_ERROR;   /*不满足空值条件,数据错误非法*/	
			}
		}break;
		
		case REMOT_DATA_MIN:	/*判断数据是不是最小值*/
		{
			if ((ChannelData >= (REMOT_DATA_MIN_VALUE - REMOT_DATA_PRECISION)) && \
				(ChannelData <= (REMOT_DATA_MIN_VALUE + REMOT_DATA_PRECISION + REMOT_DATA_MIN_COMPENSATE)))
			{
				retStatus = REMOT_DATA_MIN;	  /*满足最小值条件*/
			}
			else
			{
				retStatus = REMOT_DATA_ERROR;  /*不满足最小值条件,数据错误非法*/	
			}
		}break;

		case REMOT_DATA_MID:	/*判断数据是不是中值,摇杆置中操作时,可能并不是严格的中间,所以需要算上置中误差*/
		{
			if ((ChannelData >= (REMOT_DATA_MID_VALUE - REMOT_DATA_PRECISION - REMOT_DATA_MID_ERROR)) && \
				(ChannelData <= (REMOT_DATA_MID_VALUE + REMOT_DATA_PRECISION + REMOT_DATA_MID_ERROR)))
			{
				retStatus = REMOT_DATA_MID;	  /*满足中值条件*/
			}
			else
			{
				retStatus = REMOT_DATA_ERROR;  /*不满足中值条件,数据错误非法*/	
			}		
		}break;

		case REMOT_DATA_MAX:	/*判断数据是不是最大值*/
		{
			if ((ChannelData >= (REMOT_DATA_MAX_VALUE - REMOT_DATA_PRECISION)) && \
				(ChannelData <= (REMOT_DATA_MAX_VALUE + REMOT_DATA_PRECISION)))
			{
				retStatus = REMOT_DATA_MAX;	  /*满足最大值条件*/
			}
			else
			{
				retStatus = REMOT_DATA_ERROR;  /*不满足最大值条件,数据错误非法*/	
			}		
		}break;	

		default:break;
	}
	
	return retStatus;
}


#if defined(REMOTE_DATA_RECV__PWM)
/*姿态通道(TIM4)捕获+解析*/
void remot_Attitude_Channel_Analy(void)
{	
	/*捕获CH1发生捕获事件*/
	if (TIM_GetITStatus(g_sTimPwmIn_Attitude.Tim, TIM_IT_CC1) != RESET)	
	{	
		if(g_psReceiverAnaly->Attitude.Roll.CAP_EDGE == CAP_EDGE_FALLING)	/*本次捕获到是下降沿 */
		{	  			
			g_psReceiverAnaly->Attitude.Roll.CAP_EDGE = CAP_EDGE_RISING;	/*标记下次捕获到的边沿状态为上升沿*/
					
			/*保存下降沿时刻计数*/
			g_psReceiverAnaly->Attitude.Roll.fallingEdgeTicks = TIM_GetCapture1(g_sTimPwmIn_Attitude.Tim);
			
			if (g_psReceiverAnaly->Attitude.Roll.fallingEdgeTicks > \
				g_psReceiverAnaly->Attitude.Roll.risingEdgeTicks)
			{
				/*得到一个PWM周期的高电平计数*/
				g_psReceiverAnaly->Attitude.Roll.avaTicks = g_psReceiverAnaly->Attitude.Roll.fallingEdgeTicks - \
															g_psReceiverAnaly->Attitude.Roll.risingEdgeTicks;
				
				/*pwm数据状态更新*/
				g_psReceiverAnaly->Attitude.Roll.UPDATE_STATUS = WAVE_STATUS_NEW;
			}
			
	   		TIM_OC1PolarityConfig(g_sTimPwmIn_Attitude.Tim, TIM_ICPolarity_Rising); /*设置为上升沿捕获*/
		}
		else  						/*本次捕获到是上升沿 */
		{
			/*保存上升沿时刻计数*/
 			g_psReceiverAnaly->Attitude.Roll.risingEdgeTicks = TIM_GetCapture1(g_sTimPwmIn_Attitude.Tim);
			
			g_psReceiverAnaly->Attitude.Roll.CAP_EDGE = CAP_EDGE_FALLING;	/*标记下次捕获到的边沿状态为下降沿*/
			
	   		TIM_OC1PolarityConfig(g_sTimPwmIn_Attitude.Tim, TIM_ICPolarity_Falling);		/*CC1P=1 设置为下降沿捕获*/
		}
		
		TIM_ClearITPendingBit(g_sTimPwmIn_Attitude.Tim, TIM_IT_CC1);	/*清除中断标志位*/	  	    
	}	
	
	/*捕获CH2发生捕获事件*/
	if (TIM_GetITStatus(g_sTimPwmIn_Attitude.Tim, TIM_IT_CC2) != RESET)	
	{	
		if(g_psReceiverAnaly->Attitude.Pitch.CAP_EDGE == CAP_EDGE_FALLING)	/*本次捕获到是下降沿 */
		{	  			
			g_psReceiverAnaly->Attitude.Pitch.CAP_EDGE = CAP_EDGE_RISING;	/*标记下次捕获到的边沿状态为上升沿*/
					
			/*保存下降沿时刻计数*/
			g_psReceiverAnaly->Attitude.Pitch.fallingEdgeTicks = TIM_GetCapture2(g_sTimPwmIn_Attitude.Tim);
			
			if (g_psReceiverAnaly->Attitude.Pitch.fallingEdgeTicks > \
				g_psReceiverAnaly->Attitude.Pitch.risingEdgeTicks)
			{
				/*得到一个PWM周期的有效计数*/
				g_psReceiverAnaly->Attitude.Pitch.avaTicks = g_psReceiverAnaly->Attitude.Pitch.fallingEdgeTicks - \
															 g_psReceiverAnaly->Attitude.Pitch.risingEdgeTicks;

				/*pwm数据状态更新*/
				g_psReceiverAnaly->Attitude.Pitch.UPDATE_STATUS = WAVE_STATUS_NEW;				
			}
			
	   		TIM_OC2PolarityConfig(g_sTimPwmIn_Attitude.Tim, TIM_ICPolarity_Rising); /*设置为上升沿捕获*/
		}
		else  						/*本次捕获到是上升沿 */
		{
			/*保存上升沿时刻计数*/
 			g_psReceiverAnaly->Attitude.Pitch.risingEdgeTicks = TIM_GetCapture2(g_sTimPwmIn_Attitude.Tim);
			
			g_psReceiverAnaly->Attitude.Pitch.CAP_EDGE = CAP_EDGE_FALLING;	/*标记下次捕获到的边沿状态为下降沿*/
			
	   		TIM_OC2PolarityConfig(g_sTimPwmIn_Attitude.Tim, TIM_ICPolarity_Falling);		/*CC1P=1 设置为下降沿捕获*/
		}
		
		TIM_ClearITPendingBit(g_sTimPwmIn_Attitude.Tim, TIM_IT_CC2);	/*清除中断标志位*/	    
	}	
	
	/*捕获CH3发生捕获事件*/
	if (TIM_GetITStatus(g_sTimPwmIn_Attitude.Tim, TIM_IT_CC3) != RESET)	
	{	
		if(g_psReceiverAnaly->Attitude.Throttle.CAP_EDGE == CAP_EDGE_FALLING)	/*本次捕获到是下降沿 */
		{	  			
			g_psReceiverAnaly->Attitude.Throttle.CAP_EDGE = CAP_EDGE_RISING;	/*标记下次捕获到的边沿状态为上升沿*/
					
			/*保存下降沿时刻计数*/
			g_psReceiverAnaly->Attitude.Throttle.fallingEdgeTicks = TIM_GetCapture3(g_sTimPwmIn_Attitude.Tim);
			
			if (g_psReceiverAnaly->Attitude.Throttle.fallingEdgeTicks > \
				g_psReceiverAnaly->Attitude.Throttle.risingEdgeTicks)
			{
				/*得到一个PWM周期的有效计数*/
				g_psReceiverAnaly->Attitude.Throttle.avaTicks = g_psReceiverAnaly->Attitude.Throttle.fallingEdgeTicks - \
																g_psReceiverAnaly->Attitude.Throttle.risingEdgeTicks;
				
				/*pwm数据状态更新*/
				g_psReceiverAnaly->Attitude.Throttle.UPDATE_STATUS = WAVE_STATUS_NEW;	
			}
			
	   		TIM_OC3PolarityConfig(g_sTimPwmIn_Attitude.Tim, TIM_ICPolarity_Rising); /*设置为上升沿捕获*/
		}
		else  						/*本次捕获到是上升沿 */
		{
			/*保存上升沿时刻计数*/
 			g_psReceiverAnaly->Attitude.Throttle.risingEdgeTicks = TIM_GetCapture3(g_sTimPwmIn_Attitude.Tim);
			
			g_psReceiverAnaly->Attitude.Throttle.CAP_EDGE = CAP_EDGE_FALLING;	/*标记下次捕获到的边沿状态为下降沿*/
			
	   		TIM_OC3PolarityConfig(g_sTimPwmIn_Attitude.Tim, TIM_ICPolarity_Falling);		/*CC1P=1 设置为下降沿捕获*/
		}
		
		TIM_ClearITPendingBit(g_sTimPwmIn_Attitude.Tim, TIM_IT_CC3);	/*清除中断标志位*/	  	    
	}	
	
	/*捕获CH4发生捕获事件*/
	if (TIM_GetITStatus(g_sTimPwmIn_Attitude.Tim, TIM_IT_CC4) != RESET)	
	{	
		if(g_psReceiverAnaly->Attitude.Yaw.CAP_EDGE == CAP_EDGE_FALLING)	/*本次捕获到是下降沿 */
		{	  			
			g_psReceiverAnaly->Attitude.Yaw.CAP_EDGE = CAP_EDGE_RISING;	/*标记下次捕获到的边沿状态为上升沿*/
					
			/*保存下降沿时刻计数*/
			g_psReceiverAnaly->Attitude.Yaw.fallingEdgeTicks = TIM_GetCapture4(g_sTimPwmIn_Attitude.Tim);
			
			if (g_psReceiverAnaly->Attitude.Yaw.fallingEdgeTicks > \
				g_psReceiverAnaly->Attitude.Yaw.risingEdgeTicks)
			{
				/*得到一个PWM周期的有效计数*/
				g_psReceiverAnaly->Attitude.Yaw.avaTicks = g_psReceiverAnaly->Attitude.Yaw.fallingEdgeTicks - \
														   g_psReceiverAnaly->Attitude.Yaw.risingEdgeTicks;
				
				/*pwm数据状态更新*/
				g_psReceiverAnaly->Attitude.Yaw.UPDATE_STATUS = WAVE_STATUS_NEW;					
			}
			
	   		TIM_OC4PolarityConfig(g_sTimPwmIn_Attitude.Tim, TIM_ICPolarity_Rising); /*设置为上升沿捕获*/
		}
		else  						/*本次捕获到是上升沿 */
		{
			/*保存上升沿时刻计数*/
 			g_psReceiverAnaly->Attitude.Yaw.risingEdgeTicks = TIM_GetCapture4(g_sTimPwmIn_Attitude.Tim);
			
			g_psReceiverAnaly->Attitude.Yaw.CAP_EDGE = CAP_EDGE_FALLING;	/*标记下次捕获到的边沿状态为下降沿*/
			
	   		TIM_OC4PolarityConfig(g_sTimPwmIn_Attitude.Tim, TIM_ICPolarity_Falling);		/*CC1P=1 设置为下降沿捕获*/
		}
		
		TIM_ClearITPendingBit(g_sTimPwmIn_Attitude.Tim, TIM_IT_CC4);	/*清除中断标志位*/	    
	}	

	/*定时器计数溢出中断*/
	if (TIM_GetITStatus(g_sTimPwmIn_Attitude.Tim, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(g_sTimPwmIn_Attitude.Tim, TIM_IT_Update);	/*清除中断标志位*/	  	    
	}	
}


/*云台万向节旋钮通道(TIM1)捕获+解析*/
void remot_Switch_Channel_Analy(void)
{	
	/*捕获CH1发生捕获事件*/
	if (TIM_GetITStatus(g_sTimPwmIn_Switch.Tim, TIM_IT_CC1) != RESET)	
	{	
		if(g_psReceiverAnaly->Switch.SWA.CAP_EDGE == CAP_EDGE_FALLING)	/*本次捕获到是下降沿 */
		{	  			
			g_psReceiverAnaly->Switch.SWA.CAP_EDGE = CAP_EDGE_RISING;	/*标记下次捕获到的边沿状态为上升沿*/
					
			/*保存下降沿时刻计数*/
			g_psReceiverAnaly->Switch.SWA.fallingEdgeTicks = TIM_GetCapture1(g_sTimPwmIn_Switch.Tim);
			
			if (g_psReceiverAnaly->Switch.SWA.fallingEdgeTicks > \
				g_psReceiverAnaly->Switch.SWA.risingEdgeTicks)
			{
				/*得到一个PWM周期的有效计数*/
				g_psReceiverAnaly->Switch.SWA.avaTicks = g_psReceiverAnaly->Switch.SWA.fallingEdgeTicks - \
														 g_psReceiverAnaly->Switch.SWA.risingEdgeTicks;
				
				/*pwm数据状态更新*/
				g_psReceiverAnaly->Switch.SWA.UPDATE_STATUS = WAVE_STATUS_NEW;					
			}
			
	   		TIM_OC1PolarityConfig(g_sTimPwmIn_Switch.Tim, TIM_ICPolarity_Rising); /*设置为上升沿捕获*/
		}
		else  						/*本次捕获到是上升沿 */
		{
			/*保存上升沿时刻计数*/
 			g_psReceiverAnaly->Switch.SWA.risingEdgeTicks = TIM_GetCapture1(g_sTimPwmIn_Switch.Tim);
			
			g_psReceiverAnaly->Switch.SWA.CAP_EDGE = CAP_EDGE_FALLING;	/*标记下次捕获到的边沿状态为下降沿*/
			
	   		TIM_OC1PolarityConfig(g_sTimPwmIn_Switch.Tim, TIM_ICPolarity_Falling);		/*CC1P=1 设置为下降沿捕获*/
		}
		
		TIM_ClearITPendingBit(g_sTimPwmIn_Switch.Tim, TIM_IT_CC1);	/*清除中断标志位*/	  	    
	}	
	
	/*捕获CH2发生捕获事件*/
	if (TIM_GetITStatus(g_sTimPwmIn_Switch.Tim, TIM_IT_CC2) != RESET)	
	{	
		if(g_psReceiverAnaly->Switch.SWB.CAP_EDGE == CAP_EDGE_FALLING)	/*本次捕获到是下降沿 */
		{	  			
			g_psReceiverAnaly->Switch.SWB.CAP_EDGE = CAP_EDGE_RISING;	/*标记下次捕获到的边沿状态为上升沿*/
					
			/*保存下降沿时刻计数*/
			g_psReceiverAnaly->Switch.SWB.fallingEdgeTicks = TIM_GetCapture2(g_sTimPwmIn_Switch.Tim);
			
			if (g_psReceiverAnaly->Switch.SWB.fallingEdgeTicks > \
				g_psReceiverAnaly->Switch.SWB.risingEdgeTicks)
			{
				/*得到一个PWM周期的有效计数*/
				g_psReceiverAnaly->Switch.SWB.avaTicks = g_psReceiverAnaly->Switch.SWB.fallingEdgeTicks - \
													     g_psReceiverAnaly->Switch.SWB.risingEdgeTicks;
				
				/*pwm数据状态更新*/
				g_psReceiverAnaly->Switch.SWB.UPDATE_STATUS = WAVE_STATUS_NEW;					
			}
			
	   		TIM_OC2PolarityConfig(g_sTimPwmIn_Switch.Tim, TIM_ICPolarity_Rising); /*设置为上升沿捕获*/
		}
		else  						/*本次捕获到是上升沿 */
		{
			/*保存上升沿时刻计数*/
 			g_psReceiverAnaly->Switch.SWB.risingEdgeTicks = TIM_GetCapture2(g_sTimPwmIn_Switch.Tim);
			
			g_psReceiverAnaly->Switch.SWB.CAP_EDGE = CAP_EDGE_FALLING;	/*标记下次捕获到的边沿状态为下降沿*/
			
	   		TIM_OC2PolarityConfig(g_sTimPwmIn_Switch.Tim, TIM_ICPolarity_Falling);		/*CC1P=1 设置为下降沿捕获*/
		}
		
		TIM_ClearITPendingBit(g_sTimPwmIn_Switch.Tim, TIM_IT_CC2);	/*清除中断标志位*/	    
	}

	/*捕获CH3发生捕获事件*/
	if (TIM_GetITStatus(g_sTimPwmIn_Switch.Tim, TIM_IT_CC3) != RESET)	
	{	
		if(g_psReceiverAnaly->Switch.SWC.CAP_EDGE == CAP_EDGE_FALLING)	/*本次捕获到是下降沿 */
		{	  			
			g_psReceiverAnaly->Switch.SWC.CAP_EDGE = CAP_EDGE_RISING;	/*标记下次捕获到的边沿状态为上升沿*/
					
			/*保存下降沿时刻计数*/
			g_psReceiverAnaly->Switch.SWC.fallingEdgeTicks = TIM_GetCapture3(g_sTimPwmIn_Switch.Tim);
			
			if (g_psReceiverAnaly->Switch.SWC.fallingEdgeTicks > \
				g_psReceiverAnaly->Switch.SWC.risingEdgeTicks)
			{
				/*得到一个PWM周期的高电平计数*/
				g_psReceiverAnaly->Switch.SWC.avaTicks = g_psReceiverAnaly->Switch.SWC.fallingEdgeTicks - \
														 g_psReceiverAnaly->Switch.SWC.risingEdgeTicks;
				
				/*pwm数据状态更新*/
				g_psReceiverAnaly->Switch.SWC.UPDATE_STATUS = WAVE_STATUS_NEW;
			}
			
	   		TIM_OC3PolarityConfig(g_sTimPwmIn_Switch.Tim, TIM_ICPolarity_Rising); /*设置为上升沿捕获*/
		}
		else  						/*本次捕获到是上升沿 */
		{
			/*保存上升沿时刻计数*/
 			g_psReceiverAnaly->Switch.SWC.risingEdgeTicks = TIM_GetCapture3(g_sTimPwmIn_Switch.Tim);
			
			g_psReceiverAnaly->Switch.SWC.CAP_EDGE = CAP_EDGE_FALLING;	/*标记下次捕获到的边沿状态为下降沿*/
			
	   		TIM_OC3PolarityConfig(g_sTimPwmIn_Switch.Tim, TIM_ICPolarity_Falling);		/*CC1P=1 设置为下降沿捕获*/
		}
		
		TIM_ClearITPendingBit(g_sTimPwmIn_Switch.Tim, TIM_IT_CC3);	/*清除中断标志位*/	  	    
	}	
	
	/*捕获CH4发生捕获事件*/
	if (TIM_GetITStatus(g_sTimPwmIn_Switch.Tim, TIM_IT_CC4) != RESET)	
	{	
		if(g_psReceiverAnaly->Switch.SWD.CAP_EDGE == CAP_EDGE_FALLING)	/*本次捕获到是下降沿 */
		{	  			
			g_psReceiverAnaly->Switch.SWD.CAP_EDGE = CAP_EDGE_RISING;	/*标记下次捕获到的边沿状态为上升沿*/
					
			/*保存下降沿时刻计数*/
			g_psReceiverAnaly->Switch.SWD.fallingEdgeTicks = TIM_GetCapture4(g_sTimPwmIn_Switch.Tim);
			
			if (g_psReceiverAnaly->Switch.SWD.fallingEdgeTicks > \
				g_psReceiverAnaly->Switch.SWD.risingEdgeTicks)
			{
				/*得到一个PWM周期的有效计数*/
				g_psReceiverAnaly->Switch.SWD.avaTicks = g_psReceiverAnaly->Switch.SWD.fallingEdgeTicks - \
													     g_psReceiverAnaly->Switch.SWD.risingEdgeTicks;

				/*pwm数据状态更新*/
				g_psReceiverAnaly->Switch.SWD.UPDATE_STATUS = WAVE_STATUS_NEW;				
			}
			
	   		TIM_OC4PolarityConfig(g_sTimPwmIn_Switch.Tim, TIM_ICPolarity_Rising); /*设置为上升沿捕获*/
		}
		else  						/*本次捕获到是上升沿 */
		{
			/*保存上升沿时刻计数*/
 			g_psReceiverAnaly->Switch.SWD.risingEdgeTicks = TIM_GetCapture4(g_sTimPwmIn_Switch.Tim);
			
			g_psReceiverAnaly->Switch.SWD.CAP_EDGE = CAP_EDGE_FALLING;	/*标记下次捕获到的边沿状态为下降沿*/
			
	   		TIM_OC4PolarityConfig(g_sTimPwmIn_Switch.Tim, TIM_ICPolarity_Falling);		/*CC1P=1 设置为下降沿捕获*/
		}
		
		TIM_ClearITPendingBit(g_sTimPwmIn_Switch.Tim, TIM_IT_CC4);	/*清除中断标志位*/	    
	}

	/*定时器计数溢出中断*/
	if (TIM_GetITStatus(g_sTimPwmIn_Switch.Tim, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(g_sTimPwmIn_Switch.Tim, TIM_IT_Update);	/*清除中断标志位*/	  	    
	}	
}

/*万向节(TIM1)捕获+解析*/
void remot_Gimbal_Channel_Analy(void)
{
	
}
#endif

#if defined(REMOTE_DATA_RECV__PPM)
Remot_PPM_Recv_Tick g_sRemot_PPM_Recv_Tick = {0};
/*=== MCU PPM解析 ===*/
void remot_PPM_AllChannel_Analy(void)
{
	/*系统运行时间获取,us*/
	g_sRemot_PPM_Recv_Tick.lastTick = g_sRemot_PPM_Recv_Tick.curTick;
	
	/*计算当前时刻us*/
	g_sRemot_PPM_Recv_Tick.curTick = (g_sTimExecutePeriod.Period * \
									  g_vu32_Time_Period_Cnt_Foc_Ms) + \
									  g_sTimExecutePeriod.Tim->CNT; /*us*/
	
	/*计算时间差*/
	g_sRemot_PPM_Recv_Tick.deltaTick = g_sRemot_PPM_Recv_Tick.curTick - g_sRemot_PPM_Recv_Tick.lastTick;
	
	if (g_psReceiverAnaly->PPM_RECV_STATUS == REMOT_PPM_RECV_START)
	{
		/*有效持续周期*/
		if ((800 <= g_sRemot_PPM_Recv_Tick.deltaTick) && \
			(g_sRemot_PPM_Recv_Tick.deltaTick <= 2200))
		{
			g_psReceiverAnaly->PPM_Buff[g_psReceiverAnaly->PPM_SAMPLE_CHANNLE] = g_sRemot_PPM_Recv_Tick.deltaTick;			
			g_psReceiverAnaly->PPM_SAMPLE_CHANNLE++;
			
			if (g_psReceiverAnaly->PPM_SAMPLE_CHANNLE >= REMOT_PPM_RECV_MAX_CHANNEL_NBR)
			{
				/*各通道数据*/	
				/*attitude*/
				g_psReceiverAnaly->Attitude.Roll.avaTicks     = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_1ST_CNL];
				g_psReceiverAnaly->Attitude.Pitch.avaTicks    = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_2ND_CNL];
				g_psReceiverAnaly->Attitude.Throttle.avaTicks = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_3RD_CNL];
				g_psReceiverAnaly->Attitude.Yaw.avaTicks      = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_4TH_CNL];	
				
				/*switch*/
				g_psReceiverAnaly->Switch.SWA.avaTicks 		  = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_5TH_CNL];
				g_psReceiverAnaly->Switch.SWB.avaTicks 		  = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_6TH_CNL];
				g_psReceiverAnaly->Switch.SWC.avaTicks 		  = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_7TH_CNL];
				g_psReceiverAnaly->Switch.SWD.avaTicks 		  = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_8TH_CNL];				
				
				/*标记数据更新状态*/
				/*attitude*/
				g_psReceiverAnaly->Attitude.Roll.UPDATE_STATUS     = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Attitude.Pitch.UPDATE_STATUS    = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Attitude.Throttle.UPDATE_STATUS = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Attitude.Yaw.UPDATE_STATUS      = WAVE_STATUS_NEW;

				/*switch*/
				g_psReceiverAnaly->Switch.SWA.UPDATE_STATUS		   = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Switch.SWB.UPDATE_STATUS 	   = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Switch.SWC.UPDATE_STATUS		   = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Switch.SWD.UPDATE_STATUS 	   = WAVE_STATUS_NEW;			
				
				/*标记PPM接收完成*/
				g_psReceiverAnaly->PPM_RECV_STATUS = REMOT_PPM_RECV_FINISH;
			}
		}
		else
		{
			if (g_sRemot_PPM_Recv_Tick.deltaTick >= 2500) /*帧结束电平至少2ms = 2000us*/
			{
				/*各通道数据*/	
				/*attitude*/
				g_psReceiverAnaly->Attitude.Roll.avaTicks     = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_1ST_CNL];
				g_psReceiverAnaly->Attitude.Pitch.avaTicks    = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_2ND_CNL];
				g_psReceiverAnaly->Attitude.Throttle.avaTicks = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_3RD_CNL];
				g_psReceiverAnaly->Attitude.Yaw.avaTicks      = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_4TH_CNL];	
				
				/*switch*/
				g_psReceiverAnaly->Switch.SWA.avaTicks 		  = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_5TH_CNL];
				g_psReceiverAnaly->Switch.SWB.avaTicks 		  = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_6TH_CNL];
				g_psReceiverAnaly->Switch.SWC.avaTicks 		  = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_7TH_CNL];
				g_psReceiverAnaly->Switch.SWD.avaTicks 		  = g_psReceiverAnaly->PPM_Buff[REMOT_PPM_SAMPLE_8TH_CNL];
				
				/*标记数据更新状态*/
				/*attitude*/
				g_psReceiverAnaly->Attitude.Roll.UPDATE_STATUS     = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Attitude.Pitch.UPDATE_STATUS    = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Attitude.Throttle.UPDATE_STATUS = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Attitude.Yaw.UPDATE_STATUS      = WAVE_STATUS_NEW;

				/*switch*/
				g_psReceiverAnaly->Switch.SWA.UPDATE_STATUS		   = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Switch.SWB.UPDATE_STATUS 	   = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Switch.SWC.UPDATE_STATUS		   = WAVE_STATUS_NEW;
				g_psReceiverAnaly->Switch.SWD.UPDATE_STATUS 	   = WAVE_STATUS_NEW;			
				
				/*标记PPM接收完成*/
				g_psReceiverAnaly->PPM_RECV_STATUS    = REMOT_PPM_RECV_START;
				g_psReceiverAnaly->PPM_SAMPLE_CHANNLE = REMOT_PPM_SAMPLE_1ST_CNL;
			}
			else
			{
				g_psReceiverAnaly->PPM_RECV_STATUS = REMOT_PPM_RECV_FINISH;
			}
		}
	}
	else if (g_sRemot_PPM_Recv_Tick.deltaTick >= 2500) /*帧结束电平至少2ms = 2000us*/
	{
		g_psReceiverAnaly->PPM_RECV_STATUS    = REMOT_PPM_RECV_START;
		g_psReceiverAnaly->PPM_SAMPLE_CHANNLE = REMOT_PPM_SAMPLE_1ST_CNL;
	}
}
#endif
