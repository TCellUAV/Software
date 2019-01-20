#include "control_AltHold.h"
#include "control_Aircraft.h"

/*获取油门相对中位死区推动情况*/
CONTROL_THROTTLE_PUSH altitude_Get_Throttle_Relative_MidDead(ControlThrottlePush *controlThrottlePush)
{
	/*记录上次油门状态*/
	controlThrottlePush->LAST_STATUS = controlThrottlePush->CUR_STATUS;
	
	/*单次油门状态判断*/
	if (g_psControlAircraft->throttleUpDownJudgeValue <= REMOT_THROTTLE_DOWN_CROSSOVER)	/*油门值在死区之下*/
	{
		controlThrottlePush->CUR_STATUS = CONTROL_THROTTLE_PUSH_LOCATE_DOWN_MIDDEAD;
	}
	else if (g_psControlAircraft->throttleUpDownJudgeValue >= REMOT_THROTTLE_UP_CROSSOVER) /*油门值在死区之上*/
	{
		controlThrottlePush->CUR_STATUS = CONTROL_THROTTLE_PUSH_LOCATE_UP_MIDDEAD;		
	}
	else	/*油门值在死区之间*/
	{
		controlThrottlePush->CUR_STATUS = CONTROL_THROTTLE_PUSH_LOCATE_MIDDEAD;		
	}
	
	/*本和上次油门状态判断*/	
	if ((controlThrottlePush->LAST_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_DOWN_MIDDEAD) && \
		(controlThrottlePush->CUR_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_UP_MIDDEAD)) 	 	/*向上过死*/
	{
		controlThrottlePush->REAL_STATUS = CONTROL_THROTTLE_PUSH_UP_OVER_MIDDEAD;
	}
	else if ((controlThrottlePush->LAST_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_UP_MIDDEAD) && \
			 (controlThrottlePush->CUR_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_DOWN_MIDDEAD)) /*向下过死*/
	{
		controlThrottlePush->REAL_STATUS = CONTROL_THROTTLE_PUSH_DOWM_OVER_MIDDEAD;	
	}
	else if ((controlThrottlePush->LAST_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_DOWN_MIDDEAD) && \
			 (controlThrottlePush->CUR_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_MIDDEAD)) /*向上到中*/
	{
		controlThrottlePush->REAL_STATUS = CONTROL_THROTTLE_PUSH_UP_TO_MIDDEAD;	
	}
	else if ((controlThrottlePush->LAST_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_UP_MIDDEAD) && \
			 (controlThrottlePush->CUR_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_MIDDEAD)) /*向下到中*/
	{
		controlThrottlePush->REAL_STATUS = CONTROL_THROTTLE_PUSH_DOWM_TO_MIDDEAD;	
	}
	else if ((controlThrottlePush->LAST_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_UP_MIDDEAD) && \
			 (controlThrottlePush->CUR_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_UP_MIDDEAD)) /*暂位于上位区*/
	{
		controlThrottlePush->REAL_STATUS = CONTROL_THROTTLE_PUSH_UPDOWN_UPBAND;	
	}
	else if ((controlThrottlePush->LAST_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_DOWN_MIDDEAD) && \
			 (controlThrottlePush->CUR_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_DOWN_MIDDEAD)) /*暂位于下位区*/
	{
		controlThrottlePush->REAL_STATUS = CONTROL_THROTTLE_PUSH_UPDOWN_DOWMBAND;	
	}	
	else if ((controlThrottlePush->LAST_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_MIDDEAD) && \
			 (controlThrottlePush->CUR_STATUS == CONTROL_THROTTLE_PUSH_LOCATE_MIDDEAD)) /*暂位于中位区*/
	{
		controlThrottlePush->REAL_STATUS = CONTROL_THROTTLE_PUSH_UPDOWN_DEADBAND;	
	}	
	
	return (controlThrottlePush->REAL_STATUS);
}

vu16 g_vu16HighPosControlTicks         = 0; /*高度位置控制计数器*/
vu16 g_vu16HighSpeedControlTicks       = 0; /*高度速度控制计数器*/

fp32 g_HeightYawSpeedFeedforwardOutput = 0.0f;   /*竖直速度前馈控制器输出*/
fp32 g_HeightYawSpeedFeedforwardRate   = 1.0f;   /*竖直速度前馈控制器,APM里面为1、0.45*/
fp32 g_HeightYawSpeedFeedforwardDelta  = 0.0f;   /*竖直期望速度变化率*/
fp32 g_CurHeightYawSpeedExpect         = 0.0f;   /*本次竖直速度期望*/
fp32 g_LastHeightYawSpeedExpect        = 0.0f;	 /*上次竖直速度期望*/

/*竖直高度(定高)控制器*/
void vertical_Control_AltHold(fp32 controlDeltaT)
{
	/*悬停油门设置为默认悬停油门*/
	#if (CTRL_THROTTLE_HOVER_ENABLE_STATUS == SYS_ENABLE)
	g_psControlAircraft->heightHoldThrottle = CTRL_THROTTLE_HOVER_DEFAULT_VALUE;
	#endif
	
	/*高度控制器第1步*/
	/*  **
	   ***
	  ****
	    **
		**
		**
		**   
	  *******	
	  *******/
	/************************** 竖直高度控制器 开始 ********************************/
	/****** 定高：高度位置环+速度环+加速度环，控制周期分别为10ms、5ms、5ms ********/
	/************************** 竖直位置控制器 开始 ********************************/
	/*油门摇杆位于中间位置死区内(既不是上推,也不是下推)*/
	if ((g_psControlAircraft->throttleUpDownJudgeValue >= REMOT_THROTTLE_DOWN_CROSSOVER) && \
		(g_psControlAircraft->throttleUpDownJudgeValue <= REMOT_THROTTLE_UP_CROSSOVER)) /*400 <= && 600 <=*/
	{
		/*高度位置环输出给定速度期望*/
		if (g_psPidSystem->HighPosition.expect == 0) /*油门上/下拉时,竖直位置期望为0,油门回中后,只设置一次竖直位置期望*/
		{
			g_psPidSystem->HighPosition.expect = g_psSinsReal->curPosition[EARTH_FRAME_Z]; /*油门回中后,更新位置(高度)期望*/
		}
		
		/*ticks++*/
		g_vu16HighPosControlTicks++;	/*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS 执行一次*/
		
		/*竖直高度控制周期2*5=10ms*/
		if (g_vu16HighPosControlTicks >= CTRL_VERTICAL_POSITION_CONTROL_PERIOD)
		{
			/*ticks清0*/
			g_vu16HighPosControlTicks = 0;
			
			/*更新高度位置反馈*/
			g_psPidSystem->HighPosition.feedback = g_psSinsReal->curPosition[EARTH_FRAME_Z];
			
			/*外环 竖直高度位置PID运算及输出*/
			g_psPidSystem->HighPosition.controlOutput = pid_Control_General_Dp(&g_psPidSystem->HighPosition); 
	
			/*是否启用速度前馈控制器*/
			#ifdef CTRL_HEIGHT_YAW_SPEED_FEEDFORWARD 
			g_CurHeightYawSpeedExpect         = g_psPidSystem->HighPosition.controlOutput; /*本次竖直速度期望*/
			g_HeightYawSpeedFeedforwardDelta  = (g_CurHeightYawSpeedExpect - g_LastHeightYawSpeedExpect) / (2 * controlDeltaT); /*速度期望变化率*/
			g_HeightYawSpeedFeedforwardOutput = g_HeightYawSpeedFeedforwardRate * g_HeightYawSpeedFeedforwardDelta; /*竖直速度前馈控制器输出*/
			g_LastHeightYawSpeedExpect        = g_CurHeightYawSpeedExpect; /*竖直速度期望推迭*/
			#endif
			
			/*更新 内环 竖直速度期望*/
			g_psPidSystem->HighSpeed.expect = g_psPidSystem->HighPosition.controlOutput; /*等于外环控制输出*/
		}
		
		/*飞行器竖直方向运动趋势保持中立*/
		g_psControlAircraft->AIRCRAFT_VER_MOVE_TREND = CTRL_AIRCRAFT_MOVE_VER_HOLD;
	}
	/*油门摇杆位于中间位置死区之上(上推),给定上升速度期望*/
	else if (g_psControlAircraft->throttleUpDownJudgeValue > REMOT_THROTTLE_UP_CROSSOVER) /*> 600*/
	{
		/*油门杆上推,直接给定竖直速度期望*/
		g_psPidSystem->HighSpeed.expect = CTRL_HEIGHT_CLIMB_UP_MAX_SPEED * \
										  (g_psControlAircraft->throttleUpDownJudgeValue - REMOT_THROTTLE_UP_CROSSOVER) / \
										  (CTRL_THROTTLE_TOP_VALUE - REMOT_THROTTLE_UP_CROSSOVER);
		
		g_psPidSystem->HighPosition.expect = 0; /*竖直位置环期望置0*/
		
		/*飞行器竖直方向运动趋势向上*/
		g_psControlAircraft->AIRCRAFT_VER_MOVE_TREND = CTRL_AIRCRAFT_MOVE_VER_UP;
	}
	/*油门摇杆位于中间位置死区之下(下推),给定下降速度期望*/
	else if (g_psControlAircraft->throttleUpDownJudgeValue < REMOT_THROTTLE_DOWN_CROSSOVER) /*< 400*/
	{
		/*油门杆上推,直接给定竖直速度期望*/
		g_psPidSystem->HighSpeed.expect = CTRL_HEIGHT_CLIMB_DOWN_MAX_SPEED * \
										  (g_psControlAircraft->throttleUpDownJudgeValue - REMOT_THROTTLE_DOWN_CROSSOVER) / \
										  (REMOT_THROTTLE_DOWN_CROSSOVER - CTRL_THROTTLE_BOTTOM_VALUE);
		
		g_psPidSystem->HighPosition.expect = 0; /*竖直位置环期望置0*/
		
		/*飞行器竖直方向运动趋势向下*/
		g_psControlAircraft->AIRCRAFT_VER_MOVE_TREND = CTRL_AIRCRAFT_MOVE_VER_DOWN;		
	}
	/************************** 竖直位置控制器 结束 ********************************/		

	/*高度控制器第2步*/
	/**********
	 **********
		     **
	 **********
	 **********
	 **
	 **********
	 ********** */		
	/************************** 竖直速度控制器 开始 ********************************/		
	/*ticks++*/
	g_vu16HighSpeedControlTicks++;
	
	/*竖直高度控制周期1*5=5ms*/
	if (g_vu16HighSpeedControlTicks >= CTRL_VERTICAL_SPEED_CONTROL_PERIOD) /*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS 执行一次*/
	{
		/*ticks清0*/
		g_vu16HighSpeedControlTicks = 0;
		
		/*更新竖直速度反馈,即竖直惯导速度*/
		g_psPidSystem->HighSpeed.feedback = g_psSinsReal->curSpeed[EARTH_FRAME_Z];
		
		/*竖直高度速度控制及输出*/
		g_psPidSystem->HighSpeed.controlOutput = pid_Control_Div_LPF(&g_psPidSystem->HighSpeed, PID_CONTROLER_HIGH_SPEED);  /*PID DIV控制低通滤波*/
	}
	
	/*上升下降过程中期望加速度限幅单独处理*/
	if (g_psControlAircraft->AIRCRAFT_VER_MOVE_TREND == CTRL_AIRCRAFT_MOVE_VER_UP)	/*上升*/
	{  
		g_psPidSystem->HighSpeed.controlOutput = math_Constrain(g_psPidSystem->HighSpeed.controlOutput, \
															    CTRL_HEIGHT_CLIMB_UP_MAX_ACCELERATION, -CTRL_HEIGHT_CLIMB_UP_MAX_ACCELERATION);
	}
	else if (g_psControlAircraft->AIRCRAFT_VER_MOVE_TREND == CTRL_AIRCRAFT_MOVE_VER_DOWN) /*下降*/
	{
		g_psPidSystem->HighSpeed.controlOutput = math_Constrain(g_psPidSystem->HighSpeed.controlOutput, \
															    CTRL_HEIGHT_CLIMB_DOWN_MAX_ACCELERATION, -CTRL_HEIGHT_CLIMB_DOWN_MAX_ACCELERATION);		
	}
	
	/*标记飞行器空中悬停状态*/
	/*竖直方向悬停*/
	if (g_psControlAircraft->AIRCRAFT_VER_MOVE_TREND == CTRL_AIRCRAFT_MOVE_VER_HOLD)
	{
		g_psUav_Status->AIRSTOP_TYPE |= UAV_AIRSTOP_ONLY_VERTICAL;
	}
	/*竖直方向未悬停*/
	else
	{
		g_psUav_Status->AIRSTOP_TYPE &= UAV_AIRSTOP_ONLY_HORIZONTAL;
	}
	
	/************************** 竖直速度控制器 结束 ********************************/
	/*高度控制器第3步*/
	/**********
	 **********
              ***
             ***
              ***
               ***
        ***********
	 ***********/
	/************************** 竖直加速度控制器 开始 ********************************/
	/*更新竖直加速度期望*/
	g_psPidSystem->HighAcc.expect = g_psPidSystem->HighSpeed.controlOutput; /*等于外环控制输出*/
	
	#ifdef CTRL_HEIGHT_YAW_SPEED_FEEDFORWARD /*是否启用速度前馈控制器*/
	g_psPidSystem->HighAcc.expect += g_HeightYawSpeedFeedforwardOutput; /*加上前馈控制器输出*/
	#endif
	
	/*更新竖直加速度反馈*/
	g_psPidSystem->HighAcc.feedback = g_psSinsFilterFeedback->curAcc[EARTH_FRAME_Z]; /*加速度原始数据校准+巴特沃斯滤波后的值作为加速度反馈值*/
	
	/*竖直加速度控制及输出*/
	g_psPidSystem->HighAcc.controlOutput = pid_Control_Err_LPF(&g_psPidSystem->HighAcc, PID_CONTROLER_HIGH_ACC);  /*PID ERR控制低通滤波*/
	/************************** 竖直加速度控制器 结束 ********************************/
	
	/*悬停油门 = 加速度环积分值 + 基准悬停油门
	  此时输出力 F = mg	(F即悬停油门产生的(拉)力)
  	  当需要输出a的加速度时，输出力 F1=m(g+a)
	  F1/F = 1 + a/g
	  因此此时应输出：悬停油门*(1 + a/g)*/
	
	/*根据需要达到的加速度来计算需要净增加的油门量*/
	g_psPidSystem->HighAcc.controlOutput += (g_psControlAircraft->heightHoldThrottle + g_psPidSystem->HighAcc.integrate - CTRL_THROTTLE_START_TURN) * \
										    (g_psPidSystem->HighAcc.expect / 980);
	
	/*是否 开启三环定高模式: 即竖直高度位置+竖直速度+竖直加速度:1开启,0关闭*/ 
	if (CTRL_HEIGHT_POS_CONTROL_ACC_STATUS == SYS_ENABLE)
	{
		/*油门来源于高度加速度控制器输出*/
		g_psControlAircraft->ctrlThrottle = (u16)(g_psControlAircraft->heightHoldThrottle + \
												  g_psPidSystem->HighAcc.controlOutput);
	}
	else /*两环定高模式:即竖直高度位置+竖直速度*/
	{
		/*油门来源于高度速度控制器输出*/
		g_psControlAircraft->ctrlThrottle = (u16)(g_psControlAircraft->heightHoldThrottle + \
												  g_psPidSystem->HighSpeed.controlOutput);		
	}
	
	/********************* 高度控制器结束,给定油门控制量 结束 ************************/		
}
