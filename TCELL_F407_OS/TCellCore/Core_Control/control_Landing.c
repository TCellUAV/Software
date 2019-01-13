#include "control_Landing.h"
#include "ahrs_Caculation.h"
#include "control_Aircraft.h"

/*着陆返航控制*/
ControlLand	g_sControlLand = 
{
	.verticalSpeedChangePeriod = 0,
};

ControlLand *g_psControlLand = &g_sControlLand;


vu16 g_vu16LandCheckContinueTicks = 0; /*飞行器检测着陆持续ticks*/

/*自检触地进入怠速模式*/
UAV_LAND_STATUS ctrl_Landing_Status_Check(Uav_Status *uavStatus)
{
	/*油门控制处于较低行程:
	  1.姿态模式下,油门杆处于低位
      2.定高模式下,期望速度向下,单加速度环反馈为角小值，
      3.加速度控制输出由于长时间积分,到负的较大值,使得油门控制较低*/
	
	/*记录上次飞行状态*/
	uavStatus->UavLandStatus.LastTime = uavStatus->UavLandStatus.ThisTime;
	
	/*1.当前油门输出,2.触地后无旋转(合角速度小于20deg/s),3.惯导竖直方向速度<±20cm/s,4.无水平手动操作*/
	if ((g_psControlAircraft->throttleOutput <= CTRL_LAND_CHECK_THROTTLE_THRESHOLD_MIN) && \
		(g_GyroLenth <= 20.0f) && \
		(math_Abs(g_psSinsReal->curSpeed[EARTH_FRAME_Z]) <= 20.0f) && \
		(ctrl_Go_Home_Horizontal_Hand_Meddle() == CTRL_GO_HOME_HAND_MEDDLE_FALSE))
	{
		g_vu16LandCheckContinueTicks++; /*5ms执行一次*/	
	}
	else
	{
		g_vu16LandCheckContinueTicks /= 2; /*快速削减*/		
	}
	
	/*检测Ticks计数状态*/
	if (g_vu16LandCheckContinueTicks >= 5000)
	{
		g_vu16LandCheckContinueTicks = 5000; /*防止溢出*/
	}
	
	if (g_vu16LandCheckContinueTicks >= 300) /*持续1.5S检测满足着陆标记*/
	{
		/*标记飞行器飞行状态为着陆状态*/		
		uavStatus->UavLandStatus.ThisTime = UAV_LAND_YES;
		
		/*允许一键起飞,禁止一键着陆*/
//		uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.ENABLE_STATUS = UAV_MISSION_ENABLE; 
//		uavStatus->UavCurrentFlyMission.Onekey_Mission.LandHome.ENABLE_STATUS       = UAV_MISSION_DISABLE;
		
		/*着陆后标记禁止 失联自动返航*/
		g_psControlAircraft->GO_HOME_STATUS = CTRL_AIRCRAFT_GO_HOME_DISABLE;
		
		/*着陆状态,禁止清除起飞任务*/
		if ((uavStatus->UavCurrentFlyMission.CURRENT_FLY_MISSION != UAV_FLY_MISSION_ONEKEY_FLY) && \
			(uavStatus->UavCurrentFlyMission.CURRENT_FLY_MISSION != UAV_FLY_MISSION_NULL))
		{
			control_fly_mission_clear(uavStatus, uavStatus->UavCurrentFlyMission.CURRENT_FLY_MISSION);
		}
	}
	else
	{
		/*标记飞行器飞行状态为飞行状态*/		
		uavStatus->UavLandStatus.ThisTime = UAV_LAND_NOT;	

		/*允许一键降落,禁止一键起飞*/
//		uavStatus->UavCurrentFlyMission.Onekey_Mission.LandHome.ENABLE_STATUS       = UAV_MISSION_ENABLE;		
//		uavStatus->UavCurrentFlyMission.Onekey_Mission.FixedHeightFly.ENABLE_STATUS = UAV_MISSION_DISABLE; 

		/*起飞后标记使能 失联自动返航*/
		g_psControlAircraft->GO_HOME_STATUS = CTRL_AIRCRAFT_GO_HOME_ENABLE;
	}
	
	return (uavStatus->UavLandStatus.ThisTime);	/*返回飞行器飞行状态*/
}


/*返航竖直高度控制*/
vu16 g_vu16LandVerticalPosControlTicks   = 0;	/*返航位置控制计数器*/
vu16 g_vu16LandVerticalSpeedControlTicks = 0;	/*返航速制控制计数器*/

void ctrl_Go_Home_Vertical_Control(fp32 verticalSpeedExpect, fp32 heightExpect, fp32 controlDeltaT)
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
	/*高度未达目标高度/手动期望高度*/
	if (heightExpect == 0)
	{	
		/*一直更新高度期望*/
		g_psPidSystem->HighPosition.expect = g_psSinsReal->curPosition[EARTH_FRAME_Z];
	}
	else
	{	
		/*更新高度期望*/
		g_psPidSystem->HighPosition.expect = heightExpect;
	}
	
	/************************** 竖直位置控制器 开始 ********************************/
	/*油门摇杆位于中间位置死区内(既不是上推,也不是下推)*/
	if ((g_psControlAircraft->throttleUpDownJudgeValue >= REMOT_THROTTLE_DOWN_CROSSOVER) && \
		(g_psControlAircraft->throttleUpDownJudgeValue <= REMOT_THROTTLE_UP_CROSSOVER))
	{
		/*ticks++*/
		g_vu16LandVerticalPosControlTicks++;	/*5ms执行一次*/
		
		/*竖直高度控制周期2*5=10ms*/
		if (g_vu16LandVerticalPosControlTicks >= CTRL_VERTICAL_POSITION_CONTROL_PERIOD)
		{
			/*ticks清0*/
			g_vu16LandVerticalPosControlTicks = 0;
			
			/*更新高度位置反馈*/
			g_psPidSystem->HighPosition.feedback = g_psSinsReal->curPosition[EARTH_FRAME_Z];
			
			/*外环 竖直高度位置PID运算及输出*/
			g_psPidSystem->HighPosition.controlOutput = pid_Control_General_Dp(&g_psPidSystem->HighPosition);
			
			/*判断速度期望来源*/
			if (heightExpect == 0)
			{
				/*本次竖直速度期望来源于外部直接给定*/
				g_CurHeightYawSpeedExpect = verticalSpeedExpect;
			}
			else
			{
				 /*本次竖直速度期望来源于位置环输出*/
				g_CurHeightYawSpeedExpect = g_psPidSystem->HighPosition.controlOutput;
			}
			
			#ifdef CTRL_HEIGHT_YAW_SPEED_FEEDFORWARD /*是否启用速度前馈控制器*/
			g_HeightYawSpeedFeedforwardDelta  = (g_CurHeightYawSpeedExpect - g_LastHeightYawSpeedExpect) / (2 * controlDeltaT); /*速度期望变化率*/
			g_HeightYawSpeedFeedforwardOutput = g_HeightYawSpeedFeedforwardRate * g_HeightYawSpeedFeedforwardDelta; /*竖直速度前馈控制器输出*/
			g_LastHeightYawSpeedExpect        = g_CurHeightYawSpeedExpect; /*竖直速度期望推迭*/
			#endif
			
			/*更新 内环 竖直速度期望*/
			g_psPidSystem->HighSpeed.expect = g_CurHeightYawSpeedExpect;
		}
		
		/*飞行器竖直方向运动趋势保持中立*/
		g_psControlAircraft->AIRCRAFT_VER_MOVE_TREND = CTRL_AIRCRAFT_MOVE_VER_HOLD;
	}
	/*油门摇杆位于中间位置死区之上(上推),给定上升速度期望*/
	else if (g_psControlAircraft->throttleUpDownJudgeValue > REMOT_THROTTLE_UP_CROSSOVER)
	{
		/*油门杆上推,直接给定竖直速度期望*/
		g_psPidSystem->HighSpeed.expect = CTRL_HEIGHT_CLIMB_UP_MAX_SPEED * \
										  (g_psControlAircraft->throttleUpDownJudgeValue - REMOT_THROTTLE_UP_CROSSOVER) / \
										  (CTRL_THROTTLE_TOP_VALUE - REMOT_THROTTLE_UP_CROSSOVER);
		
		/*飞行器竖直方向运动趋势向上*/
		g_psControlAircraft->AIRCRAFT_VER_MOVE_TREND = CTRL_AIRCRAFT_MOVE_VER_UP;
	}
	/*油门摇杆位于中间位置死区之下(下推),给定下降速度期望*/
	else if (g_psControlAircraft->throttleUpDownJudgeValue < REMOT_THROTTLE_DOWN_CROSSOVER)
	{
		/*油门杆上推,直接给定竖直速度期望*/
		g_psPidSystem->HighSpeed.expect = CTRL_HEIGHT_CLIMB_DOWN_MAX_SPEED * \
										  (g_psControlAircraft->throttleUpDownJudgeValue - REMOT_THROTTLE_DOWN_CROSSOVER) / \
										  (REMOT_THROTTLE_DOWN_CROSSOVER - CTRL_THROTTLE_BOTTOM_VALUE);
		
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
	g_vu16LandVerticalSpeedControlTicks++;
	
	/*竖直高度控制周期1*5=5ms*/
	if (g_vu16LandVerticalSpeedControlTicks >= CTRL_VERTICAL_SPEED_CONTROL_PERIOD) /*5ms执行一次*/
	{
		/*ticks清0*/
		g_vu16LandVerticalSpeedControlTicks = 0;
		
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
	
	/************************** 竖直速度控制器 结束 ********************************/
	/*高度控制器第3步*/
	/************
	 ************
              ***
             ***
              ***
               ***
        ***********
	 **************/
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

/*返航水平位置控制*/
vu16 g_vu16LandHorizontalPosControlTicks   = 0;	/*返航位置控制计数器*/
vu16 g_vu16LandHorizontalSpeedControlTicks = 0;	/*返航速度控制计数器*/

void ctrl_Go_Home_Horizontal_Control(Vector2s_Nav targPos, Vector2s_Nav curPos, CTRL_GO_HOME_DISTANCE HOME_DISTANCE)
{
	/*得到相对目标点N、E方向偏移,即期望位置偏移*/
	g_psControlLand->RelativeHomeDistance = land_Gps_Offset_Relative_To_Home(targPos, curPos);
	
	if (g_psUav_Status->UavLandStatus.ThisTime == UAV_LAND_NOT)
	{
		if (status_GPS_Fix_Ava_Check(g_psUav_Status) == UAV_SENMOD_USE_CONTROL_ALLOW)
		{
			/************************** 水平位置控制器 开始 ********************************/
			/*遥控居中,水平方向无遥控给定期望角度*/
			if ((g_psControlAircraft->RemotExpectAngle.pitch == 0) && \
				(g_psControlAircraft->RemotExpectAngle.roll == 0))
			{
				/*ticks++*/
				g_vu16LandHorizontalPosControlTicks++; /*5ms执行一次*/
				
				/************************** 水平位置控制器 开始 ********************************/
				if (g_vu16LandHorizontalPosControlTicks >= CTRL_GPS_HORIZONTAL_POSITION_CONTROL_PERIOD) /*5ms执行一次,20ms*/
				{
					/*ticks清0*/
					g_vu16LandHorizontalPosControlTicks = 0;
					
					/*位置期望,经纬、航行速度、高度*/
					g_psPidSystem->LatitudePosition.expect  = g_psSinsReal->curPosition[EARTH_FRAME_Y];
					g_psPidSystem->LongitudePosition.expect = g_psSinsReal->curPosition[EARTH_FRAME_X];
					
					/*导航坐标系下E、N方向上位置偏差*/
					g_psAttitudeAll->EarthFramePosError.north = g_psControlLand->RelativeHomeDistance.north - g_psSinsReal->curPosition[EARTH_FRAME_Y];
					g_psAttitudeAll->EarthFramePosError.east  = g_psControlLand->RelativeHomeDistance.east - g_psSinsReal->curPosition[EARTH_FRAME_X];
					
					/*导航坐标系下机体Pitch、Roll方向上位置偏差*/
					g_psAttitudeAll->BodyFramePosError.pitch = -g_psAttitudeAll->EarthFramePosError.east * SIN_YAW + \
																g_psAttitudeAll->EarthFramePosError.north * COS_YAW;
			
					g_psAttitudeAll->BodyFramePosError.roll  =  g_psAttitudeAll->EarthFramePosError.east * COS_YAW + \
																g_psAttitudeAll->EarthFramePosError.north * SIN_YAW;		
					
					/*根据当前点距离HOME点水平距离来决定当前水平速度*/
					if (HOME_DISTANCE == CTRL_GO_HOME_FARAWAY_DISTANCE)
					{
						g_psControlLand->horizontalSpeedExpect = CTRL_AUTO_GO_HOME_A2B_FIRST_NAV_SPEED_CM_S;
					}
					else if (HOME_DISTANCE == CTRL_GO_HOME_FARAWAY_DISTANCE)
					{
						g_psControlLand->horizontalSpeedExpect = CTRL_AUTO_GO_HOME_B2C_SECOND_NAV_SPEED_CM_S;					
					}
					else if (HOME_DISTANCE == CTRL_GO_HOME_FARAWAY_DISTANCE)
					{
						g_psControlLand->horizontalSpeedExpect = CTRL_AUTO_GO_HOME_NEARC_THIRD_NAV_SPEED_CM_S;						
					}
					else 
					{
						g_psControlLand->horizontalSpeedExpect = CTRL_AUTO_GO_HOME_DEFAULT_NAV_SPEED_CM_S;						
					}
				
					/*导航坐标系下机体Pitch、Roll,方向上期望刹车速度,这里为单比例运算不调用PID计算函数*/
					g_psAttitudeAll->BodyFrameBrakeSpeed.pitch = math_Constrain(g_psPidSystem->LatitudePosition.PID.kP * g_psAttitudeAll->BodyFramePosError.pitch, \
																				g_psControlLand->horizontalSpeedExpect, \
																			   -g_psControlLand->horizontalSpeedExpect); /*位置偏差限幅,单位cm*/
		
					g_psAttitudeAll->BodyFrameBrakeSpeed.roll  = math_Constrain(g_psPidSystem->LongitudePosition.PID.kP * g_psAttitudeAll->BodyFramePosError.roll, \
																				g_psControlLand->horizontalSpeedExpect, \
																			   -g_psControlLand->horizontalSpeedExpect); /*位置偏差限幅,单位cm*/
					
					/*更新水平方向Pitch、Roll速度控制器期望*/
					g_psPidSystem->LatitudeSpeed.expect  = g_psAttitudeAll->BodyFrameBrakeSpeed.pitch;  /*横纬-沿y轴方向(N向)*/					
					g_psPidSystem->LongitudeSpeed.expect = g_psAttitudeAll->BodyFrameBrakeSpeed.roll;   /*竖经-沿x轴方向(E向)*/			
						
					/******************** 水平位置控制器结束,给出期望刹车速度(期望速度) 结束 *********************/
				}			
		
				/************************** 水平速度控制器 开始 ********************************/				
				/*导航系的水平速度，转化到机体坐标系X-Y方向上*/
				/*沿载体Pitch、Roll方向水平速度控制*/
				g_vu16LandHorizontalSpeedControlTicks++; /*5ms执行一次*/
			
				if (g_vu16LandHorizontalSpeedControlTicks >= CTRL_GPS_HORIZONTAL_SPEED_CONTROL_PERIOD) /*10ms*/
				{
					/*ticks清0*/
					g_vu16LandHorizontalSpeedControlTicks = 0;
				
					/*机体系水平方向速度反馈量*/
					g_psAttitudeAll->BodyFrameSpeedFeedback.pitch = -g_psSinsReal->curSpeed[EARTH_FRAME_X] * SIN_YAW + \
																	 g_psSinsReal->curSpeed[EARTH_FRAME_Y] * COS_YAW;
				
					g_psAttitudeAll->BodyFrameSpeedFeedback.roll  =  g_psSinsReal->curSpeed[EARTH_FRAME_X] * COS_YAW + \
																     g_psSinsReal->curSpeed[EARTH_FRAME_Y] * SIN_YAW;					
				
					/*更新沿载体水平方向速度反馈量*/
					g_psPidSystem->LatitudeSpeed.feedback  = g_psAttitudeAll->BodyFrameSpeedFeedback.pitch; /*横纬-沿y轴方向(N向)*/		
					g_psPidSystem->LongitudeSpeed.feedback = g_psAttitudeAll->BodyFrameSpeedFeedback.roll;  /*竖经-沿x轴方向(E向)*/
				
					/*沿载体水平方向速度控制及输出*/
					g_psPidSystem->LatitudeSpeed.controlOutput  = pid_Control_Div_LPF(&g_psPidSystem->LatitudeSpeed, PID_CONTROLER_LATITUDE_SPEED);  /*PID DIV控制低通滤波*/		
					g_psPidSystem->LongitudeSpeed.controlOutput = pid_Control_Div_LPF(&g_psPidSystem->LongitudeSpeed, PID_CONTROLER_LONGITUDE_SPEED);  /*PID DIV控制低通滤波*/	
				
					/*水平方向期望加速度即水平速度控制环的输出*/
					g_sHorizontalExpectAcc.y = -g_psPidSystem->LatitudeSpeed.controlOutput;  /*机头pitch前后方向*/
					g_sHorizontalExpectAcc.x =  g_psPidSystem->LongitudeSpeed.controlOutput; /*与机头pitch垂直roll左右方向*/
					
					/*水平期望加速度与垂直的重力加速度计算出期望倾角*/
					horizontal_Acc_Convert_To_Dip_Angle(g_sHorizontalExpectAcc, &g_sHorizontalExpectAngle);
					
					/*更新角度环期望角(来源期望水平加速度和重力加速度换算出的期望倾角)*/
					g_psPidSystem->PitchAngle.expect = g_sHorizontalExpectAngle.y;	/*机头pitch前后方向*/
					g_psPidSystem->RollAngle.expect  = g_sHorizontalExpectAngle.x;	/*与机头pitch垂直roll左右方向*/			
				}
			
				/************************** 水平速度控制器 结束 ********************************/
			}
			/*拨动摇杆,只进行水平速度控制,无水平位置控制*/
			else if ((g_psControlAircraft->RemotExpectAngle.pitch != 0) || \
					 (g_psControlAircraft->RemotExpectAngle.roll != 0)) 
			{
				/*ticks++*/
				g_vu16LandHorizontalSpeedControlTicks++; /*5ms执行一次*/
					
				if (g_vu16LandHorizontalSpeedControlTicks >= CTRL_GPS_HORIZONTAL_SPEED_CONTROL_PERIOD) /*10ms*/
				{
					/*ticks清0*/
					g_vu16LandHorizontalSpeedControlTicks = 0;
				
					/*N向(沿PITCH方向)最大移动速度*/
					g_psPidSystem->LatitudeSpeed.expect  = g_psControlAircraft->RemotExpectAutoAngle.pitch * \
														   CTRL_HORIZONTAL_MAX_MOVE_SPEED; /*最大期望速度45*6/100=3.6m/s*/

					/*E向(沿ROLL方向)最大移动速度*/
					g_psPidSystem->LongitudeSpeed.expect = g_psControlAircraft->RemotExpectAutoAngle.roll * \
														   CTRL_HORIZONTAL_MAX_MOVE_SPEED; /*最大期望速度45*6/100=3.6m/s*/						
				
					/*导航系的水平速度，转化到机体坐标系X-Y方向上*/
					/*沿载体Pitch、Roll方向水平速度控制*/
					g_psAttitudeAll->BodyFrameSpeedFeedback.pitch = -g_psSinsReal->curSpeed[EARTH_FRAME_X] * SIN_YAW + \
																	 g_psSinsReal->curSpeed[EARTH_FRAME_Y] * COS_YAW;

					g_psAttitudeAll->BodyFrameSpeedFeedback.roll  =  g_psSinsReal->curSpeed[EARTH_FRAME_X] * COS_YAW + \
																	 g_psSinsReal->curSpeed[EARTH_FRAME_Y] * SIN_YAW;
		
					/*更新水平速度反馈*/
					g_psPidSystem->LatitudeSpeed.feedback  = g_psAttitudeAll->BodyFrameSpeedFeedback.pitch; /*横纬-沿y轴方向(N向)(沿pitch旋转方向不是绕)*/	
					g_psPidSystem->LongitudeSpeed.feedback = g_psAttitudeAll->BodyFrameSpeedFeedback.roll;  /*竖经-沿x轴方向(E向)(沿roll旋转方向不是绕)*/
				
					/*水平速度PID计算及输出 (水平控速)*/
					g_psPidSystem->LatitudeSpeed.controlOutput  = pid_Control_Div_LPF(&g_psPidSystem->LatitudeSpeed, PID_CONTROLER_LATITUDE_SPEED);    /*PID DIV控制低通滤波*/
					g_psPidSystem->LongitudeSpeed.controlOutput = pid_Control_Div_LPF(&g_psPidSystem->LongitudeSpeed, PID_CONTROLER_LONGITUDE_SPEED);  /*PID DIV控制低通滤波*/	
				
					/*水平方向期望加速度即水平速度控制环的输出*/
					g_sHorizontalExpectAcc.y = -g_psPidSystem->LatitudeSpeed.controlOutput;  /*机头pitch前后方向*/
					g_sHorizontalExpectAcc.x =  g_psPidSystem->LongitudeSpeed.controlOutput; /*与机头pitch垂直roll左右方向*/
				
					/*水平期望加速度与垂直的重力加速度计算出期望倾角*/
					horizontal_Acc_Convert_To_Dip_Angle(g_sHorizontalExpectAcc, &g_sHorizontalExpectAngle);
				
					/*更新角度环期望角(来源期望水平加速度和重力加速度换算出的期望倾角)*/
					g_psPidSystem->PitchAngle.expect = g_sHorizontalExpectAngle.y;	/*机头pitch前后方向*/
					g_psPidSystem->RollAngle.expect  = g_sHorizontalExpectAngle.x;	/*与机头pitch垂直roll左右方向*/				
				}
			}
		}
	}
	else if (g_psUav_Status->UavLandStatus.ThisTime == UAV_LAND_YES)		
	{
		/*水平期望姿态角为0*/
		g_psPidSystem->PitchAngle.expect = 0;
		g_psPidSystem->RollAngle.expect  = 0;
		
		/*起飞前清空积分控制器*/
		pid_Horizontal_Takeoff_Integrate_Reset();		
	}
}


/*返航控制*/
void ctrl_Go_Home_Control(fp32 controlDeltaT)
{
	/*GPS可用且HOME已设置,且处于定点模式,执行返航*/
	if ((status_GPS_Fix_Ava_Check(g_psUav_Status) == UAV_SENMOD_USE_CONTROL_ALLOW) && \
		(g_psUav_Status->UavControlMode.Horizontal.CONTROL_MODE == UAV_HORIZONTAL_CONTROL_FIX_POS))
	{
		/*GPS参与返航*/
		ctrl_Go_Home_GPS_Control(controlDeltaT);
	}
	/*GPS无效时,执行降落*/
	else
	{
		/*普通原地着陆控制*/
		ctrl_Land_Ground_Control(controlDeltaT, g_psUav_Status);
	}
}

/*检测返航时手动干预高度*/
CTRL_GO_HOME_HAND_MEDDLE_STATUS ctrl_Go_Home_Vertical_Hand_Meddle(void)
{
	CTRL_GO_HOME_HAND_MEDDLE_STATUS verticalStatus = CTRL_GO_HOME_HAND_MEDDLE_FALSE;
	
	/*判断是否有竖直给定量*/
	if ((g_psControlAircraft->throttleUpDownJudgeValue <= REMOT_THROTTLE_DOWN_CROSSOVER) || \
		(g_psControlAircraft->throttleUpDownJudgeValue >= REMOT_THROTTLE_UP_CROSSOVER))
	{
		verticalStatus = CTRL_GO_HOME_HAND_MEDDLE_TRUE;
	}	
	
	return verticalStatus;	
}

/*检测返航时手动干预水平*/
CTRL_GO_HOME_HAND_MEDDLE_STATUS ctrl_Go_Home_Horizontal_Hand_Meddle(void)
{
	CTRL_GO_HOME_HAND_MEDDLE_STATUS horizontalStatus = CTRL_GO_HOME_HAND_MEDDLE_FALSE;
	
	/*判断是否有水平给定量*/
	if ((g_psControlAircraft->RemotExpectAngle.roll != 0) || \
		(g_psControlAircraft->RemotExpectAngle.pitch != 0))
	{
		horizontalStatus = CTRL_GO_HOME_HAND_MEDDLE_TRUE;
	}	
	
	return horizontalStatus;
}

/*返航点状态更新*/
void ctrl_Go_Home_Status_Update(void)
{
	/*当前点距离HOME点直线距离*/
	g_psAttitudeAll->gpsRelativeHome = gps_Two_Pos_Segment_Distance(g_psAttitudeAll->HomePos.Coordinate_s4, g_psAttitudeAll->GpsData.Coordinate_s4) * 100.0f; /*cm*/
	
	/*更迭上次返航流程*/
	g_psControlLand->LAST_GO_HOME_PROCESS = g_psControlLand->CUR_GO_HOME_PROCESS;
	
	/*离HOME点水平距离超过 FARAWAY_DISTANCE 半径*/
	if (g_psAttitudeAll->gpsRelativeHome >= CTRL_AUTO_GO_HOME_FARAWAY_DISTANCE_CM)
	{
		/*远离HOME点,先攀升到安全高度,返航*/
		g_psControlLand->CUR_GO_HOME_PROCESS = CTRL_LAND_FARAWAY_CLIMB_SAFE_HEIGHT_GO_HOME;
		
		/*返航点距离标定:遥远*/
		g_psControlLand->HOME_DISTANCE = CTRL_GO_HOME_FARAWAY_DISTANCE;
	}
	/*离HOME点水平距离超过 NEAR 半径*/
	else if (g_psAttitudeAll->gpsRelativeHome >= CTRL_AUTO_GO_HOME_NEAR_DISTANCE_CM)
	{
		/*靠近HOME点,保持安全高度,返航*/
		g_psControlLand->CUR_GO_HOME_PROCESS = CTRL_LAND_NEAR_HOLD_HEIGHT_GO_HOME;			
		
		/*返航点距离标定:靠近*/
		g_psControlLand->HOME_DISTANCE = CTRL_GO_HOME_NEAR_DISTANCE;		
	}
	/*离HOME点水平距离小于 NEAR 半径,到达HOME点*/	
	else
	{
		/*HOME点附近(默认到达HOME点),降落*/
		g_psControlLand->CUR_GO_HOME_PROCESS = CTRL_LAND_ARRIVE_DESCEND_GO_HOME;

		/*返航点距离标定:到达*/
		g_psControlLand->HOME_DISTANCE = CTRL_GO_HOME_ARRIVE_DISTANCE;		
	}
	
	/*判断水平位置方向有无干预*/
	g_psControlLand->HORIZONTAL_HAND_MEDDLE_STATUS = ctrl_Go_Home_Horizontal_Hand_Meddle();	
	
	/*判断竖直高度方向有无干预*/
	g_psControlLand->VERTICAL_HAND_MEDDLE_STATUS = ctrl_Go_Home_Vertical_Hand_Meddle();
	
	/*水平控制模式更迭*/
	g_psControlLand->LAST_HORIZONTAL_CTRL_MODE = g_psControlLand->CUR_HORIZONTAL_CTRL_MODE;
	
	/*若在返航过程中,水平位置有手动干预*/
	if (g_psControlLand->HORIZONTAL_HAND_MEDDLE_STATUS == CTRL_GO_HOME_HAND_MEDDLE_TRUE)
	{
		/*将自动降落流程置为NULL,下一次循环会重新判断*/
		g_psControlLand->LAST_GO_HOME_PROCESS = CTRL_GO_HOME_NULL_PROCESS;
		
		/*将返航点距离HOME距离标定为无效,下一次循环会重新判断*/
		g_psControlLand->HOME_DISTANCE = CTRL_GO_HOME_NULL_DISTANCE;
		
		/*标记本次水平控制为手动模式*/
		g_psControlLand->CUR_HORIZONTAL_CTRL_MODE = CTRL_GO_HOME_HAND_CTRL;
		
		/*更新为第一次(无手动干涉/消除手动干涉后)切换返航模式,*/
		g_psControlLand->FIRST_SWITCH_STATUS = CTRL_GO_HOME_FIRST_SWITCH_TURE;

		/*手动干预后,标定安全高度未设置,等待重新设置安全高度*/
		g_psControlLand->SAFE_HEIGHT_STATUS = CTRL_GO_HOME_SAFE_HEIGHT_DISARRIVE;		
	}
	else
	{
		/*标记本次水平控制为自动模式*/
		g_psControlLand->CUR_HORIZONTAL_CTRL_MODE = CTRL_GO_HOME_AUTO_CTRL;
	}
	
	/*若在返航过程中,竖直高度有手动干预*/
	if (g_psControlLand->VERTICAL_HAND_MEDDLE_STATUS == CTRL_GO_HOME_HAND_MEDDLE_TRUE)
	{
		/*将自动降落流程置为NULL,下一次循环会重新判断*/		
		g_psControlLand->LAST_GO_HOME_PROCESS = CTRL_GO_HOME_NULL_PROCESS;	
		
		/*将返航点距离HOME距离标定为无效,下一次循环会重新判断*/
		g_psControlLand->HOME_DISTANCE = CTRL_GO_HOME_NULL_DISTANCE;		
		
		/*更新为第一次(无手动干涉/消除手动干涉后)切换返航模式,*/
		g_psControlLand->FIRST_SWITCH_STATUS = CTRL_GO_HOME_FIRST_SWITCH_TURE;
		
		/*手动干预后,标定安全高度未设置,等带重新设置安全高度*/
		g_psControlLand->SAFE_HEIGHT_STATUS = CTRL_GO_HOME_SAFE_HEIGHT_DISARRIVE;
	}	
}

/*返航控制GPS*/
void ctrl_Go_Home_GPS_Control(fp32 controlDeltaT)
{
	/*记录家位置坐标*/
	g_psControlLand->LandHomePosition.lat = g_psAttitudeAll->HomePos.Coordinate_s4.lat;
	g_psControlLand->LandHomePosition.lon = g_psAttitudeAll->HomePos.Coordinate_s4.lon;	
	
	/*更新当前位置坐标*/
	g_psControlLand->LandCurPosition.lat = g_psAttitudeAll->GpsData.Coordinate_s4.lat;
	g_psControlLand->LandCurPosition.lon = g_psAttitudeAll->GpsData.Coordinate_s4.lon;
	
	/*持续更新当前点与HOME点位置关系*/
	ctrl_Go_Home_Status_Update();
	
	/*未着地,有姿态控制参与*/
	if (g_psUav_Status->UavLandStatus.ThisTime != UAV_LAND_YES)
	{
		/*无水平打杆动作*/
		if (g_psControlLand->HORIZONTAL_HAND_MEDDLE_STATUS == CTRL_GO_HOME_HAND_MEDDLE_FALSE)
		{
			/*首次切自动返航,且远离HOME点时,先上升到设置的安全高度*/
			if ((g_psControlLand->CUR_GO_HOME_PROCESS == CTRL_LAND_FARAWAY_CLIMB_SAFE_HEIGHT_GO_HOME) && \
				(g_psControlLand->FIRST_SWITCH_STATUS == CTRL_GO_HOME_FIRST_SWITCH_TURE))
			{
				/*如果当前高度低于远距离返航高度,则持续上升,直到满足竖直安全高度*/
				if (g_psSinsReal->curPosition[EARTH_FRAME_Z] < CTRL_AUTO_GO_HOME_FARAWAY_DISTANCE_SAFE_HEIGHT_CM)
				{
					/*水平定点,竖直上升*/
					g_psControlLand->LandTargetPosition.lat = g_psAttitudeAll->GpsData.Coordinate_s4.lat;
					g_psControlLand->LandTargetPosition.lon = g_psAttitudeAll->GpsData.Coordinate_s4.lon;
					
					/*速度变化最大量,然后递减*/
					g_psControlLand->verticalCalimbSpeedRate = CTRL_AUTO_GO_HOME_FARAWAY_DISTANCE_CLIMB_CM_S;
					
					/*重置竖直攀升速度变化周期*/
					g_psControlLand->verticalSpeedChangePeriod = 0;
					
					/*切换时刻速度期望为0*/
					g_psControlLand->verticalSpeedExpect = 0;
					
					/*目标高度期望为0*/
					g_psControlLand->heightExpect = 0;
					
					/*标记未达安全高度*/
					g_psControlLand->SAFE_HEIGHT_STATUS = CTRL_GO_HOME_SAFE_HEIGHT_DISARRIVE;
				}
				/*当切返航[**瞬间**]的高度大于安全高度时,保持当前高度,执行返航*/
				else
				{
					/*水平向HOME点靠拢,高度保持安全高度*/
					g_psControlLand->LandTargetPosition.lat = g_psAttitudeAll->HomePos.Coordinate_s4.lat;
					g_psControlLand->LandTargetPosition.lon = g_psAttitudeAll->HomePos.Coordinate_s4.lon;
					
					/*切换时刻速度期望为0*/
					g_psControlLand->verticalSpeedExpect = 0;
					
					/*设置目标高度期望为安全高度*/
					g_psControlLand->heightExpect = g_psSinsReal->curPosition[EARTH_FRAME_Z];
					
					/*标记已达安全高度*/
					g_psControlLand->SAFE_HEIGHT_STATUS = CTRL_GO_HOME_SAFE_HEIGHT_ARRIVE;					
				}

				/*远距离起飞置安全高度/切换时处于安全高度继续返航已完成,下次不再设置*/
				g_psControlLand->FIRST_SWITCH_STATUS = CTRL_GO_HOME_FIRST_SWITCH_FALSE;
				
				/*确保会进入下一阶段,防止临界位置判断失败,无法进入下一阶段*/
				g_psControlLand->LAST_GO_HOME_PROCESS = CTRL_LAND_FARAWAY_CLIMB_SAFE_HEIGHT_GO_HOME;
			}
			/*持续处于遥远位置*/
			else if ((g_psControlLand->CUR_GO_HOME_PROCESS == CTRL_LAND_FARAWAY_CLIMB_SAFE_HEIGHT_GO_HOME) && \
					 (g_psControlLand->LAST_GO_HOME_PROCESS == CTRL_LAND_FARAWAY_CLIMB_SAFE_HEIGHT_GO_HOME))
			{
				/*飞行器当前时刻已飞至安全高度,但未做到达标记*/
				if ((g_psSinsReal->curPosition[EARTH_FRAME_Z] >= CTRL_AUTO_GO_HOME_FARAWAY_DISTANCE_SAFE_HEIGHT_CM) && \
					(g_psControlLand->SAFE_HEIGHT_STATUS == CTRL_GO_HOME_SAFE_HEIGHT_DISARRIVE))
				{
					/*标记已达安全高度*/
					g_psControlLand->SAFE_HEIGHT_STATUS = CTRL_GO_HOME_SAFE_HEIGHT_ARRIVE;

					/*设置高度爬升结束到水平位置靠拢的过渡时间(2s)*/
					g_psControlLand->climb2CloseTransitionTicks = CTRL_AUTO_GO_HOME_CLIMB_THEN_CLOSE_TRANSITION_TICKS;
					
					/*标记缓冲时区内的飞行状态未设置*/
					g_psControlLand->TRANSITION_SET_STATUS = CTRL_GO_GOME_TRANSITION_SET_NO;
				}
				/*飞行器离HOME点较远,未达到安全高度,任然处于保持原水平位置,继续上升*/
				else if (g_psControlLand->SAFE_HEIGHT_STATUS == CTRL_GO_HOME_SAFE_HEIGHT_DISARRIVE)
				{	
					/*速率变化周期ticks++*/
					g_psControlLand->verticalSpeedChangePeriod++;
					
					/*8 * 5ms = 40ms速度变化率改变一次*/
					if (g_psControlLand->verticalSpeedChangePeriod >= CTRL_AUTO_GO_HOME_VERTICAL_SPEED_CHANGE_MAX_PERIOD)
					{
						if (g_psControlLand->verticalCalimbSpeedRate >= 1)
						{
							g_psControlLand->verticalCalimbSpeedRate--;
						}
						else
						{
							g_psControlLand->verticalCalimbSpeedRate = 0;
						}
						
						g_psControlLand->verticalSpeedChangePeriod = 0;
					}
					
					/*攀升速度依次递增，避免油门控制量突变*/
					g_psControlLand->verticalSpeedExpect = CTRL_AUTO_GO_HOME_FARAWAY_DISTANCE_CLIMB_CM_S - \
														   g_psControlLand->verticalCalimbSpeedRate;
					
					/*目标高度期望为0*/
					g_psControlLand->heightExpect = 0;					
				}
				
				/*飞行器离HOME点较远,已达到安全高度*/
				if (g_psControlLand->SAFE_HEIGHT_STATUS == CTRL_GO_HOME_SAFE_HEIGHT_ARRIVE)
				{
					/*竖直高度到达安全高度,向目标点水平靠近的等待过渡时间,5ms减一次*/
					if (g_psControlLand->climb2CloseTransitionTicks >= 1)
					{
						g_psControlLand->climb2CloseTransitionTicks--;
					}
					else
					{
						g_psControlLand->climb2CloseTransitionTicks = 0;
					}
					
					/*过渡时间已结束,设置返航点*/
					if (g_psControlLand->climb2CloseTransitionTicks == 0)
					{
						/*水平向HOME点靠拢,高度保持安全高度*/
						g_psControlLand->LandTargetPosition.lat = g_psAttitudeAll->HomePos.Coordinate_s4.lat;
						g_psControlLand->LandTargetPosition.lon = g_psAttitudeAll->HomePos.Coordinate_s4.lon;
						
						/*设置竖直爬升速度为0*/
						g_psControlLand->verticalSpeedExpect = 0;
						
						/*保持当前目标高度*/
						g_psControlLand->heightExpect = g_psSinsReal->curPosition[EARTH_FRAME_Z];
						
						/*标记目标高度已到达,相关状态已设置,无需再设置*/
						g_psControlLand->SAFE_HEIGHT_STATUS = CTRL_GO_HOME_SAFE_HEIGHT_NULL;
					}
					/*竖直高度达到安全高度,向水平靠拢的过渡时间内,需要悬停在当前位置*/
					else if (g_psControlLand->climb2CloseTransitionTicks != 0)
					{
						/*过渡时区内的过渡状态未设置,则设置(三维定点),并等待过渡时间结束,再向HOME点靠近*/
						if (g_psControlLand->TRANSITION_SET_STATUS == CTRL_GO_GOME_TRANSITION_SET_NO)
						{
							/*标记过渡时区内的状态已设置*/
							g_psControlLand->TRANSITION_SET_STATUS = CTRL_GO_GOME_TRANSITION_SET_OK;
							
							/*设置三维位置期望*/
							g_psControlLand->LandTargetPosition.lat = g_psAttitudeAll->GpsData.Coordinate_s4.lat;
							g_psControlLand->LandTargetPosition.lon = g_psAttitudeAll->GpsData.Coordinate_s4.lon;
							g_psControlLand->heightExpect           = g_psSinsReal->curPosition[EARTH_FRAME_Z];
							
							/*设置竖直速度期望为0*/
							g_psControlLand->verticalSpeedExpect = 0;
						}
					}
				}
			}
			/*首次切返航模式,距离home较近时,保持当前高度,将Home点作为目标点,飞至home正上方*/
			else if ((g_psControlLand->CUR_GO_HOME_PROCESS == CTRL_LAND_NEAR_HOLD_HEIGHT_GO_HOME) && \
					 (g_psControlLand->LAST_GO_HOME_PROCESS != CTRL_LAND_NEAR_HOLD_HEIGHT_GO_HOME))
			{
				/*目标座标为HOME位置*/
				g_psControlLand->LandTargetPosition.lat = g_psAttitudeAll->HomePos.Coordinate_s4.lat;
				g_psControlLand->LandTargetPosition.lon = g_psAttitudeAll->HomePos.Coordinate_s4.lon;
				
				/*设置竖直速度期望为0*/
				g_psControlLand->verticalSpeedExpect = 0;
				
				/*目标高度期望为当前高度*/
				g_psControlLand->heightExpect = g_psSinsReal->curPosition[EARTH_FRAME_Z];
				
				/*标记目标高度已到达,相关状态已设置,无需再设置*/
				g_psControlLand->SAFE_HEIGHT_STATUS = CTRL_GO_HOME_SAFE_HEIGHT_NULL;		

				/*确保会进入下一阶段,防止临界位置判断失败,无法进入下一阶段*/
				g_psControlLand->LAST_GO_HOME_PROCESS = CTRL_LAND_NEAR_HOLD_HEIGHT_GO_HOME;
			}
			/*持续在HOME点附近*/
			else if ((g_psControlLand->CUR_GO_HOME_PROCESS == CTRL_LAND_NEAR_HOLD_HEIGHT_GO_HOME) && \
					 (g_psControlLand->LAST_GO_HOME_PROCESS == CTRL_LAND_NEAR_HOLD_HEIGHT_GO_HOME))
			{
				/*目标座标为HOME位置*/
				g_psControlLand->LandTargetPosition.lat = g_psAttitudeAll->HomePos.Coordinate_s4.lat;
				g_psControlLand->LandTargetPosition.lon = g_psAttitudeAll->HomePos.Coordinate_s4.lon;	
				
				/*设置竖直速度期望为0*/
				g_psControlLand->verticalSpeedExpect = 0;
				
				/*标记目标高度已到达,相关状态已设置,无需再设置*/
				g_psControlLand->SAFE_HEIGHT_STATUS = CTRL_GO_HOME_SAFE_HEIGHT_NULL;		
			}
			/*首次进入HOME点正上方*/
			else if ((g_psControlLand->CUR_GO_HOME_PROCESS == CTRL_LAND_ARRIVE_DESCEND_GO_HOME) && \
					 (g_psControlLand->LAST_GO_HOME_PROCESS != CTRL_LAND_ARRIVE_DESCEND_GO_HOME))
			{
				/*目标座标为HOME位置*/
				g_psControlLand->LandTargetPosition.lat = g_psAttitudeAll->HomePos.Coordinate_s4.lat;
				g_psControlLand->LandTargetPosition.lon = g_psAttitudeAll->HomePos.Coordinate_s4.lon;
				
				/*速度变化最大量,然后递减*/
				g_psControlLand->verticalDescendSpeedRate = CTRL_AUTO_GO_HOME_ARRIVE_DESCEND_CM_S;
					
				/*设置竖直速度期望为0*/
				g_psControlLand->verticalSpeedExpect = 0;
				
				/*重置竖直下降速度变化周期*/
				g_psControlLand->verticalSpeedChangePeriod = 0;
				
				/*设置竖直位置期望为0*/
				g_psControlLand->heightExpect = 0;
				
				/*下降过程,无需再设置相关状态*/
				g_psControlLand->SAFE_HEIGHT_STATUS = CTRL_GO_HOME_SAFE_HEIGHT_NULL;				
			}
			/*持续在HOME点正上方*/			
			else if ((g_psControlLand->CUR_GO_HOME_PROCESS == CTRL_LAND_ARRIVE_DESCEND_GO_HOME) && \
					 (g_psControlLand->LAST_GO_HOME_PROCESS == CTRL_LAND_ARRIVE_DESCEND_GO_HOME))
			{
				/*目标座标为HOME位置*/
				g_psControlLand->LandTargetPosition.lat = g_psAttitudeAll->HomePos.Coordinate_s4.lat;
				g_psControlLand->LandTargetPosition.lon = g_psAttitudeAll->HomePos.Coordinate_s4.lon;
				
				/*速率变化周期ticks++*/
				g_psControlLand->verticalSpeedChangePeriod++;
				
				/*8 * 5ms = 40ms速度变化率改变一次*/
				if (g_psControlLand->verticalSpeedChangePeriod >= CTRL_AUTO_GO_HOME_VERTICAL_SPEED_CHANGE_MAX_PERIOD)
				{
					if (g_psControlLand->verticalDescendSpeedRate <= -1)
					{
						g_psControlLand->verticalDescendSpeedRate++;
					}
					else
					{
						g_psControlLand->verticalDescendSpeedRate = 0;
					}
						
					g_psControlLand->verticalSpeedChangePeriod = 0;
				}
					
				/*下降速度依次递增,避免油门控制量突变*/
				g_psControlLand->verticalSpeedExpect = CTRL_AUTO_GO_HOME_ARRIVE_DESCEND_CM_S - \
													   g_psControlLand->verticalDescendSpeedRate;				
				
				/*设置竖直期望高度为0*/
				g_psControlLand->heightExpect = 0;

				/*下降过程,无需再设置相关状态*/
				g_psControlLand->SAFE_HEIGHT_STATUS = CTRL_GO_HOME_SAFE_HEIGHT_NULL;
			}
			
			/*返航水平位置控制*/
			ctrl_Go_Home_Horizontal_Control(g_psControlLand->LandTargetPosition, g_psControlLand->LandCurPosition, g_psControlLand->HOME_DISTANCE); 
			
			/*返航竖直高度控制*/
			ctrl_Go_Home_Vertical_Control(g_psControlLand->verticalSpeedExpect, g_psControlLand->heightExpect, controlDeltaT); 
		}
		/*有水平打杆动作*/
		else if (g_psControlLand->HORIZONTAL_HAND_MEDDLE_STATUS == CTRL_GO_HOME_HAND_MEDDLE_TRUE)
		{
			/*返航水平位置控制*/
			ctrl_Go_Home_Horizontal_Control(g_psControlLand->LandTargetPosition, g_psControlLand->LandCurPosition, g_psControlLand->HOME_DISTANCE); 			
			
			/*手动调整*/
			if ((g_psControlLand->CUR_HORIZONTAL_CTRL_MODE == CTRL_GO_HOME_HAND_CTRL) && \
				(g_psControlLand->LAST_HORIZONTAL_CTRL_MODE == CTRL_GO_HOME_AUTO_CTRL))
			{
				/*设置竖直速度期望为0*/
				g_psControlLand->verticalSpeedExpect = 0;
				
				/*目标高度为手动调整后的高度*/
				g_psControlLand->heightExpect = g_psSinsReal->curPosition[EARTH_FRAME_Z];				
			}
			/*返航完全由手动接管*/
			else if ((g_psControlLand->CUR_HORIZONTAL_CTRL_MODE == CTRL_GO_HOME_HAND_CTRL) && \
					 (g_psControlLand->LAST_HORIZONTAL_CTRL_MODE == CTRL_GO_HOME_HAND_CTRL))
			{
				/*设置竖直速度期望为0*/
				g_psControlLand->verticalSpeedExpect = 0;			
			}
			
			/*返航竖直高度控制*/
			ctrl_Go_Home_Vertical_Control(g_psControlLand->verticalSpeedExpect, g_psControlLand->heightExpect, controlDeltaT); 
		}
	}
	/*飞行器已着地*/
	else if (g_psUav_Status->UavLandStatus.ThisTime == UAV_LAND_YES)
	{
		/*水平期望角给0*/
		g_psPidSystem->PitchAngle.expect = 0;
		g_psPidSystem->RollAngle.expect  = 0;

		/*起飞前清空积分控制器*/
		pid_Horizontal_Takeoff_Integrate_Reset();
		
		/*以2*Nav_Rapid_Decline_rate速度下降,使得油门量迅速满足怠速条件*/
		g_psControlLand->verticalSpeedExpect = 2 * CTRL_AUTO_GO_HOME_RAPID_DESCEND_CM_S;
		
		/*竖直高度期望为0*/
		g_psControlLand->heightExpect = 0;
		
		/*返航竖直高度控制*/
		ctrl_Go_Home_Vertical_Control(g_psControlLand->verticalSpeedExpect, g_psControlLand->heightExpect, controlDeltaT); 		
	}
}

/*普通原地着陆控制(NO GPS)*/
void ctrl_Land_Ground_Control(fp32 controlDeltaT, Uav_Status *uavStatus)
{
	/*未着地,有姿态控制参与*/
	if (uavStatus->UavLandStatus.ThisTime != UAV_LAND_YES)		
	{
		/*更新水平自稳控制下的ROLL(横滚)和PITCH(俯仰)期望角*/
		g_psPidSystem->PitchAngle.expect = g_psControlAircraft->RemotExpectAutoAngle.pitch; /*遥控期待自稳角pitch*/
		g_psPidSystem->RollAngle.expect  = g_psControlAircraft->RemotExpectAutoAngle.roll;	/*遥控期待自稳角roll*/
		
		/*根据当前高度,来决定降落速度,安全高度外快速,安全高度内慢速*/
		if (g_psSinsReal->curPosition[EARTH_FRAME_Z] <= CTRL_AUTO_GO_HOME_FARAWAY_DISTANCE_SAFE_HEIGHT_CM)
		{
			g_psControlLand->verticalSpeedExpect = CTRL_AUTO_GO_HOME_ARRIVE_DESCEND_CM_S;	/*慢速*/
		}
		else 
		{
			g_psControlLand->verticalSpeedExpect = CTRL_AUTO_GO_HOME_RAPID_DESCEND_CM_S;	/*快速*/
		}
		
		/*竖直高度期望为0*/
		g_psControlLand->heightExpect = 0;
		
		ctrl_Go_Home_Vertical_Control(g_psControlLand->verticalSpeedExpect, g_psControlLand->heightExpect, controlDeltaT);
	}
	/*飞行器已着地*/
	else if (uavStatus->UavLandStatus.ThisTime == UAV_LAND_YES)
	{
		/*水平期望角给0*/
		g_psPidSystem->PitchAngle.expect = 0;
		g_psPidSystem->RollAngle.expect  = 0;

		/*起飞前清空积分控制器*/
		pid_Horizontal_Takeoff_Integrate_Reset();		
		
		/*以2*Nav_Rapid_Decline_rate速度下降,使得油门量迅速满足怠速条件*/
		g_psControlLand->verticalSpeedExpect = 2 * CTRL_AUTO_GO_HOME_RAPID_DESCEND_CM_S;
		
		/*竖直高度期望为0*/
		g_psControlLand->heightExpect = 0;
		
		ctrl_Go_Home_Vertical_Control(g_psControlLand->verticalSpeedExpect, g_psControlLand->heightExpect, controlDeltaT);		
	}
}


/*得到相对目标点机体Pit、Rol方向偏移*/
Vector2f_Nav land_Gps_Offset_Relative_To_Home(Vector2s_Nav targPos, Vector2s_Nav curPos)
{
	Vector2f_Nav twoPosDealt;
	static s32 lastLandLat = 0;
	static fp32 landScale = 1.0f;
	
	/*比较两次纬度相差值，避免重复运算余弦函数*/
	if (math_Abs(lastLandLat - targPos.lat) < 100000)
	{
		/*保持scale*/
	}
	else
	{
		landScale = cosf(targPos.lat * 1.0e-7f * DEG2RAD);
		landScale = math_Constrain(landScale, 1.0f, 0.01f);
	}
	
	lastLandLat = targPos.lat;
	
	twoPosDealt.north = (targPos.lat - curPos.lat) * GPS_LOCATION_SCALING_FACTOR;				/*m*/
	twoPosDealt.east  = (targPos.lon - curPos.lon) * GPS_LOCATION_SCALING_FACTOR * landScale;	/*m*/
	
	/***********************************************************************************
	明确下导航系方向，这里正北、正东为正方向:
	沿着正东，经度增加,当无人机相对home点，往正东向移动时，此时GPS_Present.lng>GPS_Home.lng，得到的location_delta.x大于0;
	沿着正北，纬度增加,当无人机相对home点，往正北向移动时，此时GPS_Present.lat>GPS_Home.lat，得到的location_delta.y大于0;
	******************************************************************************/
	twoPosDealt.north *= 100;//沿地理坐标系，正北方向位置偏移,单位为CM
	twoPosDealt.east  *= 100;//沿地理坐标系，正东方向位置偏移,单位为CM
	
	return twoPosDealt;	
}
