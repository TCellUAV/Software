#include "control_PosHold.h"
#include "control_Aircraft.h"

Vector2f_Nav g_sUavGpsStopPoint = {0}; 		  /*GPS定点模式导航系中的刹车停止点*/
Vector2f g_sUavOpticFlowStopPoint = {0}; 		  /*光流定点模式下刹车停止点*/

Vector2f g_sHorizontalExpectAcc   = {0}; /*水平期望加速度*/
Vector2f g_sHorizontalExpectAngle = {0}; /*水平期望角度*/

/*定点模式下,遥杆回中后,先用水平速度控制刹车,待刹停后再赋值位置选点*/
SYS_RETSTATUS horizontal_GPS_Get_Stop_Point_XY(Vector2f_Nav *stopPoint)
{
	Vector2f_Nav curPos, curSpeed, curAcc;      /*导航系下当前位置,速度,加速度*/
	fp32 resultantSpeed = 0, resultantAcc = 0;	/*合速度,合加速度*/
	
	/*当前水平方向(x,y)的位置,速度,加速度*/
	curPos.north   = g_psSinsReal->curPosition[EARTH_FRAME_Y];
	curPos.east    = g_psSinsReal->curPosition[EARTH_FRAME_X];
	curSpeed.north = g_psSinsReal->curSpeed[EARTH_FRAME_Y];
	curSpeed.east  = g_psSinsReal->curSpeed[EARTH_FRAME_X];	
	curAcc.north   = g_psSinsReal->curAcc[EARTH_FRAME_Y];
	curAcc.east    = g_psSinsReal->curAcc[EARTH_FRAME_X];

	/*合速度和合加速度*/
	resultantSpeed = pythagorous2(curSpeed.north, curSpeed.east);
	resultantAcc   = pythagorous2(curAcc.north, curAcc.east);
	
	/*合水平速度小于等于20cm/s
	  合水平加速度小于等于40cm/s^2
	  Cos_Pitch*Cos_Roll、单个方向水平姿态约为15deg，两个方向水平姿态角小于10deg(rMatrix[2][2] = COS_PITCH * COS_ROLL)*/
	if ((resultantSpeed <= 20.0f) && (resultantAcc <= 40.0f) && (rMatrix[2][2] >= 0.97f))
	{
		stopPoint->north = curPos.north;
		stopPoint->east  = curPos.east;
		
		return SYS_RET_SUCC;
	}
	
	return SYS_RET_FAIL;
}

/*定点模式下,遥杆回中后,先用水平速度控制刹车,待刹停后再赋值光流位置选点*/
SYS_RETSTATUS horizontal_OpticFlow_Get_Stop_Point_XY(Vector2f *stopPoint)
{
	Vector2f curSpeed, curAcc; /*当前位置,速度,加速度*/
	fp32 resultantSpeed = 0, resultantAcc = 0;	/*合速度,合加速度*/
	
	/*当前水平方向(x,y)的速度,加速度*/
	curSpeed.x = g_psAttitudeAll->OpticFlowData.RealSpeed.x;				/*光流水平速度*/
	curSpeed.y = g_psAttitudeAll->OpticFlowData.RealSpeed.y;		
	curAcc.x   = g_psSinsFilterFeedback->curAcc[EARTH_FRAME_X];				/*水平加速度*/
	curAcc.y   = g_psSinsFilterFeedback->curAcc[EARTH_FRAME_Y];

	/*合速度和合加速度*/
	resultantSpeed = pythagorous2(curSpeed.x, curSpeed.y);
	resultantAcc   = pythagorous2(curAcc.x, curAcc.y);
	
	/*合水平速度的小于等于20cm/s
	  合水平加速度的小于等于40cm/s^2
	  Cos_Pitch*Cos_Roll、单个方向水平姿态约为15deg，两个方向水平姿态角约为10deg*/
	if ((resultantSpeed <= 20.0f) && (resultantAcc <= 40.0f) && (rMatrix[2][2] >= 0.97f))
	{
		/*光流定点,刹车后,期望位移为0*/
		stopPoint->x = 0;
		stopPoint->y = 0;
		
		return SYS_RET_SUCC;
	}
	
	return SYS_RET_FAIL;	
}

/*水平方向的加速度换算倾角*/
Vector2f *horizontal_Acc_Convert_To_Dip_Angle(Vector2f acc2f, Vector2f *angle2f)
{
	/*机头所在轴速度(前后运动加速度),与机头所在轴垂直轴速度(左右运动加速度)*/
	fp32 accY, accX; 
	fp32 maxDipAngle = REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX;
	
	accY = acc2f.y; /*机头所在轴为前后速度*/
	accX = acc2f.x; /*与机头所在轴垂直轴为左右速度*/
	
	/*根据前后左右运动加速度(cm/s^2)和垂直的重力加速度(cm/s^2)的夹角来,计算出倾角*/
	angle2f->y = math_Constrain(math_fast_atan((accY * COS_ROLL) / (GRAVITY_STD * 100)) * RAD2DEG, maxDipAngle, -maxDipAngle); /*pitch*/
	angle2f->x = math_Constrain(math_fast_atan(accX / (GRAVITY_STD * 100)) * RAD2DEG, maxDipAngle, -maxDipAngle);			   /*roll*/
	
	return (angle2f);
}

/*水平位置控制器*/
void horizontal_Control_PosHold(fp32 controlDeltaT)
{
	/*GPS定点模式(切定点时判断)*/
	if (g_sUav_Status.UavSenmodStatus.Horizontal.CURRENT_USE == UAV_HORIZONTAL_SENMOD_CURRENT_GPS)
	{
		horizontal_Control_GPS_PosHold(controlDeltaT);
	}
	/*光流定点模式(切定点时判断)*/
	else if (g_sUav_Status.UavSenmodStatus.Horizontal.CURRENT_USE == UAV_HORIZONTAL_SENMOD_CURRENT_OPTICFLOW)
	{
//		horizontal_Control_OpticFlow_PosHold(controlDeltaT);
	}	
}

vu16 g_vu16GpsHorizontalPosControlTicks   = 0; /*GPS水平位置控制计数器*/
vu16 g_vu16GpsHorizontalSpeedControlTicks = 0; /*GPS水平速度控制计数器*/

/*水平GPS位置控制器*/
void horizontal_Control_GPS_PosHold(fp32 controlDeltaT)
{
	/************************** 水平位置控制器 开始 ********************************/
	/*遥控居中,水平方向无遥控给定期望角度/速度*/
	if ((g_psControlAircraft->RemotExpectAngle.pitch == 0) && \
		(g_psControlAircraft->RemotExpectAngle.roll == 0))
	{
		/*ticks++*/
		g_vu16GpsHorizontalPosControlTicks++; /*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS 执行一次*/
		
		/************************** 水平位置控制器 开始 ********************************/
		if (g_vu16GpsHorizontalPosControlTicks >= CTRL_GPS_HORIZONTAL_POSITION_CONTROL_PERIOD)
		{
			/*ticks清0*/
			g_vu16GpsHorizontalPosControlTicks = 0;
			
			/*位置期望,经纬、航行速度、高度,方向杆回中后,期望为0,重新设置期望(只设置一次)*/
			if ((g_psPidSystem->LatitudePosition.expect == 0) && \
				(g_psPidSystem->LongitudePosition.expect == 0))
			{
				/*当刹车停住时,才更新水平位置期望*/
				if (horizontal_GPS_Get_Stop_Point_XY(&g_sUavGpsStopPoint) == SYS_RET_SUCC)
				{
					g_psPidSystem->LatitudePosition.expect  = g_sUavGpsStopPoint.north;
					g_psPidSystem->LongitudePosition.expect = g_sUavGpsStopPoint.east;
					
					/*标记水平方向是悬停*/
					g_psUav_Status->AIRSTOP_TYPE |= UAV_AIRSTOP_ONLY_HORIZONTAL;
				}
				else /*只采用水平速度刹车*/
				{
					/*水平刹车速度期望为0,即期望停住*/
					g_psPidSystem->LatitudeSpeed.expect  = 0;
					g_psPidSystem->LongitudeSpeed.expect = 0;
					
					/*标记水平方向非悬停*/
					g_psUav_Status->AIRSTOP_TYPE &= UAV_AIRSTOP_ONLY_VERTICAL;					
				}	
			}
			else /*水平位置靠水平速度环刹车后,更新水平位置期望,然后进入水平位置控制环*/
			{
				/*更新水平位置反馈,来源于当前惯导的位置估计*/
				g_psPidSystem->LatitudePosition.feedback  = g_psSinsReal->curPosition[EARTH_FRAME_Y];					
				g_psPidSystem->LongitudePosition.feedback = g_psSinsReal->curPosition[EARTH_FRAME_X];
			
				/*导航坐标系下E、N方向上位置偏差*/
				g_psAttitudeAll->EarthFramePosError.north = g_psPidSystem->LatitudePosition.expect - g_psPidSystem->LatitudePosition.feedback;
				g_psAttitudeAll->EarthFramePosError.east  = g_psPidSystem->LongitudePosition.expect - g_psPidSystem->LongitudePosition.feedback;
			
				/*导航坐标系 旋转到 机体坐标系 位置偏差*/
				g_psAttitudeAll->BodyFramePosError.pitch = -g_psAttitudeAll->EarthFramePosError.east * SIN_YAW + \
															g_psAttitudeAll->EarthFramePosError.north * COS_YAW;
	
				g_psAttitudeAll->BodyFramePosError.roll  =  g_psAttitudeAll->EarthFramePosError.east * COS_YAW + \
															g_psAttitudeAll->EarthFramePosError.north * SIN_YAW;

				/*机体坐标系下方向上期望刹车速度,这里为单比例运算不调用PID计算函数*/
				g_psAttitudeAll->BodyFramePosError.pitch = math_Constrain( g_psAttitudeAll->BodyFramePosError.pitch, \
																		   g_psPidSystem->LatitudePosition.errorMax, \
																		  -g_psPidSystem->LatitudePosition.errorMax); /*位置偏差限幅,单位cm*/

				g_psAttitudeAll->BodyFramePosError.roll  = math_Constrain( g_psAttitudeAll->BodyFramePosError.roll, \
																		   g_psPidSystem->LongitudePosition.errorMax, \
																		  -g_psPidSystem->LongitudePosition.errorMax); /*位置偏差限幅,单位cm*/
			
				/*更新水平方向刹车速度:比例运算(水平位置控制输出)*/
				g_psAttitudeAll->BodyFrameBrakeSpeed.pitch = g_psPidSystem->LatitudePosition.PID.kP * g_psAttitudeAll->BodyFramePosError.pitch;  /*y*/
				g_psAttitudeAll->BodyFrameBrakeSpeed.roll  = g_psPidSystem->LongitudePosition.PID.kP * g_psAttitudeAll->BodyFramePosError.roll;  /*x*/
			
				/*更新【机体系】水平方向Pitch、Roll速度控制器期望*/
				g_psPidSystem->LatitudeSpeed.expect  = g_psAttitudeAll->BodyFrameBrakeSpeed.pitch;  /*横纬-沿y轴方向(N向)*/
				g_psPidSystem->LongitudeSpeed.expect = g_psAttitudeAll->BodyFrameBrakeSpeed.roll;   /*竖经-沿x轴方向(E向)*/
				
				/******************** 水平位置控制器结束,给出期望刹车速度(期望速度) 结束 *********************/
			}
		}				
	
		/************************** 水平速度控制器(定点) 开始 ********************************/				
	    /*导航系的水平速度，转化到机体坐标系X-Y方向上*/
		/*沿载体Pitch、Roll方向水平速度控制*/
		g_vu16GpsHorizontalSpeedControlTicks++; /*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS 执行一次*/
		
		if (g_vu16GpsHorizontalSpeedControlTicks >= CTRL_GPS_HORIZONTAL_SPEED_CONTROL_PERIOD)
		{
			/*ticks清0*/
			g_vu16GpsHorizontalSpeedControlTicks = 0;
			
			/*导航系 转 机体系 水平方向速度反馈量*/
			g_psAttitudeAll->BodyFrameSpeedFeedback.pitch = -g_psSinsReal->curSpeed[EARTH_FRAME_X] * SIN_YAW + \
															 g_psSinsReal->curSpeed[EARTH_FRAME_Y] * COS_YAW;
			
			g_psAttitudeAll->BodyFrameSpeedFeedback.roll  =  g_psSinsReal->curSpeed[EARTH_FRAME_X] * COS_YAW + \
															 g_psSinsReal->curSpeed[EARTH_FRAME_Y] * SIN_YAW;					
			
			/*更新 机体系 水平方向速度反馈量*/
			g_psPidSystem->LatitudeSpeed.feedback  = g_psAttitudeAll->BodyFrameSpeedFeedback.pitch; /*y轴方向*/
			g_psPidSystem->LongitudeSpeed.feedback = g_psAttitudeAll->BodyFrameSpeedFeedback.roll;  /*x轴方向*/
			
			/*机体系 水平方向速度控制及输出*/
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
		
		/************************** 水平速度控制器(定点) 结束 ********************************/
	}
	/*拨动摇杆,只进行水平速度控制,无水平位置控制*/
	else if ((g_psControlAircraft->RemotExpectAngle.pitch != 0) || \
			 (g_psControlAircraft->RemotExpectAngle.roll != 0)) 
	{
		/*分两种情况:
		1、载体坐标系的姿态角控制;
		2、载体坐标系方向上的速度控制;*/
		
		/*推动方向杆,对应期望角度*/
		if (CTRL_HORIZONTAL_SENSOR_MODE_REMOT_EXPECT_ANGLE == SYS_ENABLE)
		{
			g_psPidSystem->PitchAngle.expect = g_psControlAircraft->RemotExpectAutoAngle.pitch;
			g_psPidSystem->RollAngle.expect  = g_psControlAircraft->RemotExpectAutoAngle.roll;			
		}
		/*推动方向杆,对应给定载体坐标系的沿Pitch,Roll方向运动速度*/
		/************************** 水平速度控制器(摇杆控速) 开始 ********************************/			
		else
		{
			/*ticks++*/
			g_vu16GpsHorizontalSpeedControlTicks++; /*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS 执行一次*/
			
			if (g_vu16GpsHorizontalSpeedControlTicks >= CTRL_GPS_HORIZONTAL_SPEED_CONTROL_PERIOD) 
			{
				/*ticks清0*/
				g_vu16GpsHorizontalSpeedControlTicks = 0;
				
				/*N向(沿PITCH方向)最大移动速度*/
				g_psPidSystem->LatitudeSpeed.expect  = -(g_psControlAircraft->RemotExpectAutoAngle.pitch / REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX) * \
													     CTRL_HORIZONTAL_MAX_MOVE_SPEED; /*最大期望速度,前speed>0;后speed<0*/

				/*E向(沿ROLL方向)最大移动速度*/
				g_psPidSystem->LongitudeSpeed.expect =  (g_psControlAircraft->RemotExpectAutoAngle.roll / REMOT_PITCH_ROLL_ANGLE_EXPECT_MAX) * \
													     CTRL_HORIZONTAL_MAX_MOVE_SPEED; /*最大期望速度,前speed>0;后speed<0*/						
				
				/*导航系的水平速度，转化到机体坐标系方向上*/
				/*沿机体Pitch、Roll方向水平速度控制*/
				g_psAttitudeAll->BodyFrameSpeedFeedback.pitch = -g_psSinsReal->curSpeed[EARTH_FRAME_X] * SIN_YAW + \
																 g_psSinsReal->curSpeed[EARTH_FRAME_Y] * COS_YAW;

				g_psAttitudeAll->BodyFrameSpeedFeedback.roll  =  g_psSinsReal->curSpeed[EARTH_FRAME_X] * COS_YAW + \
																 g_psSinsReal->curSpeed[EARTH_FRAME_Y] * SIN_YAW;
				
				/*更新水平速度反馈*/
				g_psPidSystem->LatitudeSpeed.feedback  = g_psAttitudeAll->BodyFrameSpeedFeedback.pitch; /*y轴方向*/	
				g_psPidSystem->LongitudeSpeed.feedback = g_psAttitudeAll->BodyFrameSpeedFeedback.roll;  /*x轴方向*/
				
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
		
		/************************** 水平速度控制器(摇杆控速) 结束 ********************************/
		/*推杆间隙,水平位置期望置0,遥控值直接给水平 速度/角度 期望*/
		g_psPidSystem->LatitudePosition.expect  = 0;
		g_psPidSystem->LongitudePosition.expect = 0;				
	}
}

vu16 g_vu16OpflowHorizontalPosControlTicks   = 0; /*OpticFlow水平位置控制计数器*/
vu16 g_vu16OpflowHorizontalSpeedControlTicks = 0; /*OpticFlow水平速度控制计数器*/

/*水平光流位置控制器*/
void horizontal_Control_OpticFlow_PosHold(fp32 controlDeltaT)
{
	/************************** 水平位置控制器 开始 ********************************/	
	/*遥控居中,水平方向无遥控给定期望角度*/
	if ((g_psControlAircraft->RemotExpectAngle.pitch == 0) && \
		(g_psControlAircraft->RemotExpectAngle.roll == 0))
	{
		/*ticks++*/
		g_vu16OpflowHorizontalPosControlTicks++; /*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS 执行一次*/
		
		/************************** 水平位置控制器 开始 ********************************/
		if (g_vu16OpflowHorizontalPosControlTicks >= CTRL_OPFLOW_HORIZONTAL_POSITION_CONTROL_PERIOD)
		{
			/*ticks清0*/
			g_vu16OpflowHorizontalPosControlTicks = 0;
			
			/*位置期望,x,y,方向杆回中后,期望为0,重新设置期望(只设置一次)*/
			if ((g_psPidSystem->OpticFlowXPosition.expect == 0) && \
				(g_psPidSystem->OpticFlowYPosition.expect == 0))
			{
				/*当刹车停住时,才更新水平位置期望*/
				if (horizontal_OpticFlow_Get_Stop_Point_XY(&g_sUavOpticFlowStopPoint) == SYS_RET_SUCC)
				{
					g_psPidSystem->OpticFlowXPosition.expect = g_sUavOpticFlowStopPoint.y;
					g_psPidSystem->OpticFlowYPosition.expect = g_sUavOpticFlowStopPoint.x;
				}
				else /*只采用水平速度刹车*/
				{
					/*水平刹车速度期望为0,即期望停住*/
					g_psPidSystem->OpticFlowXSpeed.expect = 0;
					g_psPidSystem->OpticFlowYSpeed.expect = 0;						
				}	
			}
			else /*水平位置靠水平速度环刹车后,更新水平位置期望,然后进入水平位置控制环*/
			{
				/*更新水平位置反馈,来源于当前惯导的位置估计*/
				g_psPidSystem->OpticFlowXPosition.feedback = g_psAttitudeAll->OpticFlowData.RealPosition.x;	
				g_psPidSystem->OpticFlowYPosition.feedback = g_psAttitudeAll->OpticFlowData.RealPosition.y;	
			
				/*导航坐标系下E、N方向上位置偏差*/
				g_psAttitudeAll->EarthFramePosError.north = g_psPidSystem->OpticFlowYPosition.expect - g_psPidSystem->OpticFlowYPosition.feedback;
				g_psAttitudeAll->EarthFramePosError.east  = g_psPidSystem->OpticFlowXPosition.expect - g_psPidSystem->OpticFlowXPosition.feedback;
			
				/*导航坐标系下机体Pitch(沿pitch旋转方向不是绕,即y轴方向(N向))、Roll(沿Roll旋转方向不是绕,即x轴方向(E向))方向上位置偏差*/
				g_psAttitudeAll->BodyFramePosError.pitch = -g_psAttitudeAll->EarthFramePosError.east * SIN_YAW + \
															g_psAttitudeAll->EarthFramePosError.north * COS_YAW;
	
				g_psAttitudeAll->BodyFramePosError.roll  =  g_psAttitudeAll->EarthFramePosError.east * COS_YAW + \
															g_psAttitudeAll->EarthFramePosError.north * SIN_YAW;		

				/*导航坐标系下机体Pitch(沿pitch旋转方向不是绕,即y轴方向(N向))、Roll(沿Roll旋转方向不是绕,
				  即x轴方向(E向))方向上期望刹车速度,这里为单比例运算不调用PID计算函数*/
				g_psAttitudeAll->BodyFramePosError.pitch = math_Constrain(g_psAttitudeAll->BodyFramePosError.pitch, \
																		  g_psPidSystem->OpticFlowYPosition.errorMax, \
																		  -g_psPidSystem->OpticFlowYPosition.errorMax); /*(沿pitch旋转方向不是绕,即y轴方向)方向,位置偏差限幅,单位cm*/

				g_psAttitudeAll->BodyFramePosError.roll  = math_Constrain(g_psAttitudeAll->BodyFramePosError.roll, \
																		  g_psPidSystem->OpticFlowXPosition.errorMax, \
																		  -g_psPidSystem->OpticFlowXPosition.errorMax); /*(沿Roll旋转方向不是绕,即x轴方向)方向,位置偏差限幅,单位cm*/
			
				/*更新水平方向Pitch(沿pitch旋转方向不是绕,即y轴方向(N向))、Roll(沿Roll旋转方向不是绕,即x轴方向(E向))刹车速度:比例运算(水平位置控制输出)*/
				g_psAttitudeAll->BodyFrameBrakeSpeed.pitch = g_psPidSystem->OpticFlowXPosition.PID.kP * g_psAttitudeAll->BodyFramePosError.pitch;
				g_psAttitudeAll->BodyFrameBrakeSpeed.roll  = g_psPidSystem->OpticFlowYPosition.PID.kP * g_psAttitudeAll->BodyFramePosError.roll;  
			
				/*更新水平方向Pitch(沿pitch旋转方向不是绕,即y轴方向(N向))、Roll(沿Roll旋转方向不是绕,即x轴方向(E向))速度控制器期望*/
				g_psPidSystem->OpticFlowYSpeed.expect = g_psAttitudeAll->BodyFrameBrakeSpeed.pitch;  /*横纬-沿y轴方向(N向)(沿pitch旋转方向不是绕)*/					
				g_psPidSystem->OpticFlowXSpeed.expect = g_psAttitudeAll->BodyFrameBrakeSpeed.roll;   /*竖经-沿x轴方向(E向)(沿roll旋转方向不是绕)*/			
				
				/******************** 水平位置控制器结束,给出期望刹车速度(期望速度) 结束 *********************/
			}
		}				
	
		/************************** 水平速度控制器 开始 ********************************/				
	    /*导航系的水平速度，转化到机体坐标系X-Y方向上*/
		/*沿载体Pitch、Roll方向水平速度控制*/
		g_vu16OpflowHorizontalSpeedControlTicks++; /*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS 执行一次*/
		
		if (g_vu16OpflowHorizontalSpeedControlTicks >= CTRL_OPFLOW_HORIZONTAL_SPEED_CONTROL_PERIOD)
		{
			/*ticks清0*/
			g_vu16OpflowHorizontalSpeedControlTicks = 0;
			
			/*机体系水平方向速度反馈量*/
			g_psAttitudeAll->BodyFrameSpeedFeedback.pitch = -g_psAttitudeAll->OpticFlowData.RealSpeed.y * SIN_YAW + \
															 g_psAttitudeAll->OpticFlowData.RealSpeed.x * COS_YAW;
			
			g_psAttitudeAll->BodyFrameSpeedFeedback.roll  =  g_psAttitudeAll->OpticFlowData.RealSpeed.y * COS_YAW + \
															 g_psAttitudeAll->OpticFlowData.RealSpeed.x * SIN_YAW;					
			
			/*更新沿载体水平方向速度反馈量*/
			g_psPidSystem->OpticFlowXSpeed.feedback = g_psAttitudeAll->BodyFrameSpeedFeedback.pitch; /*横纬-沿y轴方向(N向)(沿pitch旋转方向不是绕)*/		
			g_psPidSystem->OpticFlowYSpeed.feedback = g_psAttitudeAll->BodyFrameSpeedFeedback.roll;  /*竖经-沿x轴方向(E向)(沿roll旋转方向不是绕)*/
			
			/*沿载体水平方向速度控制及输出*/
			g_psPidSystem->OpticFlowXSpeed.controlOutput = pid_Control_Div_LPF(&g_psPidSystem->OpticFlowXSpeed, PID_CONTROLER_LATITUDE_SPEED);  /*PID DIV控制低通滤波*/		
			g_psPidSystem->OpticFlowYSpeed.controlOutput = pid_Control_Div_LPF(&g_psPidSystem->OpticFlowYSpeed, PID_CONTROLER_LONGITUDE_SPEED);  /*PID DIV控制低通滤波*/	
			
			/*水平方向期望加速度即水平速度控制环的输出*/
			g_sHorizontalExpectAcc.y = -g_psPidSystem->OpticFlowYSpeed.controlOutput;  /*机头pitch前后方向*/
			g_sHorizontalExpectAcc.x =  g_psPidSystem->OpticFlowXSpeed.controlOutput;  /*与机头pitch垂直roll左右方向*/
				
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
		/*分两种情况:
		1、导航坐标系的航向速度控制;
		2、载体坐标系方向上的速度控制;*/
		
		/*打杆后,光流水平位移清0*/
		g_psAttitudeAll->OpticFlowData.RealPosition.x = 0;
		g_psAttitudeAll->OpticFlowData.RealPosition.y = 0;
		
		/*推动方向杆,对应期望角度*/
		if (CTRL_HORIZONTAL_SENSOR_MODE_REMOT_EXPECT_ANGLE == SYS_ENABLE)
		{
			g_psPidSystem->PitchAngle.expect = g_psControlAircraft->RemotExpectAutoAngle.pitch;
			g_psPidSystem->RollAngle.expect  = g_psControlAircraft->RemotExpectAutoAngle.roll;					
		}
		/*推动方向杆,对应给定载体坐标系的沿Pitch,Roll方向运动速度*/
		/************************** 水平速度控制器 开始 ********************************/			
		else
		{
			/*ticks++*/
			g_vu16OpflowHorizontalSpeedControlTicks++; /*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS 执行一次*/
			
			if (g_vu16OpflowHorizontalSpeedControlTicks >= CTRL_OPFLOW_HORIZONTAL_SPEED_CONTROL_PERIOD)
			{
				/*ticks清0*/
				g_vu16OpflowHorizontalSpeedControlTicks = 0;
				
				/*N向(沿PITCH方向)最大移动速度*/
				g_psPidSystem->OpticFlowYSpeed.expect = g_psControlAircraft->RemotExpectAutoAngle.pitch * \
														CTRL_HORIZONTAL_MAX_MOVE_SPEED; /*最大期望速度45*6/100=3.6m/s*/

				/*E向(沿ROLL方向)最大移动速度*/
				g_psPidSystem->OpticFlowXSpeed.expect = g_psControlAircraft->RemotExpectAutoAngle.roll * \
														CTRL_HORIZONTAL_MAX_MOVE_SPEED; /*最大期望速度45*6/100=3.6m/s*/						
				
				/*导航系的水平速度，转化到机体坐标系X-Y方向上*/
				/*沿载体Pitch、Roll方向水平速度控制*/
				g_psAttitudeAll->BodyFrameSpeedFeedback.pitch = -g_psAttitudeAll->OpticFlowData.RealSpeed.y * SIN_YAW + \
																 g_psAttitudeAll->OpticFlowData.RealSpeed.x * COS_YAW;

				g_psAttitudeAll->BodyFrameSpeedFeedback.roll  =  g_psAttitudeAll->OpticFlowData.RealSpeed.y * COS_YAW + \
																 g_psAttitudeAll->OpticFlowData.RealSpeed.x * SIN_YAW;
				
				/*更新水平速度反馈*/
				g_psPidSystem->OpticFlowXSpeed.feedback = g_psAttitudeAll->BodyFrameSpeedFeedback.pitch; /*横纬-沿y轴方向(N向)(沿pitch旋转方向不是绕)*/	
				g_psPidSystem->OpticFlowYSpeed.feedback = g_psAttitudeAll->BodyFrameSpeedFeedback.roll;  /*竖经-沿x轴方向(E向)(沿roll旋转方向不是绕)*/
				
				/*水平速度PID计算及输出 (水平控速)*/
				g_psPidSystem->OpticFlowXSpeed.controlOutput = pid_Control_Div_LPF(&g_psPidSystem->OpticFlowXSpeed, PID_CONTROLER_LATITUDE_SPEED);    /*PID DIV控制低通滤波*/
				g_psPidSystem->OpticFlowYSpeed.controlOutput = pid_Control_Div_LPF(&g_psPidSystem->OpticFlowYSpeed, PID_CONTROLER_LONGITUDE_SPEED);  /*PID DIV控制低通滤波*/	
				
				/*水平方向期望加速度即水平速度控制环的输出*/
				g_sHorizontalExpectAcc.y = -g_psPidSystem->OpticFlowYSpeed.controlOutput;  /*机头pitch前后方向*/
				g_sHorizontalExpectAcc.x =  g_psPidSystem->OpticFlowXSpeed.controlOutput; /*与机头pitch垂直roll左右方向*/
				
				/*水平期望加速度与垂直的重力加速度计算出期望倾角*/
				horizontal_Acc_Convert_To_Dip_Angle(g_sHorizontalExpectAcc, &g_sHorizontalExpectAngle);
				
				/*更新角度环期望角(来源期望水平加速度和重力加速度换算出的期望倾角)*/
				g_psPidSystem->PitchAngle.expect = g_sHorizontalExpectAngle.y;	/*机头pitch前后方向*/
				g_psPidSystem->RollAngle.expect  = g_sHorizontalExpectAngle.x;	/*与机头pitch垂直roll左右方向*/				
			}
		}
		
		/************************** 水平速度控制器 结束 ********************************/
		
		/*推杆间隙,水平位置期望置0,遥控值直接给水平速度期望*/
		g_psPidSystem->OpticFlowXPosition.expect = 0;
		g_psPidSystem->OpticFlowYPosition.expect = 0;				
	}
}
