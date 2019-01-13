#include "pid_System.h"
#include "filter_DataProcess.h"
#include "hci_oledshow.h"
#include "control_Config.h"   /*用于控制系统配置*/

/*PID控制系统*/
PidSystem g_sPidSystem =
{
	.PidSettingSystem.DO_STATUS            = PID_PARAMETER_DO_NULL, /*PID调参系统默认什么都不做*/
	.PidSettingSystem.AVA_STATUS           = PID_PARAMETER_DISAVA,
	.PidSettingSystem.PARA_DOWNLOAD_STATUS = PID_PARA_DOWNLOAD_FINISH,
};

PidSystem *g_psPidSystem = &g_sPidSystem;

/*
1偏差限幅标志；   2积分限幅标志；   3积分分离标志；   4期望；
5反馈             6偏差；           7上次偏差；       8偏差限幅值；
9积分分离偏差值； 10积分值          11积分限幅值；    12控制参数Kp；
13控制参数Ki；    14控制参数Kd；    15控制器总输出；  16上次控制器总输出
17总输出限幅度    18控制参数Kp比例  19控制参数Ki比例  20控制参数Kd比例
*/
const fp32 g_PidLinkInit[PID_PARAMETER_SETTING_NBR][20]=
{
	/*姿态 内环(角速度)+外环(角度)*/
/*                                       	   Kp        Ki        Kd                kPscale  kIscale kDscale*/
	/*1  2  3  4  5  6  7  8   9   10   11     12        13        14   15  16   17    18       19      20*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,450 ,0  ,0  ,225  ,0.75    ,5.5000    ,1.80  ,0  ,0  ,500   ,1       ,1      ,1}, /*Pitch_Gyro  ;俯仰角速度*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,450 ,0  ,0  ,225  ,0.75    ,5.5000    ,1.80  ,0  ,0  ,500   ,1       ,1      ,1}, /*Roll_Gyro   ;横滚角速度*/	
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,275 ,0  ,0  ,125  ,1.00    ,0.5000    ,0.00  ,0  ,0  ,300   ,1       ,1      ,1}, /*Yaw_Gyro    ;偏航角速度*/	
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,30  ,0  ,0  ,80   ,4.00    ,0.0000    ,0.00  ,0  ,0  ,300   ,1       ,1      ,1}, /*Pitch_Angle ;俯仰角*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,30  ,0  ,0  ,80   ,4.00    ,0.0000    ,0.00  ,0  ,0  ,300   ,1       ,1      ,1}, /*Roll_Angle  ;横滚角*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,45  ,0  ,0  ,175  ,5.00    ,0.0000    ,0.00  ,0  ,0  ,300   ,1       ,1      ,1}, /*Yaw_Angle   ;偏航角*/

	//定高参数
	//高度单项比例控制，有偏差限幅、总输出即为最大攀升、下降速度400cm/s
	//Z轴速度比例+积分控制，无偏差限幅		
#if (CTRL_HEIGHT_POS_CONTROL_ACC_STATUS == SYS_DISABLE) /*是否开启竖直高度3环控制(位置+速度+加速度)*/		
/*                                       	   Kp        Ki        Kd                kPscale  kIscale kDscale*/
	/*1  2  3  4  5  6  7  8   9   10   11     12        13        14   15  16   17    18       19      20*/		
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,400 ,0  ,0  ,400  ,2.0     ,10.00     ,0.15  ,0  ,0  ,600   ,1       ,1      ,1}, /*High_Speed    ;海拔攀升速度*/		
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,200 ,0  ,0  ,50   ,1.0     ,0.000     ,0     ,0  ,0  ,400   ,1       ,1      ,1}, /*High_Position ;海拔高度位置*/
#else
/*                                       	   Kp        Ki        Kd                kPscale  kIscale kDscale*/
	/*1  2  3  4  5  6  7  8   9   10   11     12        13        14   15  16   17    18       19      20*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,400 ,0  ,0  ,500  ,3.0     ,0.000     ,0.1   ,0  ,0  ,500   ,1       ,1      ,1}, /*High_Speed    ;海拔攀升速度*/	
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,200 ,0  ,0  ,100  ,0.4     ,0.000     ,0     ,0  ,0  ,400   ,1       ,1      ,1}, /*High_Position ;海拔高度位置*/
#endif
 /*
1偏差限幅标志；  2积分限幅标志；3积分分离标志；   4期望；
5反馈            6偏差；        7上次偏差；       8偏差限幅值；
9积分分离偏差值；10积分值       11积分限幅值；    12控制参数Kp；
13控制参数Ki；   14控制参数Kd； 15控制器总输出；  16上次控制器总输出
17总输出限幅度
*/
	/*GPS(水平) 内环(速度)+外环(位置)*/
/*                                       	   Kp        Ki        Kd                kPscale  kIscale kDscale*/
	/*1  2  3  4  5  6  7  8   9   10   11     12        13        14   15  16   17    18       19      20*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,300 ,0  ,0  ,240  ,1.800   ,0.450     ,0     ,0  ,0  ,500   ,1       ,1      ,1}, /*Latitude_Speed     ;水平纬度速度*/	
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,180 ,0  ,0  ,8    ,0.200   ,0.000     ,0     ,0  ,0  ,150   ,1       ,1      ,1}, /*Latitude_Position  ;水平纬度位置*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,300 ,0  ,0  ,240  ,1.800   ,0.450     ,0     ,0  ,0  ,500   ,1       ,1      ,1}, /*Longitude_Speed    ;水平经度速度*/		
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,180 ,0  ,0  ,8    ,0.200   ,0.000     ,0     ,0  ,0  ,150   ,1       ,1      ,1}, /*Longitude_Position ;水平经度位置*/
	
	/*Opticflow(水平) 内环(速度)+外环(位置)*/
/*                                       	   Kp        Ki        Kd                kPscale  kIscale kDscale*/
	/*1  2  3  4  5  6  7  8   9   10   11     12        13        14   15  16   17    18       19      20*/		
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,30  ,15 ,0  ,200  ,4.50    ,0.10      ,0.0   ,0  ,0  ,450   ,1       ,1      ,1}, /*光流(水平X方向)速度*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,100 ,15 ,0  ,15   ,0.25    ,0.00      ,0     ,0  ,0  ,30    ,1       ,1      ,1}, /*光流(水平X方向)位置*/	
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,30  ,15 ,0  ,200  ,4.50    ,0.10      ,0.0   ,0  ,0  ,450   ,1       ,1      ,1}, /*光流(水平Y方向)速度*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,100 ,15 ,0  ,15   ,0.25    ,0.00      ,0     ,0  ,0  ,30    ,1       ,1      ,1}, /*光流(水平Y方向)位置*/	
	
	/*加速度控制器*/
	//最大加速度200cm/s^2
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,600 ,0  ,0  ,500  ,0.10    ,0.8000    ,0     ,0  ,0  ,600   ,1       ,1      ,1}, /*垂直加速度控制器*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,100 ,0  ,0  ,3    ,0.32    ,0.0000    ,0     ,0  ,0  ,150   ,1       ,1      ,1}, /*水平经度方向加速度控制器*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,100 ,0  ,0  ,15   ,0.45    ,0.0000    ,0     ,0  ,0  ,25    ,1       ,1      ,1}, /*水平维度方向加速度控制器*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,0   ,0  ,0  ,0    ,0.00    ,0.0000    ,0     ,0  ,0  ,0     ,1       ,1      ,1}, /*光流(水平X方向)加速度*/
	{1  ,1 ,0 ,0 ,0 ,0 ,0 ,0   ,0  ,0  ,0    ,0.00    ,0.0000    ,0     ,0  ,0  ,0     ,1       ,1      ,1}, /*光流(水平Y方向)加速度*/		
};

/*PID控制系统初始化*/
void pid_System_Init(PidSystem *pidSystem)
{ 
	/*1.角度和角速度控制 初始化*/
	pid_Link_Init(&(pidSystem->PitchGyro), PID_CONTROLER_PITCH_GYRO);
	pid_Link_Init(&(pidSystem->RollGyro), PID_CONTROLER_ROLL_GYRO);	
	pid_Link_Init(&(pidSystem->YawGyro), PID_CONTROLER_YAW_GYRO);	
	pid_Link_Init(&(pidSystem->PitchAngle), PID_CONTROLER_PITCH_ANGLE);	
	pid_Link_Init(&(pidSystem->RollAngle), PID_CONTROLER_ROLL_ANGLE);
	pid_Link_Init(&(pidSystem->YawAngle), PID_CONTROLER_YAW_ANGLE);

	
	/*2.竖直高度控制 初始化*/
	pid_Link_Init(&(pidSystem->HighSpeed), PID_CONTROLER_HIGH_SPEED);	
	pid_Link_Init(&(pidSystem->HighPosition), PID_CONTROLER_HIGH_POSITION);	
	
	/*3.(GPS)位置和速度控制 初始化*/
	pid_Link_Init(&(pidSystem->LatitudeSpeed), PID_CONTROLER_LATITUDE_SPEED);	
	pid_Link_Init(&(pidSystem->LatitudePosition), PID_CONTROLER_LATITUDE_POSITION);	
	pid_Link_Init(&(pidSystem->LongitudeSpeed), PID_CONTROLER_LONGITUDE_SPEED);	
	pid_Link_Init(&(pidSystem->LongitudePosition), PID_CONTROLER_LONGITUDE_POSITION);

	/*4.(光流) 位置和速度控制 初始化*/
	pid_Link_Init(&(pidSystem->OpticFlowXSpeed), PID_CONTROLER_OPTICFLOW_X_SPEED);	
	pid_Link_Init(&(pidSystem->OpticFlowXPosition), PID_CONTROLER_OPTICFLOW_X_POSITION);
	pid_Link_Init(&(pidSystem->OpticFlowYSpeed), PID_CONTROLER_OPTICFLOW_Y_SPEED);
	pid_Link_Init(&(pidSystem->OpticFlowYPosition), PID_CONTROLER_OPTICFLOW_Y_POSITION);	
	
	
	/*5.加速度控制 初始化*/
	pid_Link_Init(&(pidSystem->HighAcc), PID_CONTROLER_HIGH_ACC);
	pid_Link_Init(&(pidSystem->LongitudeAcc), PID_CONTROLER_LONGITUDE_ACC);
	pid_Link_Init(&(pidSystem->LatitudeAcc), PID_CONTROLER_LATITUDE_ACC);	
	pid_Link_Init(&(pidSystem->OpticFlowXAcc), PID_CONTROLER_OPTICFLOW_X_ACC);
	pid_Link_Init(&(pidSystem->OpticFlowYAcc), PID_CONTROLER_OPTICFLOW_Y_ACC);		
}

/*PID控制单环初始化*/
void pid_Link_Init(PidLink *pidLink, PID_CONTROLER_LINK CONTROLER_LINK)
{
	/*1.偏差限幅标志*/
	pidLink->ERROR_LIMIT_FLAG = (PID_LIMIT_FLAG)(g_PidLinkInit[CONTROLER_LINK][0]);
	/*2.积分限幅标志*/
	pidLink->INTEGRATE_LIMIT_FLAG = (PID_LIMIT_FLAG)(g_PidLinkInit[CONTROLER_LINK][1]);
	/*3.积分分离标志*/
	pidLink->INTEGRATE_SEPARATION_FLAG = (PID_LIMIT_FLAG)(g_PidLinkInit[CONTROLER_LINK][2]);
	/*4.期望*/
	pidLink->expect = g_PidLinkInit[CONTROLER_LINK][3];
	/*5.反馈值*/
	pidLink->feedback = g_PidLinkInit[CONTROLER_LINK][4];
	/*6.偏差*/
	pidLink->error = g_PidLinkInit[CONTROLER_LINK][5];
	/*7.上次偏差*/
	pidLink->lastError = g_PidLinkInit[CONTROLER_LINK][6];
	/*8.偏差限幅值*/
	pidLink->errorMax = g_PidLinkInit[CONTROLER_LINK][7];
	/*9.积分分离偏差值*/
	pidLink->integrateSeparationError = g_PidLinkInit[CONTROLER_LINK][8];
	/*10.积分值*/
	pidLink->integrate = g_PidLinkInit[CONTROLER_LINK][9];
	/*11.积分限幅值*/
	pidLink->integrateMax = g_PidLinkInit[CONTROLER_LINK][10];
	/*12.控制参数kP*/
	pidLink->PID.kP = g_PidLinkInit[CONTROLER_LINK][11];
	/*13.控制参数kI*/
	pidLink->PID.kI = g_PidLinkInit[CONTROLER_LINK][12];	
	/*14.控制参数kD*/
	pidLink->PID.kD = g_PidLinkInit[CONTROLER_LINK][13];		
	/*15.控制器总输出*/
	pidLink->controlOutput = g_PidLinkInit[CONTROLER_LINK][14];
	/*16.上次控制器总输出*/
	pidLink->lastControlOutPut = g_PidLinkInit[CONTROLER_LINK][15];
	/*17.控制器总输出限幅*/
	pidLink->controlOutPutLimit = g_PidLinkInit[CONTROLER_LINK][16];
	
	/*PID Scale默认1.0倍*/
	pidLink->PidScale.kP = g_PidLinkInit[CONTROLER_LINK][17];
	pidLink->PidScale.kI = g_PidLinkInit[CONTROLER_LINK][18];
	pidLink->PidScale.kD = g_PidLinkInit[CONTROLER_LINK][19];
}

/*PID通用控制计算*/
fp32 pid_Control_General_Dp(PidLink *pidLink)
{
	fp32 controlDeltaT;
	
	/*获取精确执行间隔时间*/
	get_Period_Execute_Time_Info(&(pidLink->PidControlDT));
	controlDeltaT = pidLink->PidControlDT.DeltaTime / 1000.0f; /*换算成s*/
	
	/*最小控制间隔周期*/
	if (controlDeltaT < 0.001f)
	{
		return 0;
	}
	
	/*1.偏差计算*/
	/*保存上次偏差*/
	pidLink->lastError = pidLink->error;	
	/*期望减去反馈得到偏差*/
	pidLink->error = pidLink->expect - pidLink->feedback; 
	/*偏差限幅标志位*/
	if (pidLink->ERROR_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->error >= pidLink->errorMax)
		{
			pidLink->error = pidLink->errorMax;
		}
		
		if (pidLink->error <= -pidLink->errorMax)
		{
			pidLink->error = -pidLink->errorMax;
		}
	}
	
	/*2.积分计算*/
	/*积分分离标志位*/
	if (pidLink->INTEGRATE_SEPARATION_FLAG == PID_LIMIT_ENABLE)
	{
		if (math_Abs(pidLink->error) <= pidLink->integrateSeparationError)
		{
			pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->error * controlDeltaT;
		}
	}
	else
	{
		pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->error * controlDeltaT;		
	}
	
	/*3.积分限幅*/
	/*积分限制幅度标志*/
	if (pidLink->INTEGRATE_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->integrate >= pidLink->integrateMax)
		{
			pidLink->integrate = pidLink->integrateMax;
		}
		
		if (pidLink->integrate <= -pidLink->integrateMax)
		{
			pidLink->integrate = -pidLink->integrateMax;
		}
	}
	
	/*4.总输出计算*/
	/*输出值递推*/
	pidLink->lastControlOutPut = pidLink->controlOutput;
	/*PID运算*/
	pidLink->controlOutput = (pidLink->PidScale.kP * pidLink->PID.kP) * pidLink->error + \
							  pidLink->integrate + \
							  pidLink->PID.kD * (pidLink->error - pidLink->lastError);

	/*5.总输出限幅*/
	if (pidLink->controlOutput >= pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = pidLink->controlOutPutLimit;
	}
	
	if (pidLink->controlOutput <= -pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = -pidLink->controlOutPutLimit;
	}
	
	/*6.返回总输出*/
	return (pidLink->controlOutput);
}

/*PID Yaw角控制计算*/
fp32 pid_Control_Yaw_Dp(PidLink *pidLink)
{
	fp32 controlDeltaT;
	
	/*获取精确执行间隔时间*/
	get_Period_Execute_Time_Info(&(pidLink->PidControlDT));
	controlDeltaT = pidLink->PidControlDT.DeltaTime / 1000.0f; /*换算成s*/	

	/*最小控制间隔周期*/
	if (controlDeltaT < 0.001f)
	{
		return 0;
	}
	
	/*1.偏差计算*/
	/*保存上次偏差*/
	pidLink->lastError = pidLink->error;	
	/*期望减去反馈得到偏差*/
	pidLink->error = pidLink->expect - pidLink->feedback; 
	
	/*偏航角偏差超过±180处理*/
	if (pidLink->error < -180)
	{
		pidLink->error = pidLink->error + 360;
	}

	if (pidLink->error > 180)
	{
		pidLink->error = pidLink->error - 360;
	}	
	
	/*偏差限幅标志位*/
	if (pidLink->ERROR_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->error >= pidLink->errorMax)
		{
			pidLink->error = pidLink->errorMax;
		}
		
		if (pidLink->error <= -pidLink->errorMax)
		{
			pidLink->error = -pidLink->errorMax;
		}
	}
	
	/*2.积分计算*/
	/*积分分离标志位*/
	if (pidLink->INTEGRATE_SEPARATION_FLAG == PID_LIMIT_ENABLE)
	{
		if (math_Abs(pidLink->error) <= pidLink->integrateSeparationError)
		{
			pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->error * controlDeltaT; /*kI运算*/
		}
	}
	else
	{
		pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->error * controlDeltaT;		
	}
	
	/*3.积分限幅*/
	/*积分限制幅度标志*/
	if (pidLink->INTEGRATE_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->integrate >= pidLink->integrateMax)
		{
			pidLink->integrate = pidLink->integrateMax;
		}
		
		if (pidLink->integrate <= -pidLink->integrateMax)
		{
			pidLink->integrate = -pidLink->integrateMax;
		}
	}
	
	/*4.总输出计算*/
	/*输出值递推*/
	pidLink->lastControlOutPut = pidLink->controlOutput;
	/*PID运算*/
	pidLink->controlOutput = (pidLink->PidScale.kP * pidLink->PID.kP) * pidLink->error + \
							  pidLink->integrate + \
							  pidLink->PID.kD * (pidLink->error - pidLink->lastError);

	/*5.总输出限幅*/
	if (pidLink->controlOutput >= pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = pidLink->controlOutPutLimit;
	}
	
	if (pidLink->controlOutput <= -pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = -pidLink->controlOutPutLimit;
	}
	
	/*6.返回总输出*/
	return (pidLink->controlOutput);	
}

/*PID DIV控制低通滤波*/
fp32 pid_Control_Div_LPF(PidLink *pidLink, PID_CONTROLER_LINK CONTROLER_LINK)
{
	u8 i;
	fp32 controlDeltaT;

	/*获取精确执行间隔时间*/
	get_Period_Execute_Time_Info(&(pidLink->PidControlDT));
	controlDeltaT = pidLink->PidControlDT.DeltaTime / 1000.0f; /*换算成s*/	

	/*最小控制间隔周期*/
	if (controlDeltaT < 0.001f)
	{
		return 0;
	}
	
	/*1.偏差计算*/
	/*保存上次偏差*/
	pidLink->lastError = pidLink->error;	
	/*期望减去反馈得到偏差*/
	pidLink->error = pidLink->expect - pidLink->feedback; 
	/*原始微分*/
	pidLink->disErr = pidLink->error - pidLink->lastError;
	
	/******************************************/
	#if (PID_SYSTEM_CONFIG_USE_AVERAGE_FILTER == SYS_ENABLE)
	/*均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常*/		
	fp32 tempa, tempb, tempc, max, min;  /*用于防跳变滤波*/	

	tempa = pidLink->lastLastDisErrLPF; 
	tempb = pidLink->lastDisErrLPF;
	tempc = pidLink->disErr;
	max = tempa > tempb ? tempa : tempb;
	max = max > tempc ? max : tempc;
	min = tempa < tempb ? tempa : tempb;
	min = min < tempc ? min : tempc;
	if (tempa > min && tempa < max)    pidLink->disErr = tempa;
	if (tempb > min  && tempb < max )  pidLink->disErr = tempb;
	if (tempc > min  &&  tempc < max)  pidLink->disErr = tempc;
	pidLink->lastLastDisErrLPF = pidLink->lastDisErrLPF;
	pidLink->lastDisErrLPF     = pidLink->disErr;
	#endif
	/*****************************************/	
	
	/*数字低通后微分项滑动移除老数据*/
	for (i = 4; i > 0; i--)
	{
		pidLink->disErrHistory[i] = pidLink->disErrHistory[i - 1];
	}
	
	/*加入新数据*/
	pidLink->disErrHistory[0] = filter_Pid_Control_Device_LPF(pidLink->disErr, \
															  &(g_sFilterTarg.PidControlBuff[CONTROLER_LINK]), \
															  &(g_sFilterTarg.PidControlDivPara[FILTER_LPBW_PID_CONTROLER_DIV_200HZ_20HZ_IDX])); /*巴特沃斯低通后得到的微分项,20hz*/
	
	/*偏差限幅标志位*/
	if (pidLink->ERROR_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->error >= pidLink->errorMax)
		{
			pidLink->error = pidLink->errorMax;
		}
		
		if (pidLink->error <= -pidLink->errorMax)
		{
			pidLink->error = -pidLink->errorMax;
		}
	}
	
	/*2.积分计算*/
	/*积分分离标志位*/
	if (pidLink->INTEGRATE_SEPARATION_FLAG == PID_LIMIT_ENABLE)
	{
		if (math_Abs(pidLink->error) <= pidLink->integrateSeparationError)
		{
			pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->error * controlDeltaT; /*kI运算*/
		}
	}
	else
	{
		pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->error * controlDeltaT;		
	}
	
	/*3.积分限幅*/
	/*积分限制幅度标志*/
	if (pidLink->INTEGRATE_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->integrate >= pidLink->integrateMax)
		{
			pidLink->integrate = pidLink->integrateMax;
		}
		
		if (pidLink->integrate <= -pidLink->integrateMax)
		{
			pidLink->integrate = -pidLink->integrateMax;
		}
	}
	
	/*4.总输出计算*/
	/*输出值递推*/
	pidLink->lastControlOutPut = pidLink->controlOutput;
	/*PID运算*/
	pidLink->controlOutput = (pidLink->PidScale.kP * pidLink->PID.kP) * pidLink->error + \
							  pidLink->integrate + \
							  pidLink->PID.kD * pidLink->disErrHistory[0]; /*微分项来源于巴特沃斯低通滤波器*/

	/*5.总输出限幅*/
	if (pidLink->controlOutput >= pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = pidLink->controlOutPutLimit;
	}
	
	if (pidLink->controlOutput <= -pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = -pidLink->controlOutPutLimit;
	}
	
	/*6.返回总输出*/
	return (pidLink->controlOutput);	
}

/*PID ERR控制低通滤波*/
fp32 pid_Control_Err_LPF(PidLink *pidLink, PID_CONTROLER_LINK CONTROLER_LINK)
{
	fp32 controlDeltaT;

	/*获取精确执行间隔时间*/
	get_Period_Execute_Time_Info(&(pidLink->PidControlDT));
	controlDeltaT = pidLink->PidControlDT.DeltaTime / 1000.0f; /*换算成s*/	

	/*最小控制间隔周期*/
	if (controlDeltaT < 0.001f)
	{
		return 0;
	}
	
	/*1.偏差计算*/
	/*保存上次偏差*/
	pidLink->lastError = pidLink->error;	
	/*期望减去反馈得到偏差*/
	pidLink->error = pidLink->expect - pidLink->feedback; 
	/*原始微分*/
	pidLink->disErr = pidLink->error - pidLink->lastError;
	
	pidLink->lastErrLPF = pidLink->errLPF;
	pidLink->errLPF = filter_Pid_Control_Device_LPF(pidLink->error, \
													&(g_sFilterTarg.PidControlBuff[CONTROLER_LINK]), \
													&(g_sFilterTarg.PidControlErrPara[FILTER_LPBW_PID_CONTROLER_ERR_200HZ_5HZ_IDX])); /*巴特沃斯低通后得到的微分项,5hz*/
	
	/*微分量*/
	pidLink->disErrLPF = pidLink->errLPF - pidLink->lastErrLPF;
	
	/*偏差限幅标志位*/
	if (pidLink->ERROR_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->errLPF >= pidLink->errorMax)
		{
			pidLink->errLPF = pidLink->errorMax;
		}
		
		if (pidLink->errLPF <= -pidLink->errorMax)
		{
			pidLink->errLPF = -pidLink->errorMax;
		}
	}
	
	/*2.积分计算*/
	/*积分分离标志位*/
	if (pidLink->INTEGRATE_SEPARATION_FLAG == PID_LIMIT_ENABLE)
	{
		if (math_Abs(pidLink->errLPF) <= pidLink->integrateSeparationError)
		{
			pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->errLPF * controlDeltaT; /*kI运算*/
		}
	}
	else
	{
		pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->errLPF * controlDeltaT; /*kI运算*/		
	}
	
	/*3.积分限幅*/
	/*积分限制幅度标志*/
	if (pidLink->INTEGRATE_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->integrate >= pidLink->integrateMax)
		{
			pidLink->integrate = pidLink->integrateMax;
		}
		
		if (pidLink->integrate <= -pidLink->integrateMax)
		{
			pidLink->integrate = -pidLink->integrateMax;
		}
	}
	
	/*4.总输出计算*/
	/*输出值递推*/
	pidLink->lastControlOutPut = pidLink->controlOutput;
	/*PID运算*/
	pidLink->controlOutput = (pidLink->PidScale.kP * pidLink->PID.kP) * pidLink->errLPF + \
							  pidLink->integrate + \
							  pidLink->PID.kD * pidLink->disErrLPF; /*已对偏差低通,此处不再对微分项单独低通*/

	/*5.总输出限幅*/
	if (pidLink->controlOutput >= pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = pidLink->controlOutPutLimit;
	}
	
	if (pidLink->controlOutput <= -pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = -pidLink->controlOutPutLimit;
	}
	
	/*6.返回总输出*/
	return (pidLink->controlOutput);		
}

/*PID DIV控制低通滤波 Gyro*/
fp32 pid_Control_Div_LPF_Gyro(PidLink *pidLink, PID_CONTROLER_LINK CONTROLER_LINK)
{
	u8 i;
	fp32 controlDeltaT;

	/*获取精确执行间隔时间*/
	get_Period_Execute_Time_Info(&(pidLink->PidControlDT));
	controlDeltaT = pidLink->PidControlDT.DeltaTime / 1000.0f; /*换算成s*/
	
	/*最小控制间隔周期*/
	if (controlDeltaT < 0.001f)
	{
		return 0;
	}
	
	/*1.偏差计算*/
	/*保存上上次偏差*/
	pidLink->lastLastError = pidLink->lastError;
	/*保存上次偏差*/
	pidLink->lastError = pidLink->error;	
	/*期望减去反馈得到偏差*/
	pidLink->error = pidLink->expect - pidLink->feedback; 
	/*间隔了一次采样的微分*/
	pidLink->disErr = pidLink->error - pidLink->lastLastError;
	
	/******************************************/
	#if (PID_SYSTEM_CONFIG_USE_AVERAGE_FILTER == SYS_ENABLE)
	/*均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常*/		
	fp32 tempa, tempb, tempc, max, min;  /*用于防跳变滤波*/	
	
	tempa = pidLink->lastLastDisErrLPF; 
	tempb = pidLink->lastDisErrLPF;
	tempc = pidLink->disErr;
	max = tempa > tempb ? tempa : tempb;
	max = max > tempc ? max : tempc;
	min = tempa < tempb ? tempa : tempb;
	min = min < tempc ? min : tempc;
	if (tempa > min && tempa < max)    pidLink->disErr = tempa;
	if (tempb > min  && tempb < max )  pidLink->disErr = tempb;
	if (tempc > min  &&  tempc < max)  pidLink->disErr = tempc;
	pidLink->lastLastDisErrLPF = pidLink->lastDisErrLPF;
	pidLink->lastDisErrLPF     = pidLink->disErr;
	#endif
	/*****************************************/		
	
	/*数字低通后微分项滑动移除老数据*/
	for (i = 4; i > 0; i--)
	{
		pidLink->disErrHistory[i] = pidLink->disErrHistory[i - 1];
	}
	
	/*加入新数据*/
	pidLink->disErrHistory[0] = filter_Pid_Control_Device_LPF(pidLink->disErr, \
															  &(g_sFilterTarg.PidControlBuff[CONTROLER_LINK]), \
															  &(g_sFilterTarg.PidControlDivPara[FILTER_LPBW_PID_CONTROLER_DIV_200HZ_30HZ_IDX])); /*巴特沃斯低通后得到的微分项,30hz*/
	/*微分项限幅*/
	pidLink->disErrHistory[0] = math_Constrain(pidLink->disErrHistory[0], 500, -500);
	
	/*自适应微分参数*/
	pidLink->adaptableKd = pidLink->PID.kD * (1 + pidLink->disErrHistory[0] / 500);
	
	/*偏差限幅标志位*/
	if (pidLink->ERROR_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->error >= pidLink->errorMax)
		{
			pidLink->error = pidLink->errorMax;
		}
		
		if (pidLink->error <= -pidLink->errorMax)
		{
			pidLink->error = -pidLink->errorMax;
		}
	}
	
	/*2.积分计算*/
	/*积分分离标志位*/
	if (pidLink->INTEGRATE_SEPARATION_FLAG == PID_LIMIT_ENABLE)
	{
		if (math_Abs(pidLink->error) <= pidLink->integrateSeparationError)
		{
			pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->error * controlDeltaT; /*kI运算*/
		}
	}
	else
	{
		pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->error * controlDeltaT;		
	}
	
	/*3.积分限幅*/
	/*积分限制幅度标志*/
	if (pidLink->INTEGRATE_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->integrate >= pidLink->integrateMax)
		{
			pidLink->integrate = pidLink->integrateMax;
		}
		
		if (pidLink->integrate <= -pidLink->integrateMax)
		{
			pidLink->integrate = -pidLink->integrateMax;
		}
	}
	
	/*4.总输出计算*/
	/*输出值递推*/
	pidLink->lastControlOutPut = pidLink->controlOutput;
	/*PID运算*/
	pidLink->controlOutput = (pidLink->PidScale.kP * pidLink->PID.kP) * pidLink->error + \
							  pidLink->integrate + \
							  pidLink->adaptableKd * pidLink->disErrHistory[0]; /*微分项来源于巴特沃斯低通滤波器*/

	/*5.总输出限幅*/
	if (pidLink->controlOutput >= pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = pidLink->controlOutPutLimit;
	}
	
	if (pidLink->controlOutput <= -pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = -pidLink->controlOutPutLimit;
	}
	
	/*6.返回总输出*/
	return (pidLink->controlOutput);	
}

/*微分先行PID控制器*/
fp32 pid_Control_Div_LPF_Differential_Forward(PidLink *pidLink, PID_CONTROLER_LINK CONTROLER_LINK)
{
	u8 i;
	fp32 controlDeltaT;

	/*获取精确执行间隔时间*/
	get_Period_Execute_Time_Info(&(pidLink->PidControlDT));
	controlDeltaT = pidLink->PidControlDT.DeltaTime / 1000.0f; /*换算成s*/	

	/*最小控制间隔周期*/
	if (controlDeltaT < 0.001f)
	{
		return 0;
	}
	
	/*1.偏差计算*/
	/*保存上次偏差*/
	pidLink->lastError = pidLink->error;	
	/*期望减去反馈得到偏差*/
	pidLink->error = pidLink->expect - pidLink->feedback; 
	/*只对反馈信号微分*/
	pidLink->disErr = -(pidLink->feedback - pidLink->lastFeedBack);
	/*记录上次反馈值*/
	pidLink->lastFeedBack = pidLink->feedback;
	
	/******************************************/
	#if (PID_SYSTEM_CONFIG_USE_AVERAGE_FILTER == SYS_ENABLE)
	/*均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常*/		
	fp32 tempa, tempb, tempc, max, min;  /*用于防跳变滤波*/	
	
	tempa = pidLink->lastLastDisErrLPF; 
	tempb = pidLink->lastDisErrLPF;
	tempc = pidLink->disErr;
	max = tempa > tempb ? tempa : tempb;
	max = max > tempc ? max : tempc;
	min = tempa < tempb ? tempa : tempb;
	min = min < tempc ? min : tempc;
	if (tempa > min && tempa < max)    pidLink->disErr = tempa;
	if (tempb > min  && tempb < max )  pidLink->disErr = tempb;
	if (tempc > min  &&  tempc < max)  pidLink->disErr = tempc;
	pidLink->lastLastDisErrLPF = pidLink->lastDisErrLPF;
	pidLink->lastDisErrLPF     = pidLink->disErr;
	#endif
	/*****************************************/		
	
	/*数字低通后微分项滑动移除老数据*/
	for (i = 4; i > 0; i--)
	{
		pidLink->disErrHistory[i] = pidLink->disErrHistory[i - 1];
	}
	
	/*加入新数据*/
	pidLink->disErrHistory[0] = filter_Pid_Control_Device_LPF(pidLink->disErr, \
															  &(g_sFilterTarg.PidControlBuff[CONTROLER_LINK]), \
															  &(g_sFilterTarg.PidControlDivPara[0])); /*巴特沃斯低通后得到的微分项,20hz*/
	
	/*偏差限幅标志位*/
	if (pidLink->ERROR_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->error >= pidLink->errorMax)
		{
			pidLink->error = pidLink->errorMax;
		}
		
		if (pidLink->error <= -pidLink->errorMax)
		{
			pidLink->error = -pidLink->errorMax;
		}
	}
	
	/*2.积分计算*/
	/*积分分离标志位*/
	if (pidLink->INTEGRATE_SEPARATION_FLAG == PID_LIMIT_ENABLE)
	{
		if (math_Abs(pidLink->error) <= pidLink->integrateSeparationError)
		{
			pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->error * controlDeltaT; /*kI运算*/
		}
	}
	else
	{
		pidLink->integrate += (pidLink->PidScale.kI * pidLink->PID.kI) * pidLink->error * controlDeltaT;		
	}
	
	/*3.积分限幅*/
	/*积分限制幅度标志*/
	if (pidLink->INTEGRATE_LIMIT_FLAG == PID_LIMIT_ENABLE)
	{
		if (pidLink->integrate >= pidLink->integrateMax)
		{
			pidLink->integrate = pidLink->integrateMax;
		}
		
		if (pidLink->integrate <= -pidLink->integrateMax)
		{
			pidLink->integrate = -pidLink->integrateMax;
		}
	}
	
	/*4.总输出计算*/
	/*输出值递推*/
	pidLink->lastControlOutPut = pidLink->controlOutput;
	/*PID运算*/
	pidLink->controlOutput = (pidLink->PidScale.kP * pidLink->PID.kP) * pidLink->error + \
							  pidLink->integrate + \
							  pidLink->PID.kD * pidLink->disErrHistory[0]; /*微分项来源于巴特沃斯低通滤波器*/

	/*5.总输出限幅*/
	if (pidLink->controlOutput >= pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = pidLink->controlOutPutLimit;
	}
	
	if (pidLink->controlOutput <= -pidLink->controlOutPutLimit)
	{
		pidLink->controlOutput = -pidLink->controlOutPutLimit;
	}
	
	/*6.返回总输出*/
	return (pidLink->controlOutput);	
}

/*PID单环积分复位*/
void pid_Link_Integrate_Reset(PidLink *pidLink)
{
	pidLink->integrate = 0.0f;
}

/*PID水平起飞前积分复位*/
void pid_Horizontal_Takeoff_Integrate_Reset(void)
{
	/*起飞前屏蔽姿态角/角速度环积分*/
	pid_Link_Integrate_Reset(&(g_sPidSystem.PitchAngle));
	pid_Link_Integrate_Reset(&(g_sPidSystem.PitchGyro));	
	pid_Link_Integrate_Reset(&(g_sPidSystem.RollAngle));
	pid_Link_Integrate_Reset(&(g_sPidSystem.RollGyro));	
	pid_Link_Integrate_Reset(&(g_sPidSystem.YawAngle));
	pid_Link_Integrate_Reset(&(g_sPidSystem.YawGyro));

	/*GPS 位置控制速度环积分项*/
	pid_Link_Integrate_Reset(&(g_sPidSystem.LongitudeSpeed));
	pid_Link_Integrate_Reset(&(g_sPidSystem.LatitudeSpeed));
	pid_Link_Integrate_Reset(&(g_sPidSystem.OpticFlowYSpeed));
	pid_Link_Integrate_Reset(&(g_sPidSystem.OpticFlowXSpeed));	
	
	/*光流 控制积分项*/	
}

/*PID水平控制积分复位*/
void pid_Horizontal_GPS_Ctrl_Integrate_Reset(void)
{
	/*位置控制速度环积分项*/
	/*gps*/
	pid_Link_Integrate_Reset(&(g_sPidSystem.LongitudeSpeed));
	pid_Link_Integrate_Reset(&(g_sPidSystem.LatitudeSpeed));
	pid_Link_Integrate_Reset(&(g_sPidSystem.LongitudePosition));
	pid_Link_Integrate_Reset(&(g_sPidSystem.LatitudePosition));		
}

/*PID竖直控制积分复位*/
void pid_Vertical_Ctrl_Integrate_Reset(void)
{
	pid_Link_Integrate_Reset(&(g_sPidSystem.HighAcc));	
	pid_Link_Integrate_Reset(&(g_sPidSystem.HighSpeed));
	pid_Link_Integrate_Reset(&(g_sPidSystem.HighPosition));
}	

/*PID参数保存或重置*/
SYS_RETSTATUS pid_parameter_save_or_reset(PidSystem *pidSystem)
{
	SYS_RETSTATUS retStatus = SYS_RET_SUCC;
	u8 i;
	u32 pidSaveAddr = 0;
	
	/*判断是否满足将地面站设置PID参数写入存储器*/
	if (pidSystem->PidSettingSystem.DO_STATUS == PID_PARAMETER_DO_SAVE)
	{
		/*1.角速度环参数赋值给存储BUFF*/ /*=== PID Group 1 ===*/
		pidSystem->PidSettingSystem.RWBuff[0].value  = pidSystem->PitchGyro.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[1].value  = pidSystem->PitchGyro.PID.kI;
		pidSystem->PidSettingSystem.RWBuff[2].value  = pidSystem->PitchGyro.PID.kD;
		
		pidSystem->PidSettingSystem.RWBuff[3].value  = pidSystem->RollGyro.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[4].value  = pidSystem->RollGyro.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[5].value  = pidSystem->RollGyro.PID.kD;

		pidSystem->PidSettingSystem.RWBuff[6].value  = pidSystem->YawGyro.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[7].value  = pidSystem->YawGyro.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[8].value  = pidSystem->YawGyro.PID.kD;

		/*2.角度环参数赋值给存储BUFF*/ /*=== PID Group 2 ===*/
		pidSystem->PidSettingSystem.RWBuff[9].value  = pidSystem->PitchAngle.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[10].value = pidSystem->PitchAngle.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[11].value = pidSystem->PitchAngle.PID.kD;
		
		pidSystem->PidSettingSystem.RWBuff[12].value = pidSystem->RollAngle.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[13].value = pidSystem->RollAngle.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[14].value = pidSystem->RollAngle.PID.kD;

		pidSystem->PidSettingSystem.RWBuff[15].value = pidSystem->YawAngle.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[16].value = pidSystem->YawAngle.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[17].value = pidSystem->YawAngle.PID.kD;

		/*3.速度和位置环参数赋值给存储BUFF*/ /*=== PID Group 3 ===*/	
		pidSystem->PidSettingSystem.RWBuff[18].value = pidSystem->HighSpeed.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[19].value = pidSystem->HighSpeed.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[20].value = pidSystem->HighSpeed.PID.kD;
		
		pidSystem->PidSettingSystem.RWBuff[21].value = pidSystem->HighPosition.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[22].value = pidSystem->HighPosition.PID.kI;
		pidSystem->PidSettingSystem.RWBuff[23].value = pidSystem->HighPosition.PID.kD;
		
		/*GPS 水平位置和速度*/
		pidSystem->PidSettingSystem.RWBuff[24].value = pidSystem->LatitudeSpeed.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[25].value = pidSystem->LatitudeSpeed.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[26].value = pidSystem->LatitudeSpeed.PID.kD;	
		
		pidSystem->PidSettingSystem.RWBuff[27].value = pidSystem->LatitudePosition.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[28].value = pidSystem->LatitudePosition.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[29].value = pidSystem->LatitudePosition.PID.kD;
		
		pidSystem->PidSettingSystem.RWBuff[30].value = pidSystem->LongitudeSpeed.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[31].value = pidSystem->LongitudeSpeed.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[32].value = pidSystem->LongitudeSpeed.PID.kD;

		pidSystem->PidSettingSystem.RWBuff[33].value = pidSystem->LongitudePosition.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[34].value = pidSystem->LongitudePosition.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[35].value = pidSystem->LongitudePosition.PID.kD;

		/*光流 水平位置和速度*/		
		pidSystem->PidSettingSystem.RWBuff[36].value = pidSystem->OpticFlowXSpeed.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[37].value = pidSystem->OpticFlowXSpeed.PID.kI;
		pidSystem->PidSettingSystem.RWBuff[38].value = pidSystem->OpticFlowXSpeed.PID.kD;
		
		pidSystem->PidSettingSystem.RWBuff[39].value = pidSystem->OpticFlowXPosition.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[40].value = pidSystem->OpticFlowXPosition.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[41].value = pidSystem->OpticFlowXPosition.PID.kD;
		
		pidSystem->PidSettingSystem.RWBuff[42].value = pidSystem->OpticFlowYSpeed.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[43].value = pidSystem->OpticFlowYSpeed.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[44].value = pidSystem->OpticFlowYSpeed.PID.kD;

		pidSystem->PidSettingSystem.RWBuff[45].value = pidSystem->OpticFlowYPosition.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[46].value = pidSystem->OpticFlowYPosition.PID.kI;	
		pidSystem->PidSettingSystem.RWBuff[47].value = pidSystem->OpticFlowYPosition.PID.kD;
		
		/*4.加速度环参数赋值给存储BUFF*/	
		pidSystem->PidSettingSystem.RWBuff[48].value = pidSystem->HighAcc.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[49].value = pidSystem->HighAcc.PID.kI;
		pidSystem->PidSettingSystem.RWBuff[50].value = pidSystem->HighAcc.PID.kD;
		
		/*GPS 水平加速度*/
		pidSystem->PidSettingSystem.RWBuff[51].value = pidSystem->LongitudeAcc.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[52].value = pidSystem->LongitudeAcc.PID.kI;
		pidSystem->PidSettingSystem.RWBuff[53].value = pidSystem->LongitudeAcc.PID.kD;	

		pidSystem->PidSettingSystem.RWBuff[54].value = pidSystem->LatitudeAcc.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[55].value = pidSystem->LatitudeAcc.PID.kI;
		pidSystem->PidSettingSystem.RWBuff[56].value = pidSystem->LatitudeAcc.PID.kD;

		/*光流 水平加速度*/
		pidSystem->PidSettingSystem.RWBuff[57].value = pidSystem->OpticFlowXAcc.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[58].value = pidSystem->OpticFlowXAcc.PID.kI;
		pidSystem->PidSettingSystem.RWBuff[59].value = pidSystem->OpticFlowXAcc.PID.kD;

		pidSystem->PidSettingSystem.RWBuff[60].value = pidSystem->OpticFlowYAcc.PID.kP;
		pidSystem->PidSettingSystem.RWBuff[61].value = pidSystem->OpticFlowYAcc.PID.kI;
		pidSystem->PidSettingSystem.RWBuff[62].value = pidSystem->OpticFlowYAcc.PID.kD;
		
		
		/*将更新参数BUFF写入存储器*/
		pidSaveAddr = AT24CXX_STOR_PID_PARA_ADDR;
		
		for (i = 0; i < PID_PARAMETER_STOR_BUFF_LENTH; i += 3) /*每次写一组参数*/
		{
			bsp_AT24CXX_Write_3_S16Data(&g_sAt24cxx, pidSaveAddr, \
										 pidSystem->PidSettingSystem.RWBuff[i].value, \
										 pidSystem->PidSettingSystem.RWBuff[i + 1].value, \
										 pidSystem->PidSettingSystem.RWBuff[i + 2].value);
			
			/*存储地址偏移:3 * 2Byte*/
			pidSaveAddr += sizeof(s16) * 3;
		}
		
		/*标记存储参数任务状态为无效状态*/
		pidSystem->PidSettingSystem.DO_STATUS = PID_PARAMETER_DO_NULL;
		
		/*完成一次读写,才表示本次写入操作成功*/
		retStatus = pid_Parameter_Read_And_Init(&g_sPidSystem);
		
		/*新参数写入成功自动,软件复位重启*/
		if (retStatus == SYS_RET_SUCC)
		{
			/*中断屏蔽*/
			__disable_irq();		
			
			/*系统复位*/
			NVIC_SystemReset();
		}
	}
	else if (pidSystem->PidSettingSystem.DO_STATUS == PID_PARAMETER_DO_RESET)	/*重置为默认参数*/
	{
		/*PID控制系统初始化*/
		pid_System_Init(g_psPidSystem);	
		
		/*1.角速度环参数赋值给存储BUFF*/
		pidSystem->PidSettingSystem.RWBuff[0].value  = (s16)pidSystem->PitchGyro.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[1].value  = (s16)pidSystem->PitchGyro.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[2].value  = (s16)pidSystem->PitchGyro.PID.kD * 1000;	
		
		pidSystem->PidSettingSystem.RWBuff[3].value  = (s16)pidSystem->RollGyro.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[4].value  = (s16)pidSystem->RollGyro.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[5].value  = (s16)pidSystem->RollGyro.PID.kD * 1000;

		pidSystem->PidSettingSystem.RWBuff[6].value  = (s16)pidSystem->YawGyro.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[7].value  = (s16)pidSystem->YawGyro.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[8].value  = (s16)pidSystem->YawGyro.PID.kD * 1000;

		/*2.角度环参数赋值给存储BUFF*/		
		pidSystem->PidSettingSystem.RWBuff[9].value  = (s16)pidSystem->PitchAngle.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[10].value = (s16)pidSystem->PitchAngle.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[11].value = (s16)pidSystem->PitchAngle.PID.kD * 1000;
		
		pidSystem->PidSettingSystem.RWBuff[12].value = (s16)pidSystem->RollAngle.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[13].value = (s16)pidSystem->RollAngle.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[14].value = (s16)pidSystem->RollAngle.PID.kD * 1000;

		pidSystem->PidSettingSystem.RWBuff[15].value = (s16)pidSystem->YawAngle.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[16].value = (s16)pidSystem->YawAngle.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[17].value = (s16)pidSystem->YawAngle.PID.kD * 1000;

		/*3.速度和位置环参数赋值给存储BUFF*/	
		pidSystem->PidSettingSystem.RWBuff[18].value = (s16)pidSystem->HighSpeed.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[19].value = (s16)pidSystem->HighSpeed.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[20].value = (s16)pidSystem->HighSpeed.PID.kD * 1000;
		
		pidSystem->PidSettingSystem.RWBuff[21].value = (s16)pidSystem->HighPosition.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[22].value = (s16)pidSystem->HighPosition.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[23].value = (s16)pidSystem->HighPosition.PID.kD * 1000;

		/*GPS 水平位置+速度*/
		pidSystem->PidSettingSystem.RWBuff[24].value = (s16)pidSystem->LatitudeSpeed.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[25].value = (s16)pidSystem->LatitudeSpeed.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[26].value = (s16)pidSystem->LatitudeSpeed.PID.kD * 1000;	
		
		pidSystem->PidSettingSystem.RWBuff[27].value = (s16)pidSystem->LatitudePosition.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[28].value = (s16)pidSystem->LatitudePosition.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[29].value = (s16)pidSystem->LatitudePosition.PID.kD * 1000;
		
		pidSystem->PidSettingSystem.RWBuff[30].value = (s16)pidSystem->LongitudeSpeed.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[31].value = (s16)pidSystem->LongitudeSpeed.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[32].value = (s16)pidSystem->LongitudeSpeed.PID.kD * 1000;

		pidSystem->PidSettingSystem.RWBuff[33].value = (s16)pidSystem->LongitudePosition.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[34].value = (s16)pidSystem->LongitudePosition.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[35].value = (s16)pidSystem->LongitudePosition.PID.kD * 1000;
		
		/*光流水平位置+速度*/
		pidSystem->PidSettingSystem.RWBuff[36].value = (s16)pidSystem->OpticFlowXSpeed.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[37].value = (s16)pidSystem->OpticFlowXSpeed.PID.kI * 1000;
		pidSystem->PidSettingSystem.RWBuff[38].value = (s16)pidSystem->OpticFlowXSpeed.PID.kD * 1000;
		
		pidSystem->PidSettingSystem.RWBuff[39].value = (s16)pidSystem->OpticFlowXPosition.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[40].value = (s16)pidSystem->OpticFlowXPosition.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[41].value = (s16)pidSystem->OpticFlowXPosition.PID.kD * 1000;
		
		pidSystem->PidSettingSystem.RWBuff[42].value = (s16)pidSystem->OpticFlowYSpeed.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[43].value = (s16)pidSystem->OpticFlowYSpeed.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[44].value = (s16)pidSystem->OpticFlowYSpeed.PID.kD * 1000;

		pidSystem->PidSettingSystem.RWBuff[45].value = (s16)pidSystem->OpticFlowYPosition.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[46].value = (s16)pidSystem->OpticFlowYPosition.PID.kI * 1000;	
		pidSystem->PidSettingSystem.RWBuff[47].value = (s16)pidSystem->OpticFlowYPosition.PID.kD * 1000;		
		
		/*4.加速度环参数赋值给存储BUFF*/	
		pidSystem->PidSettingSystem.RWBuff[48].value = (s16)pidSystem->HighAcc.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[49].value = (s16)pidSystem->HighAcc.PID.kI * 1000;
		pidSystem->PidSettingSystem.RWBuff[50].value = (s16)pidSystem->HighAcc.PID.kD * 1000;

		/*GPS 水平加速度*/
		pidSystem->PidSettingSystem.RWBuff[51].value = (s16)pidSystem->LongitudeAcc.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[52].value = (s16)pidSystem->LongitudeAcc.PID.kI * 1000;
		pidSystem->PidSettingSystem.RWBuff[53].value = (s16)pidSystem->LongitudeAcc.PID.kD * 1000;	

		pidSystem->PidSettingSystem.RWBuff[54].value = (s16)pidSystem->LatitudeAcc.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[55].value = (s16)pidSystem->LatitudeAcc.PID.kI * 1000;
		pidSystem->PidSettingSystem.RWBuff[56].value = (s16)pidSystem->LatitudeAcc.PID.kD * 1000;

		/*光流 水平加速度*/
		pidSystem->PidSettingSystem.RWBuff[57].value = (s16)pidSystem->OpticFlowXAcc.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[58].value = (s16)pidSystem->OpticFlowXAcc.PID.kI * 1000;
		pidSystem->PidSettingSystem.RWBuff[59].value = (s16)pidSystem->OpticFlowXAcc.PID.kD * 1000;

		pidSystem->PidSettingSystem.RWBuff[60].value = (s16)pidSystem->OpticFlowYAcc.PID.kP * 1000;
		pidSystem->PidSettingSystem.RWBuff[61].value = (s16)pidSystem->OpticFlowYAcc.PID.kI * 1000;
		pidSystem->PidSettingSystem.RWBuff[62].value = (s16)pidSystem->OpticFlowYAcc.PID.kD * 1000;
		
		/*将初始化参数BUFF写入存储器*/
		pidSaveAddr = AT24CXX_STOR_PID_PARA_ADDR;
		
		for (i = 0; i < PID_PARAMETER_STOR_BUFF_LENTH; i += 3)
		{
			bsp_AT24CXX_Write_3_FloatData(&g_sAt24cxx, pidSaveAddr, \
										  pidSystem->PidSettingSystem.RWBuff[i].value, \
										  pidSystem->PidSettingSystem.RWBuff[i + 1].value, \
										  pidSystem->PidSettingSystem.RWBuff[i + 2].value);
			
			/*存储地址偏移:3 * 2Byte*/
			pidSaveAddr += sizeof(s16) * 3;
		}
		
		/*标记存储参数状态机为无效状态*/
		pidSystem->PidSettingSystem.DO_STATUS = PID_PARAMETER_DO_NULL;

		/*完成一次读写,才表示本次写入操作成功*/
		retStatus = pid_Parameter_Read_And_Init(&g_sPidSystem);		
	}
	
	return retStatus;
}

/*读取存储器内部PID参数(初始化)*/
SYS_RETSTATUS pid_Parameter_Read_And_Init(PidSystem *pidSystem)
{
	SYS_RETSTATUS readStatus[PID_PARAMETER_SETTING_NBR] = {SYS_RET_SUCC};
	u8 i, j = 0;
	u16 pidSaveAddr = 0;
	
	/*PID参数在存储器中的起始地址*/
	pidSaveAddr = AT24CXX_STOR_PID_PARA_ADDR;
	
	/*清空PID参数的读写BUFF*/
	memset(pidSystem->PidSettingSystem.RWBuff, 0, PID_PARAMETER_STOR_BUFF_LENTH);
	
	for (i = 0; i < PID_PARAMETER_STOR_BUFF_LENTH; i += 3)
	{
		readStatus[j++] = bsp_AT24CXX_Read_3_S16Data(&g_sAt24cxx, pidSaveAddr, 
													 &(pidSystem->PidSettingSystem.RWBuff[i].value), \
												     &(pidSystem->PidSettingSystem.RWBuff[i + 1].value), \
												     &(pidSystem->PidSettingSystem.RWBuff[i + 2]).value);

		/*读取地址偏移:3 * 2Byte*/
		pidSaveAddr += sizeof(s16) * 3;		
		
		/*开机显示PID控制参数*/
		#if (HCI_OLED_SHOW_PID_CONTROL_PARA_WHEN_BOOT == SYS_ENABLE)
		/*OLED上显示PID读取结果*/
		hci_Show_Control_System_Parameter(readStatus[j - 1], (fp32)pidSystem->PidSettingSystem.RWBuff[i].value * 0.001f, \
															 (fp32)pidSystem->PidSettingSystem.RWBuff[i + 1].value * 0.001f, \
															 (fp32)pidSystem->PidSettingSystem.RWBuff[i + 2].value * 0.001f, \
														     (PID_CONTROLER_LINK)(j - 1), 2000);
		#endif
		
		/*显示PID控制参数*/
		#if (HCI_OLED_SHOW_PID_CONTROL_PARA_WHEN_BOOT != SYS_ENABLE)
		if (g_sUav_Status.UavProgrameStatus.INIT_STATUS == UAV_PROGRAME_INIT_FINISH) /*运行过程中才能显示*/
		{
			/*OLED上显示PID读取结果*/
			hci_Show_Control_System_Parameter(readStatus[j - 1], (fp32)pidSystem->PidSettingSystem.RWBuff[i].value * 0.001f, \
																 (fp32)pidSystem->PidSettingSystem.RWBuff[i + 1].value * 0.001f, \
																 (fp32)pidSystem->PidSettingSystem.RWBuff[i + 2].value * 0.001f, \
																 (PID_CONTROLER_LINK)(j - 1), 2000);
		}
		#endif
	}
	
	/*检测每个PID参数读出状态*/
	for (i = 0; i < j; i++)
	{	
		if (readStatus[i] != SYS_RET_SUCC)
		{
			/*读出失败就用默认参数进行初始化*/
			pid_System_Init(pidSystem);
			
			/*PID参数有效性标记为无效*/
			pidSystem->PidSettingSystem.AVA_STATUS = PID_PARAMETER_DISAVA;
			
			return SYS_RET_FAIL;	/*返回初始化失败*/
		}
	}
	
	/*PID参数读出成功,初始化PID其余参数*/
	pid_System_Init(pidSystem);  /*除了PID三个参数外的其他参数初始化*/
	
	/*1.角速度环参数赋值给存储BUFF*/
	pidSystem->PitchGyro.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[0].value * 0.001f; 
	pidSystem->PitchGyro.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[1].value * 0.001f; 
	pidSystem->PitchGyro.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[2].value * 0.001f; 
	                            
	pidSystem->RollGyro.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[3].value * 0.001f;
	pidSystem->RollGyro.PID.kI = (fp32) pidSystem->PidSettingSystem.RWBuff[4].value * 0.001f;
	pidSystem->RollGyro.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[5].value * 0.001f;
                                   
	pidSystem->YawGyro.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[6].value * 0.001f; 
	pidSystem->YawGyro.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[7].value * 0.001f; 
	pidSystem->YawGyro.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[8].value * 0.001f; 

	/*2.角度环参数赋值给存储BUFF*/		
	pidSystem->PitchAngle.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[9].value * 0.001f; 
	pidSystem->PitchAngle.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[10].value * 0.001f; 	
	pidSystem->PitchAngle.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[11].value * 0.001f; 
                                           
	pidSystem->RollAngle.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[12].value * 0.001f; 
	pidSystem->RollAngle.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[13].value * 0.001f; 
	pidSystem->RollAngle.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[14].value * 0.001f; 
                                           
	pidSystem->YawAngle.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[15].value * 0.001f; 
	pidSystem->YawAngle.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[16].value * 0.001f; 
	pidSystem->YawAngle.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[17].value * 0.001f; 

	/*3.速度和位置环参数赋值给存储BUFF*/	
	pidSystem->HighSpeed.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[18].value * 0.001f; 
	pidSystem->HighSpeed.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[19].value * 0.001f; 
	pidSystem->HighSpeed.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[20].value * 0.001f; 
	
	pidSystem->HighPosition.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[21].value * 0.001f; 
	pidSystem->HighPosition.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[22].value * 0.001f; 
	pidSystem->HighPosition.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[23].value * 0.001f; 
    
	/*GPS 水平速度+位置*/	
	pidSystem->LatitudeSpeed.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[24].value * 0.001f; 
	pidSystem->LatitudeSpeed.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[25].value * 0.001f; 
	pidSystem->LatitudeSpeed.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[26].value * 0.001f; 
	
	pidSystem->LatitudePosition.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[27].value * 0.001f; 
	pidSystem->LatitudePosition.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[28].value * 0.001f; 
	pidSystem->LatitudePosition.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[29].value * 0.001f; 
	                                   
	pidSystem->LongitudeSpeed.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[30].value * 0.001f; 
	pidSystem->LongitudeSpeed.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[31].value * 0.001f; 
	pidSystem->LongitudeSpeed.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[32].value * 0.001f; 
       
	pidSystem->LongitudePosition.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[33].value * 0.001f;
	pidSystem->LongitudePosition.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[34].value * 0.001f;
	pidSystem->LongitudePosition.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[35].value * 0.001f;
	
	/*光流 水平速度+位置*/
	pidSystem->OpticFlowXSpeed.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[36].value * 0.001f;
	pidSystem->OpticFlowXSpeed.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[37].value * 0.001f;
	pidSystem->OpticFlowXSpeed.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[38].value * 0.001f;
	
	pidSystem->OpticFlowXPosition.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[39].value * 0.001f;
	pidSystem->OpticFlowXPosition.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[40].value * 0.001f;
	pidSystem->OpticFlowXPosition.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[41].value * 0.001f;
	
	pidSystem->OpticFlowYSpeed.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[42].value * 0.001f;
	pidSystem->OpticFlowYSpeed.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[43].value * 0.001f;
	pidSystem->OpticFlowYSpeed.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[44].value * 0.001f;
	
	pidSystem->OpticFlowYPosition.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[45].value * 0.001f;
	pidSystem->OpticFlowYPosition.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[46].value * 0.001f;
	pidSystem->OpticFlowYPosition.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[47].value * 0.001f;

	/*4.加速度环参数赋值给存储BUFF*/	
	pidSystem->HighAcc.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[48].value * 0.001f;
	pidSystem->HighAcc.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[49].value * 0.001f; 
	pidSystem->HighAcc.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[50].value * 0.001f; 

	/*GPS 水平加速度*/
	pidSystem->LongitudeAcc.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[51].value * 0.001f;
	pidSystem->LongitudeAcc.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[52].value * 0.001f;
	pidSystem->LongitudeAcc.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[53].value * 0.001f;	

	pidSystem->LatitudeAcc.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[54].value * 0.001f;
	pidSystem->LatitudeAcc.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[55].value * 0.001f;
	pidSystem->LatitudeAcc.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[56].value * 0.001f;

	/*光流 水平加速度*/
	pidSystem->OpticFlowXAcc.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[57].value * 0.001f;
	pidSystem->OpticFlowXAcc.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[58].value * 0.001f;
	pidSystem->OpticFlowXAcc.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[59].value * 0.001f;

	pidSystem->OpticFlowYAcc.PID.kP = (fp32)pidSystem->PidSettingSystem.RWBuff[60].value * 0.001f;
	pidSystem->OpticFlowYAcc.PID.kI = (fp32)pidSystem->PidSettingSystem.RWBuff[61].value * 0.001f;
	pidSystem->OpticFlowYAcc.PID.kD = (fp32)pidSystem->PidSettingSystem.RWBuff[62].value * 0.001f;	
	
	/*PID参数有效性标记为有效*/
	pidSystem->PidSettingSystem.AVA_STATUS = PID_PARAMETER_AVA;	
	
	return SYS_RET_SUCC;	/*返回初始化成功*/
}
