#include "ahrs_Caculation.h"
#include "filter_DataProcess.h"
#include "calib_SensorData.h"
#include "sins_Strapdown.h"
#include "host_Transceive.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

/*传感器原始数据和滤波数据*/
/*1.加速度原始数据*/
Acc3s g_sAccRaw = {0};
Acc3s *g_psAccRaw = &g_sAccRaw;
/*加速度校准后的数据*/
Acc3f g_sAccCorrect = {0};
Acc3f *g_psAccCorrect = &g_sAccCorrect;
/*加速度滤波处理*/
/*带阻*/
Acc3f g_sAccBwBF = {0};
Acc3f *g_psAccBwBF = &g_sAccBwBF;
/*低通*/
Acc3f g_sAccBwLP = {0};
Acc3f *g_psAccBwLP = &g_sAccBwLP;
/*姿态*/
Acc3f g_sAccAttitude = {0};
Acc3f *g_psAccAttitude = &g_sAccAttitude;

/*2.陀螺仪原始数据*/
Gyro3s g_sGyroRaw = {0};
Gyro3s *g_psGyroRaw = &g_sGyroRaw;
/*角速度校准后的数据*/
Gyro3f g_sGyroCorrect = {0};
Gyro3f *g_psGyroCorrect = &g_sGyroCorrect;
/*陀螺仪滤波处理*/
/*带阻*/
Gyro3f g_sGyroBwBF = {0};
Gyro3f *g_psGyroBPF = &g_sGyroBwBF;
/*低通*/
Gyro3f g_sGyroBwLP = {0};
Gyro3f *g_psGyroBwLP = &g_sGyroBwLP;
/*姿态*/
Gyro3f g_sGyroAttitude = {0};
Gyro3f *g_psGyroAttitude = &g_sGyroAttitude;

/*3.磁力计原始数据*/
Mag3s g_sMagRaw = {0};
Mag3s *g_psMagRaw = &g_sMagRaw;
/*磁力计校准后的数据*/
Mag3f g_sMagCorrect = {0};
Mag3f *g_psMagCorrect = &g_sMagCorrect;
/*磁力计滤波后的数据*/
Mag3f g_sMagFilter = {0};
Mag3f *g_psMagFilter = &g_sMagFilter;
/*磁力计姿态解算*/
MagAtt g_sMagAttitude = {0};
MagAtt *g_psMagAttitude = &g_sMagAttitude;

/*AHRS表示姿态*/
AhrsAttitude g_sAhrsAttitude = {0};
AhrsAttitude *g_psAhrsAttitude = &g_sAhrsAttitude;

/*角速度积分姿态角,用于磁力计校准*/
AhrsAttitude g_sCircleAngle = {0};
AhrsAttitude *g_psCircleAngle = &g_sCircleAngle;

/*四元数值*/
AhrsQuater g_sAhrsQuater = 
{
	.q0 = 1.0f,
	.q1 = 0,
	.q2 = 0,
	.q3 = 0,
};

AhrsQuater *g_psAhrsQuater = &g_sAhrsQuater;

/*四元数初始化值*/
AhrsQuaterInit g_sAhrsQuaterInit = {0};
AhrsQuaterInit *g_psAhrsQuaterInit = &g_sAhrsQuaterInit;

/*用于磁力计6面校准*/
Acc3s g_sAccCalib = {0};
Acc3s *g_psAccCalib = &g_sAccCalib;

/*经过椭球校正后的三轴加速度量*/
Acc3f g_sAccOrigion = {0};	
Acc3f *g_psAccOrigion = &g_sAccOrigion;

/*惯导加速度*/
Acc3f g_sAccSINS = {0};
Acc3f *g_psAccSINS = &g_sAccSINS;

/*控制加速度*/
Acc3f g_sAccControl = {0};
Acc3f *g_psAccControl = &g_sAccControl;

/*控制反馈加速度*/
Acc3f g_sAccCtlFeedback = {0};
Acc3f *g_psAccCtlFeedback = &g_sAccCtlFeedback;

/*方向余弦矩阵*/
fp32 rMatrix[3][3] = {0}; 
Vector3f g_sBodyFrame = {0}, g_sEarthFrame = {0};	/*机体座标系和导航坐标系*/
Vector3f *g_psBodyFrame  = &g_sBodyFrame;
Vector3f *g_psEarthFrame = &g_sEarthFrame;

/*欧拉角三角函数*/
fp32 SIN_PITCH = 0, SIN_ROLL = 0, SIN_YAW = 0, COS_PITCH = 0, COS_ROLL = 0, COS_YAW = 0;

/*角度,角速度反馈*/
AngleGyro g_sGyroFeedback = {0};
AngleGyro *g_psGyroFeedback = &g_sGyroFeedback;

/*传感器 原始/处理后的数据更新标志*/
/*加速度计数据状态*/
AccSensorDataStatus g_sAccSensorDataStatus = 
{
	.raw   = SENSOR_DATA_OLD,
	.calib = SENSOR_DATA_OLD,	
};
AccSensorDataStatus *g_psAccSensorDataStatus = &g_sAccSensorDataStatus;

/*磁力计数据状态*/
MagSensorDataStatus g_sMagSensorDataStatus = 
{
	.raw   = SENSOR_DATA_OLD,
	.calib = SENSOR_DATA_OLD,	
};
MagSensorDataStatus *g_psMagSensorDataStatus = &g_sMagSensorDataStatus;


/*====== 初始化欧拉角和四元素 ======*/
/*初始化欧拉角来初始化四元数*/
AhrsQuaterInit* ahrs_euler_to_quaternion(AhrsQuaterInit* ahrsQuaterInit)
{
	AhrsEulerInit ahrsEulerInit;
	fp32 eulerPitch, eulerRoll, eulerYaw;
	
	/*获取初始化的欧拉角*/
	ahrs_get_init_euler(&ahrsEulerInit);
	
	/*赋值*/
	eulerPitch = ahrsEulerInit.pitchInit;
	eulerRoll  = ahrsEulerInit.rollInit;
	eulerYaw   = ahrsEulerInit.yawInit;
	
	/*初始化欧拉角求初始化四元数:X-Y-Z轴一次转过α(Roll),β(Pitch),γ(Yaw)*/
	ahrsQuaterInit->q0_Init = cos(eulerRoll/2) * cos(eulerPitch/2) * cos(eulerYaw/2) + \
							  sin(eulerRoll/2) * sin(eulerPitch/2) * sin(eulerYaw/2);
	
	ahrsQuaterInit->q1_Init = sin(eulerRoll/2) * cos(eulerPitch/2) * cos(eulerYaw/2) - \
							  cos(eulerRoll/2) * sin(eulerPitch/2) * sin(eulerYaw/2);
	
	ahrsQuaterInit->q2_Init = cos(eulerRoll/2) * sin(eulerPitch/2) * cos(eulerYaw/2) + \
							  sin(eulerRoll/2) * cos(eulerPitch/2) * sin(eulerYaw/2);
	
	ahrsQuaterInit->q3_Init = cos(eulerRoll/2) * cos(eulerPitch/2) * sin(eulerYaw/2) - \
							  sin(eulerRoll/2) * sin(eulerPitch/2) * cos(eulerYaw/2);						  
	
	return ahrsQuaterInit;
}

/*获取初始化的欧拉角*/
AhrsEulerInit* ahrs_get_init_euler(AhrsEulerInit* ahrsEulerInit)
{
	u8 i;
	
	/*获取IMU数据(加速度和角速度)并校准滤波(二阶巴特沃斯低通)*/
	ahrs_imu_data_get_and_dp();
	
	/*加速度解算Pitch和Roll*/
	g_psAhrsAttitude = ahrs_euler_by_acc(g_psAccAttitude);
	
	/*等待磁力计的yaw角数据稳定*/
	for (i = 0; i < 5; i++)
	{
		/*获取MAG数据(磁力计)并滤波(窗口滑动滤波)*/
		ahrs_mag_data_get_and_dp();
		
		g_psAhrsAttitude->yaw = g_sMagAttitude.magYaw;

		sys_DelayMs(5);
	}
	
	/*初始化欧拉角的度转弧度*/
	ahrsEulerInit->rollInit  = g_psAhrsAttitude->roll  * DEG2RAD;
	ahrsEulerInit->pitchInit = g_psAhrsAttitude->pitch * DEG2RAD;
	ahrsEulerInit->yawInit   = g_psAhrsAttitude->yaw   * DEG2RAD;
	
	return ahrsEulerInit;
}

/*====== 姿态解算 ======*/
/*1. 加速度直接解算姿态角*/
AhrsAttitude *ahrs_euler_by_acc(Acc3f *acc3f)
{
	fp32 ax, ay, az;
	
	ax = acc3f->x;
	ay = acc3f->y;
	az = acc3f->z;
	
	/*pitch俯仰角_绕X轴旋转(弧度转度)*/
	g_sAhrsAttitude.pitch = atan(ay * math_InvSqrt(ax * ax + az * az)) * RAD2DEG;	
	
	/*roll横滚角_绕Y轴旋转(弧度转度)*/
	g_sAhrsAttitude.roll  = -atan(ax * math_InvSqrt(ay * ay + az * az)) * RAD2DEG;
	
	return &g_sAhrsAttitude;
}

/*有效重力加速度处理函数*/
Vector3f g_sGpsBodyMotionAcc  = {0}; /*GPS_观测机体系运动加速度*/
Vector3f g_sGpsEarthMotionAcc = {0}; /*GPS_观测导航系运动加速度*/

SYS_RETSTATUS get_effective_gravity_acc(Uav_Status *uavStatus, GPS_Data gpsData)
{
	/*判断GPS定位数据可用性*/
	if (uavStatus->UavSenmodStatus.Horizontal.Gps.DATA_STATUS != UAV_SENMOD_DATA_OK)
	{
		return SYS_RET_FAIL;
	}
	
	/*计算GPS导航系下机体运动速度*/
	g_sGpsEarthMotionAcc.x = gpsData.DeltaSpeed.east * 0.01f;  /*m/s^2*/
	g_sGpsEarthMotionAcc.y = gpsData.DeltaSpeed.north * 0.01f; /*m/s^2*/
	g_sGpsEarthMotionAcc.z = gpsData.DeltaSpeed.up * 0.01f;    /*m/s^2*/
	
	/*导航系旋转到机体系*/
	ahrs_vector_earth_to_body(&g_sGpsEarthMotionAcc, &g_sGpsBodyMotionAcc);
	
	/*将GPS观测的运动加速度，量化为数字量*/
	g_sGpsBodyMotionAcc.x *= (ACC_MAX_ONE_G / GRAVITY_STD);
	g_sGpsBodyMotionAcc.y *= (ACC_MAX_ONE_G / GRAVITY_STD);
	g_sGpsBodyMotionAcc.z *= (ACC_MAX_ONE_G / GRAVITY_STD);
	
	return SYS_RET_SUCC;
}

/*四元数互补滤波解算姿态*/
AhrsAttitude* ahrs_quaternion_complement_calculat_attitude(AhrsQuater *ahrsQuater, Acc3f *nowAcc, Gyro3f *nowGyro, Mag3f *nowMag)
{
	return g_psAhrsAttitude;
}


Gyro3f g_sGyroDelta;			/*角速度增量*/
fp32 g_GyroDeltaLenth = 0;		/*角加速度模长*/
fp32 g_GyroLenth = 0;			/*角速度模长*/
//fp32 g_GyroLenthFilter = 0;		/*角速度模长滤波*/

/*历史四元数*/
#define QUAD_HISTORY_DEPTH		(20)
Vector4q g_sQuadHistory[QUAD_HISTORY_DEPTH] = {0};
#define QUAD_HISTORY_SELECT 	(9)

/*历史加速度*/
#define ACC_HISTORY_DEPTH		(20)
Vector3f g_sAccHistory[QUAD_HISTORY_DEPTH] = {0};
#define ACC_HISTORY_SELECT 		(9)

/*历史角速度*/
#define GYRO_HISTORY_DEPTH		(10)
Vector3f g_sGyroHistory[GYRO_HISTORY_DEPTH] = {0};

/*自适应互补滤波*/
fp32 g_BetaAdjust[5] = {0.015, 0.003, 0.010, 0.02, 0.005};//{0.04,0.03,0.025,0.02,0.01};{0.05,0.03,0.025,0.02,0.01};
fp32 g_BetaDef = 0.02;

/*陀螺仪z轴角速度积分得到的Yaw角*/	
vfp32 g_IntYaw = 0;	

/*陀螺仪x,y轴角速度求z轴角速度*/
vfp32 g_yawGyroEarthFrame = 0;

/*梯度下降法解算姿态*/
AhrsAttitude* ahrs_grades_calculat_attitude(AhrsQuater *ahrsQuater, Acc3f *attAcc, Gyro3f *attGyro, Gyro3f *filterGyro, fp32 magYaw, AhrsAttitude *ahrsAtt)
{
	u8 i;
	fp32 unitNorm;	/*单位化*/
	fp32 s0, s1, s2, s3;				/*梯度下降算子求出来的姿态*/
	fp32 qDot0, qDot1, qDot2, qDot3;	/*四元数微分方程矫正四元数*/
	fp32 _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	fp32 delta;
	static fp32 vx = 0, vy = 0, vz = 0, ex = 0, ey = 0, ez = 0;
	static fp32 exInt = 0, eyInt = 0,ezInt = 0; 
	static fp32 ahrs_kP = 0, ahrs_kI = 0.01;
	static vu16 sync_cnt = 0;
	fp32 ahrsDeltaT;
	
	/*获取姿态更新的时间信息*/
	get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->AhrsAttitude));
	
	/*换算成秒*/
	ahrsDeltaT = g_psSystemPeriodExecuteTime->AhrsAttitude.DeltaTime / 1000.0f;
	
	/*同步cnt*/
	sync_cnt++;
	
	/*5*4=20ms滑动一次*/
	if (sync_cnt >= 4)
	{		
		/*cnt清0*/
		sync_cnt = 0;
		
		/*数组滑动,移除最老数据*/
		for (i = QUAD_HISTORY_DEPTH - 1; i > 0; i--)
		{
			/*四元数*/
			g_sQuadHistory[i].q0 = g_sQuadHistory[i - 1].q0;
			g_sQuadHistory[i].q1 = g_sQuadHistory[i - 1].q1;
			g_sQuadHistory[i].q2 = g_sQuadHistory[i - 1].q2;
			g_sQuadHistory[i].q3 = g_sQuadHistory[i - 1].q3;
			
			/*加速度*/
			g_sAccHistory[i].x = g_sAccHistory[i - 1].x;
			g_sAccHistory[i].y = g_sAccHistory[i - 1].y;
			g_sAccHistory[i].z = g_sAccHistory[i - 1].z;			
		}
	
		/*上一次的四元数加入历史四元数数组*/
		g_sQuadHistory[0].q0 = ahrsQuater->q0;
		g_sQuadHistory[0].q1 = ahrsQuater->q1;
		g_sQuadHistory[0].q2 = ahrsQuater->q2;
		g_sQuadHistory[0].q3 = ahrsQuater->q3;	
	}
	
	/*新的加速度加入历史加速度数组*/
	g_sAccHistory[0].x = attAcc->x;
	g_sAccHistory[0].y = attAcc->y;
	g_sAccHistory[0].z = attAcc->z;

	/*数组滑动,移除最老数据*/
	for (i = GYRO_HISTORY_DEPTH - 1; i > 0; i--)
	{
		/*角速度*/
		g_sGyroHistory[i].x = g_sGyroHistory[i - 1].x;
		g_sGyroHistory[i].y = g_sGyroHistory[i - 1].y;
		g_sGyroHistory[i].z = g_sGyroHistory[i - 1].z;			
	}	
	
	/*新的角速度加入历史角速度数组*/
	g_sGyroHistory[0].x = g_psGyroFeedback->Pitch;
	g_sGyroHistory[0].y = g_psGyroFeedback->Roll;
	g_sGyroHistory[0].z = g_psGyroFeedback->Yaw;
	
	/*(姿态解算的)角速度的数据求角速度(单位:度/秒(deg/s))*/
	attGyro->x *= MPU_GYRO_RANGE;
	attGyro->y *= MPU_GYRO_RANGE;
	attGyro->z *= MPU_GYRO_RANGE;

	/*角速度,用于姿态控制内环,角速度反馈*/
	g_psGyroFeedback->Pitch = filterGyro->x * MPU_GYRO_RANGE;
	g_psGyroFeedback->Roll  = filterGyro->y * MPU_GYRO_RANGE;
	g_psGyroFeedback->Yaw   = filterGyro->z * MPU_GYRO_RANGE;
	
	/*{-sinθ          cosθsin Φ          cosθcosΦ}*/
	g_yawGyroEarthFrame = -SIN_ROLL * attGyro->x + \
						   COS_ROLL * SIN_PITCH * attGyro->y + \
						   COS_PITCH * COS_ROLL * attGyro->z;
	
	/*角速度反馈量的增量:历史buff[0] - 历史buff[1]*/
	g_sGyroDelta.x = g_sGyroHistory[0].x - g_sGyroHistory[1].x;
	g_sGyroDelta.y = g_sGyroHistory[0].y - g_sGyroHistory[1].y;
	g_sGyroDelta.z = g_sGyroHistory[0].z - g_sGyroHistory[1].z;
	
	/*角加速度模长*/
	g_GyroDeltaLenth = sqrt(g_sGyroDelta.x * g_sGyroDelta.x + \
							g_sGyroDelta.y * g_sGyroDelta.y + \
					        g_sGyroDelta.z * g_sGyroDelta.z);
							  
	/*角速度模长*/
	g_GyroLenth = sqrt(g_psGyroFeedback->Pitch * g_psGyroFeedback->Pitch + \
					   g_psGyroFeedback->Roll  * g_psGyroFeedback->Roll  + \
					   g_psGyroFeedback->Yaw   * g_psGyroFeedback->Yaw);

//	g_GyroLenthFilter = filter_GyroFuncLpButterworth_Dp(g_GyroLenth, &(g_sFilterTarg.GyroLenthLpBwBuff[0]), \
//												        &(g_sFilterTarg.GyroLenthLpBwPara[FILTER_LPBW_GYROLENTH_200HZ_5HZ_IDX]));	
	
	/*加速度计输出有效时,利用加速度计补偿陀螺仪*/
	if (!((attAcc->x == 0.0f) && (attAcc->y == 0.0f) && (attAcc->z == 0.0f)))
	{
		/*提取GPS运动加速度*/
		if (get_effective_gravity_acc(g_psUav_Status, g_psAttitudeAll->GpsData) == SYS_RET_SUCC)
		{
			attAcc->x = g_sAccHistory[ACC_HISTORY_SELECT].x - g_sGpsBodyMotionAcc.x;	/*剔除运动加速度*/
			attAcc->y = g_sAccHistory[ACC_HISTORY_SELECT].y - g_sGpsBodyMotionAcc.y;	/*剔除运动加速度*/
			attAcc->z = g_sAccHistory[ACC_HISTORY_SELECT].z - g_sGpsBodyMotionAcc.z;	/*剔除运动加速度*/			
		}
		else
		{
			attAcc->x = g_sAccHistory[ACC_HISTORY_SELECT].x;
			attAcc->y = g_sAccHistory[ACC_HISTORY_SELECT].y;
			attAcc->z = g_sAccHistory[ACC_HISTORY_SELECT].z;
		}
	
		/*加速度单位化并取倒数*/		
		unitNorm = math_InvSqrt(attAcc->x * attAcc->x + attAcc->y * attAcc->y + attAcc->z * attAcc->z);		
		
		attAcc->x *= unitNorm;
		attAcc->y *= unitNorm;
		attAcc->z *= unitNorm;	
		
		/*避免重复运算*/
		_2q0 = 2.0f * g_sQuadHistory[QUAD_HISTORY_SELECT].q0;
		_2q1 = 2.0f * g_sQuadHistory[QUAD_HISTORY_SELECT].q1;
		_2q2 = 2.0f * g_sQuadHistory[QUAD_HISTORY_SELECT].q2;
		_2q3 = 2.0f * g_sQuadHistory[QUAD_HISTORY_SELECT].q3;
		_4q0 = 4.0f * g_sQuadHistory[QUAD_HISTORY_SELECT].q0;
		_4q1 = 4.0f * g_sQuadHistory[QUAD_HISTORY_SELECT].q1;
		_4q2 = 4.0f * g_sQuadHistory[QUAD_HISTORY_SELECT].q2;
		_8q1 = 8.0f * g_sQuadHistory[QUAD_HISTORY_SELECT].q1;
		_8q2 = 8.0f * g_sQuadHistory[QUAD_HISTORY_SELECT].q2;
		q0q0 = g_sQuadHistory[QUAD_HISTORY_SELECT].q0 * g_sQuadHistory[QUAD_HISTORY_SELECT].q0;
		q1q1 = g_sQuadHistory[QUAD_HISTORY_SELECT].q1 * g_sQuadHistory[QUAD_HISTORY_SELECT].q1;
		q2q2 = g_sQuadHistory[QUAD_HISTORY_SELECT].q2 * g_sQuadHistory[QUAD_HISTORY_SELECT].q2;
		q3q3 = g_sQuadHistory[QUAD_HISTORY_SELECT].q3 * g_sQuadHistory[QUAD_HISTORY_SELECT].q3;
		
		/*梯度下降算法,计算误差函数的梯度*/
		s0 = _4q0 * q2q2 + _2q2 * attAcc->x + _4q0 * q1q1 - _2q1 * attAcc->y;
		
		s1 = _4q1 * q3q3 - _2q3 * attAcc->x + 4.0f * q0q0 * g_sQuadHistory[QUAD_HISTORY_SELECT].q1 - \
			 _2q0 * attAcc->y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * attAcc->z;
			
		s2 = 4.0f * q0q0 * g_sQuadHistory[QUAD_HISTORY_SELECT].q2 + _2q0 * attAcc->x + _4q2 * q3q3 - \
			 _2q3 * attAcc->y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * attAcc->z;
			
		s3 = 4.0f * q1q1 * g_sQuadHistory[QUAD_HISTORY_SELECT].q3 - _2q1 * attAcc->x + 4.0f * q2q2 * \
			 g_sQuadHistory[QUAD_HISTORY_SELECT].q2 - _2q2 * attAcc->y;
		
		/*梯度归一化*/
		unitNorm = math_InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		
		s0 *= unitNorm;
		s1 *= unitNorm;
		s2 *= unitNorm;
		s3 *= unitNorm;		
		
		/*求自适应滤波参数*/
//		g_BetaDef =  g_BetaAdjust[1] + 0.025f * math_Constrain(g_GyroLenthFilter, 400, 0) * ahrsDeltaT;
//		g_BetaDef -= 0.01f * (math_Constrain(g_SinsAccLenth, 1000, 0) / 1000); /*动态步长,正常悬停在500以内*/
//		g_BetaDef =  math_Constrain(g_BetaDef, 0.06, 0.0075);

		g_BetaDef = g_BetaAdjust[1] + 0.01f * g_GyroLenth * ahrsDeltaT;
		g_BetaDef = math_Constrain(g_BetaDef, 0.04f, 0.0f);
			
		vx = 2 * (g_sQuadHistory[QUAD_HISTORY_SELECT].q1 * g_sQuadHistory[QUAD_HISTORY_SELECT].q3 - \
				  g_sQuadHistory[QUAD_HISTORY_SELECT].q0 * g_sQuadHistory[QUAD_HISTORY_SELECT].q2);
				  
		vy = 2 * (g_sQuadHistory[QUAD_HISTORY_SELECT].q2 * g_sQuadHistory[QUAD_HISTORY_SELECT].q3 + \
				  g_sQuadHistory[QUAD_HISTORY_SELECT].q0 * g_sQuadHistory[QUAD_HISTORY_SELECT].q1);		
				  
		vz = 1 - 2 * (g_sQuadHistory[QUAD_HISTORY_SELECT].q1 * g_sQuadHistory[QUAD_HISTORY_SELECT].q1 + \
				      g_sQuadHistory[QUAD_HISTORY_SELECT].q2 * g_sQuadHistory[QUAD_HISTORY_SELECT].q2);				  
		
		ex = (attAcc->y * vz) - (attAcc->z * vy);
		ey = (attAcc->z * vx) - (attAcc->x * vz);
		ez = (attAcc->x * vy) - (attAcc->y * vx);
		
		exInt += ex * ahrs_kI * ahrsDeltaT;
		eyInt += ey * ahrs_kI * ahrsDeltaT;
		ezInt += ez * ahrs_kI * ahrsDeltaT;		
	}
	
	/* 转换为弧度制,用于姿态更新*/
	attGyro->x = attGyro->x * DEG2RAD + exInt + ahrs_kP * ex;
	attGyro->y = attGyro->y * DEG2RAD + eyInt + ahrs_kP * ey;
	attGyro->z = attGyro->z * DEG2RAD + ezInt + ahrs_kP * ez;
	
	/*四元数微分方程计算本次待矫正四元数*/
	qDot0 = 0.5f * (-ahrsQuater->q1 * attGyro->x - \
					 ahrsQuater->q2 * attGyro->y - \
					 ahrsQuater->q3 * attGyro->z);
					
	qDot1 = 0.5f * ( ahrsQuater->q0 * attGyro->x + \
					 ahrsQuater->q2 * attGyro->z - \
					 ahrsQuater->q3 * attGyro->y);
						
	qDot2 = 0.5f * ( ahrsQuater->q0 * attGyro->y - \
					 ahrsQuater->q1 * attGyro->z + \
					 ahrsQuater->q3 * attGyro->x);
						
	qDot3 = 0.5f * ( ahrsQuater->q0 * attGyro->z + \
					 ahrsQuater->q1 * attGyro->y - \
					 ahrsQuater->q2 * attGyro->x);
			
	qDot0 -= g_BetaDef * s0;
	qDot1 -= g_BetaDef * s1;
	qDot2 -= g_BetaDef * s2;
	qDot3 -= g_BetaDef * s3;	
		
	/*补偿由四元数微分方程引入的姿态误差*/
	delta  = (ahrsDeltaT * attGyro->x) * (ahrsDeltaT * attGyro->x) + \
	 	 	 (ahrsDeltaT * attGyro->y) * (ahrsDeltaT * attGyro->y) + \
			 (ahrsDeltaT * attGyro->z) * (ahrsDeltaT * attGyro->z);
		
	/*将四元数姿态导数积分,得到当前四元数姿态*/
	/*二阶毕卡求解微分方程*/		
	ahrsQuater->q0 = (1.0f - delta / 8.0f) * ahrsQuater->q0 + qDot0 * ahrsDeltaT;
	ahrsQuater->q1 = (1.0f - delta / 8.0f) * ahrsQuater->q1 + qDot1 * ahrsDeltaT;
	ahrsQuater->q2 = (1.0f - delta / 8.0f) * ahrsQuater->q2 + qDot2 * ahrsDeltaT;
	ahrsQuater->q3 = (1.0f - delta / 8.0f) * ahrsQuater->q3 + qDot3 * ahrsDeltaT;
		
	/*四元数单位化*/
	unitNorm = math_InvSqrt(ahrsQuater->q0 * ahrsQuater->q0 + \
							ahrsQuater->q1 * ahrsQuater->q1 + \
							ahrsQuater->q2 * ahrsQuater->q2 + \
							ahrsQuater->q3 * ahrsQuater->q3 );
		
	ahrsQuater->q0 *= unitNorm;
	ahrsQuater->q1 *= unitNorm;
	ahrsQuater->q2 *= unitNorm;
	ahrsQuater->q3 *= unitNorm;
	
	/* 四元数到欧拉角转换,转换顺序为Z-Y-X,参见<Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors>.pdf一文,P24 */
	ahrsAtt->pitch = atan2( 2.0f * ahrsQuater->q2 * ahrsQuater->q3 + 2.0f * ahrsQuater->q0 * ahrsQuater->q1, \
						   -2.0f * ahrsQuater->q1 * ahrsQuater->q1 - 2.0f * ahrsQuater->q2 * ahrsQuater->q2 + 1.0f) * RAD2DEG;
						   
	ahrsAtt->roll  = asin(2.0f * ahrsQuater->q0 * ahrsQuater->q2 - 2.0f * ahrsQuater->q1 * ahrsQuater->q3) * RAD2DEG;;
	
//	ahrsAtt->yaw   = atan2( 2.0f * g_psAhrsQuater->q1 * g_psAhrsQuater->q2 + 2.0f * g_psAhrsQuater->q0 * g_psAhrsQuater->q3, \
//						   -2.0f * g_psAhrsQuater->q3 * g_psAhrsQuater->q3 - 2.0f * g_psAhrsQuater->q2 * g_psAhrsQuater->q2 + 1.0f) * RAD2DEG;
	
	/*陀螺仪积分 + 磁力计Yaw角一阶互补求姿态Yaw角*/
	g_IntYaw += g_yawGyroEarthFrame * ahrsDeltaT;
		
	if (((g_psMagAttitude->magYaw > 90.0f) && (g_IntYaw < -90.0f)) || \
		((g_psMagAttitude->magYaw < -90.0f) && (g_IntYaw > 90.0f)))
	{
		g_IntYaw = -g_IntYaw * 0.99f + g_psMagAttitude->magYaw * 0.01f; /*一阶互补*/
	}
	else
	{
		g_IntYaw = g_IntYaw * 0.99f + g_psMagAttitude->magYaw * 0.01f;  /*一阶互补*/
	} 
	
	if (g_IntYaw < 0)
	{
		ahrsAtt->yaw = g_IntYaw + 360.0f;	/*0~360°*/
	}
	else
	{
		ahrsAtt->yaw = g_IntYaw;
	}
	
	/*如果GPS home点已设置,获取当地磁偏角,得到地理真北*/
	if (get_gps_home_set_status(g_psUav_Status) == UAV_HOME_SET_YES)
	{
		ahrsAtt->yaw -= g_psAttitudeAll->declination;
	}
	
	/*将AHRS姿态(PITCH、ROLL、YAW)赋值给飞行器姿态,后者用于控制系统的反馈量*/
	g_psAttitudeAll->Ahrs.pitch = ahrsAtt->pitch;
	g_psAttitudeAll->Ahrs.roll  = ahrsAtt->roll;
	g_psAttitudeAll->Ahrs.yaw   = ahrsAtt->yaw;
	
	/*计算积分角度,用于磁力计校准*/
	g_psCircleAngle->pitch += g_psGyroFeedback->Pitch * ahrsDeltaT;
	g_psCircleAngle->roll  += g_psGyroFeedback->Roll * ahrsDeltaT;	
	g_psCircleAngle->yaw   += g_psGyroFeedback->Yaw * ahrsDeltaT;
	
	if (g_psCircleAngle->pitch < 0)
	{
		g_psCircleAngle->pitch += 360;
	}
	
	if (g_psCircleAngle->pitch > 360)
	{
		g_psCircleAngle->pitch -= 360;
	}

	if (g_psCircleAngle->roll < 0)
	{
		g_psCircleAngle->roll += 360;
	}
	
	if (g_psCircleAngle->roll > 360)
	{
		g_psCircleAngle->roll -= 360;
	}

	if (g_psCircleAngle->yaw < 0)
	{
		g_psCircleAngle->yaw += 360;
	}
	
	if (g_psCircleAngle->yaw > 360)
	{
		g_psCircleAngle->yaw -= 360;
	}	
	
	return &(g_psAttitudeAll->Ahrs);
}

/*4. 四元数+估算姿态角*/
/*四元数初始化*/
void ahrs_quaternion_init(AhrsQuater *ahrsQuater)
{
	u16 i = 0;
	
	/*初始化欧拉角转初始化四元数*/
	ahrs_euler_to_quaternion(g_psAhrsQuaterInit);
	
	/*初始化的四元数*/
	ahrsQuater->q0 = g_psAhrsQuaterInit->q0_Init;
	ahrsQuater->q1 = g_psAhrsQuaterInit->q1_Init;
	ahrsQuater->q2 = g_psAhrsQuaterInit->q2_Init;
	ahrsQuater->q3 = g_psAhrsQuaterInit->q3_Init;
	
	/*将四元数历史值保存起来*/
	for (i = QUAD_HISTORY_DEPTH - 1; i > 0; i--)
	{
		g_sQuadHistory[i].q0 = ahrsQuater->q0;
		g_sQuadHistory[i].q1 = ahrsQuater->q1;
		g_sQuadHistory[i].q2 = ahrsQuater->q2;
		g_sQuadHistory[i].q3 = ahrsQuater->q3;		
	}
	
	/*加入新值*/
	g_sQuadHistory[0].q0 = ahrsQuater->q0;
	g_sQuadHistory[0].q1 = ahrsQuater->q1;
	g_sQuadHistory[0].q2 = ahrsQuater->q2;
	g_sQuadHistory[0].q3 = ahrsQuater->q3;		
}

/*更新方向余弦矩阵(n系->b系)*/
void ahrs_compute_rotation_matrix(void)
{	
	SIN_PITCH = sin(g_psAhrsAttitude->pitch * DEG2RAD);
	SIN_ROLL  = sin(g_psAhrsAttitude->roll  * DEG2RAD);
	SIN_YAW   = sin(g_psAhrsAttitude->yaw   * DEG2RAD);
	COS_PITCH = cos(g_psAhrsAttitude->pitch * DEG2RAD);
	COS_ROLL  = cos(g_psAhrsAttitude->roll  * DEG2RAD);
	COS_YAW   = cos(g_psAhrsAttitude->yaw   * DEG2RAD);
	
	/*上面变量已宏定义为以下变量*/
	rMatrix[0][0] = COS_YAW * COS_ROLL;
	rMatrix[0][1] = SIN_PITCH * SIN_ROLL * COS_YAW - COS_PITCH * SIN_YAW;
	rMatrix[0][2] = SIN_PITCH * SIN_YAW + COS_PITCH * SIN_ROLL * COS_YAW;	
	
	rMatrix[1][0] = SIN_YAW * COS_ROLL;
	rMatrix[1][1] = SIN_PITCH * SIN_ROLL * SIN_YAW + COS_PITCH * COS_YAW;
	rMatrix[1][2] = COS_PITCH * SIN_ROLL * SIN_YAW - SIN_PITCH * COS_YAW;
	
	rMatrix[2][0] = -SIN_ROLL;
	rMatrix[2][1] = SIN_PITCH * COS_ROLL;
	rMatrix[2][2] = COS_PITCH * COS_ROLL;		
}


/*====== IMU和MAG数据获取及处理(校准、滤波) ======*/
/*IMU数据获取和处理 */
void ahrs_imu_data_get_and_dp(void)
{
	/***1. IMU原始值获取***/
#ifdef MD_IMU__MPU6050
	/*获取加速度值*/
	g_psAccRaw  = bsp_MPU6050_GetAcc(&g_sMpu6050);

	/*获取陀螺仪值*/
	g_psGyroRaw = bsp_MPU6050_GetGyro(&g_sMpu6050);

#endif	
	
#ifdef MD_IMU__MPU6000
	/*获取加速度值*/
	g_psAccRaw  = bsp_MPU6000_GetAcc(&g_sMpu6000);
	
	/*获取陀螺仪值*/
	g_psGyroRaw = bsp_MPU6000_GetGyro(&g_sMpu6000);
#endif
	
	/***2. Acc原始数据校准及滤波*/
	/*Acc: 2rd lpButterWorth FS:200HZ, FC:1HZ (用于加速度计6面校准,和磁力计校准动作条件)*/
	g_psAccCalib->x = (s16)(filter_AccFuncLpButterworth_Dp((fp32)g_psAccRaw->x, &(g_sFilterTarg.AccLpBwCalib[0]), \
														   &(g_sFilterTarg.AccLpBwPara[FILTER_LPBW_ACC_200HZ_1HZ_IDX])));
	g_psAccCalib->y = (s16)(filter_AccFuncLpButterworth_Dp((fp32)g_psAccRaw->y, &(g_sFilterTarg.AccLpBwCalib[1]), \
								                           &(g_sFilterTarg.AccLpBwPara[FILTER_LPBW_ACC_200HZ_1HZ_IDX])));
	g_psAccCalib->z = (s16)(filter_AccFuncLpButterworth_Dp((fp32)g_psAccRaw->z, &(g_sFilterTarg.AccLpBwCalib[2]), \
													       &(g_sFilterTarg.AccLpBwPara[FILTER_LPBW_ACC_200HZ_1HZ_IDX])));

	/*用于加速度计校准,确保数据每次都是更新的*/
	g_psAccSensorDataStatus->calib = SENSOR_DATA_NEW;
	
	/*加速度数据椭球校准*/
	g_psAccCorrect = calib_Acc_Data_Dp(g_psAccRaw);
	
	/*加速度Origion*/
	g_psAccOrigion->x = g_psAccCorrect->x;	
	g_psAccOrigion->y = g_psAccCorrect->y;
	g_psAccOrigion->z = g_psAccCorrect->z;
	
	/*Acc: 2rd lpButterWorth FS:200HZ, FC:30HZ (用于惯导融合的加速度计量)*/
	g_psAccSINS->x = filter_AccFuncLpButterworth_Dp(g_psAccOrigion->x, &(g_sFilterTarg.AccLpBwSINS[0]), \
											        &(g_sFilterTarg.AccLpBwPara[FILTER_LPBW_ACC_200HZ_30HZ_IDX]));
	g_psAccSINS->y = filter_AccFuncLpButterworth_Dp(g_psAccOrigion->y, &(g_sFilterTarg.AccLpBwSINS[1]), \
											        &(g_sFilterTarg.AccLpBwPara[FILTER_LPBW_ACC_200HZ_30HZ_IDX]));
	g_psAccSINS->z = filter_AccFuncLpButterworth_Dp(g_psAccOrigion->z, &(g_sFilterTarg.AccLpBwSINS[2]), \
											        &(g_sFilterTarg.AccLpBwPara[FILTER_LPBW_ACC_200HZ_30HZ_IDX]));		
	
	/*Acc: 2rd lpButterWorth FS:200HZ, FC:15HZ (用于控制反馈)*/
	g_psAccCtlFeedback->x = filter_AccFuncLpButterworth_Dp(g_psAccOrigion->x, &(g_sFilterTarg.AccLpBwFeedback[0]), \
														   &(g_sFilterTarg.AccLpBwPara[FILTER_LPBW_ACC_200HZ_15HZ_IDX]));
	g_psAccCtlFeedback->y = filter_AccFuncLpButterworth_Dp(g_psAccOrigion->y, &(g_sFilterTarg.AccLpBwFeedback[1]), \
														   &(g_sFilterTarg.AccLpBwPara[FILTER_LPBW_ACC_200HZ_15HZ_IDX]));
	g_psAccCtlFeedback->z = filter_AccFuncLpButterworth_Dp(g_psAccOrigion->z, &(g_sFilterTarg.AccLpBwFeedback[2]), \
														   &(g_sFilterTarg.AccLpBwPara[FILTER_LPBW_ACC_200HZ_15HZ_IDX]));	

	/*机体系方向余弦矩阵转导航系方向余弦矩阵*/													
	g_psBodyFrame->x = g_psAccCtlFeedback->x;
	g_psBodyFrame->y = g_psAccCtlFeedback->y;	
	g_psBodyFrame->z = g_psAccCtlFeedback->z;	
	
	g_psEarthFrame = ahrs_vector_body_to_earth(g_psBodyFrame, g_psEarthFrame);
	g_psSinsFilterFeedback->curAcc[EARTH_FRAME_X] = g_psEarthFrame->x;
	g_psSinsFilterFeedback->curAcc[EARTH_FRAME_Y] = g_psEarthFrame->y;
	g_psSinsFilterFeedback->curAcc[EARTH_FRAME_Z] = g_psEarthFrame->z;	
	
	/*加速度转化为G,单位为cm/s^2*/
	g_psSinsFilterFeedback->curAcc[EARTH_FRAME_X] *= SINS_ACC_GRAVITY * MPU_ACC_RANGE;
	g_psSinsFilterFeedback->curAcc[EARTH_FRAME_X] *= 100;	/*m/s^2 -> cm/s^s*/
	
	g_psSinsFilterFeedback->curAcc[EARTH_FRAME_Y] *= SINS_ACC_GRAVITY * MPU_ACC_RANGE;
	g_psSinsFilterFeedback->curAcc[EARTH_FRAME_Y] *= 100;	/*m/s^2 -> cm/s^s*/
	
	g_psSinsFilterFeedback->curAcc[EARTH_FRAME_Z] *= SINS_ACC_GRAVITY * MPU_ACC_RANGE;
	g_psSinsFilterFeedback->curAcc[EARTH_FRAME_Z] -= SINS_ACC_GRAVITY;	/*减去Z轴重力加速度*/
	g_psSinsFilterFeedback->curAcc[EARTH_FRAME_Z] *= 100;	/*m/s^2 -> cm/s^s*/
	
	/*Acc: 2rd lpButterWorth(用于姿态解算)*/
	/*带阻滤波:FS:200HZ, BL:30HZ, BH:94HZ*/	
//	g_psAccBwBF = filter_AccAttLpButterworth_Dp(g_psAccOrigion, g_sFilterTarg.AccBsBwBuff, \
//											    &(g_sFilterTarg.AccBsBwPara[FILTER_BSBW_ACC_200HZ_30HZ_94HZ_IDX]));
	
	/*低通滤波:FS:200HZ, FC:30HZ */	
	g_psAccBwLP = filter_AccAttLpButterworth_Dp(g_psAccOrigion, g_sFilterTarg.AccLpBwAttitude, \
											    &(g_sFilterTarg.AccLpBwPara[FILTER_LPBW_ACC_200HZ_30HZ_IDX]));
										
												   
	/****** 参与姿态解算的加速度数据 ******/	
	g_psAccAttitude->x = g_psAccBwLP->x;
	g_psAccAttitude->y = g_psAccBwLP->y;
	g_psAccAttitude->z = g_psAccBwLP->z;
	
	/***3. Gyro原始数据校准及滤波*/
	/*陀螺仪数据减去零偏值*/
#ifdef MD_IMU__MPU6050
	g_psGyroCorrect->x = g_psGyroRaw->x - g_sMpu6050.GyroZero.x;
	g_psGyroCorrect->y = g_psGyroRaw->y - g_sMpu6050.GyroZero.y;
	g_psGyroCorrect->z = g_psGyroRaw->z - g_sMpu6050.GyroZero.z;	
#endif	
	
#ifdef MD_IMU__MPU6000
	g_psGyroCorrect->x = g_psGyroRaw->x - g_sMpu6000.GyroZero.x;
	g_psGyroCorrect->y = g_psGyroRaw->y - g_sMpu6000.GyroZero.y;
	g_psGyroCorrect->z = g_psGyroRaw->z - g_sMpu6000.GyroZero.z;
#endif

	/*Gyro: 2rd lpButterWorth  (用于姿态解算和反馈)*/
//	
	/*带阻滤波:FS:200HZ, BL:30HZ, BH:98HZ*/
//	g_psGyroAttBPF->x = filter_GyroFuncLpButterworth_Dp(g_psGyroCorrect->x, &(g_sFilterTarg.GyroBsBwBuff[0]), \
//												        &(g_sFilterTarg.GyroBsBwPara[FILTER_BSBW_GYRO_200HZ_30HZ_98HZ_IDX]));
//														
//	g_psGyroAttBPF->y = filter_GyroFuncLpButterworth_Dp(g_psGyroCorrect->y, &(g_sFilterTarg.GyroBsBwBuff[1]), \
//												        &(g_sFilterTarg.GyroBsBwPara[FILTER_BSBW_GYRO_200HZ_30HZ_98HZ_IDX]));

//	g_psGyroAttBPF->z = filter_GyroFuncLpButterworth_Dp(g_psGyroCorrect->z, &(g_sFilterTarg.GyroBsBwBuff[2]), \
//												        &(g_sFilterTarg.GyroBsBwPara[FILTER_BSBW_GYRO_200HZ_30HZ_98HZ_IDX]));
														
	/*低通滤波:FS:200HZ, FC:51HZ*/
	g_psGyroBwLP->x = filter_GyroFuncLpButterworth_Dp(g_psGyroCorrect->x, &(g_sFilterTarg.GyroLpBwBuff[0]), \
												      &(g_sFilterTarg.GyroLpBwPara[FILTER_LPBW_GYRO_200HZ_51HZ_IDX]));	
	g_psGyroBwLP->y = filter_GyroFuncLpButterworth_Dp(g_psGyroCorrect->y, &(g_sFilterTarg.GyroLpBwBuff[1]), \
												      &(g_sFilterTarg.GyroLpBwPara[FILTER_LPBW_GYRO_200HZ_51HZ_IDX]));														
	g_psGyroBwLP->z = filter_GyroFuncLpButterworth_Dp(g_psGyroCorrect->z, &(g_sFilterTarg.GyroLpBwBuff[2]), \
												      &(g_sFilterTarg.GyroLpBwPara[FILTER_LPBW_GYRO_200HZ_51HZ_IDX]));											  
		
	/*参与姿态解算的角速度数据*/		
	g_psGyroAttitude->x = g_psGyroCorrect->x;
	g_psGyroAttitude->y = g_psGyroCorrect->y;
	g_psGyroAttitude->z = g_psGyroCorrect->z;
}

/*Mag数据获取和处理*/
void ahrs_mag_data_get_and_dp(void)
{		
	/*1. Mag原始数据获取*/
#ifdef HW_CUT__USE_GPS_MAG	/*磁力计优先采用GPS磁力计*/
	/*GPS MAG*/
	#ifdef GPS_MAG__AK8975
	/*AK8975*/
	g_psMagRaw = bsp_AK8975_Get_Mag_Data(&g_sAk8975);
	#endif

	#ifdef GPS_MAG__HMC5883L
	/*HMC5883L*/
	g_psMagRaw = bsp_HMC5883L_Get_Mag_Data(&g_sHmc5883l);
	#endif

	#ifdef GPS_MAG__HMC5983
	/*HMC5983*/
	g_psMagRaw = bsp_HMC5983_Get_Mag_Data(&g_sHmc5983);
	#endif

	#ifdef MD_MAG__IST8310
	/*IST8310*/
	g_psMagRaw = bsp_IST8310_Get_Mag_Data(&g_sIst8310);	
	#endif	
	
	#else
	
	/*MD_MAG*/
	#ifdef MD_MAG__AK8975
	/*AK8975*/
	g_psMagRaw = bsp_AK8975_Get_Mag_Data(&g_sAk8975);
	#endif
	
	#ifdef MD_MAG__HMC5883L
	/*HMC5883L*/
	g_psMagRaw = bsp_HMC5883L_Get_Mag_Data(&g_sHmc5883l);
	#endif
	
	#ifdef MD_MAG__IST8310
	/*IST8310*/
	g_psMagRaw = bsp_IST8310_Get_Mag_Data(&g_sIst8310);	
	#endif	
#endif

	/*用于磁力计校准,确保数据每次都是更新的*/
	g_psMagSensorDataStatus->calib = SENSOR_DATA_NEW;
	
	/*2.磁力计数据校准*/
	g_psMagCorrect = calib_Mag_Data_Dp(g_psMagRaw);
	
	/*3.磁力计数据滑动窗口滤波*/
	g_psMagFilter->x = filter_Slider_Average_Dp(&(g_sFilterTarg.MagxSliderAverage), g_psMagCorrect->x);
	g_psMagFilter->y = filter_Slider_Average_Dp(&(g_sFilterTarg.MagySliderAverage), g_psMagCorrect->y);
	g_psMagFilter->z = filter_Slider_Average_Dp(&(g_sFilterTarg.MagzSliderAverage), g_psMagCorrect->z);
	
	/*4.倾角补偿(机体坐标系到导航坐标系)*/
	g_psMagAttitude->hx = g_psMagFilter->x * COS_ROLL + g_psMagFilter->z * SIN_ROLL;
	
	g_psMagAttitude->hy = g_psMagFilter->x * SIN_PITCH * SIN_ROLL + \
						  g_psMagFilter->y * COS_PITCH - \
						  g_psMagFilter->z * COS_ROLL * SIN_PITCH;

	/*5.反正切得到磁力计观测Yaw(偏航)角度*/
	g_psMagAttitude->magYaw = atan2(g_psMagAttitude->hx, g_psMagAttitude->hy) * RAD2DEG;
}


/*向量转换:机体座标系到导航坐标系(frame)*/
Vector3f *ahrs_vector_body_to_earth(Vector3f *bf, Vector3f *ef)
{
	ef->x = rMatrix[0][0] * bf->x + rMatrix[0][1] * bf->y + rMatrix[0][2] * bf->z;
	ef->y = rMatrix[1][0] * bf->x + rMatrix[1][1] * bf->y + rMatrix[1][2] * bf->z;
	ef->z = rMatrix[2][0] * bf->x + rMatrix[2][1] * bf->y + rMatrix[2][2] * bf->z;	
	
	return ef;
}

/*向量旋转:导航座标系到机体座标系*/
Vector3f *ahrs_vector_earth_to_body(Vector3f *ef, Vector3f *bf)
{
	bf->x = rMatrix[0][0] * ef->x + rMatrix[1][0] * ef->y + rMatrix[2][0]*ef->z;
    bf->y = rMatrix[0][1] * ef->x + rMatrix[1][1] * ef->y + rMatrix[2][1]*ef->z;
	bf->z = rMatrix[0][2] * ef->x + rMatrix[1][2] * ef->y + rMatrix[2][2]*ef->z;
	
	return bf;
}
