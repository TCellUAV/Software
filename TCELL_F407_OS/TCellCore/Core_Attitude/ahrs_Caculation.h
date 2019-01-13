#ifndef _AHRS_CACULATION_H_
#define _AHRS_CACULATION_H_

#include "sys_Platform.h"
#include "math_Function.h"
#include "remot_DataAnaly.h"

/****** 姿态表示 ******/
/*IMU姿态*/
typedef struct
{
	fp32 pitch;
	fp32 roll;
}ImuAttitude;

/*Ahrs姿态*/
typedef struct
{
	fp32 pitch;
	fp32 roll;
	fp32 yaw;
}AhrsAttitude;

/*Mag磁力计姿态数据及姿态角*/
typedef struct
{
	fp32 hx;
	fp32 hy;
	fp32 hz;
	fp32 magYaw;
}MagAtt;

/*角度角速度惯导反馈*/
typedef struct
{
	fp32 Pitch;
	fp32 Roll;
	fp32 Yaw;
}AngleGyro;

/*====== 初始化欧拉角和四元数 ======*/
typedef struct
{
	fp32 q0_Init;
	fp32 q1_Init;
	fp32 q2_Init;
	fp32 q3_Init;
}AhrsQuaterInit;

/*初始化欧拉角转初始化四元数*/
AhrsQuaterInit* ahrs_caculation_euler_to_quaternion(AhrsQuaterInit* ahrsQuaterInit);

typedef struct
{
	fp32 pitchInit;
	fp32 rollInit;
	fp32 yawInit;	
}AhrsEulerInit;

/*获取初始化的欧拉角*/
AhrsEulerInit* ahrs_get_init_euler(AhrsEulerInit* ahrsEulerInit);

/*传感器 原始/处理后的数据更新标志*/
typedef enum
{
	SENSOR_DATA_NEW = 0,
	SENSOR_DATA_OLD = 1,
}SENSOR_DATA_STATUS;

/*加速度计数据状态*/
typedef struct
{
	volatile SENSOR_DATA_STATUS raw;		/*原始数据*/
	volatile SENSOR_DATA_STATUS calib;	/*用于加速度计校准数据*/
}AccSensorDataStatus;

/*磁力计数据状态*/
typedef struct
{
	volatile SENSOR_DATA_STATUS raw;		/*原始数据*/
	volatile SENSOR_DATA_STATUS calib;	/*用于磁力计校准数据*/
}MagSensorDataStatus;


/*====== 姿态解算 ======*/
/*1. 加速度直接解算姿态角*/
AhrsAttitude *ahrs_euler_by_acc(Acc3f *acc3f);

/*4. 梯度下降法算姿态角*/
typedef struct
{
	fp32 q0;
	fp32 q1;
	fp32 q2;
	fp32 q3;
}AhrsQuater;

/*四元数初始化*/
void ahrs_quaternion_init(AhrsQuater *ahrsQuater);

/*四元数互补滤波解算姿态*/
AhrsAttitude* ahrs_quaternion_complement_calculat_attitude(AhrsQuater *ahrsQuater, Acc3f *nowAcc, Gyro3f *nowGyro, Mag3f *nowMag);

/*梯度下降法解算姿态*/
AhrsAttitude* ahrs_grades_calculat_attitude(AhrsQuater *ahrsQuater, Acc3f *attAcc, Gyro3f *attGyro, Gyro3f *filterGyro, fp32 magYaw, AhrsAttitude *ahrsAtt);

/*更新方向余弦矩阵*/
void ahrs_compute_rotation_matrix(void);

/*====== IMU和MAG数据获取及处理(校准、滤波) ======*/
/*IMU数据获取和处理 */
void ahrs_imu_data_get_and_dp(void);

/*Mag数据获取和处理*/
void ahrs_mag_data_get_and_dp(void);

/*向量旋转:机体座标系到导航座标系*/
Vector3f *ahrs_vector_body_to_earth(Vector3f *bf, Vector3f *ef);

/*向量旋转:导航座标系到机体座标系*/
Vector3f *ahrs_vector_earth_to_body(Vector3f *ef, Vector3f *bf);


/*传感器原始数据和滤波数据*/
/*1.加速度原始数据*/
extern Acc3s g_sAccRaw;
extern Acc3s *g_psAccRaw;
/*加速度校准后的数据*/
extern Acc3f g_sAccCorrect;
extern Acc3f *g_psAccCorrect;
/*加速度滤波处理*/
/*带阻*/
extern Acc3f g_sAccBwBF;
extern Acc3f *g_psAccBwBF;
/*低通*/
extern Acc3f g_sAccBwLP;
extern Acc3f *g_psAccBwLP;
/*姿态*/
extern Acc3f g_sAccAttitude;
extern Acc3f *g_psAccAttitude;

/*2.陀螺仪原始数据*/
extern Gyro3s g_sGyroRaw;
extern Gyro3s *g_psGyroRaw;
/*角速度校准后的数据*/
extern Gyro3f g_sGyroCorrect;
extern Gyro3f *g_psGyroCorrect;
/*陀螺仪滤波处理*/
/*带阻*/
extern Gyro3f g_sGyroBwBF;
extern Gyro3f *g_psGyroBPF;
/*低通*/
extern Gyro3f g_sGyroBwLP;
extern Gyro3f *g_psGyroBwLP;
/*姿态*/
extern Gyro3f g_sGyroAttitude;
extern Gyro3f *g_psGyroAttitude;

/*3.磁力计原始数据*/
extern Mag3s g_sMagRaw;
extern Mag3s *g_psMagRaw;
/*磁力计校准后的数据*/
extern Mag3f g_sMagCorrect;
extern Mag3f *g_psMagCorrect;
/*磁力计滤波后的数据*/
extern Mag3f g_sMagFilter;
extern Mag3f *g_psMagFilter;
/*磁力计姿态解算*/
extern MagAtt g_sMagAttitude;
extern MagAtt *g_psMagAttitude;

/*AHRS表示姿态*/
extern AhrsAttitude g_sAhrsAttitude;
extern AhrsAttitude *g_psAhrsAttitude;

/*角速度积分姿态角,用于磁力计校准*/
extern AhrsAttitude g_sCircleAngle;
extern AhrsAttitude *g_psCircleAngle;

/*四元数值*/
extern AhrsQuater g_sAhrsQuater;
extern AhrsQuater *g_psAhrsQuater;

/*四元数初始化值*/
extern AhrsQuaterInit g_sAhrsQuaterInit;
extern AhrsQuaterInit *g_psAhrsQuaterInit;

/*用于磁力计6面校准*/
extern Acc3s g_sAccCalib;
extern Acc3s *g_psAccCalib;

/*经过椭球校正后的三轴加速度量*/
extern Acc3f g_sAccOrigion;	
extern Acc3f *g_psAccOrigion;

/*惯导加速度*/
extern Acc3f g_sAccSINS;
extern Acc3f *g_psAccSINS;

/*控制加速度*/
extern Acc3f g_sAccControl;
extern Acc3f *g_psAccControl;

/*控制反馈加速度*/
extern Acc3f g_sAccCtlFeedback;
extern Acc3f *g_psAccCtlFeedback;

/*角度,角速度反馈*/
extern AngleGyro g_sGyroFeedback;
extern AngleGyro *g_psGyroFeedback;

/*方向余弦矩阵*/
extern fp32 rMatrix[3][3]; 
extern Vector3f g_sBodyFrame, g_sEarthFrame;	/*机体座标系和导航坐标系*/
extern Vector3f *g_psBodyFrame;
extern Vector3f *g_psEarthFrame;

/*欧拉角三角函数*/
extern fp32 SIN_PITCH, SIN_ROLL, SIN_YAW, COS_PITCH, COS_ROLL, COS_YAW;

/*角速度模长*/
extern fp32 g_GyroLenth;

/*传感器 原始/处理后的数据更新标志*/
/*加速度计数据状态*/
extern AccSensorDataStatus g_sAccSensorDataStatus;
extern AccSensorDataStatus *g_psAccSensorDataStatus;

/*磁力计数据状态*/
extern MagSensorDataStatus g_sMagSensorDataStatus;
extern MagSensorDataStatus *g_psMagSensorDataStatus;

/*GPS_观测运动加速度*/
extern Vector3f g_sGpsBodyMotionAcc; /*GPS_观测机体系运动加速度*/
extern Vector3f g_sGpsEarthMotionAcc; /*GPS_观测导航系运动加速度*/

#endif
