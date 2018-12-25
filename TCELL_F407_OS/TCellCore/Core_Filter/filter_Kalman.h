#ifndef _FILTER_KALMAN_H_
#define _FILTER_KALMAN_H_

#include "sys_Platform.h"
#include "sins_Strapdown.h"

#define KALMAN_GPS_PROCESS_NOISE_CONSTANT	(1.0f)

typedef enum
{
	KALMAN_POS = 0, /*位置对象*/
	KALMAN_SPD = 1, /*速度对象*/
}KALMAN_TARG;

/*竖直卡尔曼观测器*/
typedef struct
{
	fp32 R[2];
	fp32 Q;				/*quality factor(品质因数)*/
	fp32 accBiasGain[3];
	fp32 pre_conv[4];	/*present covariance*/
}Filter_Kalman_Vertical;

/*水平卡尔曼观测器*/
typedef struct
{
	fp32 R[2];
	fp32 Q[2];		     /*quality factor(品质因数)*/
	fp32 R_AccBias[2];
	fp32 pre_conv[2][4]; /*present covariance*/
	fp64 k[2][2];		 /*增益矩阵*/
	fp32 accBiasGain[2];
}Filter_Kalman_Horizontal;

/*竖直卡尔曼观测估计*/
void filter_Kalman_Estimate_Vertical(fp32 pos_observation,  /*位置观测量*/
									 u16 estimateDelay, 	/*观测传感器延时*/
									 SINS *SinsKf, 	   		/*惯导结构体*/
									 fp32 driveTarg,	    /*系统原始驱动量*/
									 Filter_Kalman_Vertical *Kalman,
									 EARTH_FRAME_AXIS AXIS,
								     fp32 deltaT);

/*水平(GPS)卡尔曼观测估计*/
void filter_Kalman_Estimate_GPS_Horizontal(fp32 pos_observation,    /*位置观测量*/
									       fp32 speeed_observation, /*速度观测量*/
									       fp32 gpsQuality,			/*GPS定位质量*/
									       u16 estimateDelay, 	    /*观测传感器延时*/
									       SINS *SinsKf, 	   		/*惯导结构体*/
									       Filter_Kalman_Horizontal *Kalman,
									       EARTH_FRAME_AXIS AXIS,
								           fp32 deltaT);

/*竖直卡尔曼观测器*/
extern Filter_Kalman_Vertical g_sFilterKalmanVertical;

/*水平GPS卡尔曼观测器*/
extern Filter_Kalman_Horizontal g_sFilter_Kalman_GPS_Horizontal;

#endif
