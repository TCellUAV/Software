#include "filter_Kalman.h"

/*竖直卡尔曼观测器*/
Filter_Kalman_Vertical g_sFilterKalmanVertical = 
{
	/*R*/
	.R = 
	{
		5.0e-4f, 6.0e-4f,
	},
	
	/*quality factor*/
	.Q = 30,
	
	/*bias Gain*/
	.accBiasGain = 
	{
		0.0005f, 0.0005f, 0.0005f,
	},
	
	/*present covariance*/
	.pre_conv = 
	{
		0.18f, 0.1f, 0.1f, 0.18f,
	},
};

/*竖直卡尔曼观测估计*/
void filter_Kalman_Estimate_Vertical(fp32 pos_observation,  /*位置观测量*/
									 u16 estimateDelay, 	/*观测传感器延时*/
									 SINS *SinsKf, 	   		/*惯导结构体*/
									 fp32 driveTarg,	    /*系统原始驱动量*/
									 Filter_Kalman_Vertical *Kalman,
									 EARTH_FRAME_AXIS AXIS,
								     fp32 deltaT)
{
	fp32 conv_z = 0, z_cor = 0;
	fp32 temp_conv[4] = {0};	   /*先验协方差*/
	fp32 k[2] = {0}; 			   /*增益矩阵*/
	fp32 c_temp = 0;
	
	/*先验状态*/
	SinsKf->curAcc[AXIS] = driveTarg;
	SinsKf->curAcc[AXIS] = SinsKf->accOffset[AXIS] + SinsKf->curAcc[AXIS];
	SinsKf->curPosition[AXIS] += SinsKf->curSpeed[AXIS] * deltaT + \
								 (SinsKf->curAcc[AXIS] * deltaT * deltaT) / 2.0f;
	SinsKf->curSpeed[AXIS] += SinsKf->curAcc[AXIS] * deltaT;
	
	/*先验协方差*/
	c_temp = Kalman->pre_conv[1] + Kalman->pre_conv[3] * deltaT;
	temp_conv[0] = Kalman->pre_conv[0] + Kalman->pre_conv[2] * deltaT + c_temp * deltaT + Kalman->R[0];
	temp_conv[1] = c_temp;
	temp_conv[2] = Kalman->pre_conv[2] + Kalman->pre_conv[3] * deltaT;
	temp_conv[3] = Kalman->pre_conv[3] + Kalman->R[1];
	
	/*计算卡尔曼增益*/
	conv_z = temp_conv[0] + Kalman->Q;
	k[0] = temp_conv[0] / conv_z;
	k[1] = temp_conv[2] / conv_z;
	
	/*数据融合输出*/
	z_cor = pos_observation - SinsKf->pos_History[AXIS][estimateDelay];
	SinsKf->curPosition[AXIS] += k[0] * z_cor;
	SinsKf->curSpeed[AXIS] += k[1] * z_cor;
	SinsKf->accOffset[AXIS] += Kalman->accBiasGain[AXIS] * z_cor;
	
	/*更新状态协方差矩阵*/
	Kalman->pre_conv[0] = (1 - k[0]) * temp_conv[0];
	Kalman->pre_conv[1] = (1 - k[0]) * temp_conv[1];
	Kalman->pre_conv[2] = temp_conv[2] - k[1] * temp_conv[0];
	Kalman->pre_conv[3] = temp_conv[3] - k[1] * temp_conv[1];
} 



/*水平GPS卡尔曼观测器*/
Filter_Kalman_Horizontal g_sFilter_Kalman_GPS_Horizontal = 
{	
	/*quality factor*/
	.Q = 
	{
		0.075f, 1.2f,
	},
	
	/*bias Gain*/
	.R_AccBias = 
	{
		0.0001f, 0,
	},
	
	/*present covariance*/
	.pre_conv = 
	{
		0.018f, 0.005f, 0.005f, 0.5f,
		0.018f, 0.005f, 0.005f, 0.5f,
	},
	
	/*k*/
	.K = 
	{
		0
	},
	
	/*accBiasGain*/
	.accBiasGain =
	{
		0, 0.001f,
	},
};

/*水平(GPS)卡尔曼观测估计*/
void filter_Kalman_Estimate_GPS_Horizontal(fp32 pos_observation,    /*位置观测量*/
									       fp32 speed_observation,  /*速度观测量*/
									       fp32 gpsQuality,			/*GPS定位质量*/
									       u16 estimateDelay, 	    /*位置观测传感器延时*/
									       SINS *SinsKf, 	   		/*惯导结构体*/
										   fp32 driveTarg,	   		/*系统原始驱动量*/
									       Filter_Kalman_Horizontal *Kalman,
									       EARTH_FRAME_AXIS AXIS,
								           fp32 deltaT)
{
	fp32 conv_z = 0;
	fp32 z_delta[2] = {0};
	fp32 conv_temp = 0;
	fp64 temp_conv[4] = {0};	/*先验协方差*/
	
	/*计算动态量*/
	Kalman->R[KALMAN_POS] = 0.5f * KALMAN_GPS_PROCESS_NOISE_CONSTANT * KALMAN_GPS_DATA_UPDATE_PERIOD_S * KALMAN_GPS_DATA_UPDATE_PERIOD_S; /*POS Noise :0.005*/
	Kalman->R[KALMAN_SPD] = KALMAN_GPS_PROCESS_NOISE_CONSTANT * KALMAN_GPS_DATA_UPDATE_PERIOD_S;				 					      /*SPEED Noise :0.1*/
	
	/*先验状态*/
	SinsKf->curPosition[AXIS] += SinsKf->curSpeed[AXIS] * deltaT + \
								 ((driveTarg + Kalman->accBiasGain[AXIS]) * deltaT * deltaT) / 2.0f; /*观测本次的位置数据*/
	SinsKf->curSpeed[AXIS] += (driveTarg + Kalman->accBiasGain[AXIS]) * deltaT; 					 /*观测本次的速度数据*/
	
	/*先验协方差*/
	conv_temp = Kalman->pre_conv[AXIS][1] + Kalman->pre_conv[AXIS][3] * deltaT;
	temp_conv[0] = Kalman->pre_conv[AXIS][0] + Kalman->pre_conv[AXIS][2] * deltaT + conv_temp * deltaT + Kalman->R[0];
	temp_conv[1] = conv_temp;
	temp_conv[2] = Kalman->pre_conv[AXIS][2] + Kalman->pre_conv[AXIS][3] * deltaT;
	temp_conv[3] = Kalman->pre_conv[AXIS][3] + Kalman->R[1];
	
	/*计算卡尔曼增益*/
	conv_z = 1.0f / ((temp_conv[0] + Kalman->Q[0] * gpsQuality) * (temp_conv[3] + Kalman->Q[1] * gpsQuality) - temp_conv[1] * temp_conv[2]);
	
	/*化简如下*/
	Kalman->K[0][0] = ( temp_conv[0] * (temp_conv[3] + Kalman->Q[1] * gpsQuality) - temp_conv[1] * temp_conv[2]) * conv_z;
	Kalman->K[0][1] = ( temp_conv[1] * Kalman->Q[0] * gpsQuality) * conv_z;
	Kalman->K[1][0] = ( temp_conv[2] * Kalman->Q[1] * gpsQuality) * conv_z;
	Kalman->K[1][1] = (-temp_conv[1] * temp_conv[2] + temp_conv[3] * (temp_conv[0] + Kalman->Q[0] * gpsQuality)) * conv_z;	
	
	/*融合数据输出*/
	z_delta[0] = pos_observation - SinsKf->pos_History[AXIS][estimateDelay];
	z_delta[1] = speed_observation - SinsKf->speed_History[AXIS][estimateDelay * 2];
	
	/*当前位置和速度更新*/
	SinsKf->curPosition[AXIS] += (Kalman->K[0][0] * z_delta[0]) + (Kalman->K[0][1] * z_delta[1]); /*得到本次的位置输出*/
	SinsKf->curSpeed[AXIS] += (Kalman->K[1][0] * z_delta[0]) + (Kalman->K[1][1] * z_delta[1]); /*得到本次的速度输出*/
	
	Kalman->accBiasGain[AXIS] += (Kalman->R_AccBias[0] * z_delta[0]) + (Kalman->R_AccBias[1] * z_delta[1]); /*加速度互补*/
	
	/*更新状态协方差矩阵*/
	Kalman->pre_conv[AXIS][0] = (1 - Kalman->K[0][0]) * temp_conv[0] - Kalman->K[0][1] * temp_conv[2];
	Kalman->pre_conv[AXIS][1] = (1 - Kalman->K[0][0]) * temp_conv[1] - Kalman->K[0][1] * temp_conv[3];
	Kalman->pre_conv[AXIS][2] = (1 - Kalman->K[1][1]) * temp_conv[2] - Kalman->K[1][0] * temp_conv[0];
	Kalman->pre_conv[AXIS][3] = (1 - Kalman->K[1][1]) * temp_conv[3] - Kalman->K[1][0] * temp_conv[1];	
}
