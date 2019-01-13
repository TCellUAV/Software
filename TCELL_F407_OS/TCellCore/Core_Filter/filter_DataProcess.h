#ifndef _FILTER_ARITHMETIC_H_
#define _FILTER_ARITHMETIC_H_

#include "ahrs_Caculation.h"

/*=========================== 原始数据滤波器 ===========================*/
/*1.限幅滤波(毛刺滤波器)*/
#define FILTER_BASE_BUFF_LEN (10)	/*限幅滤波器深度*/

typedef struct
{
	fp32 lastVal;						/*上一次的值*/
	fp32 maxAbs;						/*允许上次本次数值最大差距*/
	fp32 wvBuff[FILTER_BASE_BUFF_LEN];	/*权值数组*/
	u16	 index;							/*顺序*/
	u16	 isUp;							/*标记*/
}FilterBase;

/*毛刺滤波器初始化*/
void filter_Base_Init(FilterBase *filterBase, fp32 maxAbs);

/*对数据进行毛刺滤波*/
fp32 filter_Base_Dp(FilterBase *filterBase, fp32 newVal);

/*2.滑动滤波(抑制高频震荡)*/
#define FILTER_SLIDE_BUFF_LEN (15)	/*滑动平均数组最大长度*/

typedef struct
{
	fp32 array[FILTER_SLIDE_BUFF_LEN];
	u8   arrayLen;
}FilterSlider;

/*滑动滤波器初始化*/
void filter_Slider_Init(FilterSlider *filterSlider, u8 arrayLen);

/*对数据进行滑动平均滤波*/
fp32 filter_Slider_Dp(FilterSlider *filterSlider, fp32 newVal);

/*对数据进行滑动均值滤波*/
fp32 filter_Slider_Average_Dp(FilterSlider *filterSlider, fp32 newVal);

/*3.巴特沃斯二阶滤波器*/
/*巴特沃斯滤波器选择(采样频率&截止频率)*/
/*ACC 加速度低通*/
#define FILTER_LPBW_ACC_200HZ_80HZ_IDX						(0)
#define FILTER_LPBW_ACC_200HZ_60HZ_IDX						(1)
#define FILTER_LPBW_ACC_200HZ_51HZ_IDX						(2)
#define FILTER_LPBW_ACC_200HZ_30HZ_IDX						(3)
#define FILTER_LPBW_ACC_200HZ_20HZ_IDX						(4)
#define FILTER_LPBW_ACC_200HZ_15HZ_IDX						(5)
#define FILTER_LPBW_ACC_200HZ_10HZ_IDX						(6)
#define FILTER_LPBW_ACC_200HZ_5HZ_IDX						(7)
#define FILTER_LPBW_ACC_200HZ_2HZ_IDX						(8)
#define FILTER_LPBW_ACC_200HZ_1HZ_IDX						(9)

/*ACC 加速度带阻*/
#define FILTER_BSBW_ACC_200HZ_30HZ_98HZ_IDX					(0)
#define FILTER_BSBW_ACC_200HZ_30HZ_94HZ_IDX					(1)
		
/*GYRO 角速度低通*/					
#define FILTER_LPBW_GYRO_200HZ_51HZ_IDX						(0)
#define FILTER_LPBW_GYRO_200HZ_30HZ_IDX						(1)

/*GYRO 角速度带阻*/
#define FILTER_BSBW_GYRO_200HZ_30HZ_98HZ_IDX				(0)
#define FILTER_BSBW_GYRO_200HZ_30HZ_94HZ_IDX				(1)
					
/*BARO 气压计低通*/					
#define FILTER_LPBW_BARO_9HZ_3HZ_IDX						(0)
#define FILTER_LPBW_BARO_9HZ_2HZ_IDX						(1)

/*GYRO 角速度模长低通*/					
#define FILTER_LPBW_GYROLENTH_200HZ_5HZ_IDX					(0)

/*PID 控制低通*/
/*para*/
#define FILTER_LPBW_PID_CONTROLER_ERR_200HZ_5HZ_IDX			(0)

#define FILTER_LPBW_PID_CONTROLER_DIV_200HZ_20HZ_IDX		(0)
#define FILTER_LPBW_PID_CONTROLER_DIV_200HZ_30HZ_IDX		(1)

/*buff*/
#define FILTER_LPBW_PID_CONTROLER_PITCH_GYRO_IDX			(00)
#define FILTER_LPBW_PID_CONTROLER_ROLL_GYRO_IDX				(01)
#define FILTER_LPBW_PID_CONTROLER_YAW_GYRO_IDX 				(02)
#define FILTER_LPBW_PID_CONTROLER_PITCH_ANGLE_IDX 			(03)
#define FILTER_LPBW_PID_CONTROLER_ROLL_ANGLE_IDX			(04)
#define FILTER_LPBW_PID_CONTROLER_YAW_ANGLE_IDX				(05)
#define FILTER_LPBW_PID_CONTROLER_HIGH_SPEED_IDX			(06)
#define FILTER_LPBW_PID_CONTROLER_HIGH_POSITION_IDX			(07)
#define FILTER_LPBW_PID_CONTROLER_LATITUDE_SPEED_IDX		(08)
#define FILTER_LPBW_PID_CONTROLER_LATITUDE_POSITION _IDX	(09)
#define FILTER_LPBW_PID_CONTROLER_LONGITUDE_SPEED_IDX		(10)
#define FILTER_LPBW_PID_CONTROLER_LONGITUDE_POSITION_IDX	(11)
#define FILTER_LPBW_PID_CONTROLER_OPTICFLOW_X_SPEED_IDX		(12)
#define FILTER_LPBW_PID_CONTROLER_OPTICFLOW_X_POSITION_IDX	(13)
#define FILTER_LPBW_PID_CONTROLER_OPTICFLOW_Y_SPEED_IDX		(14)
#define FILTER_LPBW_PID_CONTROLER_OPTICFLOW_Y_POSITION_IDX	(15)
#define FILTER_LPBW_PID_CONTROLER_HIGH_ACC_IDX 				(16)
#define FILTER_LPBW_PID_CONTROLER_LONGITUDE_ACC_IDX 		(17)
#define FILTER_LPBW_PID_CONTROLER_LATITUDE_ACC_IDX			(18)
#define FILTER_LPBW_PID_CONTROLER_OPTICFLOW_X_ACC_IDX 		(19)
#define FILTER_LPBW_PID_CONTROLER_OPTICFLOW_Y_ACC_IDX		(20)

/*滤波器类型*/
typedef enum
{
	FILTER_TYPE_LOWPASS  = 0, /*低通*/
	FILTER_TYPE_BANDSTOP = 1, /*带阻*/
	FILTER_TYPE_BANDPASS = 2, /*带通*/
	FILTER_TYPE_NOTCH    = 3, /*陷波*/
}FILTER_TYPE;

/*滤波器参数*/
typedef struct
{
	FILTER_TYPE FTYPE;
	u16  		FS;	    /*采样频率*/
	/*低通*/
	u16  		FC;	    /*截止频率*/
	/*带通/带阻*/
	u16  		bandLow;   /*低带*/
	u16  		bandHigh;  /*高带*/
	/*陷波*/
	u16  		notch;     /*陷波*/
	u16  		bandWidth; /*带宽*/
	
	fp32 		A[3];	    /*分母*/
	fp32 		B[3];   	/*分子*/
}FilterLpButterworthPara;

/*滤波器输入输出缓存*/
typedef struct
{
	fp32 Input[3];		/*输入*/
	fp32 Output[3];		/*输出*/
}FilterLpButterworthBuff;

/*巴特沃斯二阶低通滤波器初始化:MATLAB计算*/
void filter_LpButterworth_Matlab_Init(void);

/*巴特沃斯二阶低通滤波器初始化:公式计算*/
void filter_LpButterworth_Formulary_Init(u16 freqSample, u16 freqCut, FilterLpButterworthPara *para);

/*加速度3轴(姿态) 巴特沃斯二阶低通滤波器滤波*/
Acc3f* filter_AccAttLpButterworth_Dp(Acc3f *targAcc, FilterLpButterworthBuff *buff, FilterLpButterworthPara *para);

/*加速度(功能:传感器校准、惯导、控制反馈) 巴特沃斯二阶低通滤波器滤波*/
fp32 filter_AccFuncLpButterworth_Dp(fp32 currInput, FilterLpButterworthBuff *buff, FilterLpButterworthPara *para);

/*角速度 巴特沃斯二阶低通滤波器滤波*/
fp32 filter_GyroFuncLpButterworth_Dp(fp32 currInput, FilterLpButterworthBuff *buff, FilterLpButterworthPara *para);

/*气压计高度 巴特沃斯二阶低通滤波器滤波*/
fp32 filter_BaroAltitudeLpButterworth_Dp(fp32 currInput, FilterLpButterworthBuff *buff, FilterLpButterworthPara *para);

/*光流累计像素点 巴特沃斯二阶低通滤波器滤波*/
fp32 filter_OpFlowIntPixLpButterworth_Dp(fp32 currInput, FilterLpButterworthBuff *buff, FilterLpButterworthPara *para);

/*PID偏差量 巴特沃斯二阶低通滤波器滤波*/
fp32 filter_Pid_Control_Device_LPF(fp32 currInput, FilterLpButterworthBuff *buff, FilterLpButterworthPara *para);

/*=========================== 姿态解算估计滤波器 ===========================*/
/*1.一阶互补滤波估计姿态角*/
typedef struct
{
	fp32 K;
	fp32 Angle;
}FilterYijieHubu;

/*一阶互补初始化*/
void filter_YijieHubu_Init(FilterYijieHubu *filterYijieHubu);

/*一阶互补滤波滤波*/
fp32 filter_YijieHubu_Dp(FilterYijieHubu *filterYijieHubu, fp32 angle_m, fp32 gyro_m, fp32 dtS);	

/*2.二阶互补滤波估计姿态角*/
typedef struct
{
	fp32 K;
	fp32 Angle;
	fp32 x1;
	fp32 x2;
	fp32 y1;
}FilterErjieHubu;

/*二阶互补初始化*/
void filter_ErjieHubu_Init(FilterErjieHubu *filterErjieHubu);

/*二阶互补滤波*/
fp32 filter_ErjieHubu_Dp(FilterErjieHubu *filterErjieHubu, fp32 angle_m, fp32 gyro_m, fp32 dtS);	


/****** 数据处理 + 姿态解算 ******/
typedef struct
{
	/*加速度 二阶巴特沃斯低通滤波器*/
	FilterLpButterworthPara AccLpBwPara[10];    /*加速度原始数据巴特沃斯二阶低通滤波器_10个*/
	FilterLpButterworthBuff AccLpBwAttitude[3];	/*加速度(姿态)巴特沃斯二阶低通滤波器输入输出缓存*/
	FilterLpButterworthBuff AccLpBwFeedback[3];	/*加速度(反馈)巴特沃斯二阶低通滤波器输入输出缓存*/
	FilterLpButterworthBuff AccLpBwSINS[3];		/*加速度(惯导融合)巴特沃斯二阶低通滤波器输入输出缓存*/
	FilterLpButterworthBuff AccLpBwCalib[3];	/*磁力计(矫正)巴特沃斯二阶低通滤波器输入输出缓存*/	
	FilterLpButterworthBuff AccLpBwControl[3];	/*加速度(控制)巴特沃斯二阶低通滤波器输入输出缓存*/	

	/*加速度 二阶巴特沃斯带阻滤波器*/
	FilterLpButterworthPara AccBsBwPara[2];
	FilterLpButterworthBuff AccBsBwBuff[3];
	
	/*角速度 二阶巴特沃斯低通滤波器*/	
	FilterLpButterworthPara GyroLpBwPara[2];	/*角速度原始数据巴特沃斯二阶低通滤波器_2个*/
	FilterLpButterworthBuff GyroLpBwBuff[3];	/*角速度原始数据巴特沃斯二阶低通滤波器输入输出缓存*/
	
	/*角速度 二阶巴特沃斯带阻滤波器*/
	FilterLpButterworthPara GyroBsBwPara[2];
	FilterLpButterworthBuff GyroBsBwBuff[3];	
	
	/*气压计 二阶巴特沃斯低通滤波器*/
	FilterLpButterworthPara BaroAboveLpBwPara[2];	/*气压计(上方)原始数据巴特沃斯二阶低通滤波器_2个*/
	FilterLpButterworthBuff BaroAboveLpBwBuff[2];	/*气压计(上方)原始数据巴特沃斯二阶低通滤波器输入输出缓存*/

	/*光流 二阶巴特沃斯低通滤波器*/
	FilterLpButterworthPara OpticFlowIntPixLpBwPara[1];	/*光流累计像素点巴特沃斯低通滤波*/
	FilterLpButterworthPara OpticFlowGyroLpBwPara[1];	/*光流角速度巴特沃斯低通滤波*/
	FilterLpButterworthBuff OpticFlowIntPixLpBwBuff[2];	/*光流巴特沃斯低通滤波输入输出缓存*/
	FilterLpButterworthBuff OpticFlowGyroLpBwBuff[2];	/*光流巴特沃斯低通滤波输入输出缓存*/	
	
	/*磁力计 窗口滑动平均滤波器*/
	FilterSlider            MagxSliderAverage;  /*磁力计(X轴)原始数据窗口滑动平均滤波*/
	FilterSlider            MagySliderAverage;	/*磁力计(Y轴)原始数据窗口滑动平均滤波*/
	FilterSlider            MagzSliderAverage;	/*磁力计(Z轴)原始数据窗口滑动平均滤波*/
	
	/*超声波 窗口滑动平均滤波器*/
	FilterSlider            UltrSliderAverage;  /*超声波原始数据窗口滑动平均滤波*/
	
	/*角速度模长滤波*/
	FilterLpButterworthPara GyroLenthLpBwPara[1];
	FilterLpButterworthBuff GyroLenthLpBwBuff[1];
	
	/*PID控制 二阶巴特沃斯低通滤波器*/
	FilterLpButterworthPara PidControlErrPara[1];
	FilterLpButterworthPara PidControlDivPara[2];
	FilterLpButterworthBuff PidControlBuff[21]; 	   /*PID 控制器低通输入输出缓冲*/

    FilterYijieHubu         YijieHubuPitch;  		    /*Pitch角一阶互补滤波器*/
    FilterYijieHubu         YijieHubuRoll;    			/*Roll角一阶互补滤波器*/

	FilterErjieHubu 		ErjieHubuPitch;   		    /*Pitch角二阶互补滤波器*/
	FilterErjieHubu 		ErjieHubuRoll;   		    /*Roll角二阶滤波器*/	   
}FilterTarg;


/*原始数据滤波器初始化*/
void filter_origin_data_filter_init(FilterTarg *filterTarg);

/*滤波器综合*/
extern FilterTarg g_sFilterTarg;
extern FilterTarg *g_psFilterTarg;

#endif
