#ifndef _CALIB_SENSORDATA_H_
#define _CALIB_SENSORDATA_H_

#include "sys_Platform.h"
#include "ahrs_Caculation.h"
#include "bsp_BoardLib.h"
#include "remot_DataAnaly.h"
#include "status_Aircraft.h"

#define CALIB_ENTRY_MIN_HOLD_TIME_MS		(1000) /*校准面切换动作最小持续时间(ms)*/
#define CALIB_SIDE_CHANGE_MIN_HOLD_TIME_MS	(500)  /*校准面切换动作最小持续时间(ms)*/

/*=== 传感器校准系统 ===*/
/*传感器是否进入校准功能*/
typedef enum
{
	ENTRY_CALIB_INTO = 1,	/*进入校准*/
	ENTRY_CALIB_EXIT = 0,   /*退出校准*/
	ENTRY_CALIB_NULL = 0xFF,
}ENTRY_CALIB_STATUS;

/*采样进行状态*/
typedef enum
{
	SAMPLE_DATA_START  = 1,
	SAMPLE_DATA_FINISH = 0,
}SAMPLE_DATA_STATUS;

/*当前执行的是采样过程/结果*/
typedef enum
{
	SAMPLE_PROCESS_SAMPLEING = 0,
	SAMPLE_PROCESS_RESULT    = 1,
}SAMPLE_PROCESS_TYPE;

/*单次采样完成状态*/
typedef enum
{
	SINGLE_SAMPLE_SUCC = 1,	/*单次采样成功*/
	SINGLE_SAMPLE_FAIL = 0,	/*单次采样失败*/
}SINGLE_SAMPLE_STATUS;

/*角点采样完成状态*/
typedef enum
{
	POSITION_SAMPLE_SUCC = 1, /*角点采样成功*/
	POSITION_SAMPLE_FAIL = 0, /*角点采样失败*/	
}POSITION_SAMPLE_STATUS;

/*整体采样完成状态*/
typedef enum
{
	WHOLE_SAMPLE_SUCC = 1,	/*整体采样成功*/
	WHOLE_SAMPLE_FAIL = 0,	/*整体采样失败*/
}WHOLE_SAMPLE_STATUS;

/*传感器是否成功校准*/
typedef enum
{
	RESULT_CALIB_SUCC = 1,	 /*传感器校准成功*/
	RESULT_CALIB_FAIL = 0,   /*传感器校准失败*/
}RESULT_CALIB_STATUS;

/*校准面的序号(数组从0开始)*/
typedef enum
{
	SIDE_INDEX_1ST  = 0,	
	SIDE_INDEX_2ND  = 1,
	SIDE_INDEX_3RD  = 2,
	SIDE_INDEX_4TH  = 3,
	SIDE_INDEX_5TH  = 4,
	SIDE_INDEX_6TH  = 5,	
	SIDE_INDEX_TOP  = 6,	
	SIDE_INDEX_NULL = 0xFF,	
}CALIB_SIDE_INDEX;

/*校准方位(点)的序号*/
typedef enum
{
	POSITION_INDEX_1ST  = 0,	
	POSITION_INDEX_2ND  = 1,
	POSITION_INDEX_3RD  = 2,
	POSITION_INDEX_4TH  = 3,
	POSITION_INDEX_5TH  = 4,
	POSITION_INDEX_6TH  = 5,	
	POSITION_INDEX_7TH  = 6,	
	POSITION_INDEX_8TH  = 7,
	POSITION_INDEX_9TH  = 8,
	POSITION_INDEX_10TH = 9,	
	POSITION_INDEX_11TH = 10,
	POSITION_INDEX_12TH = 11,
	POSITION_INDEX_13TH = 12,
	POSITION_INDEX_14TH = 13,	
	POSITION_INDEX_15TH = 14,
	POSITION_INDEX_16TH = 15,
	POSITION_INDEX_17TH = 16,	
	POSITION_INDEX_18TH = 17,	
	POSITION_INDEX_19TH = 18,
	POSITION_INDEX_20TH = 19,
	POSITION_INDEX_21TH = 20,
	POSITION_INDEX_22TH = 21,	
	POSITION_INDEX_23TH = 22,
	POSITION_INDEX_24TH = 23,
	POSITION_INDEX_25TH = 24,	
	POSITION_INDEX_26TH = 25,	
	POSITION_INDEX_27TH = 26,
	POSITION_INDEX_28TH = 27,
	POSITION_INDEX_29TH = 28,
	POSITION_INDEX_30TH = 29,	
	POSITION_INDEX_31TH = 30,
	POSITION_INDEX_32TH = 31,
	POSITION_INDEX_33TH = 32,
	POSITION_INDEX_34TH = 33,
	POSITION_INDEX_35TH = 34,
	POSITION_INDEX_36TH = 35,	
	POSITION_INDEX_NULL = 0xFF,	
}CALIB_POSITION_INDEX;

/*磁力计最小二乘中间变量*/
/*Least Squares Intermediate Variable*/
typedef struct
{
	fp32 x_sumplain;
	fp32 x_sumsq;
	fp32 x_sumcube;

	fp32 y_sumplain;
	fp32 y_sumsq;
	fp32 y_sumcube;

	fp32 z_sumplain;
	fp32 z_sumsq;
	fp32 z_sumcube;

	fp32 xy_sum;
	fp32 xz_sum;
	fp32 yz_sum;

	fp32 x2y_sum;
	fp32 x2z_sum;
	fp32 y2x_sum;
	fp32 y2z_sum;
	fp32 z2x_sum;
	fp32 z2y_sum;	
	
	u32 size;
}Mag_Calib_LSIV;

/*磁力计校准零偏*/
typedef struct
{
	fp32 x;
	fp32 y;
	fp32 z;	
}MagCalibOffset;

/*加速度计校准系统*/
typedef struct
{
	volatile ENTRY_CALIB_STATUS   ENTRY_STATUS;	    	/*加速度计进入校准的状态*/
	volatile SAMPLE_DATA_STATUS	  SAMPLE_STATUS;		/*加速度采样状态*/
	volatile SAMPLE_PROCESS_TYPE  SAMPLE_PROCESS;		/*当前是采样过程/结果*/
	volatile SINGLE_SAMPLE_STATUS SINGLE_STATUS[6];  	/*单次任务采样完成状态(加计6个面)*/
	volatile WHOLE_SAMPLE_STATUS  WHOLE_STATUS;			/*整体采样完成状态*/
	volatile CALIB_SIDE_INDEX     CUR_SIDE_INDEX;		/*当前校准面序号*/
	volatile CALIB_SIDE_INDEX     GONNA_SIDE_INDEX;		/*将要校准面序号*/	
	volatile RESULT_CALIB_STATUS  RESULT_STATUS;		/*最后校准结果*/	
	Acc3f				          sampleData[6];		/*6个面采集的数据*/ 
}AccCalibSystem;

/*磁力计校准系统*/
typedef struct
{
	volatile ENTRY_CALIB_STATUS     ENTRY_STATUS;		     /*磁力计进入校准的状态*/
	volatile SAMPLE_DATA_STATUS	    SAMPLE_STATUS;		     /*磁力计采样状态*/	
	volatile SAMPLE_PROCESS_TYPE    SAMPLE_PROCESS;			 /*当前是采样过程/结果*/	
	volatile SINGLE_SAMPLE_STATUS   SINGLE_STATUS[3];	     /*单次任务采样完成状态(磁计3个面)*/
	volatile POSITION_SAMPLE_STATUS POSITION_STATUS[3][36];  /*角点采样完成状态(磁计3个面,36个角点)*/
	volatile WHOLE_SAMPLE_STATUS    WHOLE_STATUS;		     /*整体采样完成状态*/
	volatile CALIB_SIDE_INDEX       CUR_SIDE_INDEX;	     	 /*当前校准面序号*/
	volatile CALIB_SIDE_INDEX       GONNA_SIDE_INDEX;		 /*将要校准面序号*/		
	volatile RESULT_CALIB_STATUS    RESULT_STATUS;			 /*最后校准结果*/		
	CALIB_POSITION_INDEX            POSITION_INDEX;    		 /*校准方位序号*/
	Mag3f				            sampleData[3][36];	     /*3个面采集的数据*/ 

	/*磁力计校准特征*/	
	Mag_Calib_LSIV					Mag_Calib_LSIV;			 /*磁力计最小二乘中间变量*/
	Vector4f						Sphere;					 /*球参数*/
}MagCalibSystem;


/*=== 传感器校准参数 ===*/
typedef struct
{
	fp32 x;
	fp32 y;
	fp32 z;
}Unit3f;

typedef struct
{
	Unit3f Scale;		/*灵敏度*/
	Unit3f Offset;		/*零偏值*/
}AccCalibPara;

typedef struct
{
	Unit3f Offset;		/*零偏值*/
}MagCalibPara;

/*=== 传感器进入校准&校准面检测 ===*/
/*加速度计校准条件检测*/
CALIB_SIDE_INDEX calib_acc_sensor_check(void);

/*磁力计校准条件检测*/
CALIB_SIDE_INDEX calib_mag_sensor_check(void);


/*=== 校准系统重置 ===*/
/*加速度计校准状态清除*/
void calib_Acc_Sensor_System_Reset(void);

/*磁力计校准状态清除*/
void calib_Mag_Sensor_System_Reset(void);

/*=== 传感器进行采集&校准 ===*/
/*加速度计执行采样+校准*/
SYS_RETSTATUS calib_acc_sensor_dp(CALIB_SIDE_INDEX calibSideIndex);

/*磁力计执行采样+校准*/
SYS_RETSTATUS calib_mag_sensor_dp(CALIB_SIDE_INDEX calibSideIndex);

/*检查采样状态*/
SYS_RETSTATUS calib_sample_status_check(u8 *BUFF, u8 CHECK_TARG, u16 totalNbr);

/*=== 校准参数读取&初始化 ===*/
/*加速度传感器校准准参数初始化(读取)*/
SYS_RETSTATUS calib_Acc_Sensor_parameter_Read(void);

/*磁力计传感器校准准参数初始化(读取)*/
SYS_RETSTATUS calib_Mag_Sensor_parameter_Read(void);

/*全部传感器校准参数初始化(读取)*/
SYS_RETSTATUS calib_all_sensor_parameter_read(void);


/*=== 原始数据校准 ===*/
/*加速度计原始数据(椭球)校准*/
Acc3f* calib_Acc_Data_Dp(Acc3s *acc3s);

/*磁力计原始数据(减零偏)校准*/
Mag3f* calib_Mag_Data_Dp(Mag3s *mag3s);


/*=== 磁力计最小二乘法中间值校准 ===*/
u32 calib_mag_sensor_lsiv_accumulate(Mag_Calib_LSIV *magLSIV, Mag3f magf3);
void calib_mag_sensor_lsiv_calculate(Mag_Calib_LSIV *magLSIV, u32 maxIterations, fp32 delta, Vector4f *Sphere);


/*=== 加速度椭球校准法 ===*/
SYS_RETSTATUS calib_Acc_Gauss_Newton_Dp(Acc3f sampleData[], AccCalibPara *accCalibPara);

/*矩阵重置*/
void calib_ElipsoidCorrect_Matrix_Reset(fp32 ds[], fp32 JS[][6]);

/*矩阵更新*/
void calib_ElipsoidCorrect_Matrix_Update(fp32 ds[], fp32 JS[][6], fp32 beta[], fp32 data[]);

/*找到差值*/
void calib_ElipsoidCorrect_Find_Delta(fp32 ds[], fp32 JS[][6], fp32 delta[]);


/*加速度计校准系统*/
extern AccCalibSystem g_sAccCalibSystem;	
extern AccCalibSystem *g_psAccCalibSystem;

/*磁力计校准系统*/
extern MagCalibSystem g_sMagCalibSystem;
extern MagCalibSystem *g_psMagCalibSystem;


/*加速度修正参数*/
/*Acc_Offset*/
extern fp32 g_fpAccOffsetX;
extern fp32 g_fpAccOffsetY;
extern fp32 g_fpAccOffsetZ;

/*Acc_Scale*/
extern fp32 g_fpAccScaleX;
extern fp32 g_fpAccScaleY;
extern fp32 g_fpAccScaleZ;

/*磁力计修正参数*/
/*Mag_Offset*/
extern fp32 g_fpMagOffsetX;
extern fp32 g_fpMagOffsetY;
extern fp32 g_fpMagOffsetZ;

#endif
