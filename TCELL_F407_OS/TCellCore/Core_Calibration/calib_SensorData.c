#include "calib_SensorData.h"
#include "ahrs_Caculation.h"
#include "hci_oledshow.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

/*加速度修正参数*/
/*Acc_Offset*/
fp32 g_fpAccOffsetX = 0;
fp32 g_fpAccOffsetY = 0;
fp32 g_fpAccOffsetZ = 0;

/*Acc_Scale*/
fp32 g_fpAccScaleX  = 0;
fp32 g_fpAccScaleY  = 0;
fp32 g_fpAccScaleZ  = 0;

/*磁力计修正参数*/
/*Mag_Offset*/
fp32 g_fpMagOffsetX = 0;
fp32 g_fpMagOffsetY = 0;
fp32 g_fpMagOffsetZ = 0;

/*加速度计校准系统*/
AccCalibSystem g_sAccCalibSystem   = 
{
	.ENTRY_STATUS   = ENTRY_CALIB_NULL,	  /*该值必须初始化NULL*/
	.RESULT_STATUS  = RESULT_CALIB_FAIL,  /*该值必须初始化FAIL*/
	.CUR_SIDE_INDEX = SIDE_INDEX_NULL,	  /*该值必须初始化(main函数里会执行判断)*/
};	

AccCalibSystem *g_psAccCalibSystem = &g_sAccCalibSystem;

/*磁力计校准系统*/
MagCalibSystem g_sMagCalibSystem = 
{
	.ENTRY_STATUS   = ENTRY_CALIB_NULL,	  /*该值必须初始化NULL*/
	.RESULT_STATUS  = RESULT_CALIB_FAIL,   /*该值必须初始化FAIL*/
	.CUR_SIDE_INDEX = SIDE_INDEX_NULL,	  /*该值必须初始化(main函数里会执行判断)*/	
};

MagCalibSystem *g_psMagCalibSystem = &g_sMagCalibSystem;


/*加速度计校准动作持续时长*/
vu16 g_sEntryAccCalibContinueTicks = 0;	/*进入加速度计校准动作持续时长*/
vu16 g_sTopAccContinueTicks        = 0; /*顶面加速度校准动作持续时长*/
vu16 g_sRollLeftAccContinueTicks   = 0;	/*AttRoll向左倾斜90度面动作持续时长*/
vu16 g_sRollRightAccContinueTicks  = 0;	/*AttRoll向右倾斜90度面动作持续时长*/
vu16 g_sPitchFrontAccContinueTicks = 0; /*AttPitch向前倾斜90度面动作持续时长*/
vu16 g_sPitchBackAccContinueTicks  = 0;	/*AttPitch向后倾斜90度面动作持续时长*/
vu16 g_sUnderAccContinueTicks	   = 0; /*底面加速度校准动作持续时长*/

/*磁力计校准动作持续时长*/
vu16 g_sEntryMagCalibContinueTicks = 0;	/*进入磁力计校准动作持续时长*/
vu16 g_sTopMagContinueTicks        = 0;	/*顶面磁力计校准动作持续时长(加计Z轴竖直)*/
vu16 g_sFlankPitchMagContinueTicks = 0;	/*侧面磁力计校准动作持续时长(加计Y轴竖直)*/
vu16 g_sFlankRollMagContinueTicks  = 0;	/*侧面磁力计校准动作持续时长(加计X轴竖直)*/


/*=== 传感器进入校准&校准面检测 ===*/
/*加速度计校准条件、面检测*/

CALIB_SIDE_INDEX calib_acc_sensor_check(void)
{
	/*1.加速度计进入校准动作判断*/
	/*						*	
		*******       *******
		*     *       *   * *
		*  *  *       *  *  * 
		* *   *       *     *
		*******       *******
		*                     
	*/	
	if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MAX)     == REMOT_DATA_MAX) && \
		(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MAX)    == REMOT_DATA_MAX) && \
		(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MIN) == REMOT_DATA_MIN) && \
		(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MIN)      == REMOT_DATA_MIN))
	{
		/*满足条件目标Ticks累加*/
		g_sEntryAccCalibContinueTicks++;
		
		/*非目标Ticks清0*/
		g_sEntryMagCalibContinueTicks = 0;
		
		if (g_sEntryAccCalibContinueTicks > (CALIB_ENTRY_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
		{
			/*完成一次检测,目标Ticks清0*/
			g_sEntryAccCalibContinueTicks = 0;

			/*本次校准目标加速度计全部清0,为校准做准备*/
			calib_Acc_Sensor_System_Reset();			
			
			/*进入和退出加速度计校准切换*/
			if (g_psAccCalibSystem->ENTRY_STATUS == ENTRY_CALIB_NULL)
			{
				/*标记进入加速度计校准*/
				g_psAccCalibSystem->ENTRY_STATUS = ENTRY_CALIB_INTO;
				
				/*将要进行校准的面序:第一面*/
				g_psAccCalibSystem->GONNA_SIDE_INDEX = SIDE_INDEX_1ST;
			}
			else if (g_psAccCalibSystem->ENTRY_STATUS == ENTRY_CALIB_INTO)
			{
				/*标记退出加速度计校准: 用户主动退出*/				
				g_psAccCalibSystem->ENTRY_STATUS = ENTRY_CALIB_EXIT;
				
				/*将要进行校准的面序:NULL*/
				g_psAccCalibSystem->GONNA_SIDE_INDEX = SIDE_INDEX_NULL;				
			}
			
			/*加速度传感器校准最终状态清除*/
			g_psAccCalibSystem->RESULT_STATUS = RESULT_CALIB_FAIL;
			
			/*非本次校准目标全部清0,避免校准混乱*/
			calib_Mag_Sensor_System_Reset();
			
			/*若磁力计未完成校准,则下次需要重新判断进入*/
			g_psMagCalibSystem->ENTRY_STATUS = ENTRY_CALIB_EXIT;
		}
	}	

	/*2.加速度计进入校准后,才允许检测校准某面*/
	if (g_psAccCalibSystem->ENTRY_STATUS == ENTRY_CALIB_INTO)
	{
		/*1.判断第1面(顶面)动作*/
		/*							
			*******       *******
			*     *       *     *
			*  *  *       *  *  * 
			*   * *       *     *
			*******       *******
				  *                    
		*/		
		if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MID)     == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MID)    == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MIN) == REMOT_DATA_MIN) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MAX)      == REMOT_DATA_MAX))
		{	
			/*目标面Tick++*/
			g_sTopAccContinueTicks++;
			
			/*其它面Tick清0*/
			g_sRollLeftAccContinueTicks   = 0;
			g_sRollRightAccContinueTicks  = 0;
			g_sPitchFrontAccContinueTicks = 0;
			g_sPitchBackAccContinueTicks  = 0;
			g_sUnderAccContinueTicks	  = 0;			
			
			if (g_sTopAccContinueTicks >= (CALIB_SIDE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
			{
				g_sTopAccContinueTicks = 0;
				
				g_psAccCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_1ST;
			}
		}		

		/*2.判断第2面(AttRoll向左倾斜90度面)动作*/
		/*							
			*******       *******
			*     *       *     *
			*  *  *     ******  * 
			*     *       *     *
			*******       *******
				                     
		*/		
		if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MIN)     == REMOT_DATA_MIN) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MID)    == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MID) == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MID)      == REMOT_DATA_MID))
		{	
			/*目标面Tick++*/
			g_sRollLeftAccContinueTicks++;
			
			/*其它面Tick清0*/
			g_sTopAccContinueTicks        = 0;
			g_sRollRightAccContinueTicks  = 0;
			g_sPitchFrontAccContinueTicks = 0;
			g_sPitchBackAccContinueTicks  = 0;
			g_sUnderAccContinueTicks	  = 0;			
			
			if (g_sRollLeftAccContinueTicks >= (CALIB_SIDE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
			{
				g_sRollLeftAccContinueTicks = 0;
				
				g_psAccCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_2ND; 
			}
		}
		
		/*3.判断第3面(AttRoll向右倾斜90度面)动作*/
		/*							
			*******       *******
			*     *       *     *
			*  *  *       *  ******
			*     *       *     *
			*******       *******
				                     
		*/		
		if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MAX)     == REMOT_DATA_MAX) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MID)    == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MID) == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MID)      == REMOT_DATA_MID))
		{	
			/*目标面Tick++*/
			g_sRollRightAccContinueTicks++;
			
			/*其它面Tick清0*/
			g_sTopAccContinueTicks        = 0;
			g_sRollLeftAccContinueTicks   = 0;
			g_sPitchFrontAccContinueTicks = 0;
			g_sPitchBackAccContinueTicks  = 0;
			g_sUnderAccContinueTicks	  = 0;			
			
			if (g_sRollRightAccContinueTicks >= (CALIB_SIDE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
			{
				g_sRollRightAccContinueTicks = 0;
				
				g_psAccCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_3RD; 
			}
		}			
		
		/*4.判断第4面(AttPitch向前倾斜90度面)动作*/
		/*					 *		
			*******       *******
			*     *       *  *  *
			*  *  *       *  *  * 
			*     *       *     *
			*******       *******
				                     
		*/		
		if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MID)     == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MAX)    == REMOT_DATA_MAX) && \
			(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MID) == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MID)      == REMOT_DATA_MID))
		{	
			/*目标面Tick++*/
			g_sPitchFrontAccContinueTicks++;
			
			/*其它面Tick清0*/
			g_sTopAccContinueTicks        = 0;
			g_sRollLeftAccContinueTicks   = 0;
			g_sRollRightAccContinueTicks  = 0;
			g_sPitchBackAccContinueTicks  = 0;
			g_sUnderAccContinueTicks	  = 0;			
			
			if (g_sPitchFrontAccContinueTicks >= (CALIB_SIDE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
			{
				g_sPitchFrontAccContinueTicks = 0;
				
				g_psAccCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_4TH; 
			}
		}			
		
		/*5.判断第5面(AttPitch向后倾斜90度面)动作*/
		/*					 		
			*******       *******
			*     *       *     *
			*  *  *       *  *  * 
			*     *       *  *  *
			*******       *******
				             *        
		*/		
		if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MID)     == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MIN)    == REMOT_DATA_MIN) && \
			(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MID) == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MID)      == REMOT_DATA_MID))
		{	
			/*目标面Tick++*/
			g_sPitchBackAccContinueTicks++;
			
			/*其它面Tick清0*/
			g_sTopAccContinueTicks        = 0;
			g_sRollLeftAccContinueTicks   = 0;
			g_sRollRightAccContinueTicks  = 0;
			g_sPitchFrontAccContinueTicks = 0;
			g_sUnderAccContinueTicks	  = 0;			
			
			if (g_sPitchBackAccContinueTicks >= (CALIB_SIDE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
			{
				g_sPitchBackAccContinueTicks = 0;
				
				g_psAccCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_5TH; 
			}
		}

		/*6.判断第6面(底面)动作*/
		/*					 		
			*******       *******
			*     *       *     *
			*  *  *       *  *  * 
			* *   *       *     *
			*******       *******
			*	                     
		*/		
		if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MID)     == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MID)    == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MIN) == REMOT_DATA_MIN) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MIN)      == REMOT_DATA_MIN))
		{	
			/*目标面Tick++*/
			g_sUnderAccContinueTicks++;
			
			/*其它面Tick清0*/
			g_sTopAccContinueTicks        = 0;
			g_sRollLeftAccContinueTicks   = 0;
			g_sRollRightAccContinueTicks  = 0;
			g_sPitchFrontAccContinueTicks = 0;
			g_sPitchBackAccContinueTicks  = 0;			
			
			if (g_sUnderAccContinueTicks >= (CALIB_SIDE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
			{
				g_sUnderAccContinueTicks = 0;
				
				g_psAccCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_6TH; 
			}
		}
	}
	
	return (g_psAccCalibSystem->CUR_SIDE_INDEX);
}

/*磁力计校准条件、面检测*/
CALIB_SIDE_INDEX calib_mag_sensor_check(void)
{
	/*1.磁力计进入校准动作判断*/
	/*			      *	
		*******       *******
		*     *       * *   *
		*  *  *       *  *  * 
		*   * *       *     *
		*******       *******
		      *                    
	*/	
	if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MIN)     == REMOT_DATA_MIN) && \
		(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MAX)    == REMOT_DATA_MAX) && \
		(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MIN) == REMOT_DATA_MIN) && \
		(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MAX)      == REMOT_DATA_MAX))
	{
		/*满足条件目标Ticks累加*/
		g_sEntryMagCalibContinueTicks++;
		
		/*非目标Ticks清0*/
		g_sEntryAccCalibContinueTicks = 0;
		
		if (g_sEntryMagCalibContinueTicks > (CALIB_ENTRY_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
		{
			/*完成一次检测,目标Ticks清0*/
			g_sEntryMagCalibContinueTicks = 0;
			
			/*本次校准目标加速度计全部清0,为校准做准备*/
			calib_Mag_Sensor_System_Reset();			
			
			/*进入和退出磁力计校准切换*/
			if (g_psMagCalibSystem->ENTRY_STATUS == ENTRY_CALIB_NULL)
			{
				/*标记进入磁力计校准*/
				g_psMagCalibSystem->ENTRY_STATUS = ENTRY_CALIB_INTO;

				/*将要进行校准的面序:第一面*/
				g_psMagCalibSystem->GONNA_SIDE_INDEX = SIDE_INDEX_1ST;				
			}
			else if (g_psMagCalibSystem->ENTRY_STATUS == ENTRY_CALIB_INTO)
			{
				/*标记退出磁力计校准(用户主动退出)*/				
				g_psMagCalibSystem->ENTRY_STATUS = ENTRY_CALIB_EXIT;
				
				/*将要进行校准的面序:NULL*/
				g_psMagCalibSystem->GONNA_SIDE_INDEX = SIDE_INDEX_NULL;					
			}			
			
			/*磁力计传感器校准最终状态清除*/
			g_psMagCalibSystem->RESULT_STATUS = RESULT_CALIB_FAIL;
			
			/*非本次校准目标全部清0,避免校准混乱*/
			calib_Acc_Sensor_System_Reset();
			
			/*若加速度计未完成校准,则下次需要重新判断进入*/
			g_psAccCalibSystem->ENTRY_STATUS = ENTRY_CALIB_EXIT;
		}		
	}	

	/*2.磁力计进入校准后,才允许检测校准某面*/
	if (g_psMagCalibSystem->ENTRY_STATUS == ENTRY_CALIB_INTO)
	{
		/*1.判断第1面(顶面加计Z轴朝上)动作*/
		/*					 	
			*******       *******
			*     *       *     *
			*  *  *       *  *  * 
			*   * *       *     *
			*******       *******
				  *                    
		*/		
		if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MID)     == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MID)    == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MIN) == REMOT_DATA_MIN) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MAX)      == REMOT_DATA_MAX))
		{	
			/*目标面Tick++*/
			g_sTopMagContinueTicks++;
			
			/*其它面Tick清0*/
			g_sFlankPitchMagContinueTicks = 0;
			g_sFlankRollMagContinueTicks  = 0;
			
			if (g_sTopMagContinueTicks >= (CALIB_SIDE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
			{
				g_sTopMagContinueTicks = 0;
				
				g_psMagCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_1ST; 
			}
		}		

		/*2.判断第2面(侧面加计机头(Y轴)朝上)动作*/
		/*		
               *		
			*******       *******
			*  *  *       *     *
		    *  *  *       *  *  * 
			*     *       *     *
			*******       *******
		*/		
		if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MID)     == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MID)    == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MAX) == REMOT_DATA_MAX) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MID)      == REMOT_DATA_MID))
		{	
			/*目标面Tick++*/
			g_sFlankPitchMagContinueTicks++;
			
			/*其它面Tick清0*/
			g_sTopMagContinueTicks       = 0;
			g_sFlankRollMagContinueTicks = 0;
			
			if (g_sFlankPitchMagContinueTicks >= (CALIB_SIDE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
			{
				g_sFlankPitchMagContinueTicks = 0;
				
				g_psMagCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_2ND; 
			}
		}	

		/*3.判断第3面(侧面加计(x轴)朝上)动作*/
		/*			
			*******       *******
			*     *       *     *
		    *  *******    *  *  * 
			*     *       *     *
			*******       *******
		*/		
		if ((remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MID)     == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MID)    == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttThrottle, REMOT_DATA_MID) == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MAX)      == REMOT_DATA_MAX))
		{	
			/*目标面Tick++*/
			g_sFlankRollMagContinueTicks++;
			
			/*其它面Tick清0*/
			g_sTopMagContinueTicks        = 0;
			g_sFlankPitchMagContinueTicks = 0;
			
			if (g_sFlankRollMagContinueTicks >= (CALIB_SIDE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_UAV_CALIB_FOC_MS))
			{
				g_sFlankRollMagContinueTicks = 0;
				
				g_psMagCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_2ND; 
			}
		}	
	}
	
	return (g_psMagCalibSystem->CUR_SIDE_INDEX);
}


/*=== 校准系统重置 ===*/
/*加速度计校准状态清除*/
void calib_Acc_Sensor_System_Reset(void)
{
	CALIB_SIDE_INDEX i;
	
	/*整体采样完成标志位清除去*/
	g_psAccCalibSystem->WHOLE_STATUS  = WHOLE_SAMPLE_FAIL;
			
	/*清除对应面数据和校准状态*/
	for (i = SIDE_INDEX_1ST; i <= SIDE_INDEX_6TH; i++)
	{
		/*对应面单次采样完成标志位清除*/
		g_psAccCalibSystem->SINGLE_STATUS[i] = SINGLE_SAMPLE_FAIL;
				
		/*对应面加速度计量清0*/
		g_psAccCalibSystem->sampleData[i].x  = 0;
		g_psAccCalibSystem->sampleData[i].y  = 0;				
		g_psAccCalibSystem->sampleData[i].z  = 0;				
	}
}

/*磁力计校准状态清除*/
void calib_Mag_Sensor_System_Reset(void)
{
	CALIB_SIDE_INDEX     i;
	CALIB_POSITION_INDEX j;
	
	/*整体采样完成标志位清除*/
	g_psMagCalibSystem->WHOLE_STATUS = WHOLE_SAMPLE_FAIL;
	
	/*2个面的采集状态清0*/
	for (i = SIDE_INDEX_1ST; i <= SIDE_INDEX_3RD; i++)
	{
		/*对应面单词采集完成标志位清除*/
		g_psMagCalibSystem->SINGLE_STATUS[i] = SINGLE_SAMPLE_FAIL;
		
		/*2个面的12个角点采集状态和数据清除*/
		for (j = POSITION_INDEX_1ST; j <= POSITION_INDEX_36TH; j++)
		{
			/*采集状态清0*/
			g_psMagCalibSystem->POSITION_STATUS[i][j] = POSITION_SAMPLE_FAIL;
			
			/*采集数据清0*/
			g_psMagCalibSystem->sampleData[i][j].x = 0;
			g_psMagCalibSystem->sampleData[i][j].y = 0;
			g_psMagCalibSystem->sampleData[i][j].z = 0;			
		}				
	}
	
	/*磁力计最小二乘中间变量清0*/
	memset(&(g_psMagCalibSystem->Mag_Calib_LSIV), 0, sizeof(Mag_Calib_LSIV));
}


/*=== 传感器进行采集&校准 ===*/
/*加速度计执行采样+校准*/
SYS_RETSTATUS calib_acc_sensor_dp(CALIB_SIDE_INDEX calibSideIndex)
{
	SYS_RETSTATUS retStatus = SYS_RET_FAIL;
	static volatile CALIB_SIDE_INDEX LAST_SIDE = SIDE_INDEX_NULL;
	
	/*加速度参数变量用于写入*/
	AccCalibPara sAccCalibParaWrite;
	AccCalibPara *psAccCalibParaWrite = &sAccCalibParaWrite;
	
	u16 sampleTimes = 0;		/*采集次数*/
	Acc3f sampleSum = {0};		/*采集数据之和*/	
	
	/*OLED显示操作提示*/
	hci_Show_Acc_Calib_Status(g_psAccCalibSystem->ENTRY_STATUS, g_psAccCalibSystem->GONNA_SIDE_INDEX);
	
	/*判断是否为退出校准操作: 用户主动退出 / 完成校准退出*/
	if (g_psAccCalibSystem->ENTRY_STATUS == ENTRY_CALIB_EXIT)
	{
		/*标记为NULL,退出校准系统*/
		g_psAccCalibSystem->ENTRY_STATUS = ENTRY_CALIB_NULL;
		
		/*当前面序重置*/
		g_psAccCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_NULL;
		
		/*面序状态机重置*/
		LAST_SIDE = SIDE_INDEX_NULL;
		
		/*加速度计校准状态清除*/
		calib_Acc_Sensor_System_Reset();
	}
	
	/*记录进入加速度校准模式采集的面序号*/
	if (g_psAccCalibSystem->CUR_SIDE_INDEX != LAST_SIDE)
	{
		LAST_SIDE = g_psAccCalibSystem->CUR_SIDE_INDEX;
		
		/*将要采集的下一面序*/
		g_psAccCalibSystem->GONNA_SIDE_INDEX++;
		
		/*只有每次需要采集的面序不一样时才开启采集*/
		g_psAccCalibSystem->SAMPLE_STATUS = SAMPLE_DATA_START;
	}
	
	/*第一面飞控平放，+Z轴 朝着正上方，Z axis is about 1g,X、Y is about 0g*/
	/*第二面飞控平放，+X轴 朝着正上方，X axis is about 1g,Y、Z is about 0g*/
	/*第三面飞控平放，-X轴 朝着正下方，X axis is about -1g,Y、Z is about 0g*/
	/*第四面飞控平放，-Y轴 朝着正下方，Y axis is about -1g,X、Z is about 0g*/
	/*第五面飞控平放，+Y轴 朝着正上方，Y axis is about 1g,X、Z is about 0g*/
	/*第六面飞控平放，-Z轴 朝着正下方，Z axis is about -1g,X、Y is about 0g*/	
	
	/*允许开始采集,且没有采集过,就开始采集该面数据*/
	if ((g_psAccCalibSystem->SAMPLE_STATUS == SAMPLE_DATA_START) && \
		(g_psAccCalibSystem->SINGLE_STATUS[calibSideIndex] != SINGLE_SAMPLE_SUCC))
	{	
		while(sampleTimes < 200)	/*采集200次, 5ms一次*/
		{
			/*通过陀螺仪模长来确保机体静止及校准数据每次都是最新的*/
			if ((g_GyroLenth <= 20.0f) && \
				(g_psAccSensorDataStatus->calib == SENSOR_DATA_NEW))	/*ACC数据在Task内更新5ms更新一次*/
			{
				/*加速度数据累加求和*/
				sampleSum.x += g_psAccCalib->x * ACC_TO_ONE_G;	/*加速度计转化为1g量程下*/
				sampleSum.y += g_psAccCalib->y * ACC_TO_ONE_G;	/*加速度计转化为1g量程下*/
				sampleSum.z += g_psAccCalib->z * ACC_TO_ONE_G;	/*加速度计转化为1g量程下*/
			
				/*采集次数++*/
				sampleTimes++;
				
				/*确保每次用于校准的数据都是最新的*/
				g_psAccSensorDataStatus->calib = SENSOR_DATA_OLD;
			}
			
			/*标记本次是采样过程*/
			g_psAccCalibSystem->SAMPLE_PROCESS = SAMPLE_PROCESS_SAMPLEING;
			
			/*用OLED显示加速度校准过程*/
			hci_Show_Acc_Calib_Process(calibSideIndex, g_psAccCalibSystem->SAMPLE_PROCESS, \
													   g_psAccCalibSystem->SINGLE_STATUS[calibSideIndex], \
													   g_GyroLenth, \
													   g_psAccCalib->x * ACC_TO_ONE_G, \
													   g_psAccCalib->y * ACC_TO_ONE_G, \
													   g_psAccCalib->z * ACC_TO_ONE_G, 0);			
		}
		
		/*本次加速度数据采集结束*/
		g_psAccCalibSystem->SAMPLE_STATUS = SAMPLE_DATA_FINISH;
					
		/*当前面采集完成*/
		g_psAccCalibSystem->SINGLE_STATUS[calibSideIndex] = SINGLE_SAMPLE_SUCC;
		
		/*保存对应面的加速度数据*/
		g_psAccCalibSystem->sampleData[calibSideIndex].x = sampleSum.x / sampleTimes; 
		g_psAccCalibSystem->sampleData[calibSideIndex].y = sampleSum.y / sampleTimes; 
		g_psAccCalibSystem->sampleData[calibSideIndex].z = sampleSum.z / sampleTimes; 	
		
		/*标记本次是采样结果*/	
		g_psAccCalibSystem->SAMPLE_PROCESS = SAMPLE_PROCESS_RESULT;
		
		/*用OLED显示加速度校准结果*/
		hci_Show_Acc_Calib_Process(calibSideIndex, g_psAccCalibSystem->SAMPLE_PROCESS, \
												   g_psAccCalibSystem->SINGLE_STATUS[calibSideIndex], \
												   g_GyroLenth, \
												   g_psAccCalibSystem->sampleData[calibSideIndex].x, \
												   g_psAccCalibSystem->sampleData[calibSideIndex].y, \
												   g_psAccCalibSystem->sampleData[calibSideIndex].z, 1500);
	}
	
	/*判断是否所有面已经矫正完毕,且是第一次完成全部面采集*/
	if (calib_sample_status_check((u8*)&(g_psAccCalibSystem->SINGLE_STATUS), SINGLE_SAMPLE_SUCC, 6) == SYS_RET_SUCC)
	{
		/*标记加速度校准系统退出(完成退出)*/
		g_psAccCalibSystem->ENTRY_STATUS = ENTRY_CALIB_EXIT;
		
		/*高斯牛顿法(Gauss-Newton)求加速度计校准参数*/
		retStatus = calib_Acc_Gauss_Newton_Dp(g_psAccCalibSystem->sampleData, psAccCalibParaWrite);
		
		/*加速度计校准成功*/
		if (retStatus == SYS_RET_SUCC)
		{
			/*加速度校准数据写入EEPROM*/
			/*Acc_Scale*/
			bsp_AT24CXX_Write_3_FloatData(&g_sAt24cxx, AT24CXX_STOR_ACC_SCALE_ADDR, \
										   psAccCalibParaWrite->Scale.x, \
										   psAccCalibParaWrite->Scale.y, \
										   psAccCalibParaWrite->Scale.z);
			
			/*Acc_Offset*/	
			bsp_AT24CXX_Write_3_FloatData(&g_sAt24cxx, AT24CXX_STOR_ACC_OFFSET_ADDR, \
										   psAccCalibParaWrite->Offset.x, \
										   psAccCalibParaWrite->Offset.y, \
										   psAccCalibParaWrite->Offset.z);		
			
			/*从EEPROM读出,并再次判断是否符合高斯牛顿法,并赋值给参数*/
			retStatus = calib_Acc_Sensor_parameter_Read();
			
			/*最后的校准结果*/
			if (retStatus == SYS_RET_SUCC)
			{
				g_psAccCalibSystem->RESULT_STATUS = RESULT_CALIB_SUCC;	/*成功*/
			}
			else
			{
				g_psAccCalibSystem->RESULT_STATUS = RESULT_CALIB_FAIL;	/*失败*/
			}
		}
		else	/*加速度计校准失败*/
		{
			retStatus = SYS_RET_FAIL;
		}
	}
	
	return retStatus;
}

/*36个角点的标准AttYaw角度*/
const u16 g_Mag360Define[36] = 
{  0,  10,  20,  30,  40,  50,  60,  70,  80,  90, 100, 110, 
 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230,
 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350};

/*磁力计执行采样+校准*/
SYS_RETSTATUS calib_mag_sensor_dp(CALIB_SIDE_INDEX calibSideIndex)
{
	SYS_RETSTATUS retStatus = SYS_RET_FAIL;
	static volatile CALIB_SIDE_INDEX LAST_SIDE = SIDE_INDEX_NULL;
	
	CALIB_POSITION_INDEX positionIndex;	/*角点*/
	u32 magCalibSaveAddr = 0; /*磁力计校准参数保存地址*/

	/*磁力计校准参数变量用于写入*/
	MagCalibPara sMagCalibParaWrite;
	MagCalibPara *psMagCalibParaWrite = &sMagCalibParaWrite;
	
	vu16 sampleZTimes = 0;		/*Z轴竖直面采集次数*/
	vu16 sampleYTimes = 0;		/*Y轴竖直面采集次数*/
	vu16 sampleXTimes = 0;		/*X轴竖直面采集次数*/	
	Mag3s sampleZSum  = {0};	/*Z轴竖直面采集数据之和*/	
	Mag3s sampleYSum  = {0};	/*Y轴竖直面采集数据之和*/
	Mag3s sampleXSum  = {0};	/*X轴竖直面采集数据之和*/

	/*OLED显示操作提示*/
	hci_Show_Mag_Calib_Status(g_psMagCalibSystem->ENTRY_STATUS, g_psMagCalibSystem->GONNA_SIDE_INDEX);
	
	/*判断是否为退出校准操作: 用户主动退出 / 完成校准退出*/
	if (g_psMagCalibSystem->ENTRY_STATUS == ENTRY_CALIB_EXIT)
	{
		/*标记为NULL,退出校准系统*/
		g_psMagCalibSystem->ENTRY_STATUS = ENTRY_CALIB_NULL;
		
		/*当前面序重置*/
		g_psMagCalibSystem->CUR_SIDE_INDEX = SIDE_INDEX_NULL;
		
		/*面序状态机重置*/
		LAST_SIDE = SIDE_INDEX_NULL;

		/*磁力计校准状态清除*/
		calib_Mag_Sensor_System_Reset();
	}	
	
	/*记录进入加速度校准模式采集的面序号*/
	if (g_psMagCalibSystem->CUR_SIDE_INDEX != LAST_SIDE)
	{
		LAST_SIDE = calibSideIndex;
		
		/*将要采集的下一面序*/
		g_psMagCalibSystem->GONNA_SIDE_INDEX++;		
		
		/*只有每次需要采集的面序不一样时才开启采集*/
		g_psMagCalibSystem->SAMPLE_STATUS = SAMPLE_DATA_START;	
	}
	
	/********第一面+Z轴正向朝着正上方，
	开始绕竖直轴旋转，+Z axis is about 1g,X、Y is about 0g*/
	/********第二面+Y轴正向朝着正上方，
	开始绕竖直轴旋转，+Y axis is about 1g,X、Z is about 0g*/
	/********第三面+X轴正向朝着正上方，
	开始绕竖直轴旋转，+X axis is about 1g,Y、Z is about 0g*/	
	
	/*允许开始采集,且没有采集过,就开始采集该面数据*/
	if ((g_psMagCalibSystem->SAMPLE_STATUS == SAMPLE_DATA_START) && \
		(g_psMagCalibSystem->SINGLE_STATUS[calibSideIndex] != SINGLE_SAMPLE_SUCC))
	{
		/*Z轴竖直面角点数据采集*/
		if (calibSideIndex == SIDE_INDEX_1ST)
		{
			/*36个角点*/
			for (positionIndex = POSITION_INDEX_1ST; positionIndex <= POSITION_INDEX_36TH; positionIndex++)
			{
				while(sampleZTimes < 10)	/*采集100次*/
				{
					/*判断数据是否符合校准*/
					if ((fabs(g_psCircleAngle->yaw - g_Mag360Define[positionIndex]) <= 5.0f) && \
						(g_psAccCalib->z >= (ACC_MAX_ONE_G / 2.0f)) && \
						(g_psMagSensorDataStatus->calib == SENSOR_DATA_NEW))	/*加速度计Z轴基本竖直*/
					{
						/*磁力计数据累加求和*/
						sampleZSum.x += g_psMagRaw->x;
						sampleZSum.y += g_psMagRaw->y;
						sampleZSum.z += g_psMagRaw->z;
			
						/*采集次数++*/
						sampleZTimes++;
				
						/*确保每次用于校准的数据都是最新的*/
						g_psMagSensorDataStatus->calib = SENSOR_DATA_OLD;					
					}
					
					/*标记本次是采样过程*/
					g_psMagCalibSystem->SAMPLE_PROCESS = SAMPLE_PROCESS_SAMPLEING;
					
					/*用OLED显示陀螺仪校准过程*/
					hci_Show_Mag_Calib_Process(calibSideIndex, positionIndex, g_psMagCalibSystem->POSITION_STATUS[calibSideIndex][positionIndex], \
																			  g_psMagCalibSystem->SAMPLE_PROCESS, \
																			  (u16)g_psCircleAngle->yaw, (u16)g_Mag360Define[positionIndex], 5, \
																			  g_psAccCalib->z, 0);						
				}
				
				/*标计当前面&当前角点数据采集完毕*/
				g_psMagCalibSystem->POSITION_STATUS[calibSideIndex][positionIndex] = POSITION_SAMPLE_SUCC;	
			
				/*平均值赋值*/
				g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex].x = (fp32)sampleZSum.x / sampleZTimes;	
				g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex].y = (fp32)sampleZSum.y / sampleZTimes;
				g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex].z = (fp32)sampleZSum.z / sampleZTimes;	
								
				/*计数器清0*/
				sampleZTimes = 0;
				
				/*磁力计最小二乘法计算*/
				calib_mag_sensor_lsiv_accumulate(&(g_psMagCalibSystem->Mag_Calib_LSIV), g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex]);
				calib_mag_sensor_lsiv_calculate(&(g_psMagCalibSystem->Mag_Calib_LSIV), 36 * 3, 0.0f, &(g_psMagCalibSystem->Sphere));
				
				/*标记本次是采样结果*/
				g_psMagCalibSystem->SAMPLE_PROCESS = SAMPLE_PROCESS_RESULT;			
				
				/*用OLED显示陀螺仪校准结果*/
				hci_Show_Mag_Calib_Process(calibSideIndex, positionIndex, g_psMagCalibSystem->POSITION_STATUS[calibSideIndex][positionIndex], \
																	      g_psMagCalibSystem->SAMPLE_PROCESS, \
																		  (u16)g_psCircleAngle->yaw, (u16)g_Mag360Define[positionIndex], 5, \
																		  g_psAccCalib->z, 1500);				
			}
			
			/*本次采样结束*/
			g_psMagCalibSystem->SAMPLE_STATUS = SAMPLE_DATA_FINISH;
		}
		/*Y轴竖直面角点数据采集,且只允许成功采集一次*/
		else if (calibSideIndex == SIDE_INDEX_2ND)
		{
			/*36个角点*/
			for (positionIndex = POSITION_INDEX_1ST; positionIndex <= POSITION_INDEX_36TH; positionIndex++)
			{
				while(sampleYTimes < 10)	/*采集100次*/
				{
					/*判断数据是否符合校准*/
					if ((fabs(g_psCircleAngle->roll - g_Mag360Define[positionIndex]) <= 5.0f) && \
						(g_psAccCalib->y >= (ACC_MAX_ONE_G / 2.0f)) && \
						(g_psMagSensorDataStatus->calib == SENSOR_DATA_NEW))	/*加速度计Y轴基本竖直*/
					{
						/*磁力计数据累加求和*/
						sampleYSum.x += g_psMagRaw->x;
						sampleYSum.y += g_psMagRaw->y;
						sampleYSum.z += g_psMagRaw->z;
			
						/*采集次数++*/
						sampleYTimes++;
				
						/*确保每次用于校准的数据都是最新的*/
						g_psMagSensorDataStatus->calib = SENSOR_DATA_OLD;					
					}
					
					/*标记本次是采样过程*/
					g_psMagCalibSystem->SAMPLE_PROCESS = SAMPLE_PROCESS_SAMPLEING;
					
					/*用OLED显示陀螺仪校准过程*/
					hci_Show_Mag_Calib_Process(calibSideIndex, positionIndex, g_psMagCalibSystem->POSITION_STATUS[calibSideIndex][positionIndex], \
																			  g_psMagCalibSystem->SAMPLE_PROCESS, \
																			  (u16)g_psCircleAngle->roll, (u16)g_Mag360Define[positionIndex], 5, \
																			  g_psAccCalib->y, 0);					
				}
			
				/*标计当前面&当前角点数据采集完毕*/
				g_psMagCalibSystem->POSITION_STATUS[calibSideIndex][positionIndex] = POSITION_SAMPLE_SUCC;	
			
				/*平均值赋值*/
				g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex].x = (fp32)sampleYSum.x / sampleYTimes;	
				g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex].y = (fp32)sampleYSum.y / sampleYTimes;
				g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex].z = (fp32)sampleYSum.z / sampleYTimes;
				
				/*计数器清0*/
				sampleYTimes = 0;

				/*磁力计最小二乘法计算*/
				calib_mag_sensor_lsiv_accumulate(&(g_psMagCalibSystem->Mag_Calib_LSIV), g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex]);
				calib_mag_sensor_lsiv_calculate(&(g_psMagCalibSystem->Mag_Calib_LSIV), 36 * 3, 0.0f, &(g_psMagCalibSystem->Sphere));
				
				/*标记本次是采样结果*/
				g_psMagCalibSystem->SAMPLE_PROCESS = SAMPLE_PROCESS_RESULT;			
				
				/*用OLED显示陀螺仪校准结果*/
				hci_Show_Mag_Calib_Process(calibSideIndex, positionIndex, g_psMagCalibSystem->POSITION_STATUS[calibSideIndex][positionIndex], \
																	      g_psMagCalibSystem->SAMPLE_PROCESS, \
																		  (u16)g_psCircleAngle->roll, (u16)g_Mag360Define[positionIndex], 5, \
																		  g_psAccCalib->y, 1500);
			}

			/*采样结束*/
			g_psMagCalibSystem->SAMPLE_STATUS = SAMPLE_DATA_FINISH;			
		}
		
		/*X轴竖直面角点数据采集,且只允许成功采集一次*/
		else if (calibSideIndex == SIDE_INDEX_3RD)
		{
			/*36个角点*/
			for (positionIndex = POSITION_INDEX_1ST; positionIndex <= POSITION_INDEX_36TH; positionIndex++)
			{
				while(sampleYTimes < 10)	/*采集100次*/
				{
					/*判断数据是否符合校准*/
					if ((fabs(g_psCircleAngle->pitch - g_Mag360Define[positionIndex]) <= 5.0f) && \
						(g_psAccCalib->x >= (ACC_MAX_ONE_G / 2.0f)) && \
						(g_psMagSensorDataStatus->calib == SENSOR_DATA_NEW))	/*加速度计X轴基本竖直*/
					{
						/*磁力计数据累加求和*/
						sampleXSum.x += g_psMagRaw->x;
						sampleXSum.y += g_psMagRaw->y;
						sampleXSum.z += g_psMagRaw->z;
			
						/*采集次数++*/
						sampleYTimes++;
				
						/*确保每次用于校准的数据都是最新的*/
						g_psMagSensorDataStatus->calib = SENSOR_DATA_OLD;					
					}
					
					/*标记本次是采样过程*/
					g_psMagCalibSystem->SAMPLE_PROCESS = SAMPLE_PROCESS_SAMPLEING;
					
					/*用OLED显示陀螺仪校准过程*/
					hci_Show_Mag_Calib_Process(calibSideIndex, positionIndex, g_psMagCalibSystem->POSITION_STATUS[calibSideIndex][positionIndex], \
																			  g_psMagCalibSystem->SAMPLE_PROCESS, \
																			  (u16)g_psCircleAngle->pitch, (u16)g_Mag360Define[positionIndex], 5, \
																			  g_psAccCalib->x, 0);					
				}
			
				/*标计当前面&当前角点数据采集完毕*/
				g_psMagCalibSystem->POSITION_STATUS[calibSideIndex][positionIndex] = POSITION_SAMPLE_SUCC;	
			
				/*平均值赋值*/
				g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex].x = (fp32)sampleYSum.x / sampleYTimes;	
				g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex].y = (fp32)sampleYSum.y / sampleYTimes;
				g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex].z = (fp32)sampleYSum.z / sampleYTimes;
				
				/*计数器清0*/
				sampleYTimes = 0;	
				
				/*磁力计最小二乘法计算*/
				calib_mag_sensor_lsiv_accumulate(&(g_psMagCalibSystem->Mag_Calib_LSIV), g_psMagCalibSystem->sampleData[calibSideIndex][positionIndex]);
				calib_mag_sensor_lsiv_calculate(&(g_psMagCalibSystem->Mag_Calib_LSIV), 36 * 3, 0.0f, &(g_psMagCalibSystem->Sphere));				
				
				/*标记本次是采样结果*/
				g_psMagCalibSystem->SAMPLE_PROCESS = SAMPLE_PROCESS_RESULT;			
				
				/*用OLED显示陀螺仪校准结果*/
				hci_Show_Mag_Calib_Process(calibSideIndex, positionIndex, g_psMagCalibSystem->POSITION_STATUS[calibSideIndex][positionIndex], \
																	      g_psMagCalibSystem->SAMPLE_PROCESS, \
																		  (u16)g_psCircleAngle->pitch, (u16)g_Mag360Define[positionIndex], 5, \
																		  g_psAccCalib->x, 1500);
			}

			/*采样结束*/
			g_psMagCalibSystem->SAMPLE_STATUS = SAMPLE_DATA_FINISH;			
		}		
	}

	/*判断第一面(顶面)36个角点是否采集完毕*/
	if (calib_sample_status_check((u8*)&(g_psMagCalibSystem->POSITION_STATUS[SIDE_INDEX_1ST]), POSITION_SAMPLE_SUCC, 36) == SYS_RET_SUCC)
	{
		/*标记第一面(顶面,加计Z轴朝上面)采集完毕*/
		g_psMagCalibSystem->SINGLE_STATUS[SIDE_INDEX_1ST] = SINGLE_SAMPLE_SUCC;
	}

	/*判断第二面(侧面,加计Y轴朝上面)12个角点是否采集完毕*/
	if (calib_sample_status_check((u8*)&(g_psMagCalibSystem->POSITION_STATUS[SIDE_INDEX_2ND]), POSITION_SAMPLE_SUCC, 36) == SYS_RET_SUCC)
	{
		/*标记第二面(侧面,加计Y轴朝上面)采集完毕*/
		g_psMagCalibSystem->SINGLE_STATUS[SIDE_INDEX_2ND] = SINGLE_SAMPLE_SUCC;
	}	
	
	/*判断第三面(侧面,加计X轴朝上面)36个角点是否采集完毕*/
	if (calib_sample_status_check((u8*)&(g_psMagCalibSystem->POSITION_STATUS[SIDE_INDEX_3RD]), POSITION_SAMPLE_SUCC, 36) == SYS_RET_SUCC)
	{
		/*标记第三面(顶面,加计Z轴朝上面)采集完毕*/
		g_psMagCalibSystem->SINGLE_STATUS[SIDE_INDEX_3RD] = SINGLE_SAMPLE_SUCC;
	}	
	
	/*三面都采集完毕*/
	if (calib_sample_status_check((u8*)&(g_psMagCalibSystem->SINGLE_STATUS), SINGLE_SAMPLE_SUCC, 3) == SYS_RET_SUCC)
	{	
		/*标记磁力计校准系统(完成退出)*/
		g_psMagCalibSystem->ENTRY_STATUS = ENTRY_CALIB_EXIT;
		
		psMagCalibParaWrite->Offset.x = g_psMagCalibSystem->Sphere.a;
		psMagCalibParaWrite->Offset.y = g_psMagCalibSystem->Sphere.b;
		psMagCalibParaWrite->Offset.z = g_psMagCalibSystem->Sphere.c;

		/*磁力计校准数据写入EEPROM*/
#ifdef HW_CUT__USE_MD_MAG
#ifdef HW_CUT__USE_GPS_MAG
		magCalibSaveAddr = AT24CXX_STOR_GPS_MAG_OFFSET_ADDR;	/*GPS Mag*/
#else
		magCalibSaveAddr = AT24CXX_STOR_MAG_OFFSET_ADDR;		/*Module Mag*/
#endif
#endif

		/*Mag_Offset*/
		bsp_AT24CXX_Write_3_FloatData(&g_sAt24cxx, magCalibSaveAddr, \
									   psMagCalibParaWrite->Offset.x, \
									   psMagCalibParaWrite->Offset.y, \
									   psMagCalibParaWrite->Offset.z);
			
		/*从EEPROM读出,并再次判断是否符合高斯牛顿法,并赋值给参数*/
		retStatus = calib_Mag_Sensor_parameter_Read();
		
		/*最后的校准结果*/
		if (retStatus == SYS_RET_SUCC)
		{
			g_psMagCalibSystem->RESULT_STATUS = RESULT_CALIB_SUCC;	/*成功*/
		}
		else
		{
			g_psMagCalibSystem->RESULT_STATUS = RESULT_CALIB_FAIL;	/*失败*/
		}
	}
	
	return retStatus;
}

SYS_RETSTATUS calib_sample_status_check(u8 *BUFF, u8 CHECK_TARG, u16 totalNbr)
{
	SYS_RETSTATUS RET_STATUS = SYS_RET_SUCC;
	u16 i;
	
	for (i = 0; i < totalNbr; i++)
	{
		if (BUFF[i] != CHECK_TARG)
		{
			RET_STATUS = SYS_RET_FAIL;
			
			break;
		}
	}
	
	return RET_STATUS;
}

/*=== 校准参数读取&初始化 ===*/
/*加速度传感器校准准参数初始化(读取)*/
SYS_RETSTATUS calib_Acc_Sensor_parameter_Read(void)
{
	SYS_RETSTATUS retStatus           = SYS_RET_FAIL;
	SYS_RETSTATUS readAccScaleStatus  = SYS_RET_FAIL;
	SYS_RETSTATUS readAccOffsetStatus = SYS_RET_FAIL;
	
	/*加速度参数变量用于读出*/
	AccCalibPara sAccCalibParaRead;
	AccCalibPara *psAccCalibParaRead = &sAccCalibParaRead;
	
	/*读取加速度校准参数*/
	/*Read Acc_Scale Status*/
	readAccScaleStatus = bsp_AT24CXX_Read_3_FloatData(&g_sAt24cxx, AT24CXX_STOR_ACC_SCALE_ADDR, \
												      &psAccCalibParaRead->Scale.x, \
											          &psAccCalibParaRead->Scale.y, \
											          &psAccCalibParaRead->Scale.z);
	
	/*Read Acc_Offset Status*/
	readAccOffsetStatus = bsp_AT24CXX_Read_3_FloatData(&g_sAt24cxx, AT24CXX_STOR_ACC_OFFSET_ADDR, \
												       &psAccCalibParaRead->Offset.x, \
												       &psAccCalibParaRead->Offset.y, \
												       &psAccCalibParaRead->Offset.z);			
    /*sanity check scale*/
    if ((fabsf(psAccCalibParaRead->Scale.x - 1.0f) > 0.5f) || \
		(fabsf(psAccCalibParaRead->Scale.y - 1.0f) > 0.5f) || \
        (fabsf(psAccCalibParaRead->Scale.z - 1.0f) > 0.5f))
    {
        retStatus = SYS_RET_FAIL;
    }
	
    /*sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)*/
    if ((fabsf(psAccCalibParaRead->Offset.x) > 5.0f) || \
        (fabsf(psAccCalibParaRead->Offset.y) > 5.0f) || \
        (fabsf(psAccCalibParaRead->Offset.z) > 5.0f ))
    {
		retStatus = SYS_RET_FAIL;
    }
	else
	{
		retStatus = SYS_RET_SUCC;
	}
	
	/*验证成立&数据正常,作为校准参数*/
	if ((retStatus == SYS_RET_SUCC) && \
		(readAccScaleStatus == SYS_RET_SUCC) && \
		(readAccOffsetStatus == SYS_RET_SUCC))
	{
		/*Acc_Offset*/
		g_fpAccOffsetX = psAccCalibParaRead->Offset.x;
		g_fpAccOffsetY = psAccCalibParaRead->Offset.y;
		g_fpAccOffsetZ = psAccCalibParaRead->Offset.z;

		/*Acc_Scale*/
		g_fpAccScaleX = psAccCalibParaRead->Scale.x;
		g_fpAccScaleY = psAccCalibParaRead->Scale.y;
		g_fpAccScaleZ = psAccCalibParaRead->Scale.z;

		/*ret succ*/
		retStatus = SYS_RET_SUCC;
	}
	else
	{
		retStatus = SYS_RET_FAIL;
	}

	return retStatus;
}


/*磁力计传感器校准准参数初始化(读取)*/
SYS_RETSTATUS calib_Mag_Sensor_parameter_Read(void)
{
	SYS_RETSTATUS retStatus        = SYS_RET_FAIL;
	SYS_RETSTATUS magOffsetStatus  = SYS_RET_FAIL;
	u32 magCalibSaveAddr = 0; /*磁力计校准参数保存地址*/
	
	/*磁力计参数变量用于读出*/
	MagCalibPara sMagCalibParaRead;
	MagCalibPara *psMagCalibParaRead = &sMagCalibParaRead;
	
	/*读取磁力计校准参数*/
#ifdef HW_CUT__USE_MD_MAG
#ifdef HW_CUT__USE_GPS_MAG
	magCalibSaveAddr = AT24CXX_STOR_GPS_MAG_OFFSET_ADDR;	/*GPS Mag*/
#else
	magCalibSaveAddr = AT24CXX_STOR_MAG_OFFSET_ADDR;		/*Module Mag*/
#endif
#endif	
	
	/*Mag_Offset*/
	magOffsetStatus = bsp_AT24CXX_Read_3_FloatData(&g_sAt24cxx, magCalibSaveAddr, \
												   &psMagCalibParaRead->Offset.x, \
												   &psMagCalibParaRead->Offset.y, \
												   &psMagCalibParaRead->Offset.z);
	
	/*验证成立&数据正常,作为校准参数*/
	if (magOffsetStatus == SYS_RET_SUCC)
	{
		/*Mag_Offset*/
		g_fpMagOffsetX = psMagCalibParaRead->Offset.x;
		g_fpMagOffsetY = psMagCalibParaRead->Offset.y;
		g_fpMagOffsetZ = psMagCalibParaRead->Offset.z;

		/*ret succ*/
		retStatus = SYS_RET_SUCC;	
	}	
	else
	{
		retStatus = SYS_RET_FAIL;
	}
	
	return retStatus;
}

/*全部传感器校准参数初始化(读取)*/
SYS_RETSTATUS calib_all_sensor_parameter_read(void)
{
	SYS_RETSTATUS retStatus = SYS_RET_FAIL;
	
	/*加速度计校准参数初始化(读取)*/
	retStatus = calib_Acc_Sensor_parameter_Read();
	
	if (retStatus == SYS_RET_SUCC)
	{
		/*标记加速度校准最终结果为成功,满足起飞条件之一*/
		g_psAccCalibSystem->RESULT_STATUS = RESULT_CALIB_SUCC;
	}
	else
	{		
		/*标记加速度校准最终结果为失败,需要重新校准*/
		g_psAccCalibSystem->RESULT_STATUS = RESULT_CALIB_FAIL;		
	}
	
	/*磁力计校准参数初始化(读取)*/	
	retStatus = calib_Mag_Sensor_parameter_Read();	
	
	if (retStatus == SYS_RET_SUCC)
	{
		/*标记磁力计校准最终结果为成功,满足起飞条件之一*/
		g_psMagCalibSystem->RESULT_STATUS = RESULT_CALIB_SUCC;		
	}
	else
	{
		/*标记磁力计校准最终结果为失败,需要重新校准*/		
		g_psMagCalibSystem->RESULT_STATUS = RESULT_CALIB_FAIL;
	}
	
	/*开机显示传感器(加速度计&磁力计)校准参数*/
	#if (HCI_OLED_SHOW_SENSOR_CALIB_PARA_WHEN_BOOT == SYS_ENABLE)
	if (g_sUav_Status.UavProgrameStatus.INIT_STATUS == UAV_PROGRAME_INIT_START)	/*开机过程才显示*/
	{
		/*传感器校准结果及参数读取并显示在OLED上,并保持2000ms*/
		hci_Show_Sensor_Calib_Parameter(g_psAccCalibSystem->RESULT_STATUS, g_psMagCalibSystem->RESULT_STATUS, 2000);
	}
	#endif
	
	/*运行过程中显示传感器(加速度计&磁力计)校准参数*/
	#if (HCI_OLED_SHOW_SENSOR_CALIB_PARA_WHEN_BOOT != SYS_ENABLE)
//	if (g_sUav_Status.UavProgrameStatus.INIT_STATUS == UAV_PROGRAME_INIT_FINISH) /*运行过程中才能显示*/
//	{
//		/*传感器校准结果及参数读取并显示在OLED上,并保持2000ms*/
//		hci_Show_Sensor_Calib_Parameter(g_psAccCalibSystem->RESULT_STATUS, g_psMagCalibSystem->RESULT_STATUS, 2000);
//	}
	#endif	

	return retStatus;
}


/*=== 原始数据校准 ===*/
/*加速度计原始数据(椭球)校准*/
Acc3f* calib_Acc_Data_Dp(Acc3s *acc3s)
{
	/*加速度计数据校准成功了,可对原始数据进行校准*/
	if (g_psAccCalibSystem->RESULT_STATUS == RESULT_CALIB_SUCC)
	{
		/*灵敏度修正*/
		g_psAccCorrect->x =  acc3s->x * g_fpAccScaleX;
		g_psAccCorrect->y =  acc3s->y * g_fpAccScaleY;
		g_psAccCorrect->z =  acc3s->z * g_fpAccScaleZ;
	
		/*减去零偏*/
		g_psAccCorrect->x -= g_fpAccOffsetX * ONE_G_TO_ACC;
		g_psAccCorrect->y -= g_fpAccOffsetY * ONE_G_TO_ACC;
		g_psAccCorrect->z -= g_fpAccOffsetZ * ONE_G_TO_ACC;	
	}
	else	/*加速度计数据校准失败了,需要用原始数据进行校准*/
	{
		g_psAccCorrect->x =  acc3s->x;
		g_psAccCorrect->y =  acc3s->y;
		g_psAccCorrect->z =  acc3s->z;		
	}
	
	return g_psAccCorrect;
}

/*磁力计原始数据(减零偏)校准*/
Mag3f* calib_Mag_Data_Dp(Mag3s *mag3s)
{
	/*减去零偏值*/
	g_psMagCorrect->x = mag3s->x - g_fpMagOffsetX;
	g_psMagCorrect->y = mag3s->y - g_fpMagOffsetY;
	g_psMagCorrect->z = mag3s->z - g_fpMagOffsetZ;	
	
	return g_psMagCorrect;
}

/*=== 磁力计最小二乘法中间值校准 ===*/
u32 calib_mag_sensor_lsiv_accumulate(Mag_Calib_LSIV *magLSIV, Mag3f magf3)
{
	fp32 x2 = magf3.x * magf3.x;
	fp32 y2 = magf3.y * magf3.y;
	fp32 z2 = magf3.z * magf3.z;
	
	magLSIV->x_sumplain += magf3.x;
	magLSIV->x_sumsq    += x2;
	magLSIV->x_sumcube  += x2 * magf3.x;

	magLSIV->y_sumplain += magf3.y;
	magLSIV->y_sumsq    += y2;
	magLSIV->y_sumcube  += y2 * magf3.y;

	magLSIV->z_sumplain += magf3.z;
	magLSIV->z_sumsq    += z2;
	magLSIV->z_sumcube  += z2 * magf3.z;

	magLSIV->xy_sum     += magf3.x * magf3.y;
	magLSIV->xz_sum     += magf3.x * magf3.z;
	magLSIV->yz_sum     += magf3.y * magf3.z;

	magLSIV->x2y_sum    += x2 * magf3.y;
	magLSIV->x2z_sum    += x2 * magf3.z;

	magLSIV->y2x_sum    += y2 * magf3.x;
	magLSIV->y2z_sum    += y2 * magf3.z;

	magLSIV->z2x_sum    += z2 * magf3.x;
	magLSIV->z2y_sum    += z2 * magf3.y;

	magLSIV->size++;

	return (magLSIV->size);	
}

void calib_mag_sensor_lsiv_calculate(Mag_Calib_LSIV *magLSIV, u32 maxIterations, fp32 delta, Vector4f *Sphere)
{
	//
	//Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
	//
	//    P is a structure that has been computed with the data earlier.
	//    P.npoints is the number of elements; the length of X,Y,Z are identical.
	//    P's members are logically named.
	//
	//    X[n] is the x component of point n
	//    Y[n] is the y component of point n
	//    Z[n] is the z component of point n
	//
	//    A is the x coordiante of the sphere
	//    B is the y coordiante of the sphere
	//    C is the z coordiante of the sphere
	//    Rsq is the radius squared of the sphere.
	//
	//This method should converge; maybe 5-100 iterations or more.
	//
	fp32 x_sum  = magLSIV->x_sumplain / magLSIV->size;        //sum( X[n] )
	fp32 x_sum2 = magLSIV->x_sumsq / magLSIV->size;    //sum( X[n]^2 )
	fp32 x_sum3 = magLSIV->x_sumcube / magLSIV->size;    //sum( X[n]^3 )
	fp32 y_sum  = magLSIV->y_sumplain / magLSIV->size;        //sum( Y[n] )
	fp32 y_sum2 = magLSIV->y_sumsq / magLSIV->size;    //sum( Y[n]^2 )
	fp32 y_sum3 = magLSIV->y_sumcube / magLSIV->size;    //sum( Y[n]^3 )
	fp32 z_sum  = magLSIV->z_sumplain / magLSIV->size;        //sum( Z[n] )
	fp32 z_sum2 = magLSIV->z_sumsq / magLSIV->size;    //sum( Z[n]^2 )
	fp32 z_sum3 = magLSIV->z_sumcube / magLSIV->size;    //sum( Z[n]^3 )

	fp32 XY  = magLSIV->xy_sum / magLSIV->size;        //sum( X[n] * Y[n] )
	fp32 XZ  = magLSIV->xz_sum / magLSIV->size;        //sum( X[n] * Z[n] )
	fp32 YZ  = magLSIV->yz_sum / magLSIV->size;        //sum( Y[n] * Z[n] )
	fp32 X2Y = magLSIV->x2y_sum / magLSIV->size;    //sum( X[n]^2 * Y[n] )
	fp32 X2Z = magLSIV->x2z_sum / magLSIV->size;    //sum( X[n]^2 * Z[n] )
	fp32 Y2X = magLSIV->y2x_sum / magLSIV->size;    //sum( Y[n]^2 * X[n] )
	fp32 Y2Z = magLSIV->y2z_sum / magLSIV->size;    //sum( Y[n]^2 * Z[n] )
	fp32 Z2X = magLSIV->z2x_sum / magLSIV->size;    //sum( Z[n]^2 * X[n] )
	fp32 Z2Y = magLSIV->z2y_sum / magLSIV->size;    //sum( Z[n]^2 * Y[n] )

	//Reduction of multiplications
	fp32 F0 = x_sum2 + y_sum2 + z_sum2;
	fp32 F1 =  0.5f * F0;
	fp32 F2 = -8.0f * (x_sum3 + Y2X + Z2X);
	fp32 F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
	fp32 F4 = -8.0f * (X2Z + Y2Z + z_sum3);

	//Set initial conditions:
	fp32 A = x_sum;
	fp32 B = y_sum;
	fp32 C = z_sum;

	//First iteration computation:
	fp32 A2 = A * A;
	fp32 B2 = B * B;
	fp32 C2 = C * C;
	fp32 QS = A2 + B2 + C2;
	fp32 QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);

	//Set initial conditions:
	fp32 Rsq = F0 + QB + QS;

	//First iteration computation:
	fp32 Q0 = 0.5f * (QS - Rsq);
	fp32 Q1 = F1 + Q0;
	fp32 Q2 = 8.0f * (QS - Rsq + QB + F0);
	fp32 aA, aB, aC, nA, nB, nC, dA, dB, dC;

	//Iterate N times, ignore stop condition.
	unsigned int n = 0;

	while(n < maxIterations) 
	{
		  n++;

		  //Compute denominator:
		  aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
		  aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
		  aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
		  aA = (aA == 0.0f) ? 1.0f : aA;
		  aB = (aB == 0.0f) ? 1.0f : aB;
		  aC = (aC == 0.0f) ? 1.0f : aC;

		  //Compute next iteration
		  nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
		  nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
		  nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);

		  //Check for stop condition
		  dA = (nA - A);
		  dB = (nB - B);
		  dC = (nC - C);

		  if ((dA * dA + dB * dB + dC * dC) <= delta) { break; }

		  //Compute next iteration's values
		  A = nA;
		  B = nB;
		  C = nC;
		  A2 = A * A;
		  B2 = B * B;
		  C2 = C * C;
		  QS = A2 + B2 + C2;
		  QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
		  Rsq = F0 + QB + QS;
		  Q0 = 0.5f * (QS - Rsq);
		  Q1 = F1 + Q0;
		  Q2 = 8.0f * (QS - Rsq + QB + F0);
	}

	Sphere->a = A;
	Sphere->b = B;
	Sphere->c = C;
	Sphere->r = sqrt(Rsq);
}

/*=== 加速度椭球校准法 ===*/
/*加速度椭球校准*/
SYS_RETSTATUS calib_Acc_Gauss_Newton_Dp(Acc3f sampleData[], AccCalibPara *accCalibPara)
{
    SYS_RETSTATUS retStatus = SYS_RET_SUCC;		
    s16 i, numIterations = 0;
    fp32 eps      = 0.000000001f;
    fp32 change   = 100.0f;
    fp32 data[3]  = {0};
    fp32 beta[6]  = {0};
    fp32 delta[6] = {0};
    fp32 ds[6]    = {0};
    fp32 JS[6][6] = {0};
	
	/*reset*/
	beta[3] = 1.0f / GRAVITY_STD;
	beta[4] = 1.0f / GRAVITY_STD;
	beta[5] = 1.0f / GRAVITY_STD;	
	
	while((numIterations < 20) && (change > eps))
	{
		numIterations++;
		
		/*矩阵重置*/
		calib_ElipsoidCorrect_Matrix_Reset(ds, JS);
		
		for (i = 0; i < 6; i++)
		{
			data[0] = sampleData[i].x;
			data[1] = sampleData[i].y;
			data[2] = sampleData[i].z;
			
			/*矩阵更新*/
			calib_ElipsoidCorrect_Matrix_Update(ds, JS, beta, data);
		}
		
		/*找到差值*/
		calib_ElipsoidCorrect_Find_Delta(ds, JS, delta);
		
		change = delta[0] * delta[0] + \
				 delta[0] * delta[0] + \
				 delta[1] * delta[1] + \
			     delta[2] * delta[2] + \
				 delta[3] * delta[3] / (beta[3] * beta[3]) + \
				 delta[4] * delta[4] / (beta[4] * beta[4]) + \
				 delta[5] * delta[5] / (beta[5] * beta[5]);
		
		for (i = 0; i < 6; i++)
		{
			beta[i] -= delta[i];
		}
	}
	
	/*copy results out*/
	accCalibPara->Scale.x  = beta[3] * GRAVITY_STD;
	accCalibPara->Scale.y  = beta[4] * GRAVITY_STD;
	accCalibPara->Scale.z  = beta[5] * GRAVITY_STD;
	accCalibPara->Offset.x = beta[0] * accCalibPara->Scale.x;
	accCalibPara->Offset.y = beta[1] * accCalibPara->Scale.y;
    accCalibPara->Offset.z = beta[2] * accCalibPara->Scale.z;
	
    /*sanity check scale*/
    if ((fabsf(accCalibPara->Scale.x - 1.0f) > 0.5f) || \
		(fabsf(accCalibPara->Scale.y - 1.0f) > 0.5f) || \
        (fabsf(accCalibPara->Scale.z - 1.0f) > 0.5f))
    {
        retStatus = SYS_RET_FAIL;
    }
	
    /*sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)*/
    if ((fabsf(accCalibPara->Offset.x) > 5.0f) || \
        (fabsf(accCalibPara->Offset.y) > 5.0f) || \
        (fabsf(accCalibPara->Offset.z) > 5.0f ))
    {
		retStatus = SYS_RET_FAIL;
    }
    
	/*return success or failure*/
    return retStatus;
}
/*矩阵重置*/
void calib_ElipsoidCorrect_Matrix_Reset(fp32 ds[], fp32 JS[][6])
{
	u8 j, k;
	
	for (j = 0; j < 6; j++)
	{
		ds[j] = 0.0f;
		
		for (k = 0; k < 6; k++)
		{
			JS[j][k] = 0.0f;
		}
	}
}

/*矩阵更新*/
void calib_ElipsoidCorrect_Matrix_Update(fp32 ds[], fp32 JS[][6], fp32 beta[], fp32 data[])
{
    s8 j, k;
    fp32 dx, b;
    fp32 residual = 1.0f;
    fp32 jacobian[6];
	
    for (j = 0; j < 3; j++)
    {
        b = beta[3+j];
        dx = (fp32)data[j] - beta[j];
        residual     -= b * b * dx * dx;
        jacobian[j]   = 2.0f * b * b * dx;
        jacobian[3+j] = -2.0f * b * dx * dx;
    }

    for (j = 0; j < 6; j++)
    {
        ds[j] += jacobian[j] * residual;
		
        for (k = 0; k < 6; k++)
        {
            JS[j][k] += jacobian[j] * jacobian[k];
        }
    }
}

/*找到差值*/
void calib_ElipsoidCorrect_Find_Delta(fp32 ds[], fp32 JS[][6], fp32 delta[])
{
    /*Solve 6-d matrix equation JS*x = dS
      first put in upper triangular form*/
    s16 i, j, k;
    fp32 mu;
	
    /*make upper triangular*/
    for (i = 0; i < 6; i++)
	{
        /*eliminate all nonzero entries below JS[i][i]*/
        for (j = i + 1; j < 6; j++)
		{
            mu = JS[i][j] / JS[i][i];
			
            if (mu != 0.0f)
			{
                ds[j] -= mu * ds[i];
				
                for (k = j; k < 6; k++)
				{
                    JS[k][j] -= mu * JS[k][i];
                }
            }
        }
    }
	
    /*back-substitute*/
    for (i = 5; i >= 0; i--)
	{
        ds[i] /= JS[i][i];
        JS[i][i] = 1.0f;

        for (j = 0; j < i; j++)
		{
            mu = JS[i][j];
            ds[j] -= mu * ds[i];
            JS[i][j] = 0.0f;
        }
    }
	
    for(i = 0; i < 6; i++)
	{
        delta[i] = ds[i];
    }	
}
