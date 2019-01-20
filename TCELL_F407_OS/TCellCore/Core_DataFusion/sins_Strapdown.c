#include "sins_Strapdown.h"
#include "filter_Kalman.h"
#include "control_Aircraft.h"

#include "safe_Operation.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

/*=== 导航系加速度 ===*/
/*惯导融合加速度*/
SINS g_sSinsReal   = {0};
SINS *g_psSinsReal = &g_sSinsReal;

/*导航系原始加速度*/
SINS g_sSinsOrigion   = {0};
SINS *g_psSinsOrigion = &g_sSinsOrigion;

/*滤波后的惯导反馈量*/
SINS g_sSinsFilterFeedback   = {0};
SINS *g_psSinsFilterFeedback = &g_sSinsFilterFeedback;

/*=== 三阶互补 ===*/
TOCSystem g_sTOCSystem   = {0};
TOCSystem *g_psTOCSystem = &g_sTOCSystem;

/*惯导中的加速步长*/
fp32 g_SinsAccLenth = 0;
/*机体相对于导航系的加速度(惯导状态)*/
Vector2f g_sSinsAccEarth = {0};
/*机体的加速度(惯导状态)*/
Vector2f g_sSinsAccBody  = {0};


/******************************************
原始运动加速度方向为东北天方向

g_sSinsOrigion.curAcc[SINS_ROLL]
    为载体沿着正北方向的运动加速度
g_sSinsOrigion.curAcc[SINS_PITCH]
    为载体沿着正东方向的运动加速度
g_sSinsOrigion.curAcc[SINS_YAW]
    为载体沿着‘天’方向的运动加速度

载体沿着正东(E)、正北(N)、正天(U)运动时,加速度方向为正

        U(正天、气压高度、SINS_YAW)
        *
        *
        *
        *
        *
        *
        * * * * * * * * * *N(正北、纬线方向、SINS_ROLL)
         *
           *
             *
               *
                 *
                   *
                     E(正东、经线方向、SINS_PITCH)
*/

/*机体系->导航系 加速度*/
void sins_get_body_relative_earth_acc(Acc3f *sinsAcc)
{
    /*Z-Y-X欧拉角转方向余弦矩阵
	/Pitch Roll  Yaw 分别对应Φ θ Ψ

             X轴旋转矩阵
             R（Φ）
        {1      0        0    }
        {0      cosΦ    sinΦ}
        {0    -sinΦ    cosΦ }

             Y轴旋转矩阵
             R（θ）
        {cosθ     0        -sinθ}
        {0         1        0     }
        {sinθ     0        cosθ}

             Z轴旋转矩阵
             R（θ）
        {cosΨ      sinΨ       0}
        {-sinΨ     cosΨ       0}
        {0          0           1 }

	由Z-Y-X顺规有:
    载体坐标系到导航坐标系下旋转矩阵R(b2n)
    R(b2n) =R(Ψ)^T*R(θ)^T*R(Φ)^T

    R=
      {cosΨ*cosθ     -cosΦ*sinΨ+sinΦ*sinθ*cosΨ        sinΨ*sinΦ+cosΦ*sinθ*cosΨ}
      {cosθ*sinΨ     cosΦ*cosΨ +sinΦ*sinθ*sinΨ       -cosΨ*sinΦ+cosΦ*sinθ*sinΨ}
      {-sinθ          cosθsin Φ                          cosθcosΦ                   }
    */

	/*机体系方向余弦矩阵转导航系方向余弦矩阵*/
	g_psBodyFrame->x = g_psAccSINS->x;
	g_psBodyFrame->y = g_psAccSINS->y;
	g_psBodyFrame->z = g_psAccSINS->z;
	
	g_psEarthFrame = ahrs_vector_body_to_earth(g_psBodyFrame, g_psEarthFrame);
	g_psSinsOrigion->curAcc[EARTH_FRAME_X] = g_psEarthFrame->x; /*导航系x E*/
	g_psSinsOrigion->curAcc[EARTH_FRAME_Y] = g_psEarthFrame->y; /*导航系y N*/
	g_psSinsOrigion->curAcc[EARTH_FRAME_Z] = g_psEarthFrame->z; /*导航系z U*/

	/*加速度转化为G,单位为cm/s^2*/
	g_psSinsOrigion->curAcc[EARTH_FRAME_X] *= SINS_ACC_GRAVITY * MPU_ACC_RANGE;
	g_psSinsOrigion->curAcc[EARTH_FRAME_X] *= 100;	/*m/s^2 -> cm/s^s*/
	
	g_psSinsOrigion->curAcc[EARTH_FRAME_Y] *= SINS_ACC_GRAVITY * MPU_ACC_RANGE;
	g_psSinsOrigion->curAcc[EARTH_FRAME_Y] *= 100;	/*m/s^2 -> cm/s^s*/
	
	g_psSinsOrigion->curAcc[EARTH_FRAME_Z] *= SINS_ACC_GRAVITY * MPU_ACC_RANGE;
	g_psSinsOrigion->curAcc[EARTH_FRAME_Z] -= SINS_ACC_GRAVITY;	/*减去Z轴重力加速度*/
	g_psSinsOrigion->curAcc[EARTH_FRAME_Z] *= 100;	/*m/s^2 -> cm/s^s*/	
	
	/*计算当前机体加速度的步长*/
	g_SinsAccLenth = sqrt(g_psSinsOrigion->curAcc[EARTH_FRAME_X] * g_psSinsOrigion->curAcc[EARTH_FRAME_X] + \
						  g_psSinsOrigion->curAcc[EARTH_FRAME_Y] * g_psSinsOrigion->curAcc[EARTH_FRAME_Y] + \
						  g_psSinsOrigion->curAcc[EARTH_FRAME_Z] * g_psSinsOrigion->curAcc[EARTH_FRAME_Z]);
						  
	/*将无人机在导航坐标系下的沿着正东、正北方向的运动加速度旋转到当前载体坐标系的运动加速度:机头(俯仰)+横滚*/
//	g_sSinsAccEarth.x = g_psSinsOrigion->curAcc[EARTH_FRAME_X];	/*沿地理坐标系,正东方向运动加速度,单位为CM*/
//	g_sSinsAccEarth.y = g_psSinsOrigion->curAcc[EARTH_FRAME_Y];	/*沿地理坐标系,正北方向运动加速度,单位为CM*/
//	
//	g_sSinsAccBody.x = g_sSinsAccEarth.x * COS_YAW + g_sSinsAccEarth.y * SIN_YAW;	 /*横滚正向运动加速度,X轴正向*/
//	g_sSinsAccBody.y = -g_sSinsAccEarth.x * SIN_YAW + g_sSinsAccEarth.y * COS_YAW;	 /*机头正向运动加速度,Y轴正向*/
}

/*竖直方向,气压计和超声波切换(非独立融合)*/
void sins_vertical_bero_ultr_auto_change(Uav_Status *uavStatus)
{
	u16 i;
	
	/*|------------|
		     |
		 pos > SYS_ULTR_MAX_MEAS_DISTANCE cm (bero)
			 |
	  |------------|
	         |
	    0 < pos < SYS_ULTR_MAX_MEAS_DISTANCE cm (ultr / bero)
	         |
	  |------------|
	*/
	
	/*状态更迭*/
	uavStatus->UavSenmodStatus.Vertical.LAST_USE = uavStatus->UavSenmodStatus.Vertical.CURRENT_USE;
	
	/*遥控指定: 竖直方向仅使用无限制传感器*/
	if (uavStatus->UavSenmodStatus.Vertical.WORK_STATUS == UAV_SENMOD_WORK_LIMITLESS)
	{
		/*气压计可用,就使用气压计作为当前竖直方向观测传感器*/
		if (get_bero_estimate_data_status(uavStatus) == UAV_SENMOD_DATA_OK)
		{	
			/*竖直高度观测值为气压计观测高度*/		
			g_psSinsReal->estimatePos[EARTH_FRAME_Z] = g_psAttitudeAll->BaroData.curAltitude;
			
			/*传感器数据同步cnt_5MS*/
			g_psSinsReal->sensorDataSync5ms[EARTH_FRAME_Z] = 20;
			
			/*标记气压计能用于控制*/
			uavStatus->UavSenmodStatus.Vertical.Bero.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_ALLOW;				
			
			/*标记本次使用气压计*/
			uavStatus->UavSenmodStatus.Vertical.CURRENT_USE = UAV_VERTICAL_SENMOD_CURRENT_BERO;		
		}
		else
		{	
			/*标记气压计不能用于控制*/
			uavStatus->UavSenmodStatus.Vertical.Bero.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_DISALLOW;			
			
			/*标记竖直传感器使用NULL*/
			uavStatus->UavSenmodStatus.Vertical.CURRENT_USE = UAV_VERTICAL_SENMOD_CURRENT_NULL;			
		}
	}
	/*遥控指定: 竖直方向使用限制传感器和无限制传感器自动切换*/
	else if (uavStatus->UavSenmodStatus.Vertical.WORK_STATUS == UAV_SENMOD_WORK_AUTO_SWITCH)
	{
		/*超声波可用,就使用超声波作为当前竖直方向观测传感器*/
		if (get_ultr_estimate_data_status(uavStatus) == UAV_SENMOD_DATA_OK)
		{
			/*竖直高度观测值为超声波观测高度*/
			g_psSinsReal->estimatePos[EARTH_FRAME_Z] = g_psAttitudeAll->UltrData.curAltitude;

			/*传感器数据同步cnt_5MS*/			
			g_psSinsReal->sensorDataSync5ms[EARTH_FRAME_Z] = 20;		

			/*标记超声波能用于控制*/
			uavStatus->UavSenmodStatus.Vertical.Ultr.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_ALLOW;
			
			/*标记本次使用超声波*/
			uavStatus->UavSenmodStatus.Vertical.CURRENT_USE = UAV_VERTICAL_SENMOD_CURRENT_ULTR;
		}
		/*超声波不可用,就使用气压计作为当前竖直方向观测传感器*/
		else if (get_bero_estimate_data_status(uavStatus) == UAV_SENMOD_DATA_OK)
		{			
			/*竖直高度观测值为气压计观测高度*/		
			g_psSinsReal->estimatePos[EARTH_FRAME_Z] = g_psAttitudeAll->BaroData.curAltitude;

			/*传感器数据同步cnt_5MS*/			
			g_psSinsReal->sensorDataSync5ms[EARTH_FRAME_Z] = 20;			

			/*标记气压计能用于控制*/
			uavStatus->UavSenmodStatus.Vertical.Bero.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_ALLOW;			
			
			/*标记本次使用气压计*/
			uavStatus->UavSenmodStatus.Vertical.CURRENT_USE = UAV_VERTICAL_SENMOD_CURRENT_BERO;
		}
		/*超声波和气压计都无效*/
		else
		{			
			/*标记超声波不能用于控制*/
			uavStatus->UavSenmodStatus.Vertical.Ultr.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_DISALLOW;	
			
			/*标记气压计不能用于控制*/
			uavStatus->UavSenmodStatus.Vertical.Bero.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_DISALLOW;	

			/*标记竖直传感器使用NULL*/
			uavStatus->UavSenmodStatus.Vertical.CURRENT_USE = UAV_VERTICAL_SENMOD_CURRENT_NULL;			
		}
	}
	
	/*(气压计切超声波 || 超声波切气压计) && 切换成的传感器能用于控制*/
	if ((uavStatus->UavSenmodStatus.Vertical.CURRENT_USE != uavStatus->UavSenmodStatus.Vertical.LAST_USE) && \
		(uavStatus->UavSenmodStatus.Vertical.CURRENT_USE != UAV_VERTICAL_SENMOD_CURRENT_NULL))
	{
		/*设置当前位置*/
		g_psSinsReal->curPosition[EARTH_FRAME_Z] = g_psSinsReal->estimatePos[EARTH_FRAME_Z];
		
		/*位置老数据移动*/
		for (i = SINS_HISTORY_DATA_DEEP - 1; i > 0; i--)
		{
			g_psSinsReal->pos_History[EARTH_FRAME_Z][i] = g_psSinsReal->estimatePos[EARTH_FRAME_Z];
		}
		
		/*加入位置新数据*/
		g_psSinsReal->pos_History[EARTH_FRAME_Z][0] = g_psSinsReal->estimatePos[EARTH_FRAME_Z];
		
		/*设置当前原始位置和速度*/
		g_psSinsOrigion->curPosition[EARTH_FRAME_Z] = g_psSinsReal->estimatePos[EARTH_FRAME_Z];
		g_psSinsOrigion->curSpeed[EARTH_FRAME_Z]    = 0;
		
		/*复位*/
		g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].acc   = 0;  /*加速度矫正量*/
		g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].speed = 0;  /*速度矫正量*/
		g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].pos   = 0;  /*位置矫正量*/
		
		/*重置竖直方向参考点*/
		uavStatus->UavControlMode.Vertical.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_NO;
	}
}

/*水平方向,GPS和光流切换(独立融合)*/
void sins_horizontal_gps_opticflow_auto_change(Uav_Status *uavStatus)
{	
	/*|------------|
		     |
		 pos > SYS_ULTR_MAX_MEAS_DISTANCE cm    (gps)
			 |
	  |------------|
	         |
	    0 < pos < SYS_ULTR_MAX_MEAS_DISTANCE cm (gps / opticflow)
	         |
	  |------------|
	*/
	
	/*状态更迭*/
	uavStatus->UavSenmodStatus.Horizontal.LAST_USE = uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE;
	
	/*遥控指定: 水平方向仅使用无限制传感器*/
	if (uavStatus->UavSenmodStatus.Horizontal.WORK_STATUS == UAV_SENMOD_WORK_LIMITLESS)
	{
		/*判断GPS是否可用于控制*/
		if (status_GPS_Fix_Ava_Check(uavStatus) == UAV_SENMOD_USE_CONTROL_ALLOW)
		{
			/*标记GPS可用于水平定点*/
			uavStatus->UavSenmodStatus.Horizontal.Gps.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_ALLOW;
			
			/*标记本次使用GPS*/
			uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE = UAV_HORIZONTAL_SENMOD_CURRENT_GPS;
		}
		else
		{
			/*标记GPS不可用于水平定点*/
			uavStatus->UavSenmodStatus.Horizontal.Gps.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_DISALLOW;
			
			/*标记本次使用NULL*/
			uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE = UAV_HORIZONTAL_SENMOD_CURRENT_NULL;			
		}
	}
	/*遥控指定: 水平方向使用限制传感器和无限制传感器自动切换*/
	else if (uavStatus->UavSenmodStatus.Horizontal.WORK_STATUS == UAV_SENMOD_WORK_AUTO_SWITCH)
	{
		/*判断当前竖直方向是否是用超声波(超声波可用+高度在有效高度以下) && 光流可用*/
		if ((uavStatus->UavSenmodStatus.Vertical.CURRENT_USE == UAV_VERTICAL_SENMOD_CURRENT_ULTR) && \
			(uavStatus->UavSenmodStatus.Horizontal.Opticflow.DATA_STATUS == UAV_SENMOD_DATA_OK))
		{
			/*标记光流能用于控制*/
			uavStatus->UavSenmodStatus.Horizontal.Opticflow.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_ALLOW;
			
			/*标记本次使用光流*/
			uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE = UAV_HORIZONTAL_SENMOD_CURRENT_OPTICFLOW;			
		}
		/*光流不可用,就使用GPS作为当前水平方向观测传感器*/
		else if (status_GPS_Fix_Ava_Check(uavStatus) == UAV_SENMOD_USE_CONTROL_ALLOW)
		{
			/*判断GPS是否可用于控制*/
			uavStatus->UavSenmodStatus.Horizontal.Gps.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_ALLOW;
		
			/*标记本次使用GPS*/
			uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE = UAV_HORIZONTAL_SENMOD_CURRENT_GPS;		
		}
		/*GPS和光流都无效*/		
		else
		{
			/*标记GPS不可用于水平定点*/
			uavStatus->UavSenmodStatus.Horizontal.Gps.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_DISALLOW;
			
			/*标记光流不能用于控制*/
			uavStatus->UavSenmodStatus.Horizontal.Opticflow.USE_CONTROL_STATUS = UAV_SENMOD_USE_CONTROL_DISALLOW;			
			
			/*标记水平方向本次使用NULL*/
			uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE = UAV_HORIZONTAL_SENMOD_CURRENT_NULL;			
		}		
	}
	
	/*发生传感器切换 && 传感器可用,重置水平参考点*/
	if ((uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE != uavStatus->UavSenmodStatus.Horizontal.LAST_USE) && \
		(uavStatus->UavSenmodStatus.Horizontal.CURRENT_USE != UAV_HORIZONTAL_SENMOD_CURRENT_NULL))
	{
		uavStatus->UavControlMode.Horizontal.REFERENCE_SET_STATUS = UAV_SWITCH_REFERENCE_SET_NO;
	}
}

#define TIME_CONTANST_ZER   (3.5f)
#define K_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_SPD_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_POS_ZER           (3.0f / TIME_CONTANST_ZER)

/*三阶互补求竖直方向上的加速度、速度、位置(高度)*/
vs16 g_TOCHighPosSolidTicks = 0;

void sins_thirdorder_complement_vertical(void)
{
	u8 i;
	fp32 sinsHighDeltaT;
	
	/*获取竖直方向惯导融合时间间隔*/
	get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->SINS_High));
	
	/*判断当前程序执行间隔是否是期望的执行间隔(很大情况下并不是调度的问题)*/
	if (safe_task_execute_period_is_true(g_psSystemPeriodExecuteTime->SINS_High.DeltaTime, RTOS_WAKE_UP_VER_FUSION_FOC_MS) == SYS_BOOL_FALSE)
	{	
		return;
	}		
	
	/*间隔时间换算成秒*/
	sinsHighDeltaT = g_psSystemPeriodExecuteTime->SINS_High.DeltaTime / 1000.0f;	
	
    /*超声波定高和气压计定高自动切换(选择高度观测数据来源)*/
	sins_vertical_bero_ultr_auto_change(g_psUav_Status);
	
	g_TOCHighPosSolidTicks++; /*数据存储周期*/	
	
	/*位置历史数据滑动*/
	if (g_TOCHighPosSolidTicks >= 1)	/*5ms*/
	{
		g_TOCHighPosSolidTicks = 0;
		
		for (i = SINS_HISTORY_DATA_DEEP - 1; i > 0; i--) /*20ms滑动一次*/
		{
			g_psSinsReal->pos_History[EARTH_FRAME_Z][i] = g_psSinsReal->pos_History[EARTH_FRAME_Z][i - 1];
		}
		
		g_psSinsReal->pos_History[EARTH_FRAME_Z][0] = g_psSinsReal->curPosition[EARTH_FRAME_Z];
	}	

	/*观测高度与估算(SINS)高度的误差(cm)*/
	g_psTOCSystem->estimateDealt[EARTH_FRAME_Z] = g_psSinsReal->estimatePos[EARTH_FRAME_Z] - g_psSinsReal->pos_History[EARTH_FRAME_Z][g_psSinsReal->sensorDataSync5ms[EARTH_FRAME_Z]];
	
	/*三路积分反馈量修正惯导的acc,speed,pos*/
	g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].acc   += g_psTOCSystem->estimateDealt[EARTH_FRAME_Z] * K_ACC_ZER * sinsHighDeltaT;  /*加速度矫正量*/
	g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].speed += g_psTOCSystem->estimateDealt[EARTH_FRAME_Z] * K_SPD_ZER * sinsHighDeltaT;  /*速度矫正量*/
	g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].pos   += g_psTOCSystem->estimateDealt[EARTH_FRAME_Z] * K_POS_ZER * sinsHighDeltaT;  /*位置矫正量*/
	
	/*1.加速度计矫正后更新*/
	g_psSinsReal->lastAcc[EARTH_FRAME_Z] = g_psSinsReal->curAcc[EARTH_FRAME_Z]; /*更新lastAcc*/
	g_psSinsReal->curAcc[EARTH_FRAME_Z]  = g_psSinsOrigion->curAcc[EARTH_FRAME_Z] + g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].acc; /*对加速度计的值进行校正*/
	
    /*2.速度增量矫正后更新，用于更新位置,由于步长h=0.005,相对较长
      这里采用二阶龙格库塔法更新微分方程，不建议用更高阶段，因为加速度信号非平滑*/
	g_psTOCSystem->speedDealt[EARTH_FRAME_Z] = ((g_psSinsReal->lastAcc[EARTH_FRAME_Z] + g_psSinsReal->curAcc[EARTH_FRAME_Z]) / 2.0f) * sinsHighDeltaT;
	
	/*惯导原始位置更新: x = vt + ((at) / 2)t ---> at = speedDealt*/
	g_psSinsOrigion->curPosition[EARTH_FRAME_Z] += (g_psSinsReal->curSpeed[EARTH_FRAME_Z] + \
											       (g_psTOCSystem->speedDealt[EARTH_FRAME_Z] * 0.5f)) * sinsHighDeltaT;
											  
    /*位置矫正后更新*/
	g_psSinsReal->curPosition[EARTH_FRAME_Z] = g_psSinsOrigion->curPosition[EARTH_FRAME_Z] + \
										       g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].pos; /*对SINS估算的位置值进行校正*/
										  
	/*3.原始速度更新*/
	g_psSinsOrigion->curSpeed[EARTH_FRAME_Z] += g_psTOCSystem->speedDealt[EARTH_FRAME_Z];

	/*速度矫正后更新*/
	g_psSinsReal->curSpeed[EARTH_FRAME_Z] = g_psSinsOrigion->curSpeed[EARTH_FRAME_Z] + \
									        g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].speed;	/*对SINS估算的速度值进行校正*/
											
	/*检测融合估计值和真实观测值误差*/
	if (math_Abs(g_psSinsReal->curPosition[EARTH_FRAME_Z] - g_psSinsReal->estimatePos[EARTH_FRAME_Z]) > 10)
	{
		/*标记水平融合失败*/
		g_psSinsReal->FUSION_STATUS[SINS_FUSION_VERTICAL] = SINS_FUSION_FAIL;
	}
	else
	{
		/*标记水平融合成功*/
		g_psSinsReal->FUSION_STATUS[SINS_FUSION_VERTICAL] = SINS_FUSION_SUCC;		
	}											
}


/*卡尔曼滤波求竖直方向上的加速度、速度、位置(Z竖直)*/
void sins_kalman_estimate_vertical(void)
{
	fp32 sinsHighDeltaT;
	u16 i;
	static vu16 speed_sync_cnt = 0; /*速度同步cnt*/
	
	/*获取竖直方向惯导融合时间间隔*/
	get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->SINS_High));
	
	/*判断当前程序执行间隔是否是期望的执行间隔(很大情况下并不是调度的问题)*/
	if (safe_task_execute_period_is_true(g_psSystemPeriodExecuteTime->SINS_High.DeltaTime, RTOS_WAKE_UP_VER_FUSION_FOC_MS) == SYS_BOOL_FALSE)
	{	
		return;
	}	
	
	/*间隔时间换算成秒*/
	sinsHighDeltaT = g_psSystemPeriodExecuteTime->SINS_High.DeltaTime / 1000.0f;
	
	/*超声波定高和气压计定高自动切换(选择高度观测数据来源)*/
	sins_vertical_bero_ultr_auto_change(g_psUav_Status);
	
	/*位置:老数据滑动*/
	for (i = SINS_HISTORY_DATA_DEEP - 1; i > 0; i--) /*5ms滑动*/
	{
		g_psSinsReal->pos_History[EARTH_FRAME_Z][i] = g_psSinsReal->pos_History[EARTH_FRAME_Z][i - 1];
	}
	
	/*位置:加入新数据*/
	g_psSinsReal->pos_History[EARTH_FRAME_Z][0] = g_psSinsReal->curPosition[EARTH_FRAME_Z];
	
	/*cnt++*/
	speed_sync_cnt++;
	
	if (speed_sync_cnt >= 20) /*100ms滑动*/
	{
		speed_sync_cnt = 0;
		
		/*速度:老数据滑动*/
		for (i = SINS_HISTORY_DATA_DEEP - 1; i > 0; i--)
		{
			g_psSinsReal->speed_History[EARTH_FRAME_Z][i] = g_psSinsReal->speed_History[EARTH_FRAME_Z][i - 1];
		}		
	}
	
	/*速度:加入新数据*/
	g_psSinsReal->speed_History[EARTH_FRAME_Z][0] = g_psSinsReal->curSpeed[EARTH_FRAME_Z];	
	
	filter_Kalman_Estimate_Vertical(g_psSinsReal->estimatePos[EARTH_FRAME_Z],  		  /*位置观测量*/
									g_psSinsReal->sensorDataSync5ms[EARTH_FRAME_Z],   /*观测传感器延时*/
									g_psSinsReal, 	  	   			      			  /*惯导结构体*/
									g_psSinsOrigion->curAcc[EARTH_FRAME_Z],	  		  /*系统原始驱动量*/
									&g_sFilterKalmanVertical,
									EARTH_FRAME_Z, 							  		  /*z = 2*/
									sinsHighDeltaT);
}


/*三阶互补求水平方向上的加速度、速度、位置(X,Y水平)*/
#define TIME_CONTANST_XY     (2.5f)
#define K_ACC_XY	         (1.0f / (TIME_CONTANST_XY * TIME_CONTANST_XY * TIME_CONTANST_XY))
#define K_SPD_XY             (3.0f / (TIME_CONTANST_XY * TIME_CONTANST_XY))															// XY????·′à??μêy,3.0
#define K_POS_XY             (3.0f / TIME_CONTANST_XY)

vs16 g_TOCHorizontalPosSolidTicks = 0;
u16 g_HorizontalDelayCnt = 40;	 /*200ms: buff[49] -> buff[9] = 40 * 5ms*/
Vector2f_Body  g_sAccCorrectBodyFrame = {0};    /*在载体机头方向,加速度矫正量*/
Vector2f_Body  g_sPosErrorOnBodyFrame = {0};    /*载体系 位置误差*/
Vector2f_Earth  g_sAccCorrectEarthFrame = {0};  /*在导航系,加速度矫正量*/

void sins_thirdorder_complement_horizontal(void)
{
	u8 i;
	fp32 sinsHorizontalDeltaT;
	
	/*获取水平方向惯导融合时间间隔*/
	get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->SINS_Horizontal));
	
	/*判断当前程序执行间隔是否是期望的执行间隔(很大情况下并不是调度的问题)*/
	if (safe_task_execute_period_is_true(g_psSystemPeriodExecuteTime->SINS_Horizontal.DeltaTime, RTOS_WAKE_UP_GPS_HOR_FUSION_FOC_MS) == SYS_BOOL_FALSE)
	{
		return;
	}	
	
	/*间隔时间换算成秒*/
	sinsHorizontalDeltaT = g_psSystemPeriodExecuteTime->SINS_Horizontal.DeltaTime / 1000.0f;
	
	/*GPS获取机体在导航系 相对home的水平偏移*/
	gps_Offset_Relative_To_Home();	
	
	/*导航坐标系下,正北、正东方向位置,单位cm*/
	g_psSinsReal->estimatePos[EARTH_FRAME_Y] = g_psAttitudeAll->EarthFrameRelativeHome.north;	
	g_psSinsReal->estimatePos[EARTH_FRAME_X] = g_psAttitudeAll->EarthFrameRelativeHome.east;
	
	g_TOCHorizontalPosSolidTicks++;
	
	/*10ms滑动一次*/
	if (g_TOCHorizontalPosSolidTicks >= 2) /*10ms*/
	{
		g_TOCHorizontalPosSolidTicks = 0;
		
		/*位置历史数据滑动*/
		for(i = SINS_HISTORY_DATA_DEEP - 1; i > 0; i--) /*10ms滑动一次*/
        {
			/*位置*/
			g_psSinsReal->pos_History[EARTH_FRAME_Y][i] = g_psSinsReal->pos_History[EARTH_FRAME_Y][i - 1];  /*Y正北方向*/
			g_psSinsReal->pos_History[EARTH_FRAME_X][i] = g_psSinsReal->pos_History[EARTH_FRAME_X][i - 1];  /*X正东方向*/
			
			/*速度*/
			g_psSinsReal->speed_History[EARTH_FRAME_X][i] = g_psSinsReal->speed_History[EARTH_FRAME_X][i - 1];   /*X正东方向*/
			g_psSinsReal->speed_History[EARTH_FRAME_Y][i] = g_psSinsReal->speed_History[EARTH_FRAME_Y][i - 1];  /*Y正北方向*/				
        }
		
		/*加入位置新数据*/
		g_psSinsReal->pos_History[EARTH_FRAME_X][0] = g_psSinsReal->curPosition[EARTH_FRAME_X];   /*X正东方向*/
		g_psSinsReal->pos_History[EARTH_FRAME_Y][0] = g_psSinsReal->curPosition[EARTH_FRAME_Y];  /*Y正北方向*/		
		
		/*加入速度新数据*/
		g_psSinsReal->speed_History[EARTH_FRAME_X][0] = g_psSinsReal->curSpeed[EARTH_FRAME_X];   /*X正东方向*/
		g_psSinsReal->speed_History[EARTH_FRAME_Y][0] = g_psSinsReal->curSpeed[EARTH_FRAME_Y];  /*Y正北方向*/		
	}
	
	/*导航坐标系下,正北、正东方向位置偏移与SINS估计量的差,单位cm*/
	g_psTOCSystem->estimateDealt[EARTH_FRAME_Y] = g_psSinsReal->estimatePos[EARTH_FRAME_Y] - \
										          g_psSinsReal->pos_History[EARTH_FRAME_Y][g_HorizontalDelayCnt];
	
	g_psTOCSystem->estimateDealt[EARTH_FRAME_X] = g_psSinsReal->estimatePos[EARTH_FRAME_X] - \
											      g_psSinsReal->pos_History[EARTH_FRAME_X][g_HorizontalDelayCnt];

	/*导航系 转 机体系 沿Roll方向,X轴*/
	g_sPosErrorOnBodyFrame.roll  = g_psTOCSystem->estimateDealt[EARTH_FRAME_X] * COS_YAW + g_psTOCSystem->estimateDealt[EARTH_FRAME_Y] * SIN_YAW;	
	
	/*导航系 转 机体系 沿Pitch方向,Y轴*/
	g_sPosErrorOnBodyFrame.pitch = -g_psTOCSystem->estimateDealt[EARTH_FRAME_X] * SIN_YAW + g_psTOCSystem->estimateDealt[EARTH_FRAME_Y] * COS_YAW;	

	/*机体系 沿Pitch方向,y轴 加速度矫正量*/
	g_sAccCorrectBodyFrame.pitch += g_sPosErrorOnBodyFrame.pitch * K_ACC_XY * sinsHorizontalDeltaT; 
	
	/*机体系 沿Roll方向,x轴 加速度矫正量*/
	g_sAccCorrectBodyFrame.roll  += g_sPosErrorOnBodyFrame.roll * K_ACC_XY * sinsHorizontalDeltaT; 

	/*将机体方向上加速度修正量，旋转至导航系北向，Y轴*/
	g_sAccCorrectEarthFrame.north = g_sAccCorrectBodyFrame.roll * SIN_YAW + g_sAccCorrectBodyFrame.pitch * COS_YAW;
	
	/*将机体方向上加速度修正量，旋转至导航系东向，X轴*/
	g_sAccCorrectEarthFrame.east  = g_sAccCorrectBodyFrame.roll * COS_YAW - g_sAccCorrectBodyFrame.pitch * SIN_YAW;
	
	/*三路积分反馈量修正惯导的acc,speed,pos*/
	/*Y正北*/
	g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].acc   += g_psTOCSystem->estimateDealt[EARTH_FRAME_Y] * K_ACC_XY * sinsHorizontalDeltaT; /*加速度矫正量*/
//	g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].acc   =  g_sAccCorrectEarthFrame.north;								  /*加速度矫正量*/
	g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].speed += g_psTOCSystem->estimateDealt[EARTH_FRAME_Y] * K_SPD_XY * sinsHorizontalDeltaT; /*速度矫正量*/
	g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].pos   += g_psTOCSystem->estimateDealt[EARTH_FRAME_Y] * K_POS_XY * sinsHorizontalDeltaT; /*位置矫正量*/

	/*X正东*/
	g_psTOCSystem->BackCorrect[EARTH_FRAME_X].acc   += g_psTOCSystem->estimateDealt[EARTH_FRAME_X] * K_ACC_XY * sinsHorizontalDeltaT;  /*加速度矫正量*/
//	g_psTOCSystem->BackCorrect[EARTH_FRAME_X].acc   =  g_sAccCorrectEarthFrame.east;								  /*加速度矫正量*/
	g_psTOCSystem->BackCorrect[EARTH_FRAME_X].speed += g_psTOCSystem->estimateDealt[EARTH_FRAME_X] * K_SPD_XY * sinsHorizontalDeltaT;  /*速度矫正量*/
	g_psTOCSystem->BackCorrect[EARTH_FRAME_X].pos   += g_psTOCSystem->estimateDealt[EARTH_FRAME_X] * K_POS_XY * sinsHorizontalDeltaT;  /*位置矫正量*/
	
	/*1.导航系水平正北方向*/
	g_psSinsReal->curAcc[EARTH_FRAME_Y]         =  g_psSinsOrigion->curAcc[EARTH_FRAME_Y] + g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].acc; /*水平运动加速度计校正*/
	g_psTOCSystem->speedDealt[EARTH_FRAME_Y]    =  g_psSinsReal->curAcc[EARTH_FRAME_Y] * sinsHorizontalDeltaT;						  /*速度增量矫正后更新,用于更新位置*/
	g_psSinsOrigion->curPosition[EARTH_FRAME_Y] += (g_psSinsReal->curSpeed[EARTH_FRAME_Y] + 0.5f * g_psTOCSystem->speedDealt[EARTH_FRAME_Y]) * sinsHorizontalDeltaT; /*原始位置更新*/
	g_psSinsReal->curPosition[EARTH_FRAME_Y]    =  g_psSinsOrigion->curPosition[EARTH_FRAME_Y] + g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].pos; /*位置矫正后更新*/
	g_psSinsOrigion->curSpeed[EARTH_FRAME_Y]    += g_psTOCSystem->speedDealt[EARTH_FRAME_Y];												   /*原始速度更新*/
	g_psSinsReal->curSpeed[EARTH_FRAME_Y]       =  g_psSinsOrigion->curSpeed[EARTH_FRAME_Y] + g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].speed;  /*速度矫正后更新*/

	/*2.导航系水平正东方向*/
	g_psSinsReal->curAcc[EARTH_FRAME_X]         =  g_psSinsOrigion->curAcc[EARTH_FRAME_X] + g_psTOCSystem->BackCorrect[EARTH_FRAME_X].acc; /*水平运动加速度计校正*/
	g_psTOCSystem->speedDealt[EARTH_FRAME_X]    =  g_psSinsReal->curAcc[EARTH_FRAME_X] * sinsHorizontalDeltaT;						  /*速度增量矫正后更新,用于更新位置*/
	g_psSinsOrigion->curPosition[EARTH_FRAME_X] += (g_psSinsReal->curSpeed[EARTH_FRAME_X] + 0.5f * g_psTOCSystem->speedDealt[EARTH_FRAME_X]) * sinsHorizontalDeltaT; /*原始位置更新*/
	g_psSinsReal->curPosition[EARTH_FRAME_X]    =  g_psSinsOrigion->curPosition[EARTH_FRAME_X] + g_psTOCSystem->BackCorrect[EARTH_FRAME_X].pos; /*位置矫正后更新*/
	g_psSinsOrigion->curSpeed[EARTH_FRAME_X]    += g_psTOCSystem->speedDealt[EARTH_FRAME_X];												 /*原始速度更新*/
	g_psSinsReal->curSpeed[EARTH_FRAME_X]       =  g_psSinsOrigion->curSpeed[EARTH_FRAME_X] + g_psTOCSystem->BackCorrect[EARTH_FRAME_X].speed;  /*速度矫正后更新*/
	
	/*检测融合估计值和真实观测值误差*/
	if ((math_Abs(g_psSinsReal->curPosition[EARTH_FRAME_X] - g_psSinsReal->estimatePos[EARTH_FRAME_X]) > 10000) || \
		(math_Abs(g_psSinsReal->curPosition[EARTH_FRAME_Y] - g_psSinsReal->estimatePos[EARTH_FRAME_Y]) > 10000) || \
		(math_Abs(g_psSinsReal->curSpeed[EARTH_FRAME_X] - g_psAttitudeAll->GpsData.CurSpeed.east) > 10000) || \
		(math_Abs(g_psSinsReal->curSpeed[EARTH_FRAME_Y] - g_psAttitudeAll->GpsData.CurSpeed.north) > 10000))
	{
		/*标记水平融合失败*/
		g_psSinsReal->FUSION_STATUS[SINS_FUSION_HORIZONTAL] = SINS_FUSION_FAIL;
	}
	else
	{
		/*标记水平融合成功*/
		g_psSinsReal->FUSION_STATUS[SINS_FUSION_HORIZONTAL] = SINS_FUSION_SUCC;		
	}	
}

vs16 g_KalmanHorizontalPosSolidTicks = 0;
u16 g_KalmanHorizontalPosDelayCnt = 10;	 /*100ms: buff[49] -> buff[9] = 40 * 5ms???*/

/*卡尔曼滤波求水平方向上的加速度、速度、位置(X,Y水平)*/
void sins_kalman_estimate_horizontal(void)
{
	fp32 sinsHorizontalDeltaT;	
	u16 i;
	
	/*获取水平方向惯导融合时间间隔*/
	get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->SINS_Horizontal));
	
	/*判断当前程序执行间隔是否是期望的执行间隔(很大情况下并不是调度的问题)*/
	if (safe_task_execute_period_is_true(g_psSystemPeriodExecuteTime->SINS_Horizontal.DeltaTime, RTOS_WAKE_UP_GPS_HOR_FUSION_FOC_MS) == SYS_BOOL_FALSE)
	{	
		return;
	}	
	
	/*间隔时间换算成秒*/
	sinsHorizontalDeltaT = g_psSystemPeriodExecuteTime->SINS_Horizontal.DeltaTime / 1000.0f;
	
	/*GPS获取机体在导航系 相对home的水平偏移*/
	gps_Offset_Relative_To_Home();
	
	/*位置观测量赋值*/
	g_psSinsReal->estimatePos[EARTH_FRAME_Y] = g_psAttitudeAll->EarthFrameRelativeHome.north;
	g_psSinsReal->estimatePos[EARTH_FRAME_X] = g_psAttitudeAll->EarthFrameRelativeHome.east;	
	
	g_KalmanHorizontalPosSolidTicks++;
	
	/*10ms滑动一次*/
	if (g_KalmanHorizontalPosSolidTicks >= 2) /*10ms*/
	{
		g_KalmanHorizontalPosSolidTicks = 0;		
		
		/*历史数据滑动*/
		for(i = SINS_HISTORY_DATA_DEEP - 1; i > 0; i--) /*10ms滑动一次*/
        {
			/*位置*/
			g_psSinsReal->pos_History[EARTH_FRAME_X][i] = g_psSinsReal->pos_History[EARTH_FRAME_X][i - 1];   /*X正东方向*/
			g_psSinsReal->pos_History[EARTH_FRAME_Y][i] = g_psSinsReal->pos_History[EARTH_FRAME_Y][i - 1];  /*Y正北方向*/
			
			/*速度*/
			g_psSinsReal->speed_History[EARTH_FRAME_X][i] = g_psSinsReal->speed_History[EARTH_FRAME_X][i - 1];   /*X正东方向*/
			g_psSinsReal->speed_History[EARTH_FRAME_Y][i] = g_psSinsReal->speed_History[EARTH_FRAME_Y][i - 1];  /*Y正北方向*/
        }
		
		/*加入位置新数据*/
		g_psSinsReal->pos_History[EARTH_FRAME_X][0] = g_psSinsReal->curPosition[EARTH_FRAME_X];   /*X正东方向*/
		g_psSinsReal->pos_History[EARTH_FRAME_Y][0] = g_psSinsReal->curPosition[EARTH_FRAME_Y];  /*Y正北方向*/		
		
		/*加入速度新数据*/
		g_psSinsReal->speed_History[EARTH_FRAME_X][0] = g_psSinsReal->curSpeed[EARTH_FRAME_X];   /*X正东方向*/
		g_psSinsReal->speed_History[EARTH_FRAME_Y][0] = g_psSinsReal->curSpeed[EARTH_FRAME_Y];  /*Y正北方向*/				
	}
	
	/*加速度观测量赋值*/
	g_psSinsReal->curAcc[EARTH_FRAME_X] = g_psSinsOrigion->curAcc[EARTH_FRAME_X];
	g_psSinsReal->curAcc[EARTH_FRAME_Y] = g_psSinsOrigion->curAcc[EARTH_FRAME_Y];
	
	/*GPS 原始数据是否可用,且可以开始融合*/
	if ((g_sUav_Status.UavSenmodStatus.Horizontal.Gps.DATA_STATUS == UAV_SENMOD_DATA_OK) && \
		(g_sUav_Status.UavSenmodStatus.Horizontal.Gps.FUSION_STATUS == UAV_SENMOD_FUSION_START)) /*低频更新传感器观测*/
	{
		/*水平E向(正东) 卡尔曼估计*/
		filter_Kalman_Estimate_GPS_Horizontal(g_psSinsReal->estimatePos[EARTH_FRAME_X],   /*位置观测量*/
											  g_psAttitudeAll->GpsData.CurSpeed.east, /*速度观测量*/
											  g_psAttitudeAll->GpsData.quality,			  /*GPS定位质量*/
											  g_KalmanHorizontalPosDelayCnt, 	   		  /*位置观测传感器延时*/
											  g_psSinsReal, 	   						  /*惯导结构体*/
											  g_psSinsOrigion->curAcc[EARTH_FRAME_X],     /*系统原始驱动量*/
											  &g_sFilter_Kalman_GPS_Horizontal,
											  EARTH_FRAME_X,
											  sinsHorizontalDeltaT);
	
		/*水平N向(正北) 卡尔曼估计*/
		filter_Kalman_Estimate_GPS_Horizontal(g_psSinsReal->estimatePos[EARTH_FRAME_Y],   /*位置观测量*/
											  g_psAttitudeAll->GpsData.CurSpeed.north, /*速度观测量*/
											  g_psAttitudeAll->GpsData.quality,			  /*GPS定位质量*/
											  g_KalmanHorizontalPosDelayCnt, 	   		  /*位置观测传感器延时*/
											  g_psSinsReal, 	   						  /*惯导结构体*/
											  g_psSinsOrigion->curAcc[EARTH_FRAME_Y],     /*系统原始驱动量*/		
											  &g_sFilter_Kalman_GPS_Horizontal,
											  EARTH_FRAME_Y,
											  sinsHorizontalDeltaT);
						
		/*标记不可卡尔曼融合*/
		g_sUav_Status.UavSenmodStatus.Horizontal.Gps.FUSION_STATUS = UAV_SENMOD_FUSION_FINISH;
	}
	else /*高频更新SINS*/
	{
		g_psSinsReal->curPosition[EARTH_FRAME_X] += g_psSinsReal->curSpeed[EARTH_FRAME_X] * sinsHorizontalDeltaT + \
												    ((g_psSinsReal->curAcc[EARTH_FRAME_X] * sinsHorizontalDeltaT * sinsHorizontalDeltaT)) / 2.0f;
		
		g_psSinsReal->curSpeed[EARTH_FRAME_X] += (g_psSinsReal->curAcc[EARTH_FRAME_X]) * sinsHorizontalDeltaT;
		
		g_psSinsReal->curPosition[EARTH_FRAME_Y] += g_psSinsReal->curSpeed[EARTH_FRAME_Y] * sinsHorizontalDeltaT + \
												    ((g_psSinsReal->curAcc[EARTH_FRAME_Y] * sinsHorizontalDeltaT * sinsHorizontalDeltaT)) / 2.0f;
		
		g_psSinsReal->curSpeed[EARTH_FRAME_Y] += (g_psSinsReal->curAcc[EARTH_FRAME_Y]) * sinsHorizontalDeltaT;		
	}
	
	/*检测融合估计值和真实观测值误差*/
	if ((math_Abs(g_psSinsReal->curPosition[EARTH_FRAME_X] - g_psSinsReal->estimatePos[EARTH_FRAME_X]) > 10000) || \
		(math_Abs(g_psSinsReal->curPosition[EARTH_FRAME_Y] - g_psSinsReal->estimatePos[EARTH_FRAME_Y]) > 10000) || \
		(math_Abs(g_psSinsReal->curSpeed[EARTH_FRAME_X] - g_psAttitudeAll->GpsData.CurSpeed.east) > 10000) || \
		(math_Abs(g_psSinsReal->curSpeed[EARTH_FRAME_Y] - g_psAttitudeAll->GpsData.CurSpeed.north) > 10000))
	{
		/*标记水平融合失败*/
		g_psSinsReal->FUSION_STATUS[SINS_FUSION_HORIZONTAL] = SINS_FUSION_FAIL;
	}
	else
	{
		/*标记水平融合成功*/
		g_psSinsReal->FUSION_STATUS[SINS_FUSION_HORIZONTAL] = SINS_FUSION_SUCC;		
	}	
}


/*捷联惯导对象重置:轴,位置,速度*/
void strapdown_ins_reset(SINS *sins, TOCSystem *tocSystem, EARTH_FRAME_AXIS AXIS, fp32 posTarg, fp32 speedTarg)
{
	u8 i;
	
	sins->curPosition[AXIS] = posTarg;	 /*位置重置*/
	sins->curSpeed[AXIS]    = speedTarg; /*速度重置*/
	sins->curAcc[AXIS]      = 0.0f;		 /*加速度清零*/
	sins->accOffset[AXIS]   = 0.0f;		 /*惯导加速度漂移估计量清零*/
	
	/*历史位置值,全部装载为当前观测值*/
	for (i = SINS_HISTORY_DATA_DEEP - 1; i > 0; i--)
	{
		sins->pos_History[AXIS][i] = posTarg;
	}
	
	sins->pos_History[AXIS][0] = posTarg;

	/*历史速度值,全部装载为当前观测值*/
	for (i = SINS_HISTORY_DATA_DEEP - 1; i > 0; i--)
	{
		sins->speed_History[AXIS][i] = speedTarg;
	}
	
	sins->speed_History[AXIS][0] = speedTarg;	
	
	/*清空惯导融合反馈修正量*/
	tocSystem->BackCorrect[AXIS].acc   = 0; /*加速度*/
	tocSystem->BackCorrect[AXIS].speed = 0; /*速度*/
	tocSystem->BackCorrect[AXIS].pos   = 0;	/*位置*/
}
