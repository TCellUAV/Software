#include "attitude_Aircraft.h"
#include "filter_DataProcess.h"
#include "sins_Strapdown.h"
#include "earth_Declination.h"

AttitudeAll g_sAttitudeAll = {0};

AttitudeAll *g_psAttitudeAll = &g_sAttitudeAll;

/*====== GPS数据和惯导关系 ======*/
/*gps控制数据获取*/
void gps_fix_position_data_get(GpsM8nPvtData pvtData, GPS_Data *gpsData)
{
	static vu16 sampleContinueTicks = 0;
	fp32 deltaT;
	
	/*获取精确的间隔时间*/
	get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->GpsCtrlData));
		
	/*更新时间间隔(s)*/
	deltaT = g_psSystemPeriodExecuteTime->GpsCtrlData.DeltaTime / 1000.0f;
	
	/*GPS数据更新失败*/
	if (g_sGpsM8N.UPDATE_STATUS != GPS_DATA_UPDATE_SUCC)
	{
		/*标记本次GPS观测水平为无效值*/
		g_psUav_Status->UavSenmodStatus.Horizontal.Gps.DATA_STATUS = UAV_SENMOD_DATA_NO;
		
		return;
	}
	
	/*Utc*/
	memcpy(&(gpsData->LocalTime), &pvtData.UtcTime, sizeof(GPS_Time));	/*内容完全对应*/
	gpsData->LocalTime.hour = pvtData.UtcTime.hour + 8;			  	    /*北京时间: UTC+8*/
	
	/*定位类型*/
	gpsData->POS_FIX_TYPE = pvtData.POS_FIX_TYPE;
	
	/*定位卫星个数*/
	gpsData->satelliteNbr = pvtData.satelliteNbr;
	
	/*经&纬度*10^7*/
	gpsData->Coordinate_s4.lon = pvtData.Coordinate.lon;
	gpsData->Coordinate_s4.lat = pvtData.Coordinate.lat;
	
	/*经&纬度真实精确值:deg*/
	gpsData->Coordinate_f8.lon = pvtData.Coordinate.lon * 0.0000001f;
	gpsData->Coordinate_f8.lat = pvtData.Coordinate.lat * 0.0000001f;
	
	/*GPS海拔高度*/
	gpsData->hMSL *= 0.1f;	   /*cm*/
	
	/*水平位置估计精度*/
	gpsData->HV_Accuracy.hAcc = pvtData.HV_Accuracy.hAcc * 0.01f; /*m*/
	
	/*垂直位置估计精度*/
	gpsData->HV_Accuracy.vAcc = pvtData.HV_Accuracy.vAcc * 0.01f; /*m*/
	
	/*GPS获取的沿导航系正北速度(Y Axis)*/
	gpsData->NED_Velocity.velN = pvtData.NED_Velocity.velN * 0.1f; /*cm/s*/
	
	/*GPS获取的沿导航系正东向速度(X Axis)*/
	gpsData->NED_Velocity.velE = pvtData.NED_Velocity.velE * 0.1f; /*cm/s*/
	
	/*GPS获取的沿导航系地向速度(Z Axis)*/
	gpsData->NED_Velocity.velD = pvtData.NED_Velocity.velD * 0.1f; /*cm/s*/
	
	/*机体对地速度*/
	gpsData->gSpeed = pvtData.gSpeed * 0.1f; /*cm/s*/
	
	/*机体运动航向角*/
	gpsData->headMot = pvtData.headMot * 0.00001f; /*deg*/
	
	/*速度估计精度*/
	gpsData->sAcc = pvtData.sAcc * 0.1f; /*cm/s*/
	
	/*位置估计精度*/
	gpsData->quality = pvtData.pDOP * 0.01f;
	
	
	/*记录上次速度*/
	gpsData->LastSpeed.east  = gpsData->CurSpeed.east;
	gpsData->LastSpeed.north = gpsData->CurSpeed.north;
	gpsData->LastSpeed.up    = gpsData->CurSpeed.up;
	
	/*记录本次速度*/	
	gpsData->CurSpeed.east  = gpsData->NED_Velocity.velE;
	gpsData->CurSpeed.north = gpsData->NED_Velocity.velN;
	gpsData->CurSpeed.up    = -gpsData->NED_Velocity.velD; /*down -> up*/
	
	/*速度增量*/
	gpsData->DeltaSpeed.east  = (gpsData->CurSpeed.east - gpsData->LastSpeed.east) / deltaT;	 /*单位cm/s^2*/
	gpsData->DeltaSpeed.north = (gpsData->CurSpeed.north - gpsData->LastSpeed.north) / deltaT;   /*单位cm/s^2*/
	gpsData->DeltaSpeed.up    = (gpsData->CurSpeed.up - gpsData->LastSpeed.up) / deltaT;		 /*单位cm/s^2*/	
	
	/*判断卫星信号是否满足定位条件*/
	if ((gpsData->satelliteNbr >= 9) && \
		(gpsData->quality <= 3.0f))
	{
		/*标记GPS数据可用*/
		g_psUav_Status->UavSenmodStatus.Horizontal.Gps.DATA_STATUS = UAV_SENMOD_DATA_OK;
		
		/*标记可以开始融合*/
		g_psUav_Status->UavSenmodStatus.Horizontal.Gps.FUSION_STATUS = UAV_SENMOD_FUSION_START;
	}
	else
	{
		/*标记GPS数据不可用*/		
		g_psUav_Status->UavSenmodStatus.Horizontal.Gps.DATA_STATUS = UAV_SENMOD_DATA_NO;	
	}
	
	/*数据有效,且第一次使用状态无效*/
	if ((g_psUav_Status->UavSenmodStatus.Horizontal.Gps.DATA_STATUS == UAV_SENMOD_DATA_OK) && \
		(g_psUav_Status->UavSenmodStatus.Horizontal.Gps.FIRST_USE_AVA_STATUS != UAV_SENMOD_FIRST_USE_AVA_OK))
	{
		/*多次有效,等待GPS信号稳定*/
		if (sampleContinueTicks > 100)
		{
			g_psUav_Status->UavSenmodStatus.Horizontal.Gps.FIRST_USE_AVA_STATUS = UAV_SENMOD_FIRST_USE_AVA_OK;
		}
		else
		{
			sampleContinueTicks++;
		}
	}
	
	/*本次数据已用,重新标记为未更新*/
	g_sGpsM8N.UPDATE_STATUS = GPS_DATA_UPDATE_FAIL;
}

/*GPS Home点设置*/
#if defined(HW_CUT__USE_GPS)

void gps_home_location_set(void)
{	
	/*home点只设置一次,且卫星信号持续100次检测为可用时才允许使用卫星数据设定HOME点*/
	if ((g_sUav_Status.HOME_SET_STATUS != UAV_HOME_SET_YES) && \
		(g_sUav_Status.UavSenmodStatus.Horizontal.Gps.FIRST_USE_AVA_STATUS == UAV_SENMOD_FIRST_USE_AVA_OK))
	{
		g_psAttitudeAll->HomePos.Coordinate_s4.lat = g_psAttitudeAll->GpsData.Coordinate_s4.lat;	/*纬度^7*/
		g_psAttitudeAll->HomePos.Coordinate_s4.lon = g_psAttitudeAll->GpsData.Coordinate_s4.lon;   	/*经度^7*/

		g_psAttitudeAll->HomePos.Coordinate_f8.lat = g_psAttitudeAll->GpsData.Coordinate_f8.lat;	/*纬度*/
		g_psAttitudeAll->HomePos.Coordinate_f8.lon = g_psAttitudeAll->GpsData.Coordinate_f8.lon;   	/*经度*/		
		
		/*标记已设定HOME点*/
		g_sUav_Status.HOME_SET_STATUS = UAV_HOME_SET_YES;
		
		/*复位惯导融合:导航系X轴(正东)*/
		strapdown_ins_reset(&g_sSinsReal, &g_sTOCSystem, EARTH_FRAME_X, g_psAttitudeAll->EarthFrameRelativeHome.east, 0);

		/*复位惯导融合:导航系Y轴(正北)*/
		strapdown_ins_reset(&g_sSinsReal, &g_sTOCSystem, EARTH_FRAME_Y, g_psAttitudeAll->EarthFrameRelativeHome.north, 0);			
		
		/*获取GPS定位点的地磁偏角*/
		g_psAttitudeAll->declination = get_earth_local_declination(g_psAttitudeAll->HomePos.Coordinate_f8.lat, \
														   	       g_psAttitudeAll->HomePos.Coordinate_f8.lon);
	}	
}

/*获取GPS HOME点设置状态*/
UAV_HOME_SET_STATUS get_gps_home_set_status(Uav_Status *uavStatus)
{
	return (uavStatus->HOME_SET_STATUS);
}
#endif

/*两点间的二维距离*/
Vector2f_Nav gps_Two_Pos_XY_Offset(GPS_Coordinate_s4 loc1, GPS_Coordinate_s4 loc2)
{
	Vector2f_Nav twoPosDelta;

	twoPosDelta.north = (loc2.lat - loc1.lat) * GPS_LOCATION_SCALING_FACTOR; 							     /*正北距离 m*/
	twoPosDelta.east  = (loc2.lon - loc1.lon) * GPS_LOCATION_SCALING_FACTOR * gps_Longitude_Scale(loc2);   /*正东距离 m*/	
	return twoPosDelta;
}

/*经度比例*/
fp32 gps_Longitude_Scale(GPS_Coordinate_s4 loc)
{
	static s32 lastLat = 0;
	static fp32 scale = 1.0f;
	
	/*比较两次纬度相差值，避免重复运算余弦函数*/
	if (math_Abs(lastLat - loc.lat) < 100000)
	{
		// we are within 0.01 degrees (about 1km) of the
		// same latitude. We can avoid the cos() and return
		// the same scale factor.
        return scale;		
	}

    scale = cosf(loc.lat * 1.0e-7f * DEG2RAD);
    scale = math_Constrain(scale, 1.0f, 0.01f); 
    lastLat = loc.lat;
    return scale;	
}

/*两点间的直线距离*/
fp32 gps_Two_Pos_Segment_Distance(GPS_Coordinate_s4 loc1, GPS_Coordinate_s4 loc2)
{
	fp32 disLat = (fp32)(loc2.lat - loc1.lat);
	fp32 disLon = (fp32)(loc2.lon - loc1.lon) * gps_Longitude_Scale(loc2); /*m*/
	
	/*返回直线距离*/
	return (pythagorous2(disLat, disLon) * GPS_LOCATION_SCALING_FACTOR);
}

/*获取机体相对home的水平偏移*/
#if defined(HW_CUT__USE_GPS)
/*导航(地理)坐标系，正北+Lat(纬度)+Y、正东+Lon(经度)+X 方向位置偏移*/
/*机体(载体)坐标系，机体横滚+x正+roll、机体俯仰+(y正)+pitch 方向位置偏移*/
void gps_Offset_Relative_To_Home(void)
{
	Vector2f_Nav locationDelta = {0};
	
	/*根据当前GPS定位信息与Home点位置信息计算[导航座标系]正北、正东方向位置偏移*/
	locationDelta = gps_Two_Pos_XY_Offset(g_psAttitudeAll->HomePos.Coordinate_s4, g_psAttitudeAll->GpsData.Coordinate_s4);
   /***********************************************************************************
   明确下导航系方向，这里正北、正东为正方向:
   沿着正北，纬度增加,当无人机相对home点，往正北向移动时，此时 g_psAttitudeAll->GpsData.Coordinate_s4.lat  > g_psAttitudeAll->HomePos.Coordinate_s4.lat, 
   得到的locationDelta.x大于0;	
	
   沿着正东，经度增加,当无人机相对home点，往正东向移动时，此时 g_psAttitudeAll->GpsData.Coordinate_s4.lon > g_psAttitudeAll->HomePos.Coordinate_s4.lon,
   得到的locationDelta.y大于0;
   ******************************************************************************/	
	
   locationDelta.north *= 100;    /*沿地理坐标系，正北(lat,y)方向位置偏移,单位为CM*/
   locationDelta.east  *= 100;    /*沿地理坐标系，正东(lon,x)方向位置偏移,单位为CM*/

   g_psAttitudeAll->EarthFrameRelativeHome.north = locationDelta.north; /*地理系下相对Home点正北位置偏移,CM*/	
   g_psAttitudeAll->EarthFrameRelativeHome.east  = locationDelta.east; /*地理系下相对Home点正东位置偏移,CM*/

   /*将无人机在导航坐标系下的沿着正北、正东方向的位置偏移旋转到当前航向的位置偏移:机头(俯仰)+横滚*/
   g_psAttitudeAll->BodyFrameRelativeHome.x = locationDelta.east * COS_YAW + locationDelta.north * SIN_YAW;   /*横滚正向位置偏移,X轴正向*/	
   g_psAttitudeAll->BodyFrameRelativeHome.y = -locationDelta.east * SIN_YAW + locationDelta.north * COS_YAW;  /*机头正向位置偏移,Y轴正向*/     
}
#endif

/*====== OpticFlow数据 ======*/
/*对光流数据进行处理,计算出需要的速度,位移信息*/
#if defined(HW_CUT__USE_OPTICFLOW)

void opflow_Offset_Relative_To_Home(OpFlowUpixelsLC306DataFrame OpFlowData, fp32 sinsHeight_cm, AttitudeAll *attitudeAll)
{	
	/*判断光流是否更新成功,且数据可用*/
	if ((g_sOpFlowUpixelsLC306.UPDATE_STATUS != OPFLOW_UPIXELSLC306_UPDATE_SUCC) || \
		(OpFlowData.DATA_STATUS != OPFLOW_UPIXELSLC306_DATA_AVA))
	{
		/*标记本次GPS观测水平为无效值*/
		g_psUav_Status->UavSenmodStatus.Horizontal.Opticflow.DATA_STATUS = UAV_SENMOD_DATA_NO;
		
		return;
	}
	
	/*光流未开发,标记不可用*/
	/*标记本次光流观测水平为无效值*/
	g_psUav_Status->UavSenmodStatus.Horizontal.Opticflow.DATA_STATUS = UAV_SENMOD_DATA_NO;
	
//	/*光流累计像素点低通滤波*/
//	attitudeAll->OpticFlowData.IntPixLPF.x = filter_OpFlowIntPixLpButterworth_Dp(OpFlowData.xIntegral, \
//																					 &(g_sFilterTarg.OpticFlowIntPixLpBwBuff[0]), \
//																					 &(g_sFilterTarg.OpticFlowIntPixLpBwPara[0])); /*x,50hz,20hz*/
//	
//	attitudeAll->OpticFlowData.IntPixLPF.y = filter_OpFlowIntPixLpButterworth_Dp(OpFlowData.yIntegral, \
//																					 &(g_sFilterTarg.OpticFlowIntPixLpBwBuff[0]), \
//																					 &(g_sFilterTarg.OpticFlowIntPixLpBwPara[1])); /*y,50hz,20hz*/
//	
//	/*角度补偿位移*/
//	attitudeAll->OpticFlowData.AngleCompensate.x += (600.0f * tanf(attitudeAll->Ahrs.roll * DEG2RAD) - \
//												         attitudeAll->OpticFlowData.AngleCompensate.x) * 0.2f;
//	
//	attitudeAll->OpticFlowData.AngleCompensate.y += (600.0f * tanf(attitudeAll->Ahrs.pitch * DEG2RAD) - \
//												         attitudeAll->OpticFlowData.AngleCompensate.y) * 0.2f;	
//	
//	/*像素点真实输出*/
//	attitudeAll->OpticFlowData.CurRawPosition.x = attitudeAll->OpticFlowData.IntPixLPF.x - \
//													  attitudeAll->OpticFlowData.AngleCompensate.x;
//	
//	attitudeAll->OpticFlowData.CurRawPosition.y = attitudeAll->OpticFlowData.IntPixLPF.y - \
//													  attitudeAll->OpticFlowData.AngleCompensate.y;
//													  
//	/*光流角速度 rad/s*/
//	attitudeAll->OpticFlowData.GyroSpeed.x = attitudeAll->OpticFlowData.IntPixLPF.x / 200.0f;
//	attitudeAll->OpticFlowData.GyroSpeed.y = attitudeAll->OpticFlowData.IntPixLPF.y / 200.0f;													  
//													  
//	/*高度锁定*/
//	if (sinsHeight_cm < 200)
//	{
//		sinsHeight_cm = 100;
//	}
//	else if (sinsHeight_cm < 300)
//	{
//		sinsHeight_cm = 150;		
//	}
//	else if (sinsHeight_cm < 400)
//	{
//		sinsHeight_cm = 200;		
//	}
//	else if (sinsHeight_cm < 500)
//	{
//		sinsHeight_cm = 250;		
//	}
//	else if (sinsHeight_cm < 600)
//	{
//		sinsHeight_cm = 300;		
//	}	
//	else if (sinsHeight_cm < 1000)
//	{
//		sinsHeight_cm = 350;		
//	}
//	else	
//	{
//		sinsHeight_cm = 400;			
//	}													  
//	
//	/*计算两帧像素相对真实位移*/
//	attitudeAll->OpticFlowData.DealtRealPosition.x = (attitudeAll->OpticFlowData.CurRawPosition.x - \
//														  attitudeAll->OpticFlowData.LastRawPosition.x) * \
//														  sinsHeight_cm * 10 / 10000.0f; /*像素点转mm*/
//	
//	attitudeAll->OpticFlowData.LastRawPosition.x = attitudeAll->OpticFlowData.CurRawPosition.x;
//	
//	attitudeAll->OpticFlowData.DealtRealPosition.y = (attitudeAll->OpticFlowData.CurRawPosition.y - \
//														  attitudeAll->OpticFlowData.LastRawPosition.y) * \
//														  sinsHeight_cm * 10 / 10000.0f; /*像素点转mm*/
//	
//	attitudeAll->OpticFlowData.LastRawPosition.y = attitudeAll->OpticFlowData.CurRawPosition.y;	
//	
//	/*微分水平速度 m/s*/
//	attitudeAll->OpticFlowData.DiffSpeed.x = (attitudeAll->OpticFlowData.DealtRealPosition.x * 50.0f) / 10.0f; /*cm/s*/
//	attitudeAll->OpticFlowData.DiffSpeed.y = (attitudeAll->OpticFlowData.DealtRealPosition.y * 50.0f) / 10.0f; /*cm/s*/
//	
//	/*微分水平速度低通滤波*/
//	attitudeAll->OpticFlowData.DiffSpeedLPF.x += (attitudeAll->OpticFlowData.DiffSpeed.x - attitudeAll->OpticFlowData.DiffSpeedLPF.x) * 0.08f;
//	attitudeAll->OpticFlowData.DiffSpeedLPF.y += (attitudeAll->OpticFlowData.DiffSpeed.y - attitudeAll->OpticFlowData.DiffSpeedLPF.y) * 0.08f;	
//	
//	/*速度 cm/s*/
//	attitudeAll->OpticFlowData.RealSpeed.x = attitudeAll->OpticFlowData.DiffSpeedLPF.x;
//	attitudeAll->OpticFlowData.RealSpeed.y = attitudeAll->OpticFlowData.DiffSpeedLPF.y;
//	
//	/*累积位移 cm*/
//	attitudeAll->OpticFlowData.RealPosition.x += attitudeAll->OpticFlowData.DealtRealPosition.x / 10.0f; /*cm*/
//	attitudeAll->OpticFlowData.RealPosition.y += attitudeAll->OpticFlowData.DealtRealPosition.y / 10.0f; /*cm*/
	
	/*本次数据已使用,标记数据为未更新状态*/
	g_sOpFlowUpixelsLC306.UPDATE_STATUS = OPFLOW_UPIXELSLC306_UPDATE_FAIL;
}
#endif

/*====== Bero和Ultr高度数据获取及处理(校准、滤波) ======*/
/*获取气压计相对观测高度*/
s32 baro_get_relative_altitude(fp32 currentPa, fp32 referencePa)
{
	fp32 A = 0, altM = 0;
	s32 alt_cm = 0;
	
	if (referencePa != 0)	/*初始化为0,违背除法法则*/
	{
		A = powf((currentPa / referencePa), 1 / 5.257f);
		altM = ((1 - A) * 6357000.0f) / (A + 161.1035f);	
	}

	alt_cm = (s32)(altM * 100.0f);	/*m -> cm*/
	
	return (alt_cm);
}

/*Bero Altitude数据获取和处理 */
#if defined(HW_CUT__USE_MD_BERO)

vu16 g_BeroUpdateContinueTicks     = 0;
vu16 g_BeroZeroSampleContinueTicks = 0;

void bero_altitude_data_get_and_dp(Uav_Status *uavStatus)
{
	fp32 beroDeltaT;
	
	g_BeroUpdateContinueTicks++; /*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS 执行一次*/
	
	/*判断是否满足气压计温度更新时间*/
	if (g_BeroUpdateContinueTicks == 1)	
	{		
		bsp_SPL06_Get_Temperature(&g_sSpl06);
	}
	/*判断是否满足气压计高度更新时间*/
	else if (g_BeroUpdateContinueTicks >= (SYS_BERO_MIN_MEAS_PERIOD_TICK_MS / PLATFORM_TASK_SCHEDULER_MIN_FOC_MS))	/*110ms获取1次*/
	{
		/*cntTicks清0*/
		g_BeroUpdateContinueTicks = 0;
		
		/*获取气压计当前气压值*/
		bsp_SPL06_Get_Pressure(&g_sSpl06);
		
		/*判断本次气压值是否基本符合实际(与海平面气压值比较,正常情况是小于等于海平面气压值)*/
		if (g_sSpl06.Pressure <= SEA_LEVEL_PRESSURE)
		{
			/*判断是否已设定气压计零参考点气压和参考点高度*/
			if (uavStatus->UavSenmodStatus.Vertical.Bero.ZERO_REFERENCE_SET_STATUS != UAV_SENMOD_ZERO_REFERENCE_SET_OK)				
			{	
				/*采集50次后,待气压数据稳定后(必要步骤),设定初始位置气压值*/
				if (g_BeroZeroSampleContinueTicks > 50)
				{
					/*设定参考点的气压值*/
					g_psAttitudeAll->BaroData.zeroPressure = g_sSpl06.Pressure;
				
					/*标记气压计零参考点已设置且正确*/
					uavStatus->UavSenmodStatus.Vertical.Bero.ZERO_REFERENCE_SET_STATUS = UAV_SENMOD_ZERO_REFERENCE_SET_OK;
					
					/*根据气压海拔关系,计算得到相对高度*/
					g_psAttitudeAll->BaroData.curAltitude = baro_get_relative_altitude(g_psAttitudeAll->BaroData.zeroPressure, g_psAttitudeAll->BaroData.zeroPressure);
					
					/*记录零参考点气压计高度值*/
					g_psAttitudeAll->BaroData.zeroHeight = g_psAttitudeAll->BaroData.curAltitude;					
					
					/*复位惯导融合:Z轴(天)*/					
					strapdown_ins_reset(&g_sSinsReal, &g_sTOCSystem, EARTH_FRAME_Z, g_psAttitudeAll->BaroData.curAltitude, 0);
				}
				else
				{
					g_BeroZeroSampleContinueTicks++; 
				}
			}
		
			/*初始位置气压值有效,才标记竖直方向惯导数据有效*/
			if (uavStatus->UavSenmodStatus.Vertical.Bero.ZERO_REFERENCE_SET_STATUS == UAV_SENMOD_ZERO_REFERENCE_SET_OK)
			{	
				/*标记本次气压值为有效值*/
				uavStatus->UavSenmodStatus.Vertical.Bero.DATA_STATUS = UAV_SENMOD_DATA_OK;
				
				/*获取精确的间隔时间*/
				get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->BeroAboveAltitude));
	
				beroDeltaT = (g_psSystemPeriodExecuteTime->BeroAboveAltitude.DeltaTime) / 1000.0f; /*ms换算成s*/			

				/*原始气压值*/
				g_psAttitudeAll->BaroData.rawPressure = g_sSpl06.Pressure;			

				/*Bero Altitude: 2rd lpButterWorth FS:9HZ, FC:3HZ (气压计气压原始值巴特沃斯低通滤波)*/
				g_psAttitudeAll->BaroData.filterPressure = filter_BaroAltitudeLpButterworth_Dp(g_psAttitudeAll->BaroData.rawPressure, &(g_sFilterTarg.BaroAboveLpBwBuff[0]), \
																				               &(g_sFilterTarg.BaroAboveLpBwPara[FILTER_LPBW_BARO_9HZ_3HZ_IDX])); 				
				
				/*根据气压海拔关系,计算得到原始高度*/
				g_psAttitudeAll->BaroData.rawAltitude = baro_get_relative_altitude(g_psAttitudeAll->BaroData.filterPressure, g_psAttitudeAll->BaroData.zeroPressure);
			
				/*判断当前是否是锁定状态,是则让观测高度一直为0*/
				if (uavStatus->LOCK_STATUS == UAV_LOCK_YES)
				{
					/*每次锁定重置偏移量*/
					g_psAttitudeAll->BaroData.obAltitudeOffset = 0 - g_psAttitudeAll->BaroData.rawAltitude;
				}
				
				/*当前观测高度加上偏移量*/
				g_psAttitudeAll->BaroData.curAltitude = g_psAttitudeAll->BaroData.rawAltitude + \
														g_psAttitudeAll->BaroData.obAltitudeOffset;				
				
				/*计算气压计数据计算出的Z轴垂直向上方向上的速度(cm/s)*/
				g_psAttitudeAll->BaroData.climbSpeed = (g_psAttitudeAll->BaroData.curAltitude - \
									                    g_psAttitudeAll->BaroData.lastAltitude) / beroDeltaT;
			
				/*本次观测高度,作为下次计算的上次观测高度*/
				g_psAttitudeAll->BaroData.lastAltitude = g_psAttitudeAll->BaroData.curAltitude;
			}
		}
		else /*数据不合法,本次采样无效*/
		{
			/*标记本次气压值为无效值*/
			uavStatus->UavSenmodStatus.Vertical.Bero.DATA_STATUS = UAV_SENMOD_DATA_NO;			
			
			/*海拔高度设定为无效值*/
			g_psAttitudeAll->BaroData.curAltitude = SYS_NO_AVA_MARK;
		}
	}
}
#endif

/*获取气压计观测数据状态*/
UAV_SENMOD_DATA_STATUS get_bero_estimate_data_status(Uav_Status *uavStatus)
{
	return (uavStatus->UavSenmodStatus.Vertical.Bero.DATA_STATUS);
}

/*ultr Altitude数据获取和处理 */
#if defined(HW_CUT__USE_ULTR)

vu16 g_UltrUpdateContinueTicks     = 0;
vu16 g_UltrZeroSampleContinueTicks = 0;

void ultr_altitude_data_get_and_dp(Uav_Status *uavStatus)
{
	fp32 ultrDeltaT;
	
	g_UltrUpdateContinueTicks++; /*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS 执行一次*/

	if (g_UltrUpdateContinueTicks >= (SYS_ULTR_MIN_MEAS_PERIOD_TICK_MS / PLATFORM_TASK_SCHEDULER_MIN_FOC_MS))	/*5ms执行一次,100ms获取1次*/
	{	
		/*cntTicks清0*/
		g_UltrUpdateContinueTicks = 0;
		
		/*获取超声波当前观测值,并开启下次测量值*/
		g_psAttitudeAll->UltrData.rawAltitude = bsp_US100_Get_Distance(&g_sUs100);
		
		/*判断测距值是否为有效值*/
		if (g_psAttitudeAll->UltrData.rawAltitude != SYS_NO_AVA_MARK)
		{
			/*判断是否已设定超声波零参考点高度*/
			if (uavStatus->UavSenmodStatus.Vertical.Ultr.ZERO_REFERENCE_SET_STATUS != UAV_SENMOD_ZERO_REFERENCE_SET_OK)				
			{
				/*采集10次后,待超声波数据稳定后(必要步骤),设定初始位置超声波高度值*/
				if (g_UltrZeroSampleContinueTicks > 50)
				{
					/*设定超声波零参考点的高度*/
					g_psAttitudeAll->UltrData.zeroHeight = g_psAttitudeAll->UltrData.rawAltitude;
					
					/*标记超声波零参考点已设置且正确*/
					uavStatus->UavSenmodStatus.Vertical.Ultr.ZERO_REFERENCE_SET_STATUS = UAV_SENMOD_ZERO_REFERENCE_SET_OK;
				}
				else
				{
					g_UltrZeroSampleContinueTicks++;
				}
			}
			
			/*初始位置超声波值有效,才标记竖直方向惯导数据有效*/
			if (uavStatus->UavSenmodStatus.Vertical.Ultr.ZERO_REFERENCE_SET_STATUS == UAV_SENMOD_ZERO_REFERENCE_SET_OK)
			{
				/*距离在有效高度范围内*/
				if ((0 < g_psAttitudeAll->UltrData.rawAltitude) && (g_psAttitudeAll->UltrData.rawAltitude <= SYS_ULTR_MAX_MEAS_DISTANCE))
				{
					/*标记本次超声波观测高度为有效值*/
					uavStatus->UavSenmodStatus.Vertical.Ultr.DATA_STATUS = UAV_SENMOD_DATA_OK;

					/*获取精确的间隔时间*/
					get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->UltrAltitude));
	
					ultrDeltaT = (g_psSystemPeriodExecuteTime->UltrAltitude.DeltaTime) / 1000.0f; /*ms换算成s*/

					/*超声波数据 滑动窗口滤波*/			
					g_psAttitudeAll->UltrData.filterAltitude = filter_Slider_Average_Dp(&(g_sFilterTarg.UltrSliderAverage), g_psAttitudeAll->UltrData.rawAltitude);

					g_psAttitudeAll->UltrData.curAltitude = g_psAttitudeAll->UltrData.filterAltitude;
					
					/*计算超声波数据计算出的Z轴垂直向上方向上的速度(cm/s)*/
					g_psAttitudeAll->UltrData.climbSpeed = (g_psAttitudeAll->UltrData.curAltitude - \
													        g_psAttitudeAll->UltrData.lastAltitude) / ultrDeltaT;
			
					/*本次观测高度,作为下次计算的上次观测高度*/
					g_psAttitudeAll->UltrData.lastAltitude = g_psAttitudeAll->UltrData.curAltitude;
				}
				else /*不在有效高度范围内*/
				{
					/*标记本次超声波观测高度为无效值*/
					uavStatus->UavSenmodStatus.Vertical.Ultr.DATA_STATUS = UAV_SENMOD_DATA_NO;						
				}
			}				
		}
		else
		{
			/*标记本次超声波观测高度为无效值*/
			uavStatus->UavSenmodStatus.Vertical.Ultr.DATA_STATUS = UAV_SENMOD_DATA_NO;
		}
	}
}
#endif

/*获取超声波观测数据状态*/
UAV_SENMOD_DATA_STATUS get_ultr_estimate_data_status(Uav_Status *uavStatus)
{
	return (uavStatus->UavSenmodStatus.Vertical.Ultr.DATA_STATUS);
}
