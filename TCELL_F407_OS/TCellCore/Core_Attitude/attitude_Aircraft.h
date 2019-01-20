#ifndef _ATTITUDE_AIRCRAFT_H_
#define _ATTITUDE_AIRCRAFT_H_

#include "sys_Platform.h"
#include "bsp_BoardLib.h"
#include "ahrs_Caculation.h"

/*万向节三轴角度*/
typedef struct
{
	fp32 pitch;
	fp32 roll;
	fp32 yaw;
}GimbalAngle;

/*GPS数据*/
typedef struct
{
	GPS_Time 				LocalTime;		/*本地时间*/
	GPS_M8N_POS_FIX_TYPE    POS_FIX_TYPE;	/*定位模式*/
	u8 					    satelliteNbr;	/*卫星个数*/
	GPS_Coordinate_s4		Coordinate_s4;	/*纬度 & 纬度*/
	GPS_Coordinate_f8		Coordinate_f8;	/*纬度 & 纬度*/
	fp32					hMSL;			/*海拔高度*/	
	GPS_HV_Accuracy_f4		HV_Accuracy;	/*水平&垂直位置估计精度*/	
	GPS_NED_Velocity_f4		NED_Velocity;	/*导航系正东(E,x)/正北(N,y)/正天(U)向速度*/	
	fp32 					gSpeed;			/*机体对地速度*/
	fp32		 		    headMot;		/*载体运动航向角*/	
    fp32					sAcc;			/*速度估计精度*/    
	fp32		           	quality;		/*位置精度因子*/
	
	/*计算控制数据*/
	Vector3f_Nav 			CurSpeed;	/*本次运动速度*/
	Vector3f_Nav 			LastSpeed;	/*上次运动速度*/
	Vector3f_Nav 			DeltaSpeed;	/*运动速度变化量*/
}GPS_Data;

/*光流数据*/
typedef struct
{
	/*位移量*/
	Vector2f IntPixLPF;			
	Vector2f AngleCompensate;	 /*姿态角补偿积分位移*/
	Vector2f LastRawPosition;	 /*上次处理后的积分位移*/	
	Vector2f CurRawPosition;	 /*本次处理后的积分位移*/
	Vector2f DealtRealPosition;  /*两次计算的像素点差*/
	
	/*速度量*/
	Vector2f GyroSpeed;			/*角速度rad/s*/
	Vector2f DiffSpeed;			/*微分速度*/
	Vector2f DiffSpeedLPF;		/*微分速度低通滤波*/
	
	/*分辨率*/
	fp32 cpi;
	
	/*转化后的真实位移和速度*/
	Vector2f RealPosition;
	Vector2f RealSpeed;
}OpticFlow_Data;

/*气压计姿态数据*/
typedef struct
{
	fp32 zeroPressure;	    /*零参考点气压值*/
	fp32 rawPressure;		/*气压值原始值*/	
	fp32 filterPressure;	/*气压值滤波值*/
	fp32 zeroHeight;		/*零参考点气压计高度值*/
	s32	 rawAltitude;		/*原始高度*/
	fp32 obAltitudeOffset;	/*气压计观测高度偏移*/		
	s32  lastAltitude;  	/*上次(气压计)高度(cm)*/
    s32  curAltitude;   	/*当前(气压计)高度(cm)*/
	fp32 climbSpeed;		/*气压计观测到的(向上爬升速度)(cm/s)*/	
}Baro_Data;

/*超声波姿态数据*/
typedef struct
{
	s16  zeroHeight;		/*零参考点超声波高度值*/	
	s16	 rawAltitude;		/*超声波观测高度原始值*/
	s16	 filterAltitude;	/*超声波观测高度滤波值*/	
    s16  lastAltitude;  	/*上次(超声波)高度(cm)*/
    s16  curAltitude;   	/*当前(超声波)高度(cm)*/
	fp32 climbSpeed;	    /*超声波观测到的(向上爬升速度)(cm/s)*/
}Ultr_Data;

/*GPS控制*/
typedef struct
{
	GPS_Coordinate_s4 Coordinate_s4;
	GPS_Coordinate_f8 Coordinate_f8;	
}GPS_Coordinate;

/*全姿态*/
typedef struct
{
	/*位置姿态*/
	AhrsAttitude   Ahrs;					/*Ahrs 9Dof姿态*/
	fp32		   declination;				/*地磁偏角*/
	s32 		   fixAltitude;	    		/*定点海拔高度*/
	GPS_Coordinate HomePos;					/*HOME的经纬度*/
	Vector3f_Nav   EarthFrameRelativeHome;	/*导航(地理)坐标系,天 、正北、正东方向上位置偏移*/
	Vector3f_Nav   EarthFramePosError;		/*导航(地理)坐标系，正东、正北方向位置偏差*/
	Vector2f   	   BodyFrameRelativeHome;  	/*机体坐标系,天 、正北、正东方向上位置偏移*/	
	Vector2f_Ang   BodyFramePosError;		/*机体方向上位置偏差*/
	Vector2f_Ang   BodyFrameBrakeSpeed;		/*机体方向上刹车速度*/
	Vector2f_Ang   BodyFrameSpeedFeedback;	/*导航(地理)坐标系，机体横滚(Y正)、俯仰(X正)方向目标运动速度反馈*/
	
	/*GPS数据*/
	GPS_Data	   GpsData;
	
	/*光流数据*/
	OpticFlow_Data OpticFlowData;
	
	/*竖直方向*/
	/*气压计*/
	Baro_Data	   BaroData;
	
	/*超声波*/
	Ultr_Data      UltrData;
	
	/*水平方向*/       
	fp32 		   gpsRelativeHome;			/*GPS相对HOME点直线距离*/
}AttitudeAll;

/*====== GPS数据和惯导关系 ======*/
/*gps控制数据获取*/
void gps_fix_position_data_get(GpsM8nPvtData pvtData, GPS_Data *gpsData);

/*GPS Home点设置*/
void gps_home_location_set(void);

/*获取GPS HOME点设置状态*/
UAV_HOME_SET_STATUS get_gps_home_set_status(Uav_Status *uavStatus);

/*两点间的xy距离*/
Vector2f_Nav gps_Two_Pos_XY_Offset(GPS_Coordinate_s4 loc1, GPS_Coordinate_s4 loc2);

/*经度比例*/
fp32 gps_Longitude_Scale(GPS_Coordinate_s4 loc);

/*两点间的直线距离*/
fp32 gps_Two_Pos_Segment_Distance(GPS_Coordinate_s4 loc1, GPS_Coordinate_s4 loc2);

/*获取机体相对home的水平偏移*/
void gps_Offset_Relative_To_Home(void);

/*====== OpticFlow数据 ======*/
/*对光流数据进行处理,计算出需要的速度,位移信息*/
void opflow_Offset_Relative_To_Home(OpFlowUpixelsLC306DataFrame OpFlowData, fp32 sinsHeight_cm, AttitudeAll *attitudeAll);

/*====== Bero和Ultr高度数据获取及处理(校准、滤波) ======*/
/*获取气压计相对观测高度*/
s32 baro_get_relative_altitude(fp32 currentPa, fp32 referencePa);

/*Bero Altitude数据获取和处理 */
void bero_altitude_data_get_and_dp(Uav_Status *uavStatus);

/*获取气压计观测数据状态*/
UAV_SENMOD_DATA_STATUS get_bero_estimate_data_status(Uav_Status *uavStatus);

/*Ultr Altitude数据获取和处理*/
void ultr_altitude_data_get_and_dp(Uav_Status *uavStatus);

/*获取超声波观测数据状态*/
UAV_SENMOD_DATA_STATUS get_ultr_estimate_data_status(Uav_Status *uavStatus);

extern AttitudeAll g_sAttitudeAll;
extern AttitudeAll *g_psAttitudeAll;

#endif
