#ifndef _SYS_CONTROL_MAP_H_
#define _SYS_CONTROL_MAP_H_

/*数据融合算法选择*/
#if 0
	/*竖直方向:三阶互补数据融合*/
	#define SINS_DATA_FUSION__VER_THIRDORDER
#endif

#if 1
	/*竖直方向:卡尔曼估计*/
	#define SINS_DATA_FUSION__VER_KALMAN
#endif

#if 0	
	/*水平方向:三阶互补数据融合*/
	#define SINS_DATA_FUSION__HOR_THIRDORDER
#endif

#if 1
	/*水平方向:卡尔曼惯导数据融合*/
	#define SINS_DATA_FUSION__HOR_KALMAN
#endif



/*控制算法选择*/
/*PID算法*/
#define CONTROL_SYS__ONLY_PID			    (1)
#if (CONTROL_SYS__ONLY_PID == SYS_ENABLE)
	#define CONTROL_SYS__PID
#endif

/*ADRC算法*/
#define CONTROL_SYS__ONLY_ADRC				(0)
#if (CONTROL_SYS__ONLY_ADRC == SYS_ENABLE)
	#define CONTROL_SYS__ADRC
#endif

/*PID + ADRC算法*/
#define CONTROL_SYS__PID_ADRC				(0)
#if (CONTROL_SYS__PID_ADRC == SYS_ENABLE)
	#define CONTROL_SYS__PID
	#define CONTROL_SYS__ADRC
#endif



/*遥控数据接收方式选择*/
#if 0
	/*PWM*/
	#define REMOTE_DATA_RECV__PWM
#endif

#if 1
	/*PPM*/
	#define REMOTE_DATA_RECV__PPM
#endif

#if 0
	/*SBUS*/
	#define REMOTE_DATA_RECV__SBUS
#endif

#endif
