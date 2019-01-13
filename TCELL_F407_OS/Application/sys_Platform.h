#ifndef _SYS_PLATFORM_H_
#define _SYS_PLATFORM_H_

#include "stm32f4xx.h"
#include "arm_math.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"

#define PLATFORM_RTOS__RT_THREAD

/*RTOS*/
#ifdef PLATFORM_RTOS__RT_THREAD
/*RT-Thread*/
#include <rtthread.h>
#define sys_DelayMs(foc)	 rt_thread_delay(foc)	/*delay min ms*/
#endif

/*系统调度时基本*/
#define PLATFORM_TASK_SCHEDULER_MIN_FOC_MS	(5)	/*系统调度最小时基*/

/*Processors do multiplication faster than division!(处理器做乘法比做除法更快)*/

/********* MCU信息(重点,必须根据实际设置) *********/
#define ST_STM32F407VE				(1)
#define ST_STM32F407VG				(2)
#define ST_STM32F427VG				(4)

#if 1
#define MCU_TARG				    (ST_STM32F407VE)	    		  /*MCU类型*/
#define MCU_PAKEGE_IO				(100.82)						  /*封装和IO个数*/
#define MCU_INTERNAL_RAM_SIZE		(192)	    					  /*KByte*/
#define MCU_INTERNAL_FLASH_SIZE		(512)	    					  /*KByte*/
#define MCU_SYSTEME_CLOCK_MHZ		(168.0f) 						  /*系统时钟频率(Mhz)*/
#define MCU_ONE_NOP_SYCLE_NS		(1000.0f / MCU_SYSTEME_CLOCK_MHZ) /*1个nop的时间*/	
#endif

#if 0
#define MCU_TARG				    (ST_STM32F407VG)	    		  /*MCU类型*/
#define MCU_PAKEGE_IO				(100.82)						  /*封装和IO个数*/
#define MCU_INTERNAL_RAM_SIZE		(192)	    					  /*KByte*/
#define MCU_INTERNAL_FLASH_SIZE		(1024)	    					  /*KByte*/
#define MCU_SYSTEME_CLOCK_MHZ		(168.0f) 						  /*系统时钟频率(Mhz)*/
#define MCU_ONE_NOP_SYCLE_NS		(1000.0f / MCU_SYSTEME_CLOCK_MHZ) /*1个nop的时间*/	
#endif

#if 0
#define MCU_TARG				    (ST_STM32F427VG)	    		  /*MCU类型*/
#define MCU_PAKEGE_IO				(100.82)						  /*封装和IO个数*/
#define MCU_INTERNAL_RAM_SIZE		(256)	    					  /*KByte*/
#define MCU_INTERNAL_FLASH_SIZE		(1024)	    					  /*KByte*/
#define MCU_SYSTEME_CLOCK_MHZ		(180.0f) 						  /*系统时钟频率(Mhz)*/
#define MCU_ONE_NOP_SYCLE_NS		(1000.0f / MCU_SYSTEME_CLOCK_MHZ) /*1个nop的时间*/	
#endif
/*********************************************/

/****** 飞控版本号(配合匿名上位机查看) *******/
#define PILOT_HARDWARE_TYPE			(1)
#define PILOT_HARDWARE_VERSION		(100)
#define PILOT_SOFTWARE_VERSION		(100)
#define PILOT_PROTOCOL_VERSION		(401)
#define PILOT_BOOTLOADER_VERSION	(100)
/*********************************************/

/*Hardware FPU DSP*/
#define FPU_ENABLE_DSP //FPU_DISABLE_DSP

/*DSP*/
#ifdef FPU_ENABLE_DSP

#endif

/*No DSP*/
#ifdef FPU_DISABLE_DSP

#endif

/******************* 坐标系关联方向 *******************
**************************************************************
***************************************************************
              X型安装方式,电机序号与姿态角关系
                             -
                           Pitch
                   3#                1#
                  ！！！            ！！！
                   ！！！          ！！！
                    ！！！        ！！！
                     ！！！      ！！！
                      ！！！    ！！！
       -   Roll            ！！！            Roll   +
                           ！！！
                      ！！！    ！！！
                     ！！！      ！！！
                    ！！！        ！！！
                   ！！！          ！！！
                  ！！！            ！！！
                   2#                4#
加速度传感器轴向与载体X、Y、Z同轴,沿轴向原点看，逆时针旋转角度为+
                           Pitch
                             +
                            Y Aixs
                            *
                            *
                            *
                            *
                            *
                            *
                            * * * * * * * *   X Axis Roll+
                          (0)
***************************************************************
**************************************************************
Pitch(欧拉角,惯导方向,欧拉角绕x轴) - x(惯导元器件x方向) - y(Pitch角旋转沿y轴) - Longitude(竖经) - E(east)
Roll(欧拉角,惯导方向,欧拉角绕y轴)  - y(惯导元器件y方向) - x(Roll角旋转沿x轴)  - Latitude(横纬)  - N(north)
Yaw(欧拉角角,惯导方向,欧拉角绕z轴) - z(惯导元器件z方向) - xxxxxxxxxxxxxx      - xxxxxxxxxxxxxx  - U(up)
导航(地理)坐标系，正北+Lat(纬度)+X、正东+Lon(经度)+Y 方向位置偏移
机体(载体)坐标系，机体横滚+x正+roll、机体俯仰+(y正)+pitch 方向位置偏移
******************************************************/

/**************** 遥控基本操作 ***************/
/*锁定与解锁*/
	/*1.摇杆内八解锁*/
	/*
		*******       *******
		*     *       *     *
		*  *  *       *  *  * 
		*   * *       * *   *
		*******       *******
			  *       *   
	*/
	
	/*2.摇杆外八锁定*/
	/*
		*******       *******
		*     *       *     *
		*  *  *       *  *  * 
		* *   *       *   * *
		*******       *******
		*                   *   
	*/

/*加速度计和磁力计校准*/	
	/*1.加速度计进入/退出校准动作判断*/
	/*						*	
		*******       *******
		*     *       *   * *
		*  *  *       *  *  * 
		* *   *       *     *
		*******       *******
		*                     
	*/
	
	/*2.磁力计进入/退出校准动作判断*/
	/*			      *	
		*******       *******
		*     *       * *   *
		*  *  *       *  *  * 
		*   * *       *     *
		*******       *******
		      *                    
	*/	

/*HCI_OLED显示控制*/
	/*1.开启OLED显示*/
	/*		  	      *	
		*******       *******
		*     *       * *   *
		*  *  *       *  *  * 
		*  *  *       *     *
		*******       *******		                        
		   *
	*/

	/*2.关闭OLED显示*/
	/*		  	      *	
		*******       *******
		*     *       * *   *
		*  *  *       *  *  * 
		*  *  *       *     *
		*******       *******		                        
		   *
	*/
/*********************************************/

/******************* 标记 ********************/
#define SYS_NO_AVA_MARK					  (-255)	/*无效标记*/
#define SYS_ENABLE						  (1)		/*使能*/
#define SYS_DISABLE						  (0)		/*不使能*/
/*********************************************/

/**************** 自然事物常数 ***************/
#ifndef PI
#define PI						    (3.1415926f)		  	/*π值*/
#endif

#define LOCAL_DECLINATION_YAW		(-3.61915f)				/*当前地磁偏角,查询相关网站获得*/

#define SEA_LEVEL_PRESSURE			(101325.0f)				/*海平面气压(Pa)*/
#define EARTH_RADIUS				(6371004)				/*地球半径(m)*/
#define DEG2RAD            			(PI / 180.0f) 			/*度转弧度*/
#define RAD2DEG 		   			(180.0f / PI) 			/*弧度转度*/
#define GRAVITY_STD		   			(9.80665f)				/*标准重力加速度m/s^2*/
/*********************************************/

/***** 传感器量程设置(重点,必须根据实际设置) ******/
#define GPS_LOCATION_SCALING_FACTOR (0.011131884502145034f) /*1 经/纬度 实际距离 111千米*/
#define MPU_ACC_RANGE	   			(4.0f / 32768.0f)		/*加速度量程:±4G*/
#define MPU_GYRO_RANGE	   			(1000.0f / 32768.0f) 	/*陀螺仪量程:(±1000°/s)*/
#define ACC_MAX_ONE_G	   			(8192.0f)			    /*量程*/
#define ACC_TO_ONE_G	   			(GRAVITY_STD / ACC_MAX_ONE_G)
#define ONE_G_TO_ACC	   			(ACC_MAX_ONE_G / GRAVITY_STD)
/*********************************************/

/************* 模块参数性能 *************/
/*ULTR超声波*/
#define SYS_ULTR_MAX_MEAS_DISTANCE		 	           (200)	/*超声波最大允许测量距离:200cm*/
#define SYS_ULTR_MIN_MEAS_PERIOD_TICK_MS 	           (100)  	/*超声波最小测量周期:100ms*/

/*BERO气压计*/
#define SYS_BERO_MIN_MEAS_PERIOD_TICK_MS 	           (110)  	/*SPL06气压计最小测量周期:110ms*/

/*OPFLOW光流*/
#define SYS_OPFLOW_MAX_MEAS_DISTANCE		 	       (200)	/*光流最大允许竖直距离:200cm*/

/*电子调速器(电调)*/
#define ESC_MIN_PULSE_ZERO_SPEED_VALUE	               (1000) /*Electronic Speed Control最小脉冲宽度,零速度,电机停转状态*/
#define ESC_MAX_PULSE_MAX_SPEED_VALUE	               (2000) /*Electronic Speed Control最大脉冲宽度,最大速度,电机转速最快状态*/
										               
/*电机转动驱动对象*/                                   
#define MOTOR_TURN_DRIVE_TARGET_INDEX	               (0xFF) 		/*全部转动驱动(开关作用0x00即全部停转)*/ 
										               
/*遥控FLYSKY*/                                         
#define REMOT_DATA_REAL_MIN_VALUE		               (1000)		/*遥控数据真实最小值(1000us)*/
#define REMOT_DATA_REAL_MAX_VALUE		               (2000)		/*遥控数据真实最大值(2000us)*/
#define REMOT_ANGLE_MID					               (1500)		/*遥控期望角度(Roll,Pitch,Yaw)中间值*/
#define REMOT_ANGLE_MID_DEADZONE	                   (100)		/*遥控期望角度(Roll,Pitch,Yaw)死区值*/
#define REMOT_THROTTLE_DEADZONE_MIN		               (1100)		/*遥控油门(Throttle)最小值死区*/
#define REMOT_THROTTLE_BASE_VALUE		               (1000)		/*遥控油门(Throttle)基值(最小值)*/
#define REMOT_THROTTLE_MID_DEADZONE		               (200)		/*遥控油门(Throttle)中间死区*/
/******************************************/

/********* EEPROM保存数据的地址 **********/
/*校准参数存储地址*/
#define AT24CXX_STOR_ACC_SCALE_ADDR                    (45)										  /*Acc比例因子校准参数存储位置*/
#define AT24CXX_STOR_ACC_OFFSET_ADDR                   (AT24CXX_STOR_ACC_SCALE_ADDR  + 12 + 4)	  /*Acc零偏校准参数存储位置*/
#define AT24CXX_STOR_MAG_OFFSET_ADDR                   (AT24CXX_STOR_ACC_OFFSET_ADDR + 12 + 4)	  /*磁力计中心偏执校准参数存储位置*/
#define AT24CXX_STOR_GPS_MAG_OFFSET_ADDR               (AT24CXX_STOR_MAG_OFFSET_ADDR + 12 + 4)	  /*GPS_磁力计中心偏执校准参数存储位置*/
#define AT24CXX_STOR_PID_PARA_ADDR		               (AT24CXX_STOR_GPS_MAG_OFFSET_ADDR + 12 + 4)  /*PID系数存储位置*/
/*****************************************/

/*返回值状态类型*/
typedef enum
{
	SYS_RET_SUCC  = 0,
	SYS_RET_FAIL  = 1,
	SYS_RET_BUSY  = 2,
}SYS_RETSTATUS;

/*返回值错误类型(检测错在第几步)*/
typedef enum
{
	SYS_RETERR_0ZR  = 0,
	SYS_RETERR_1ST  = 1,
	SYS_RETERR_2ND  = 2,	
	SYS_RETERR_3RD  = 3,
	SYS_RETERR_4TH  = 4,	
	SYS_RETERR_5TH  = 5,	
	SYS_RETERR_6TH  = 6,
	SYS_RETERR_7TH  = 7,
	SYS_RETERR_8TH  = 8,
	SYS_RETERR_9TH  = 9,
	SYS_RETERR_10TH = 10,
	SYS_RETERR_11TH = 11,
	SYS_RETERR_12TH = 12,
	SYS_RETERR_13TH = 13,	
	SYS_RETERR_14TH = 14,
	SYS_RETERR_15TH = 15,
	SYS_RETERR_16TH = 16,
	SYS_RETERR_17TH = 17,
	SYS_RETERR_18TH = 18,	
	SYS_RETERR_19TH = 19,
	SYS_RETERR_20TH = 20,		
	SYS_RETERR_FF   = 0xff,	/*默认的状态*/
}SYS_RETERR;

/*数据准备状态*/
typedef enum
{
	DATA_READY_YES = 0,
	DATA_READY_NOT = 1,
}DATA_READY_STATUS;

/*全局数据类型定义*/
typedef short 			s16;
typedef int64_t 		s64;
typedef int32_t  		s32;
typedef int16_t 		s16;
typedef int8_t  		s8;

typedef const int32_t 	sc32;  
typedef const int16_t 	sc16;  
typedef const int8_t 	sc8;  

typedef __IO int32_t  	vs32;
typedef __IO int16_t  	vs16;
typedef __IO int8_t   	vs8;

typedef __I int32_t 	vsc32;  
typedef __I int16_t 	vsc16; 
typedef __I int8_t 		vsc8;   
	
typedef uint64_t  		u64;
typedef uint32_t  		u32;
typedef uint16_t 		u16;
typedef uint8_t  		u8;

typedef const uint32_t 	uc32;  
typedef const uint16_t 	uc16;  
typedef const uint8_t 	uc8; 

typedef __IO uint32_t  	vu32;
typedef __IO uint16_t 	vu16;
typedef __IO uint8_t  	vu8;

typedef __I uint32_t 	vuc32;  
typedef __I uint16_t 	vuc16; 
typedef __I uint8_t 	vuc8; 

typedef const float		fpc32;
typedef const double	dbc64;

typedef __IO float		vfp32;
typedef __IO double		vfp64;

typedef __I float		vfpc32;
typedef __I double		vdbc64;

typedef float 			fp32;
typedef double 		    fp64;

/*浮点数联合体*/
#define FLOAT_BYTE_NUM 		(4)	/*fp32类型数据占用字节数*/
#define S32_BYTE_NUM 		(4)	/*s32类型数据占用字节数*/
#define U32_BYTE_NUM 		(4)	/*u32类型数据占用字节数*/

#define S16_BYTE_NUM 		(2)	/*s32类型数据占用字节数*/
#define U16_BYTE_NUM 		(2)	/*u32类型数据占用字节数*/

typedef union
{
	fp32 value;
	u8   byte[FLOAT_BYTE_NUM];
}StorFloatData;

typedef union
{
	s32 value;
	u8  byte[S32_BYTE_NUM];
}StorS32Data;

typedef union
{
	u32 value;
	u8  byte[U32_BYTE_NUM];
}StorU32Data;

typedef union
{
	s16 value;
	u8  byte[S16_BYTE_NUM];
}StorS16Data;

typedef union
{
	u16 value;
	u8  byte[U16_BYTE_NUM];
}StorU16Data;

/*状态机*/
enum
{
	BIT_FLAG_RESET = 0,	
	BIT_FLAG_SET   = 1,
};

typedef struct
{
	vu8 bit0:1;
	vu8 bit1:1;
	vu8 bit2:1;
	vu8 bit3:1;
	vu8 bit4:1;
	vu8 bit5:1;
	vu8 bit6:1;
	vu8 bit7:1;	
}U8_Bit_Flag;

typedef struct
{
	vu16 bit0:1;
	vu16 bit1:1;
	vu16 bit2:1;
	vu16 bit3:1;
	vu16 bit4:1;
	vu16 bit5:1;
	vu16 bit6:1;
	vu16 bit7:1;	
	vu16 bit8:1;
	vu16 bit9:1;
	vu16 bit10:1;
	vu16 bit11:1;
	vu16 bit12:1;
	vu16 bit13:1;
	vu16 bit14:1;
	vu16 bit15:1;	
}U16_Bit_Flag;

/*传感器数据类型(s: short / f: fp32)*/
/*IMU_Acc原始数据*/
typedef struct
{
	s16 x;
	s16 y;
	s16 z;
}Acc3s;		

/*IMU_Acc滤波后的数据*/
typedef struct
{
	fp32 x;
	fp32 y;
	fp32 z;
}Acc3f;

/*IMU_Gyro原始数据*/
typedef struct
{
	s16 x;
	s16 y;
	s16 z;
}Gyro3s;		

/*IMU_Gyro滤波后的数据*/
typedef struct
{
	fp32 x;
	fp32 y;
	fp32 z;
}Gyro3f;

/*AHRS_Mag原始数据*/
typedef struct
{
	s16 x;
	s16 y;
	s16 z;
}Mag3s;			

/*AHRS_Mag滤波后的数据*/
typedef struct
{
	fp32 x;
	fp32 y;
	fp32 z;
}Mag3f;

/*IMU(R&F)数据*/
typedef struct
{	
	Acc3f  accRaw;		/*加速度原始数据*/
	Acc3f  accFilter;	/*加速度滤波后数据*/
	Gyro3f gyroRaw;		/*角速度原始数据*/
	Gyro3f gyroFilter;	/*角速度滤波后数据*/
}Imu3fRF;

/*Mag磁力计(R&F)数据*/
typedef struct
{
	Mag3f magRaw;		/*磁力计原始数据*/
	Mag3f magFilter;	/*磁力计滤波数据*/
}Mag3fRF;

/****** 方向余弦矩阵(DCM) ******/
typedef struct
{
	s32 x;
	s32 y;
}Vector2i;

typedef struct
{
	fp32 x;
	fp32 y;
}Vector2f;

typedef struct
{
	s16 x;
	s16 y;
	s16 z;
}Vector3i;

typedef struct
{
	fp32 x;
	fp32 y;
	fp32 z;
}Vector3f;

typedef struct
{
	fp32 a;
	fp32 b;
	fp32 c;
	fp32 r;
}Vector4f;

typedef struct
{
	fp32 q0;
	fp32 q1;
	fp32 q2;
	fp32 q3;	
}Vector4q;

/****** 机体导航 ******/
typedef struct
{
	fp32 east;
	fp32 north;
	fp32 up;
}Vector3f_Nav;

typedef struct
{
	s32 lon;
	s32 lat;
}Vector2s_Nav;

typedef struct
{
	fp32 east;
	fp32 north;
}Vector2f_Nav;

typedef struct
{
	fp32 pitch;
	fp32 roll;
}Vector2f_Ang;

typedef struct
{
	fp32 pitch;
	fp32 roll;
}Vector2f_Body;

typedef struct
{
	fp32 east;	
	fp32 north;
}Vector2f_Earth;

/****** PID算法参数 ******/
typedef struct
{
	fp32 kP;
	fp32 kI;
	fp32 kD;
}Vector3f_PID;

typedef struct
{
	s16 kP;
	s16 kI;
	s16 kD;
}Vector3s_PID;

/****** 模拟看门狗 ******/
typedef enum
{
	SIM_WATCH_DOG_ALIVE = 0, /*活着*/
	SIM_WATCH_DOG_DIED  = 1, /*死去*/
}SIM_WATCH_DOG_STATUS;

typedef struct
{	
	vu32 curTicks; 			/*当前ticks*/
	vu32 nextProcessTicks;   /*下次判断ticks*/
}SimulateWatchDog;

#ifdef PLATFORM_RTOS__RT_THREAD
#include "rtos_cpuusage.h"
#endif

#include "period_Execute.h"	      /*执行周期测试: 包含 my_GetTick()*/
#include "hci_show_send_config.h" /*人机交互:OLED和发送波形配置*/
#include "sys_ControlMap.h"

#endif
