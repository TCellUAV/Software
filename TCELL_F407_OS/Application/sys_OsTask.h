#ifndef _SYS_OSTASK_H_
#define _SYS_OSTASK_H_

#include "msp_NVIC.h"
#include "sys_Platform.h"
#include "bsp_BoardLib.h"

#include "ahrs_Caculation.h"
#include "attitude_Aircraft.h"

#include "sins_Strapdown.h"
#include "remot_DataAnaly.h"
#include "calib_SensorData.h"
#include "control_Aircraft.h"
#include "hci_oledshow.h"

/*线程优先级分配*/
enum
{	
	/*基础外设*/
	RTOS_PR_BASE_MODULE				   = 12,
	
	/*参数读写系统*/
	RTOS_PR_STOR_PARA_RDWR 			   = 16,
	
	/*传感器数据获取及姿态融合*/
	RTOS_PR_EULER_ANGLE_CALCULATE	   = 6,	/*IMU, MAG, Euler*/
	RTOS_PR_DATA_VERTICAL_FUSION	   = 7, /*BERO, ULTR, Vertical*/
	RTOS_PR_GPS_CTRL_DATA_GET		   = 4,	/*GPS 控制数据获取*/
	RTOS_PR_OPFLOW_CTRL_DATA_GET	   = 5, /*OPFLOW 控制数据获取*/
	RTOS_PR_GPS_HORIZONTAL_FUSION	   = 8, /*GPS, Horizontal*/
	RTOS_PR_OPFLOW_HORIZONTAL_FUSION   = 9, /*OPTICFLOW, Horizontal*/
	
	/*控制系统*/
	RTOS_PR_UAV_CTRL_SYSTEM 		   = 3, /*remot, pid, output motor*/
	
	/*系统校准*/
	RTOS_PR_UAV_CALIB_SYSTEM		   = 13, /*ACC, MAG calib*/
	
	/*飞行相关*/
	RTOS_PR_TFSD_FLY_LOG			   = 14, /*fly log write and read_send*/
	
	/*人机交互*/
	RTOS_PR_HCI_OLED_SHOW              = 21, /*hci: oled show*/
	RTOS_PR_HCI_SLAVE_AND_HOST         = 22, /*hci: host salve exchange*/
	
	/*任务调度状态*/
	RTOS_PR_TASK_STATUS_CHECK		   = 1, /*任务执行状态检测*/
};

/*数据更新事件*/
enum
{
	/*== 系统(校准)参数写入 ===*/
	RTOS_EVENT_PID_PARA_UPDATE_BY_HOST = 1,	/*PID参数被上位机下达改变指令(调参)*/

	/*== 校准 ===*/
};

/*RTOS 线程唤醒计数器设置*/
/*PLATFORM_TASK_SCHEDULER_MIN_FOC_MS*/
/*基础外设*/
#define RTOS_WAKE_UP_BASE_MODULE_FOC_MS		  (01 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)
/*传感器数据获取及姿态融合*/	
#define RTOS_WAKE_UP_EULER_ANGLE_FOC_MS		  (01 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)
#define RTOS_WAKE_UP_VER_FUSION_FOC_MS	 	  (01 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)
#define RTOS_WAKE_UP_GPS_HOR_FUSION_FOC_MS    (01 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)
#define RTOS_WAKE_UP_OPFLOW_HOR_FUSION_FOC_MS (01 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)
/*控制系统*/
#define RTOS_WAKE_UP_UAV_CTRL_FOC_MS	      (01 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)
/*校准系统*/
#define RTOS_WAKE_UP_UAV_CALIB_FOC_MS		  (01 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)
/*飞行相关*/
#define RTOS_WAKE_UP_TFSD_FLY_LOG_FOC_MS	  (20 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)
/*人机交互*/
#define RTOS_WAKE_UP_HCI_OLED_SHOW_FOC_MS	  (30 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)
#define RTOS_WAKE_UP_HCI_SLAVE_HOST_FOC_MS	  (01 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)
/*任务调度状态*/
#define RTOS_WAKE_UP_TASK_STATUS_CHECK_FOC_MS (01 * PLATFORM_TASK_SCHEDULER_MIN_FOC_MS)

typedef vu16 rt_wakeup_t;
typedef struct
{
	/*基础外设*/
	rt_wakeup_t base_module;	
	
	/*传感器数据获取及姿态融合*/	
	rt_wakeup_t euler_angle;
	rt_wakeup_t ver_fusion;
	rt_wakeup_t gps_hor_fusion;
	rt_wakeup_t opflow_hor_fusion;
	
	/*控制系统*/
	rt_wakeup_t uav_ctrl;	
	
	/*校准系统*/
	rt_wakeup_t uav_calib;
	
	/*飞行相关*/
	rt_wakeup_t tfsd_fly_log;
	
	/*人机交互*/
	rt_wakeup_t hci_oled_show;
	rt_wakeup_t hci_slave_host;	
	
	/*任务调度状态*/	
	rt_wakeup_t task_status_check;
}rtos_thread_wake_up;

/*NVIC初始化*/
void mcu_nvic_init(void);

/*MCU初始化*/
void mcu_driver_init(void);

/*硬件初始化*/
void hardware_init(void);

/*飞控系统初始化*/
void uav_system_init(void);

/*MCU 初始化后工作*/
void work_after_system_init(void);

/*RTOS 组件初始化*/
void rtos_unit_init(void);

/*创建线程*/
void rtos_thread_create(void);

extern rtos_thread_wake_up gs_rtos_thread_wake_up;

#if defined(PLATFORM_RTOS__RT_THREAD)
/*********** 信号量(定时执行) ***********/	
/*=== 基础外设 ===*/
extern struct rt_semaphore base_module_sem;    
/*=== 传感器数据获取及姿态融合 ===*/
extern struct rt_semaphore euler_angle_sem;
extern struct rt_semaphore ver_fusion_sem;
extern struct rt_semaphore gps_update_sem;
extern struct rt_semaphore opflow_update_sem;
extern struct rt_semaphore gps_hor_fusion_sem;  
extern struct rt_semaphore opflow_hor_fusion_sem; 
/*=== 控制系统 ===*/
extern struct rt_semaphore uav_ctrl_sem;

/*=== 校准系统 ===*/
extern struct rt_semaphore uav_calib_sem;
/*=== 飞行相关 ===*/
extern struct rt_semaphore tfsd_fly_log_sem;
/*=== 人机交互 ===*/
extern struct rt_semaphore hci_oled_show;
extern struct rt_semaphore hci_host_slave_sem;
/*=== 任务调度状态 ===*/
extern struct rt_semaphore task_status_check_sem;

/*********** 事件(被动执行) ***********/
/*=== 参数读写 ===*/
extern struct rt_event para_rdwr_event;

/********** 互斥锁(资源保护) ***********/
/*=== I2C通信 ===*/
//extern struct rt_mutex i2c_mutex;
//extern struct rt_mutex gps_mag_i2c_mutex;
/*=== SPI通信 ===*/
//extern struct rt_mutex spi_mutex;
#endif

#endif
