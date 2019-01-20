#ifndef _HCI_AIRCRAFT_H_
#define _HCI_AIRCRAFT_H_

#include "sys_Platform.h"
#include "bsp_BoardLib.h"
#include "bsp_HCI_OLED0_96.h"
#include "status_Aircraft.h"
#include "calib_SensorData.h"
#include "pid_system.h"

#define HCI_PAGE_CHANGE_MIN_HOLD_TIME_MS	(1500) /*页面切换动作最小持续时间(ms)*/

/*包含字库*/
extern const u8 g_Ascii8X16[];
extern const u8 g_Ascii6X12[];
extern const u8 g_Ascii6X8[];
extern const u8 g_Hanzi16X16[][16];
extern const u8 g_Hanzi12X12[];
extern const u8 g_LogoFlashWolves128X64[];
extern const u8 g_ProgressBar96X24[];
extern const u8 g_ProgressBarFillBlock11X8[];
extern const u8 test64X64[];
extern const u8 g_EnableShowHint128X64[];

/*ACC & MAG校准提示*/
/*加速度校准提示:第一面*/
extern const u8 g_AccCalib_1st_Side_48X128[];
/*加速度校准提示:第二面*/
extern const u8 g_AccCalib_2nd_Side_48X128[];
/*加速度校准提示:第三面*/
extern const u8 g_AccCalib_3rd_Side_48X128[];
/*加速度校准提示:第四面*/
extern const u8 g_AccCalib_4th_Side_48X128[];
/*加速度校准提示:第五面*/
extern const u8 g_AccCalib_5th_Side_48X128[];
/*加速度校准提示:第六面*/
extern const u8 g_AccCalib_6th_Side_48X128[];

/*磁力计校准提示:第一面*/
extern const u8 g_MagCalib_1st_Side_48X128[];
/*磁力计校准提示:第二面*/
extern const u8 g_MagCalib_2nd_Side_48X128[];
/*磁力计校准提示:第三面*/
extern const u8 g_MagCalib_3rd_Side_48X128[];

/*初始化对象*/
typedef enum
{
	HCI_SHOW_INIT_HCI_MODE    = 0, /*_0_人机交互模块*/
	HCI_SHOW_INIT_BASE_MODE   = 1, /*_1_基础模块*/
	HCI_SHOW_INIT_STOR_MODE   = 2, /*_2_数据存储模块*/
	HCI_SHOW_INIT_AHRS_MODE   = 3, /*_3_AHRS模块*/
	HCI_SHOW_INIT_BERO_MODE   = 4, /*_4_气压计模块*/
	HCI_SHOW_INIT_ULTR_MODE   = 5, /*_5_超声波模块*/
	HCI_SHOW_INIT_GPS_MODE    = 6, /*_6_GPS模块*/
	HCI_SHOW_INIT_OPFLOW_MODE = 7, /*_7_光流模块*/
}HCI_SHOW_INIT_TARG;

/*总共页数*/
#define HCI_TOTAL_SHOW_PAGE_NUMBER			(13)  /*【需要根据实际修改】*/
/*显示页面序号*/
typedef enum
{
	HCI_SHOW_PAGE_0  = 0,
	HCI_SHOW_PAGE_1  = 1,
	HCI_SHOW_PAGE_2  = 2,
	HCI_SHOW_PAGE_3  = 3,
	HCI_SHOW_PAGE_4  = 4,
	HCI_SHOW_PAGE_5  = 5,
	HCI_SHOW_PAGE_6  = 6,
	HCI_SHOW_PAGE_7  = 7,
	HCI_SHOW_PAGE_8  = 8,
	HCI_SHOW_PAGE_9  = 9,
	HCI_SHOW_PAGE_10 = 10,	
	HCI_SHOW_PAGE_11 = 11,		
	HCI_SHOW_PAGE_12 = 12,	
}HCI_SHOW_PAGE_INDEX;

/*页面框(固定位置)显示状态*/
typedef enum
{
	HCI_SHOW_MOULD_FIRST    = 0, /*第一次显示*/
	HCI_SHOW_MOULD_NOTFIRST = 1, /*非第一次显示*/
}HCI_SHOW_MOULD_STATUS;

/*退出显示操作状态*/
typedef enum
{
	HCI_EXIT_SHOW_OP_FIRST    = 0, /*第一次退出*/
	HCI_EXIT_SHOW_OP_NOTFIRST = 1, /*非第一次退出*/
}HCI_EXIT_SHOW_OP_STATUS;

/*显示空闲状态*/
typedef enum
{
	HCI_SHOW_TASK_IDLE = 0, /*空闲(可以显示数据页)*/
	HCI_SHOW_TASK_BUSY = 1, /*忙碌(不可以显示数据页)*/
}HCI_SHOW_TASK_STATUS;

/*========= 初始化人机交互 ========= */
/*显示飞行器启动logo*/
void hci_Show_AircraftLogoHoldMs(u32 holdMs);

/*显示初始化进度*/
void hci_Show_InitRateOfProgress(u8 totalModuleNbr, HCI_SHOW_INIT_TARG INIT_TARG, SYS_RETERR INIT_STATUS);

/*显示传感器校准结果及参数*/
void hci_Show_Sensor_Calib_Parameter(RESULT_CALIB_STATUS ACC_CALIB_STATUS, RESULT_CALIB_STATUS MAG_CALIB_STATUS, u32 holdMs);

/*显示控制系统初始化参数(PID参数)*/
void hci_Show_Control_System_Parameter(SYS_RETSTATUS READ_STATUS, fp32 kP, fp32 kI, fp32 kD, PID_CONTROLER_LINK LINK_TARG, u32 holdMs);

/*使能OLED显示提示*/
void hci_Show_Enable_Hint(void);


/*=========  程序运行过程中显示 =========*/
void hci_show_on_run_progress(void);

/*==== 1.实时数据显示 ====*/
/*NO0.AHRS数据显示*/
void hci_Show_Cur_Ahrs_Data(void);

/*NO1.高度传感器(超声波+气压计)数据显示*/
void hci_Show_High_Sensor_Data(void);

/*NO2.GPS数据显示*/
void hci_Show_Gps_Data(void);

/*NO3.光流数据显示*/
void hci_Show_Opticflow_Data(void);

/*==== 2.惯导融合数据显示 ====*/
/*NO4.竖直高度惯导融合显示*/
void hci_Show_Height_SINS_Data(void);

/*NO5.水平沿PITCH方向惯导融合显示*/
void hci_Show_HorizontalPitch_SINS_Data(void);

/*NO6.水平沿ROLL方向惯导融合显示*/
void hci_Show_HorizontalRoll_SINS_Data(void);

/*==== 3.飞行器当前状态显示 ====*/
/*NO7.程序执行周期显示*/
void hci_Show_Execute_Period(void);

/*NO8.飞行器本身状态*/
void hci_Show_Aircraft_Status(void);

/*NO9.遥控状态显示*/
void hci_Show_Remot_Status(void);

/*NO10.传感器校准结果*/
void hci_Show_Sensor_Calib_Result(void);

/*==== 4.飞行器大数据显示 ====*/
/*No11.Gps home点数据*/
void hci_Show_Gps_Home_Data(void);

/*No12.控制模式和任务显示*/
void hci_Show_Ctrl_Mission_Data(void);


/*==== 5.传感器校准交互 ====*/
/*1.进入/退出加速度计校准提示*/
void hci_Show_Acc_Calib_Status(ENTRY_CALIB_STATUS ENTRY_STATUS, CALIB_SIDE_INDEX GONNA_CALIB_SIDE);

/*2.进入/退出磁力计校准提示*/
void hci_Show_Mag_Calib_Status(ENTRY_CALIB_STATUS ENTRY_STATUS, CALIB_SIDE_INDEX GONNA_CALIB_SIDE);

/*3.加速度计校准过程显示*/
void hci_Show_Acc_Calib_Process(CALIB_SIDE_INDEX CUR_SIDE_INDEX, SAMPLE_PROCESS_TYPE PROCESS_TYPE, SINGLE_SAMPLE_STATUS SIGSAMPLE_STATUS, fp32 gyroLenth, fp32 xg, fp32 yg, fp32 zg, u32 holdMs);

/*4.磁力计校准过程显示*/
void hci_Show_Mag_Calib_Process(CALIB_SIDE_INDEX CUR_SIDE_INDEX, CALIB_POSITION_INDEX POSITION_INDEX, POSITION_SAMPLE_STATUS POS_SAMP_STATUS, SAMPLE_PROCESS_TYPE PROCESS_TYPE, u16 curYaw, u16 targetYaw, u8 errorMaxYaw, s16 targAccAxis, u32 holdMs);

typedef struct
{
	volatile HCI_SHOW_PAGE_INDEX 	  curPageIndex;         /*当前显示页面*/
	volatile HCI_SHOW_PAGE_INDEX 	  lastPageIndex;        /*上次显示页面*/
	volatile HCI_SHOW_MOULD_STATUS    MOULD_STATUS;         /*每个页面的模板框显示状态*/
	volatile UAV_HCI_SHOW_STATUS      PAGE_STATUS;          /*该页面显示状态*/
	volatile UAV_HCI_SHOW_STATUS      SHOW_DATA_STATUS;	    /*显示数据状态*/
	volatile UAV_HCI_SHOW_STATUS 	  SHOW_HINT_STATUS;	    /*显示提示状态*/
	volatile HCI_EXIT_SHOW_OP_STATUS  EXIT_SHOW_STATUS;     /*退出显示状态*/
	volatile HCI_SHOW_TASK_STATUS     SHOW_TASK_STATUS;		/*显示任务状态*/
}HciShowPage;

/*=== 遥控配合HCI ===*/
/*HCI(OLED):允许显示/禁止显示/显示上一页/显示下一页/锁定当前页/不锁定当前页*/
void hci_remot_switch_show_status(HciShowPage *hciShowPage);

extern HciShowPage g_sHciShowPage;

#endif
