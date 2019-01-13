#ifndef _BSP_GPS_M8N_H_
#define _BSP_GPS_M8N_H_

#include "bsp_GPS_M8N_CMD.h"
#include "sys_Platform.h"
#include "msp_UART.h"
#include "sys_McuInit.h"

#define GPS_M8N_INIT_CMC_PERIOD_TICKS_MIN_MS    (5)
#define GPS_M8N_INIT_CMC_PERIOD_TICKS_MAX_MS    (10)

#define GPS_M8N_TX_PAYLOAD_LENTH	 			(30)
#define GPS_M8N_RX_PVT_ALL_LENTH 				(100)
#define GPS_M8N_RX_PVT_PAYLOAD_LENTH 			(92)
#define GPS_M8N_TX_BUFF_LENTH		 			(50)
#define GPS_M8N_RX_BUFF_LENTH		 			(100)

/*通信方式*/
typedef enum
{
	M8N_CMC_MODE_I2C   = 0,
	M8N_CMC_MODE_UART1 = 1,
	M8N_CMC_MODE_UART2 = 2,
	M8N_CMC_MODE_USB   = 3,
	M8N_CMC_MODE_SPI   = 4,
	M8N_CMC_MODE_NULL  = 0xFF,
}GPS_M8N_CMC_MODE;

/*串口波特率*/
typedef enum
{
	M8N_UART_4800   = 0,
	M8N_UART_9600   = 1,
	M8N_UART_19200  = 2,
	M8N_UART_38400  = 3,
	M8N_UART_57600  = 4,
	M8N_UART_115200 = 5,
	M8N_UART_230400 = 6,
	M8N_UART_460800 = 7,
	M8N_UART_921600 = 8,
	M8N_UART_NULL   = 0xFF,
}GPS_M8N_UART_BAUDRATE;

/*定位类型模式*/
typedef enum
{
	M8N_POS_FIX_NO        = 0x00,	/*No Fix*/
	M8N_POS_FIX_DEAD      = 0x01,	/*Dead Reckoning only*/
	M8N_POS_FIX_2D        = 0x02,	/*2D-Fix*/
	M8N_POS_FIX_3D        = 0x03,	/*3D-Fix*/
	M8N_POS_FIX_GNSS_DEAD = 0x04,	/*GNSS + dead reckoning combined*/
	M8N_POS_FIX_TIME      = 0x05,	/*Time only fix*/
}GPS_M8N_POS_FIX_TYPE;

/*定位有效性*/
typedef enum
{
	GPS_M8N_POS_FIX_AVA  = 0,	/*定位有效*/
	GPS_M8N_POS_FIX_NAVA = 1,	/*定位无效*/
}GPS_M8N_POS_FIX_AVA_STATUS;

/*Power Save Mode state*/
typedef enum
{
	GPS_M8N_POWER_SAVE_NA                       = 0, /*n/a (i.e no PSM is active)*/
	GPS_M8N_POWER_SAVE_ENABLED                  = 1, /*an intermediate state before ACQUISITION state*/
	GPS_M8N_POWER_SAVE_ACQUISITION              = 2,
	GPS_M8N_POWER_SAVE_TRACKING                 = 3,	
	GPS_M8N_POWER_SAVE_POWER_OPTIMIZED_TRACKING = 4,
	GPS_M8N_POWER_SAVE_INACTIVE                 = 5,
}GPS_M8N_POWER_SAVE_MODE_STATE;

/*UBX-CFG(Config)-RATE(Rates)*/
/*Time Source*/
typedef enum
{
	M8N_RATE_UTC_TIME   = 0,
	M8N_RATE_GPS_TIME   = 1,
	M8N_RATE_LOCAL_TIME = 2,
	M8N_RATE_NULL_TIME  = 0xFF,	
}GPS_M8N_RATE_TIME_SOURCE;

/*指令类ID*/
typedef enum
{
	GPS_M8N_TX_CLASS_AID = 0x0B,	/*GPS Aiding*/
	GPS_M8N_TX_CLASS_CFG = 0x06,	/*Config*/
	GPS_M8N_TX_CLASS_LOG = 0x21,	/*Data Logger*/
	GPS_M8N_TX_CLASS_MGA = 0x13,	/*Multiple GNSS Assistance*/
	GPS_M8N_TX_CLASS_NAV = 0x01,	/*Navigation*/
	GPS_M8N_TX_CLASS_RXM = 0x02,	/*Receiver Manager*/
	GPS_M8N_TX_CLASS_TIM = 0x0D,	/*Timing*/
	GPS_M8N_TX_CLASS_UPD = 0x09, 	/*Firmware Update Messages*/
}GPS_M8N_TX_CLASS_ID;

/*消息类ID*/
typedef enum
{
	/*AID*/
	GPS_M8N_TX_AID_HUI       = 0x02,  /*GPS_M8N_CMC_MODE Health, UTC and lonnsphere Parameters*/
	GPS_M8N_TX_AID_INI       = 0x01,  /*Initial Data*/
	
	/*CFG*/
	GPS_M8N_TX_CFG_ANT       = 0x13,  /*Antenna Settings*/
	GPS_M8N_TX_CFG_CFG       = 0x09,  /*Configuration*/	
	GPS_M8N_TX_CFG_DAT       = 0x06,  /*Datum*/	
	GPS_M8N_TX_CFG_DOSC      = 0x61,  /*Disciplined 0 scilator*/		
	GPS_M8N_TX_CFG_EKF       = 0x12,  /*EKF Settings*/
	GPS_M8N_TX_CFG_ESFGWT    = 0x29,  /*Gyro+Wheeltick*/	
	GPS_M8N_TX_CFG_ESRC      = 0x60,  /*External Source Config*/
	GPS_M8N_TX_CFG_FXN       = 0x0E,  /*Fix Now Mode*/	
	GPS_M8N_TX_CFG_GNSS      = 0x3E,  /*GNSS Config*/	
	GPS_M8N_TX_CFG_INF       = 0x02,  /*Inf Messages*/	
	GPS_M8N_TX_CFG_ITFM      = 0x39,  /*Jamming/Interference Monitor*/		
	GPS_M8N_TX_CFG_LOGFILTER = 0x47,  /*Log Setting*/	
	GPS_M8N_TX_CFG_MSG       = 0x01,  /*Messages*/
	GPS_M8N_TX_CFG_NAV5      = 0x24,  /*Navigation 5*/	
	GPS_M8N_TX_CFG_NAVX5     = 0x23,  /*Navigation Expert 5*/	
	GPS_M8N_TX_CFG_NMEA      = 0x17,  /*NMEA Protocol*/
	GPS_M8N_TX_CFG_ODO       = 0x1E,  /*Odometer/Low-Speed COG filter*/	
	GPS_M8N_TX_CFG_PM        = 0x32,  /*Power Management*/
	GPS_M8N_TX_CFG_PM2       = 0x3B,  /*Extended Power Management*/	
	GPS_M8N_TX_CFG_PRT       = 0x00,  /*Ports*/	
	GPS_M8N_TX_CFG_PWR       = 0x57,  /*Power*/	
	GPS_M8N_TX_CFG_RATE      = 0x08,  /*Rates*/
	GPS_M8N_TX_CFG_RINV      = 0x34,  /*Remote Inventory*/	
	GPS_M8N_TX_CFG_RST       = 0x04,  /*Reset*/
	GPS_M8N_TX_CFG_RXM       = 0x11,  /*Receiver Manager*/	
	GPS_M8N_TX_CFG_SBAS      = 0x16,  /*SBAS Settings*/
	GPS_M8N_TX_CFG_SMGR      = 0x62,  /*Sync Manager Config*/
	GPS_M8N_TX_CFG_TMODE     = 0x1D,  /*Time Mode*/	
	GPS_M8N_TX_CFG_TMODE2    = 0x3D,  /*Time Mode 2*/	
	GPS_M8N_TX_CFG_TP        = 0x07,  /*Timepulse*/
	GPS_M8N_TX_CFG_TP5       = 0x31,  /*Timepulse 5*/	
	GPS_M8N_TX_CFG_TXSLOT    = 0x53,  /*Tx Time Slots*/
	GPS_M8N_TX_CFG_USB       = 0x1B,  /*Universal Serial Bus*/
	
	/*LOG*/
	GPS_M8N_TX_LOG_CREATE    = 0x07,  /*Create Flash File*/
	GPS_M8N_TX_LOG_ERASE     = 0x03,  /*Erase Flash File*/
	GPS_M8N_TX_LOG_FINDTIME  = 0x0E,  /*Find entry number index corresponding to time*/
	GPS_M8N_TX_LOG_RETRIEVE  = 0x09,  /*Retrieve log entry data from receiver*/	
	GPS_M8N_TX_LOG_STRING    = 0x04,  /*String*/

	/*MGA*/
	GPS_M8N_TX_MGA_INI_EOP   = 0x40,  /*INI(Initial Assistance), EOP(Eearth Orientation Parameters)*/
	
	/*NAV*/
	GPS_M8N_TX_NAV_RESETODO  = 0x10,  /*Reset Odometer*/
	GPS_M8N_TX_NAV_PVT       = 0x07,  /*Navigation PVT Solution*/	
	
	/*RXM*/
	GPS_M8N_TX_RXM_PMREQ     = 0x41,  /*Power Mode Request*/
	
	/*TIM*/
	GPS_M8N_TX_TIM_HOC       = 0x17,  /*Host Oscilator Control*/
	GPS_M8N_TX_TIM_SMEAS     = 0x13,  /*Source Measurement*/
	GPS_M8N_TX_TIM_VCOCAL    = 0x15,  /*VCO Calibration*/
	
	/*UPD*/
	GPS_M8N_TX_UPD_SOS       = 0x14,  /*Save On Shutdown*/
}GPS_M8N_TX_MESSAGE_ID;

/*GPS_M8N发送协议帧*/
typedef struct
{
	u8 					  SY_NC_CHAR1;
	u8 					  SY_NC_CHAR2;	
	GPS_M8N_TX_CLASS_ID   CLASS_ID;
	GPS_M8N_TX_MESSAGE_ID MESSAGE_ID;
	u8 					  LENTH_LOW;
	u8 					  LENTH_HIGH;
	u8 					  PAYLOAD[GPS_M8N_TX_PAYLOAD_LENTH];
	u8 					  checkA;
	u8 					  checkB;
}GpsM8nCmdTxFrame;

/*GPS_M8N PVT接收协议帧*/
typedef struct
{
	u8 					  SY_NC_CHAR1;
	u8 					  SY_NC_CHAR2;	
	GPS_M8N_TX_CLASS_ID   CLASS_ID;
	GPS_M8N_TX_MESSAGE_ID MESSAGE_ID;
	u8 					  LENTH_LOW;
	u8 					  LENTH_HIGH;
	u8 					  PAYLOAD[GPS_M8N_RX_PVT_PAYLOAD_LENTH];
	u8 					  checkA;
	u8 					  checkB;	
}GpsM8nRxPvtFrame;

/*GPS时间*/
typedef struct
{
	u16 year;	/*年*/
	u8  month;	/*月*/
	u8  day;	/*日*/
	u8  hour;	/*时*/
	u8  minute; /*分*/
	u8  second; /*秒*/
}GPS_Time;

/*Longitude & Latitude*/
typedef struct
{
	s32 lon;
	s32 lat;
}GPS_Coordinate_s4;

typedef struct
{
	fp64 lon;
	fp64 lat;
}GPS_Coordinate_f8;

/*Horizontal & Vertical accuracy estimate*/
typedef struct
{
	u32 hAcc;
	u32 vAcc;	
}GPS_HV_Accuracy_u4;

typedef struct
{
	fp32 hAcc;
	fp32 vAcc;	
}GPS_HV_Accuracy_f4;

/*NED velocity*/
typedef struct
{
	s32 velN;	/*north velocity*/
	s32 velE;	/*east velocity*/
	s32 velD;	/*down velocity*/
}GPS_NED_Velocity_s4;

typedef struct
{
	fp32 velN;	/*north velocity*/
	fp32 velE;	/*east velocity*/
	fp32 velD;	/*down velocity*/
}GPS_NED_Velocity_f4;

/**/
/*GPS 导航系速度*/
typedef struct
{
	u32 east;	/*东*/
	u32 north; /*北*/
	u32 up;		/*天*/
}NavSystem_Speed_u32;

/*位置估算精度*/
typedef struct
{
	fp32 horizontal;	/*水平方向*/
	fp32 vertical;		/*垂直方向*/
}PosEstimateAccuracy_Str;

/*Fix Status Flags*/
typedef struct
{
	u8 gnssFixOK:1;	   /*A valid fix (i.e within DOP & accuracy masks)*/
	u8 diffSoln:1;	   /*1 if differential corrections were applied*/	
	u8 psmState:3;	   /*Power Save Mode state*/		
    u8 headVehValid:1; /*Heading of vehicle is valid*/
}Fix_Status_Flag;

/*GPS_M8N PVT数据*/
typedef struct
{
	GPS_Time 		   	    UtcTime;		/*Utc*/
	GPS_M8N_POS_FIX_TYPE    POS_FIX_TYPE;	/*GNSSfix Type, range 0..5*/
	Fix_Status_Flag         FixStatusFlag;	/*Fix Status Flags*/
	u8 					    satelliteNbr;	/*Number of satellites used in Nav Solution*/
	GPS_Coordinate_s4       Coordinate;		/*Longitude & Latitude 10^7*/
	s32 		  		    hMSL;			/*Height above mean sea level*/
	GPS_HV_Accuracy_u4		HV_Accuracy;	/*Horizontal accuracy estimate & Vertical accuracy estimate*/
	GPS_NED_Velocity_s4 	NED_Velocity;	/*NED north & ease & down velocity*/	
	s32		  		   	    gSpeed;		    /*Ground Speed (2-D)*/
	s32		 		    	headMot;		/*Heading of motion (2-D)*/
	u32						sAcc;			/*Speed accuracy estimate*/
	u16						pDOP;			/*Position DOP*/
}GpsM8nPvtData;

/*GPS数据更新标志*/
typedef enum
{
	GPS_DATA_UPDATE_SUCC = 0,	
	GPS_DATA_UPDATE_FAIL = 1,	
}GPS_DATA_UPDATE_STATUS;

/*数据错位后的纠正*/
typedef struct
{
	u16 backByteNbr;  	/*截取位置往后的字节数*/
	u16 frontByteNbr; 	/*截取位置往前的字节数*/
	u8 *pStart;			/*数组在内存中的起始地址*/
	u8 *pStitch;		/*拼接点*/
	u8 *pHead1;			/*PVT数据中Head1*/
	u8 *pHead2;			/*PVT数据中Head2*/	
}GpsDataReform;

typedef struct
{
	MSP_Uart        				   *UartMaster;  		    /*串口*/
	GpsM8nPvtData   	               PvtData;	    	    	/*PVT数据*/
	volatile GPS_DATA_UPDATE_STATUS    UPDATE_STATUS;			/*数据更新标志*/
}GpsM8N;

/*GPS初始化:PVT, 10Hz, 38400*/
SYS_RETSTATUS bsp_GPS_M8N_Init(GpsM8N *gpsM8N);

/*解析更新GPS数据*/
SYS_RETSTATUS bsp_GPS_M8N_PVT_Parse(GpsM8N *gpsM8N, GpsM8nRxPvtFrame *pvtFrame, u8 *rxBuff);

/*构建指令*/
void bsp_GPS_M8N_Make_Command(GpsM8nCmdTxFrame *txFrame, GPS_M8N_TX_CLASS_ID classId, GPS_M8N_TX_MESSAGE_ID messageId, u8 *payload, u8 cmdLen, u8 *sendBuff);

/*数据偏移重组*/
SYS_RETSTATUS bsp_GPS_M8N_Reform_PVT_Data(GpsDataReform *DataReform, u8 *rxBuff);


extern GpsM8N g_sGpsM8N;
extern GpsM8nCmdTxFrame g_sGpsM8nCmdTxFrame;     /*GPS M8N 发送协议帧*/
extern GpsM8nRxPvtFrame g_sGpsM8nRxPvtFrame;     /*GPS M8N PVT 接收协议帧*/
extern u8 g_GpsM8nTxBuff[GPS_M8N_TX_BUFF_LENTH]; /*串口发送buff*/
extern u8 g_GpsM8nRxBuff[GPS_M8N_RX_BUFF_LENTH]; /*串口接收buff*/

#endif
