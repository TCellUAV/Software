#ifndef _BSP_ULTR_RCWL1603_H_
#define _BSP_ULTR_RCWL1603_H_

#include "bsp_ULTR_RCWL1603_CMD.h"
#include "sys_Platform.h"
#include "msp_GPIO.h"
#include "msp_UART.h"
#include "sys_McuInit.h"

typedef struct
{
	u8 i;
}BSP_RCWL1603;

/*超声波模块初始化*/
SYS_RETSTATUS bSP_ULTR_RCWL1603_Init(BSP_RCWL1603 *rcwl1603);

extern BSP_RCWL1603 g_sRCWL1603;

#endif


//	MSP_Uart			*UartMaster;			/*超声波串口模式*/
//	u8 				    rxDataBuff[2];			/*接收Buff*/
//	u8 				    txCmd;					/*发送指令*/	
//	fp32				Distance;				/*测量距离值(mm)(DisDataBuff[0] << 8 | DisDataBuff[1])*/
//	u8 					Temperature;	
//	u16					FirmwareVersion;		/*超声波模块版本号*/
//	u8					GetTempCmd;				/*获取温度值命令*/
//	u8 					GetDistanceCmd;			/*获取测量距离值命令*/
//	DATA_READY_STATUS	TemperatureDataStatus;	/*温度数据就绪状态*/
//	DATA_READY_STATUS	DistanceDataStatus;		/*测距数据就绪状态*/

#define ULTR_US100_TX_BUFF_LENTH	(1)
#define ULTR_US100_RX_BUFF_LENTH	(2)

//typedef struct
//{
//	MSP_Uart			*UartMaster;			/*超声波串口模式*/
//	fp32				distance;				/*测量距离值(mm)(DisDataBuff[0] << 8 | DisDataBuff[1])*/
//	u16 				temperature;			/*温度值*/
//	DATA_READY_STATUS	temperatureDataStatus;	/*温度数据就绪状态*/
//	DATA_READY_STATUS	distanceDataStatus;		/*测距数据就绪状态*/
//}BSP_US100;

///*超声波模块初始化*/
//SYS_RETSTATUS bSP_ULTR_US100_Init(BSP_US100 *us100);

///*向超声波模块发送命令*/
//void BSP_Ultrasonic_Send_Command(BSP_US100 *us100);

///*从超声波模块接收应答数据*/
//u8 BSP_Ultrasonic_Recv_AckData(BSP_US100 *us100);

///*命令超声波开始测量模块温度*/
//void BSP_Ultrasonic_Start_Meas_Temperature(BSP_US100 *us100);

///*命令超声波开始测量距离*/
//void BSP_Ultrasonic_Start_Meas_Distance(BSP_US100 *us100);

///*获取超声波模块温度*/
//u16 BSP_Ultrasonic_Get_Temperature(BSP_US100 *us100);

///*获取超声波模块距离*/
//fp32 BSP_Ultrasonic_Get_Distance(BSP_US100 *us100);

//extern BSP_US100 g_sUs100;

///*串口发送Buff*/
//extern u8 g_Us100TxBuff[ULTR_US100_TX_BUFF_LENTH];

///*串口接收Buff*/
//extern u8 g_Us100RxBuff[ULTR_US100_RX_BUFF_LENTH];
