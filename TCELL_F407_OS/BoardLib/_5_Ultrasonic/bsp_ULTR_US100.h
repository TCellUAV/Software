#ifndef _BSP_ULTR_US100_H_
#define _BSP_ULTR_US100_H_

#include "bsp_ULTR_US100_CMD.h"
#include "sys_Platform.h"
#include "msp_GPIO.h"
#include "msp_UART.h"
#include "sys_McuInit.h"

#define ULTR_US100_TX_BUFF_LENTH	(1)
#define ULTR_US100_RX_BUFF_LENTH	(2)

/*US100数据更新标志*/
typedef enum
{
	US100_UPDATE_FAIL = 0,	
	US100_UPDATE_SUCC = 1,
}US100_UPDATE_STATUS;

typedef struct
{
	MSP_Uart			         *UartMaster;	/*超声波串口模式*/
	s16				             distance;		/*测量距离值(mm)(DisDataBuff[0] << 8 | DisDataBuff[1])*/
	s16						     temperature;	/*测量温度值*/
	volatile US100_UPDATE_STATUS UPDATE_STATUS;	/*数据更新状态*/
}BSP_US100;

/*超声波模块初始化*/
SYS_RETSTATUS bsp_US100_Init(BSP_US100 *us100);

/*从超声波模块接收数据并解析*/
SYS_RETSTATUS bsp_US100_Data_Parse(BSP_US100 *us100);

/*命令超声波开始测量距离*/
void bsp_US100_Start_Meas_Distance(BSP_US100 *us100);

/*获取超声波模块距离*/
s16 bsp_US100_Get_Distance(BSP_US100 *us100);


extern BSP_US100 g_sUs100;

/*串口发送Buff*/
extern u8 g_Us100TxBuff[ULTR_US100_TX_BUFF_LENTH];

/*串口接收Buff*/
extern u8 g_Us100RxBuff[ULTR_US100_RX_BUFF_LENTH];	

#endif
