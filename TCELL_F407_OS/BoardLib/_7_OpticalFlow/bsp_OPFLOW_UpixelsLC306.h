#ifndef _BSP_OPFLOW_UPIXELSLC306_H_
#define _BSP_OPFLOW_UPIXELSLC306_H_

#include "bsp_OPFLOW_UPIXELSLC306_CMD.h"
#include "sys_Platform.h"
#include "msp_UART.h"
#include "sys_McuInit.h"

#define OPFLOW_UPIXELSLC306_INIT_CMC_PERIOD_TICKS_MS    (10)

#define OPFLOW_UPIXELSLC306_TX_BUFF_LENTH				(6)
#define OPFLOW_UPIXELSLC306_RX_BUFF_LENTH				(14)
#define OPFLOW_UPIXELSLC306_TX_PAYLOAD_LENTH  			(4)
#define OPFLOW_UPIXELSLC306_RX_PAYLOAD_LENTH  			(10)
#define OPFLOW_UPIXELSLC306_DATA_BUFF_LENTH				(14)

/*初始化无应答时,继续发送指令次数*/
#define OPFLOW_UPIXELSLC306_RESEND_CMD_MAX_TIMES		(5)

/*光流数据可用性*/
typedef enum
{
	OPFLOW_UPIXELSLC306_DATA_DISAVA = 0x00, /*光流数据不可用*/
	OPFLOW_UPIXELSLC306_DATA_AVA    = 0xF5,	/*光流数据可用*/
}OPFLOW_UPIXELSLC306_DATA_STATUS;

/*光流初始化应答状态*/
typedef enum
{
	OPFLOW_UPIXELSLC306_RESPONSE_FAIL = 0, /*光流未应答*/	
	OPFLOW_UPIXELSLC306_RESPONSE_SUCC = 1, /*光流应答*/	
}OPFLOW_UPIXELSLC306_RESPONSE_STATUS;

/*光流初始化状态*/
typedef enum
{
	OPFLOW_UPIXELSLC306_INIT_START  = 0,  /*光流初始化开始*/	
	OPFLOW_UPIXELSLC306_INIT_FINISH = 1, /*光流初始化结束*/		
	OPFLOW_UPIXELSLC306_INIT_NULL   = 0xff,
}OPFLOW_UPIXELSLC306_INIT_STATUS;

/*光流数据更新状态*/
typedef enum
{
	OPFLOW_UPIXELSLC306_UPDATE_FAIL = 0, /*未更新*/
	OPFLOW_UPIXELSLC306_UPDATE_SUCC  = 1, /*已更新*/
}OPFLOW_UPIXELSLC306_UPDATE_STATUS;

/*UpixelsLC306 发送指令帧*/
typedef struct
{
	u8 command;	/*指令*/
	u8 payload[OPFLOW_UPIXELSLC306_TX_PAYLOAD_LENTH]; /*指令内容buff*/
	u8 XOR;		/*buff内容异或*/
}OpFlowUpixelsLC306TxFrame;

/*UpixelsLC306 接收数据帧*/
typedef struct
{
	u8 head;									      /*帧头*/
	u8 lenth;										  /*数据包字节数(固定值 0x0A)*/
	u8 payload[OPFLOW_UPIXELSLC306_RX_PAYLOAD_LENTH]; /*指令内容buff*/
	u8 XOR;											  /*3-12字节异或校验位*/
	u8 tail;										  /*帧尾*/
}OpFlowUpixelsLC306RxFrame;


/*UpixelsLC306 数据解析*/
typedef struct
{
	s16 							xIntegral;			 /*X 像素点累计时间内的累加位移,(radians*10000)[除以 10000 乘以高度(mm)后为实际位移(mm)]*/
	s16 							yIntegral;			 /*Y 像素点累计时间内的累加位移,(radians*10000)[除以 10000 乘以高度(mm)后为实际位移(mm)]*/
	u16 							integrationTimespan; /*上一次发送光流数据到本次发送光流数据的累计时间(us)*/
	u16 							groundDistance;		 /*预留离地距离,默认为 999(0x03E7)*/
	OPFLOW_UPIXELSLC306_DATA_STATUS DATA_STATUS;		 /*光流数据可用状态*/
	u8 							    version;			 /*光流模块版本号*/
}OpFlowUpixelsLC306DataFrame;


/*数据错位后的纠正*/
typedef struct
{
	u16 backByteNbr;  	/*截取位置往后的字节数*/
	u16 frontByteNbr; 	/*截取位置往前的字节数*/
	u8 *pStart;			/*数组在内存中的起始地址*/
	u8 *pStitch;		/*拼接点*/
	u8 *pHead;			/*数据中Head*/
	u8 *fixLenth;		/*固定数据长度*/	
}OpFlowUpixelsLC306DataReform;

/*初始化应答比对*/
typedef struct
{
	u8 *pStart;			/*数组在内存中的起始地址*/
	u8 *pCmpData1;		/*第1个比对数据*/	
	u8 *pCmpData2;		/*第2个比对数据*/	
	u8 *pCmpData3;		/*第3个比对数据*/	
}OpFlowUpixelsLC306ResponseCmpReform;

typedef struct
{
	MSP_Uart 					            	 *UartMaster;  		/*串口*/
	volatile OPFLOW_UPIXELSLC306_RESPONSE_STATUS RESPONSE_STATUS;	/*初始化应答状态*/
	volatile OPFLOW_UPIXELSLC306_INIT_STATUS 	 INIT_STATUS;		/*初始化初始化状态*/
	volatile OPFLOW_UPIXELSLC306_UPDATE_STATUS   UPDATE_STATUS;		/*光流数据更新状态*/
	OpFlowUpixelsLC306DataFrame 		         OpFlowData;	    /*光流数据*/
}OpFlowUpixelsLC306;

/*Upixels LC306光流初始化:19200*/
SYS_RETSTATUS bsp_OPFLOW_UpixelsLC306_Init(OpFlowUpixelsLC306 *opFlowUpixelsLC306);

/*Upixels LC306应答数据解析*/
SYS_RETSTATUS bsp_OPFLOW_UpixelsLC306_Response_Data_Parse(u8 *rspBuff);

/*Upixels LC306位置数据解析*/
SYS_RETSTATUS bsp_OPFLOW_UpixelsLC306_POS_Data_Parse(OpFlowUpixelsLC306 *opFlowUpixelsLC306, OpFlowUpixelsLC306RxFrame *rxFrame, u8 *rxBuff);

/*Upixels LC306 构建指令*/
u8 bsp_OPFLOW_UpixelsLC306_Make_Command(OpFlowUpixelsLC306TxFrame *txFrame, OPFLOW_UPIXELSLC306_CFG_CMD CMD_TYPE, u8 *payload, u8 cmdLen, u8 *sendBuff);

/*数据偏移重组*/
SYS_RETSTATUS bsp_OPFLOW_UpixelsLC306_Reform_Data(OpFlowUpixelsLC306DataReform *DataReform, u8 *rxBuff);

/*从机应答查询*/
SYS_RETSTATUS bsp_OPFLOW_UpixelsLC306_Slave_Response(OpFlowUpixelsLC306ResponseCmpReform *ResponseCmp, u8 cmpBuff[]);


extern OpFlowUpixelsLC306 g_sOpFlowUpixelsLC306;
extern OpFlowUpixelsLC306TxFrame g_sOpFlowUpixelsLC306TxFrame;				/*OpFlowUpixelsLC306 发送协议帧*/
extern OpFlowUpixelsLC306RxFrame g_sOpFlowUpixelsLC306RxFrame;				/*OpFlowUpixelsLC306 接收协议帧*/
extern u8 g_OpFlowUpixelsLC306TxBuff[OPFLOW_UPIXELSLC306_TX_BUFF_LENTH];	/*串口发送buff*/
extern u8 g_OpFlowUpixelsLC306RxBuff[OPFLOW_UPIXELSLC306_RX_BUFF_LENTH];	/*串口接收buff*/

#endif
