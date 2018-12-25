#ifndef _VCAN_DT_H_
#define _VCAN_DT_H_

#include "sys_Platform.h"
#include "msp_Uart.h"

#define VCAN_DT_MAX_DATA_CHANNLE_NBR	(8)

/*山外上位机消息ID*/
typedef enum
{
	VCAN_MSG_WAVE = 0x03, /*虚拟示波器0x03*/
}VCAN_MSG_ID;

/*虚拟示波器数据发送类型*/
typedef enum
{
	VCAN_WAVE_DATA_S8   = 0, /*signed char*/
	VCAN_WAVE_DATA_U8   = 1, /*unsigned char*/
	VCAN_WAVE_DATA_S16  = 2, /*signed short*/
	VCAN_WAVE_DATA_U16  = 3, /*unsigned short*/
	VCAN_WAVE_DATA_S32  = 4, /*signed int*/
	VCAN_WAVE_DATA_U32  = 5, /*unsigned int*/
	VCAN_WAVE_DATA_FP32 = 6, /*float*/
	VCAN_WAVE_DATA_FP64 = 7, /*double*/
}VCAN_WAVE_DATA_TYPE;

/*int8_t类型*/
typedef struct
{
	u8 msgId_Head;
	u8 msgIdBitNot_Head;
	s8 dataBuff[VCAN_DT_MAX_DATA_CHANNLE_NBR];
	u8 msgIdBitNot_Tail;	
	u8 msgId_Tail;

	u8 *pHead;
	s8 *pDataBuff;
	u8 *pTail;
}VCAN_SendWave_s8;

/*uint8_t类型*/
typedef struct
{
	u8 msgId_Head;
	u8 msgIdBitNot_Head;
	u8 dataBuff[8];
	u8 msgIdBitNot_Tail;	
	u8 msgId_Tail;
	
	u8 *pHead;
	u8 *pDataBuff;
	u8 *pTail;
}VCAN_SendWave_u8;

/*int16_t类型*/
typedef struct
{
	u8 msgId_Head;
	u8 msgIdBitNot_Head;
	s16 dataBuff[8];
	u8 msgIdBitNot_Tail;	
	u8 msgId_Tail;
	
	u8 *pHead;
	s16 *pDataBuff;
	u8 *pTail;
}VCAN_SendWave_s16;

/*uint16_t类型*/
typedef struct
{
	u8 msgId_Head;
	u8 msgIdBitNot_Head;
	u16 dataBuff[8];
	u8 msgIdBitNot_Tail;	
	u8 msgId_Tail;
	
	u8 *pHead;
	u16 *pDataBuff;
	u8 *pTail;
}VCAN_SendWave_u16;

/*int32_t类型*/
typedef struct
{
	u8 msgId_Head;
	u8 msgIdBitNot_Head;
	s32 dataBuff[8];
	u8 msgIdBitNot_Tail;	
	u8 msgId_Tail;
	
	u8 *pHead;
	s32 *pDataBuff;
	u8 *pTail;
}VCAN_SendWave_s32;

/*uint32_t类型*/
typedef struct
{
	u8 msgId_Head;
	u8 msgIdBitNot_Head;
	u32 dataBuff[8];
	u8 msgIdBitNot_Tail;	
	u8 msgId_Tail;
	
	u8 *pHead;
	u32 *pDataBuff;
	u8 *pTail;
}VCAN_SendWave_u32;

/*float类型*/
typedef struct
{
	u8 msgId_Head;
	u8 msgIdBitNot_Head;
	fp32 dataBuff[8];
	u8 msgIdBitNot_Tail;	
	u8 msgId_Tail;
	
	u8 *pHead;
	fp32 *pDataBuff;
	u8 *pTail;
}VCAN_SendWave_fp32;

/*double类型*/
typedef struct
{
	u8 msgId_Head;
	u8 msgIdBitNot_Head;
	fp64 dataBuff[8];
	u8 msgIdBitNot_Tail;	
	u8 msgId_Tail;
	
	u8 *pHead;
	fp64 *pDataBuff;
	u8 *pTail;
}VCAN_SendWave_fp64;


/*=== 山外上位机官方协议 ===*/

/*=== 下位机数据上传函数 ===*/
/*不同数据类型填充发送Buff*/
u16 VCAN_DT_Wave_Data_Fill_TxBuff(VCAN_WAVE_DATA_TYPE DATA_TYPE, u8 channleNbr, void *srcBuff, u8 *txBuff);

/*下位机上传上位机*/
void VCAN_DT_Send_Data(u8 *dataToSend, u8 length, MSP_UART_WORK_MODE UART_TX_MODE);


#endif
