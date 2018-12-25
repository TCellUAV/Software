#include "VCAN_DT.h"
#include "msp_UART.h"
#include "sys_Debug.h"
#include "sys_McuInit.h"

/*=== 山外上位机官方协议 ===*/

/*=== 下位机数据上传函数 ===*/
/*不同数据类型填充发送Buff*/
u16 VCAN_DT_Wave_Data_Fill_TxBuff(VCAN_WAVE_DATA_TYPE DATA_TYPE, u8 channleNbr, void *srcBuff, u8 *txBuff)
{
	u8 i;
	u16 txBuffLenth = 0;
	
	switch(DATA_TYPE)
	{
		case VCAN_WAVE_DATA_S8:
		{
			VCAN_SendWave_s8 vcan_SendWave_s8 = {0};
			s8 *ps8 = srcBuff;
			
			/*构建数据帧*/
			vcan_SendWave_s8.msgId_Head       = VCAN_MSG_WAVE;
			vcan_SendWave_s8.msgIdBitNot_Head = (u8)(~VCAN_MSG_WAVE);
			
			for (i = 0; i < channleNbr; i++)
			{
				vcan_SendWave_s8.dataBuff[i] = ps8[i]; /*1Byte*/
			}
			
			vcan_SendWave_s8.msgIdBitNot_Tail = (u8)(~VCAN_MSG_WAVE);
			vcan_SendWave_s8.msgId_Tail       = VCAN_MSG_WAVE;			
		
			vcan_SendWave_s8.pHead     = (u8*)&vcan_SendWave_s8.msgId_Head;
			vcan_SendWave_s8.pDataBuff = (s8*)&vcan_SendWave_s8.dataBuff;	
			vcan_SendWave_s8.pTail     = (u8*)&vcan_SendWave_s8.msgIdBitNot_Tail;

			/*copy head*/
			memcpy(txBuff, vcan_SendWave_s8.pHead, 2);
			/*copy data*/
			memcpy(txBuff + 2, vcan_SendWave_s8.pDataBuff, sizeof(s8) * channleNbr);
			/*copy tail*/
			memcpy(txBuff + 2 + sizeof(s8) * channleNbr, vcan_SendWave_s8.pTail, 2);		

			/*发送数组长度*/
			txBuffLenth = 2 + sizeof(s8) * channleNbr + 2;
		}break;

		case VCAN_WAVE_DATA_U8:
		{
			VCAN_SendWave_u8 vcan_SendWave_u8 = {0};
			u8 *pu8 = srcBuff;
			
			/*构建数据帧*/
			vcan_SendWave_u8.msgId_Head       = VCAN_MSG_WAVE;
			vcan_SendWave_u8.msgIdBitNot_Head = (u8)(~VCAN_MSG_WAVE);
			
			for (i = 0; i < channleNbr; i++)
			{
				vcan_SendWave_u8.dataBuff[i] = pu8[i]; /*1Byte*/
			}
			
			vcan_SendWave_u8.msgIdBitNot_Tail = (u8)(~VCAN_MSG_WAVE);
			vcan_SendWave_u8.msgId_Tail       = VCAN_MSG_WAVE;			
		
			vcan_SendWave_u8.pHead     = (u8*)&vcan_SendWave_u8.msgId_Head;
			vcan_SendWave_u8.pDataBuff = (u8*)&vcan_SendWave_u8.dataBuff;	
			vcan_SendWave_u8.pTail     = (u8*)&vcan_SendWave_u8.msgIdBitNot_Tail;

			/*copy head*/
			memcpy(txBuff, vcan_SendWave_u8.pHead, 2);
			/*copy data*/
			memcpy(txBuff + 2, vcan_SendWave_u8.pDataBuff, sizeof(u8) * channleNbr);
			/*copy tail*/
			memcpy(txBuff + 2 + sizeof(u8) * channleNbr, vcan_SendWave_u8.pTail, 2);
			
			/*发送数组长度*/
			txBuffLenth = 2 + sizeof(u8) * channleNbr + 2;			
		}break;

		case VCAN_WAVE_DATA_S16:
		{
			VCAN_SendWave_s16 vcan_SendWave_s16 = {0};
			s16 *ps16 = srcBuff;
			
			/*构建数据帧*/
			vcan_SendWave_s16.msgId_Head       = VCAN_MSG_WAVE;
			vcan_SendWave_s16.msgIdBitNot_Head = (u8)(~VCAN_MSG_WAVE);
			
			for (i = 0; i < channleNbr; i++)
			{
				vcan_SendWave_s16.dataBuff[i] = ps16[i]; /*1Byte*/
			}
			
			vcan_SendWave_s16.msgIdBitNot_Tail = (u8)(~VCAN_MSG_WAVE);
			vcan_SendWave_s16.msgId_Tail       = VCAN_MSG_WAVE;			
		
			vcan_SendWave_s16.pHead     = (u8*)&vcan_SendWave_s16.msgId_Head;
			vcan_SendWave_s16.pDataBuff = (s16*)&vcan_SendWave_s16.dataBuff;	
			vcan_SendWave_s16.pTail     = (u8*)&vcan_SendWave_s16.msgIdBitNot_Tail;

			/*copy head*/
			memcpy(txBuff, vcan_SendWave_s16.pHead, 2);
			/*copy data*/
			memcpy(txBuff + 2, vcan_SendWave_s16.pDataBuff, sizeof(s16) * channleNbr);
			/*copy tail*/
			memcpy(txBuff + 2 + sizeof(s16) * channleNbr, vcan_SendWave_s16.pTail, 2);	
			
			/*发送数组长度*/
			txBuffLenth = 2 + sizeof(s16) * channleNbr + 2;			
		}break;

		case VCAN_WAVE_DATA_U16:
		{
			VCAN_SendWave_u16 vcan_SendWave_u16 = {0};
			u16 *pu16 = srcBuff;
			
			/*构建数据帧*/
			vcan_SendWave_u16.msgId_Head       = VCAN_MSG_WAVE;
			vcan_SendWave_u16.msgIdBitNot_Head = (u8)(~VCAN_MSG_WAVE);
			
			for (i = 0; i < channleNbr; i++)
			{
				vcan_SendWave_u16.dataBuff[i] = pu16[i]; /*1Byte*/
			}
			
			vcan_SendWave_u16.msgIdBitNot_Tail = (u8)(~VCAN_MSG_WAVE);
			vcan_SendWave_u16.msgId_Tail       = VCAN_MSG_WAVE;			
		
			vcan_SendWave_u16.pHead     = (u8*)&vcan_SendWave_u16.msgId_Head;
			vcan_SendWave_u16.pDataBuff = (u16*)&vcan_SendWave_u16.dataBuff;	
			vcan_SendWave_u16.pTail     = (u8*)&vcan_SendWave_u16.msgIdBitNot_Tail;

			/*copy head*/
			memcpy(txBuff, vcan_SendWave_u16.pHead, 2);
			/*copy data*/
			memcpy(txBuff + 2, vcan_SendWave_u16.pDataBuff, sizeof(u16) * channleNbr);
			/*copy tail*/
			memcpy(txBuff + 2 + sizeof(u16) * channleNbr, vcan_SendWave_u16.pTail, 2);	

			/*发送数组长度*/
			txBuffLenth = 2 + sizeof(u16) * channleNbr + 2;			
		}break;

		case VCAN_WAVE_DATA_S32:
		{
			VCAN_SendWave_s32 vcan_SendWave_s32 = {0};
			s32 *ps32 = srcBuff;
			
			/*构建数据帧*/
			vcan_SendWave_s32.msgId_Head       = VCAN_MSG_WAVE;
			vcan_SendWave_s32.msgIdBitNot_Head = (u8)(~VCAN_MSG_WAVE);
			
			for (i = 0; i < channleNbr; i++)
			{
				vcan_SendWave_s32.dataBuff[i] = ps32[i]; /*4Byte*/
			}
			
			vcan_SendWave_s32.msgIdBitNot_Tail = (u8)(~VCAN_MSG_WAVE);
			vcan_SendWave_s32.msgId_Tail       = VCAN_MSG_WAVE;			
		
			vcan_SendWave_s32.pHead     = (u8*)&vcan_SendWave_s32.msgId_Head;
			vcan_SendWave_s32.pDataBuff = (s32*)&vcan_SendWave_s32.dataBuff;	
			vcan_SendWave_s32.pTail     = (u8*)&vcan_SendWave_s32.msgIdBitNot_Tail;

			/*copy head*/
			memcpy(txBuff, vcan_SendWave_s32.pHead, 2);
			/*copy data*/
			memcpy(txBuff + 2, vcan_SendWave_s32.pDataBuff, sizeof(s32) * channleNbr);
			/*copy tail*/
			memcpy(txBuff + 2 + sizeof(s32) * channleNbr, vcan_SendWave_s32.pTail, 2);	
			
			/*发送数组长度*/
			txBuffLenth = 2 + sizeof(s32) * channleNbr + 2;			
		}break;

		case VCAN_WAVE_DATA_U32:
		{
			VCAN_SendWave_u32 vcan_SendWave_u32 = {0};
			u32 *pu32 = srcBuff;
			
			/*构建数据帧*/
			vcan_SendWave_u32.msgId_Head       = VCAN_MSG_WAVE;
			vcan_SendWave_u32.msgIdBitNot_Head = (u8)(~VCAN_MSG_WAVE);
			
			for (i = 0; i < channleNbr; i++)
			{
				vcan_SendWave_u32.dataBuff[i] = pu32[i]; /*1Byte*/
			}
			
			vcan_SendWave_u32.msgIdBitNot_Tail = (u8)(~VCAN_MSG_WAVE);
			vcan_SendWave_u32.msgId_Tail       = VCAN_MSG_WAVE;			
		
			vcan_SendWave_u32.pHead     = (u8*)&vcan_SendWave_u32.msgId_Head;
			vcan_SendWave_u32.pDataBuff = (u32*)&vcan_SendWave_u32.dataBuff;	
			vcan_SendWave_u32.pTail     = (u8*)&vcan_SendWave_u32.msgIdBitNot_Tail;

			/*copy head*/
			memcpy(txBuff, vcan_SendWave_u32.pHead, 2);
			/*copy data*/
			memcpy(txBuff + 2, vcan_SendWave_u32.pDataBuff, sizeof(u32) * channleNbr);
			/*copy tail*/
			memcpy(txBuff + 2 + sizeof(u32) * channleNbr, vcan_SendWave_u32.pTail, 2);
			
			/*发送数组长度*/
			txBuffLenth = 2 + sizeof(u32) * channleNbr + 2;			
		}break;

		case VCAN_WAVE_DATA_FP32:
		{
			VCAN_SendWave_fp32 vcan_SendWave_fp32 = {0};
			fp32 *pfp32 = srcBuff;
			
			/*构建数据帧*/
			vcan_SendWave_fp32.msgId_Head       = VCAN_MSG_WAVE;
			vcan_SendWave_fp32.msgIdBitNot_Head = (u8)(~VCAN_MSG_WAVE);
			
			for (i = 0; i < channleNbr; i++)
			{
				vcan_SendWave_fp32.dataBuff[i] = pfp32[i]; /*1Byte*/
			}
			
			vcan_SendWave_fp32.msgIdBitNot_Tail = (u8)(~VCAN_MSG_WAVE);
			vcan_SendWave_fp32.msgId_Tail       = VCAN_MSG_WAVE;			
		
			vcan_SendWave_fp32.pHead     = (u8*)&vcan_SendWave_fp32.msgId_Head;
			vcan_SendWave_fp32.pDataBuff = (fp32*)&vcan_SendWave_fp32.dataBuff;	
			vcan_SendWave_fp32.pTail     = (u8*)&vcan_SendWave_fp32.msgIdBitNot_Tail;

			/*copy head*/
			memcpy(txBuff, vcan_SendWave_fp32.pHead, 2);
			/*copy data*/
			memcpy(txBuff + 2, vcan_SendWave_fp32.pDataBuff, sizeof(fp32) * channleNbr);
			/*copy tail*/
			memcpy(txBuff + 2 + sizeof(fp32) * channleNbr, vcan_SendWave_fp32.pTail, 2);
			
			/*发送数组长度*/
			txBuffLenth = 2 + sizeof(fp32) * channleNbr + 2;			
		}break;

		case VCAN_WAVE_DATA_FP64:
		{
			VCAN_SendWave_fp64 vcan_SendWave_fp64 = {0};
			fp64 *pfp64 = srcBuff;
			
			/*构建数据帧*/
			vcan_SendWave_fp64.msgId_Head       = VCAN_MSG_WAVE;
			vcan_SendWave_fp64.msgIdBitNot_Head = (u8)(~VCAN_MSG_WAVE);
			
			for (i = 0; i < channleNbr; i++)
			{
				vcan_SendWave_fp64.dataBuff[i] = pfp64[i]; /*1Byte*/
			}
			
			vcan_SendWave_fp64.msgIdBitNot_Tail = (u8)(~VCAN_MSG_WAVE);
			vcan_SendWave_fp64.msgId_Tail       = VCAN_MSG_WAVE;			
		
			vcan_SendWave_fp64.pHead     = (u8*)&vcan_SendWave_fp64.msgId_Head;
			vcan_SendWave_fp64.pDataBuff = (fp64*)&vcan_SendWave_fp64.dataBuff;	
			vcan_SendWave_fp64.pTail     = (u8*)&vcan_SendWave_fp64.msgIdBitNot_Tail;

			/*copy head*/
			memcpy(txBuff, vcan_SendWave_fp64.pHead, 2);
			/*copy data*/
			memcpy(txBuff + 2, vcan_SendWave_fp64.pDataBuff, sizeof(fp64) * channleNbr);
			/*copy tail*/
			memcpy(txBuff + 2 + sizeof(fp64) * channleNbr, vcan_SendWave_fp64.pTail, 2);
			
			/*发送数组长度*/
			txBuffLenth = 2 + sizeof(fp64) * channleNbr + 2;
		}break;

		default:break;
	}
	
	return txBuffLenth; /*返回发送Buff长度*/
}

/*下位机上传上位机*/
void VCAN_DT_Send_Data(u8 *dataToSend, u8 length, MSP_UART_WORK_MODE UART_TX_MODE)
{
	/*=== UART发送方式 ===*/
	msp_Uart_Send_Data(&g_sDebugUart, g_sDebugUart.pTxBuff, DEBUG_TX_BUFF_SIZE, UART_TX_MODE);	
}
