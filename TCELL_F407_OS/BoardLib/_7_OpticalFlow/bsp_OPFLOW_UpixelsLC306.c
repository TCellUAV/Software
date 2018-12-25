#include "bsp_OPFLOW_UpixelsLC306.h"

OpFlowUpixelsLC306 g_sOpFlowUpixelsLC306 = 
{
	.RESPONSE_STATUS = OPFLOW_UPIXELSLC306_RESPONSE_FAIL,
	.INIT_STATUS     = OPFLOW_UPIXELSLC306_INIT_NULL,	
};

/*OpFlowUpixelsLC306 接收协议帧*/
OpFlowUpixelsLC306TxFrame g_sOpFlowUpixelsLC306TxFrame = {0};

/*OpFlowUpixelsLC306 接收协议帧*/
OpFlowUpixelsLC306RxFrame g_sOpFlowUpixelsLC306RxFrame = {0};

/*串口发送Buff*/
u8 g_OpFlowUpixelsLC306TxBuff[OPFLOW_UPIXELSLC306_TX_BUFF_LENTH] = {0};

/*串口接收Buff*/
u8 g_OpFlowUpixelsLC306RxBuff[OPFLOW_UPIXELSLC306_RX_BUFF_LENTH] = {0};

/*光流数据buff*/
u8 g_OpFlowUpixelsLC306DataBuff[OPFLOW_UPIXELSLC306_DATA_BUFF_LENTH] = {0};

/*数组重组*/
OpFlowUpixelsLC306DataReform g_sOpFlowUpixelsLC306DataReform = 
{
	.backByteNbr  = 0,
	.frontByteNbr = 0,
};

OpFlowUpixelsLC306DataReform *g_psOpFlowUpixelsLC306DataReform = &g_sOpFlowUpixelsLC306DataReform;

/*初始化应答检测*/
OpFlowUpixelsLC306ResponseCmpReform g_sOpFlowUpixelsLC306ResponseCmpReform = 
{
	.pStart    = g_OpFlowUpixelsLC306RxBuff,
	.pCmpData1 = g_OpFlowUpixelsLC306RxBuff,
	.pCmpData2 = g_OpFlowUpixelsLC306RxBuff + 1,	
	.pCmpData3 = g_OpFlowUpixelsLC306RxBuff + 2,		
};

OpFlowUpixelsLC306ResponseCmpReform *g_psOpFlowUpixelsLC306ResponseCmpReform = &g_sOpFlowUpixelsLC306ResponseCmpReform;

/*OPFLOW_UPIXELSLC306 MODULE_CFG 配置结果成功标志*/
const u8 OPFLOW_UPIXELSLC306_MODULE_CFG_SUCC[3] = 
{
	0xAB, 0x00, 0xAB,
};

/*OPFLOW_UPIXELSLC306 MODULE_CFG 配置结果成功标志*/
const u8 OPFLOW_UPIXELSLC306_SENSOR_CFG_SUCC[3] = 
{
	0xBB, 0x00, 0xBB,
};

/*0xAB(模块内部参数配置指令)*/
const u8 OPFLOW_UpixelsLC306_AB_Tab_Focus_Buff[4] = 
{
	0x96, 0x26, 0xbc, 0x50
};

/*0xAB(模块内部参数配置指令)*/
const u8 OPFLOW_UpixelsLC306_BB_Sensor_Cfg_Buff[161][2] = 
{
	/*地址, 数据*/
	0x12, 0x80,
	0x11, 0x30,
	0x1b, 0x06,
	0x6b, 0x43,
	0x12, 0x20,
	0x3a, 0x00,
	0x15, 0x02,
	0x62, 0x81,
	0x08, 0xa0,
	0x06, 0x68,
	0x2b, 0x20,
	0x92, 0x25,
	0x27, 0x97,
	0x17, 0x01,
	0x18, 0x79,
	0x19, 0x00,
	0x1a, 0xa0,
	0x03, 0x00,
	0x13, 0x00,
	0x01, 0x13,
	0x02, 0x20,
	0x87, 0x16,
	0x8c, 0x01,
	0x8d, 0xcc,
	0x13, 0x07,
	0x33, 0x10,
	0x34, 0x1d,
	0x35, 0x46,
	0x36, 0x40,
	0x37, 0xa4,
	0x38, 0x7c,
	0x65, 0x46,
	0x66, 0x46,
	0x6e, 0x20,
	0x9b, 0xa4,
	0x9c, 0x7c,
	0xbc, 0x0c,
	0xbd, 0xa4,
	0xbe, 0x7c,
	0x20, 0x09,
	0x09, 0x03,
	0x72, 0x2f,
	0x73, 0x2f,
	0x74, 0xa7,
	0x75, 0x12,
	0x79, 0x8d,
	0x7a, 0x00,
	0x7e, 0xfa,
	0x70, 0x0f,
	0x7c, 0x84,
	0x7d, 0xba,
	0x5b, 0xc2,
	0x76, 0x90,
	0x7b, 0x55,
	0x71, 0x46,
	0x77, 0xdd,
	0x13, 0x0f,
	0x8a, 0x10,
	0x8b, 0x20,
	0x8e, 0x21,
	0x8f, 0x40,
	0x94, 0x41,
	0x95, 0x7e,
	0x96, 0x7f,
	0x97, 0xf3,
	0x13, 0x07,
	0x24, 0x58,
	0x97, 0x48,
	0x25, 0x08,
	0x94, 0xb5,
	0x95, 0xc0,
	0x80, 0xf4,
	0x81, 0xe0,
	0x82, 0x1b,
	0x83, 0x37,
	0x84, 0x39,
	0x85, 0x58,
	0x86, 0xff,
	0x89, 0x15,
	0x8a, 0xb8,
	0x8b, 0x99,
	0x39, 0x98,
	0x3f, 0x98,
	0x90, 0xa0,
	0x91, 0xe0,
	0x40, 0x20,
	0x41, 0x28,
	0x42, 0x26,
	0x43, 0x25,
	0x44, 0x1f,
	0x45, 0x1a,
	0x46, 0x16,
	0x47, 0x12,
	0x48, 0x0f,
	0x49, 0x0d,
	0x4b, 0x0b,
	0x4c, 0x0a,
	0x4e, 0x08,
	0x4f, 0x06,
	0x50, 0x06,
	0x5a, 0x56,
	0x51, 0x1b,
	0x52, 0x04,
	0x53, 0x4a,
	0x54, 0x26,
	0x57, 0x75,
	0x58, 0x2b,
	0x5a, 0xd6,
	0x51, 0x28,
	0x52, 0x1e,
	0x53, 0x9e,
	0x54, 0x70,
	0x57, 0x50,
	0x58, 0x07,
	0x5c, 0x28,
	0xb0, 0xe0,
	0xb1, 0xc0,
	0xb2, 0xb0,
	0xb3, 0x4f,
	0xb4, 0x63,
	0xb4, 0xe3,
	0xb1, 0xf0,
	0xb2, 0xa0,
	0x55, 0x00,
	0x56, 0x40,
	0x96, 0x50,
	0x9a, 0x30,
	0x6a, 0x81,
	0x23, 0x33,
	0xa0, 0xd0,
	0xa1, 0x31,
	0xa6, 0x04,
	0xa2, 0x0f,
	0xa3, 0x2b,
	0xa4, 0x0f,
	0xa5, 0x2b,
	0xa7, 0x9a,
	0xa8, 0x1c,
	0xa9, 0x11,
	0xaa, 0x16,
	0xab, 0x16,
	0xac, 0x3c,
	0xad, 0xf0,
	0xae, 0x57,
	0xc6, 0xaa,
	0xd2, 0x78,
	0xd0, 0xb4,
	0xd1, 0x00,
	0xc8, 0x10,
	0xc9, 0x12,
	0xd3, 0x09,
	0xd4, 0x2a,
	0xee, 0x4c,
	0x7e, 0xfa,
	0x74, 0xa7,
	0x78, 0x4e,
	0x60, 0xe7,
	0x61, 0xc8,
	0x6d, 0x70,
	0x1e, 0x39,
	0x98, 0x1a,
};

/*Upixels LC306光流初始化*/
#if defined(HW_CUT__USE_OPTICFLOW)

SYS_RETSTATUS bsp_OPFLOW_UpixelsLC306_Init(OpFlowUpixelsLC306 *opFlowUpixelsLC306)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;	
	u8 i, txLenth = 0;
	vu8 reSendTimes = 0;
	
	/*串口初始化*/
	opFlowUpixelsLC306->UartMaster = &g_sOpticalFlowUart;
	
	sys_DelayMs(1);
	
	/*1.开始配置:0xAA*/
	/*构建指令*/
	txLenth = bsp_OPFLOW_UpixelsLC306_Make_Command(&g_sOpFlowUpixelsLC306TxFrame, OPFLOW_UPIXELSLC306_STATR_CFG, NULL, 0, g_OpFlowUpixelsLC306TxBuff);
	
	/*发送指令*/
	msp_Uart_Send_Data(opFlowUpixelsLC306->UartMaster, g_OpFlowUpixelsLC306TxBuff, txLenth, MSP_UART_POLL);	
	
	/*标记初始化开始*/
	opFlowUpixelsLC306->INIT_STATUS = OPFLOW_UPIXELSLC306_INIT_START;
	
	/*延时等待*/
	sys_DelayMs(OPFLOW_UPIXELSLC306_INIT_CMC_PERIOD_TICKS_MS);

	/*2.模块内部参数配置指令:0xAB*/
	/*构建指令*/
	txLenth = bsp_OPFLOW_UpixelsLC306_Make_Command(&g_sOpFlowUpixelsLC306TxFrame, OPFLOW_UPIXELSLC306_MODULE_CFG, (u8 *)&OPFLOW_UpixelsLC306_AB_Tab_Focus_Buff, 4, g_OpFlowUpixelsLC306TxBuff);
	
	/*发送指令*/
	msp_Uart_Send_Data(opFlowUpixelsLC306->UartMaster, g_OpFlowUpixelsLC306TxBuff, txLenth, MSP_UART_POLL);

/*判断指令是否生效(是否应答),不应答重复发送,最多发送5次*/
	while((opFlowUpixelsLC306->RESPONSE_STATUS != OPFLOW_UPIXELSLC306_RESPONSE_SUCC) && (reSendTimes < OPFLOW_UPIXELSLC306_RESEND_CMD_MAX_TIMES))
	{
		/*发送指令*/
		msp_Uart_Send_Data(opFlowUpixelsLC306->UartMaster, g_OpFlowUpixelsLC306TxBuff, txLenth, MSP_UART_POLL);	
		
		reSendTimes++;
		
		sys_DelayMs(OPFLOW_UPIXELSLC306_INIT_CMC_PERIOD_TICKS_MS);		
	}
	
	/*重发送次数大于最大允许重发次数,初始化失败*/
	if (reSendTimes >= OPFLOW_UPIXELSLC306_RESEND_CMD_MAX_TIMES)
	{
		return SYS_RET_FAIL;	/*标记为初始化失败*/
	}
	else
	{
		reSendTimes = 0;		
	}
	
	/*应答标记复位*/
	opFlowUpixelsLC306->RESPONSE_STATUS = OPFLOW_UPIXELSLC306_RESPONSE_FAIL;
	
	/*3.传感器参数配置指令:0xBB*/	
	for (i = 0; i < 161; i++)
	{
		/*构建指令*/
		txLenth = bsp_OPFLOW_UpixelsLC306_Make_Command(&g_sOpFlowUpixelsLC306TxFrame, OPFLOW_UPIXELSLC306_SENSOR_CFG, (u8 *)&OPFLOW_UpixelsLC306_BB_Sensor_Cfg_Buff[i], 2, g_OpFlowUpixelsLC306TxBuff);
	
		/*发送指令*/
		msp_Uart_Send_Data(opFlowUpixelsLC306->UartMaster, g_OpFlowUpixelsLC306TxBuff, txLenth, MSP_UART_POLL);
	
		/*判断指令是否生效(是否应答)*/
		while((opFlowUpixelsLC306->RESPONSE_STATUS != OPFLOW_UPIXELSLC306_RESPONSE_SUCC) && (reSendTimes <= OPFLOW_UPIXELSLC306_RESEND_CMD_MAX_TIMES))
		{
			/*发送指令*/
			msp_Uart_Send_Data(opFlowUpixelsLC306->UartMaster, g_OpFlowUpixelsLC306TxBuff, txLenth, MSP_UART_POLL);	
			
			reSendTimes++;
			
			sys_DelayMs(OPFLOW_UPIXELSLC306_INIT_CMC_PERIOD_TICKS_MS);		
		}
		
		/*重发送次数大于最大允许重发次数,初始化失败*/
		if (reSendTimes >= OPFLOW_UPIXELSLC306_RESEND_CMD_MAX_TIMES)
		{
			return SYS_RET_FAIL;	/*标记为初始化失败*/
		}
		else
		{
			reSendTimes = 0;		
		}
	}

	/*4.完成配置:0xDD*/
	/*构建指令*/
	txLenth = bsp_OPFLOW_UpixelsLC306_Make_Command(&g_sOpFlowUpixelsLC306TxFrame, OPFLOW_UPIXELSLC306_FINISH_CFG, NULL, 0, g_OpFlowUpixelsLC306TxBuff);
	
	/*发送指令*/
	msp_Uart_Send_Data(opFlowUpixelsLC306->UartMaster, g_OpFlowUpixelsLC306TxBuff, txLenth, MSP_UART_POLL);	

	/*标记初始化结束*/
	opFlowUpixelsLC306->INIT_STATUS = OPFLOW_UPIXELSLC306_INIT_FINISH;	
	
	return statusRet;		
}
#endif

/*Upixels LC306应答数据解析*/
SYS_RETSTATUS bsp_OPFLOW_UpixelsLC306_Response_Data_Parse(u8 *rspBuff)
{
	SYS_RETSTATUS RET_STATUS;
	
	if ((rspBuff[0] == 0xAB) || (rspBuff[0] == 0xBB))
	{
		if (rspBuff[1] == 0x00)
		{
			if ((rspBuff[2] == 0xAB) || (rspBuff[2] == 0xBB))
			{
				RET_STATUS = SYS_RET_SUCC;
			}
			else
			{
				RET_STATUS = SYS_RET_FAIL;
			}
		}
		else
		{
			RET_STATUS = SYS_RET_FAIL;			
		}
	}
	else
	{
		RET_STATUS = SYS_RET_FAIL;	
	}
	
	return RET_STATUS;
}

/*Upixels LC306位置数据解析*/
SYS_RETSTATUS bsp_OPFLOW_UpixelsLC306_POS_Data_Parse(OpFlowUpixelsLC306 *opFlowUpixelsLC306, OpFlowUpixelsLC306RxFrame *rxFrame, u8 *rxBuff)
{
	u8 i, check = 0;

	/*0.对数据进行重新截取拼装成一帧数据*/
	if (bsp_OPFLOW_UpixelsLC306_Reform_Data(&g_sOpFlowUpixelsLC306DataReform, rxBuff) != SYS_RET_SUCC)
	{
		/*标记OPTIC FLOW数据更新失败*/
		opFlowUpixelsLC306->UPDATE_STATUS = OPFLOW_UPIXELSLC306_UPDATE_FAIL;			
		
		return SYS_RET_FAIL;	/*未找到1帧数据*/
	}
	
	/*先拼凑截取点往后的数据*/
	memcpy(g_OpFlowUpixelsLC306DataBuff, g_psOpFlowUpixelsLC306DataReform->pHead, g_psOpFlowUpixelsLC306DataReform->backByteNbr);

	/*再拼凑截取点往前的数据*/	
	memcpy((g_OpFlowUpixelsLC306DataBuff + g_psOpFlowUpixelsLC306DataReform->backByteNbr), g_psOpFlowUpixelsLC306DataReform->pStitch, g_psOpFlowUpixelsLC306DataReform->frontByteNbr);
	
	/*将重组后的数据赋值给光流接收协议帧*/
	memcpy((u8 *)&g_sOpFlowUpixelsLC306RxFrame, g_OpFlowUpixelsLC306DataBuff, OPFLOW_UPIXELSLC306_DATA_BUFF_LENTH);	
	
	/*OpFlowUpixelsLC306DataBuff 清0*/
	memset(g_OpFlowUpixelsLC306DataBuff, 0, OPFLOW_UPIXELSLC306_DATA_BUFF_LENTH);
	
	/*1.对本帧数据有效性进行判断*/
	/*对帧头和帧尾有效性进行检测*/
	if ((rxFrame->head != 0xFE) || \
		(rxFrame->tail != 0x55))
	{
		/*标记OPTIC FLOW数据更新失败*/
		opFlowUpixelsLC306->UPDATE_STATUS = OPFLOW_UPIXELSLC306_UPDATE_FAIL;			
		
		return SYS_RET_FAIL;
	}

	/*对固定长度有效性进行检测*/
	if (rxFrame->lenth != 0x0A)
	{
		/*标记OPTIC FLOW数据更新失败*/
		opFlowUpixelsLC306->UPDATE_STATUS = OPFLOW_UPIXELSLC306_UPDATE_FAIL;			
		
		return SYS_RET_FAIL;
	}		
	
	/*对数据校验有效性进行检测*/
	/*校验位计算(payload异或)*/
	check = rxFrame->payload[0];
	
	for (i = 1; i < OPFLOW_UPIXELSLC306_RX_PAYLOAD_LENTH; i++)
	{
		check ^= *(((u8 *)&g_sOpFlowUpixelsLC306RxFrame) + 2 + i);
	}
	
	if (rxFrame->XOR != check)
	{
		/*标记OPTIC FLOW数据更新失败*/
		opFlowUpixelsLC306->UPDATE_STATUS = OPFLOW_UPIXELSLC306_UPDATE_FAIL;			
		
		return SYS_RET_FAIL;		
	}
	
	/*2.数据为有效数据,进行拼装,且注意安装方向调整符号*/
	/*X 像素点累计时间内的累加位移,(radians*10000)[除以 10000 乘以高度(mm)后为实际位移(mm)]*/
	opFlowUpixelsLC306->OpFlowData.xIntegral = (rxFrame->payload[1] << 8) + rxFrame->payload[0];

	/*Y 像素点累计时间内的累加位移,(radians*10000)[除以 10000 乘以高度(mm)后为实际位移(mm)]*/
	opFlowUpixelsLC306->OpFlowData.yIntegral = (rxFrame->payload[3] << 8) + rxFrame->payload[2];	
	
	/*上一次发送光流数据到本次发送光流数据的累计时间(s)*/
	opFlowUpixelsLC306->OpFlowData.integrationTimespan = (rxFrame->payload[5] << 8) + rxFrame->payload[4];

	/*预留离地距离,默认为 999(0x03E7)*/
	opFlowUpixelsLC306->OpFlowData.groundDistance = (rxFrame->payload[7] << 8) + rxFrame->payload[6];
	
	 /*光流数据可用状态*/
	opFlowUpixelsLC306->OpFlowData.DATA_STATUS = (OPFLOW_UPIXELSLC306_DATA_STATUS)rxFrame->payload[8];
	
	/*光流版本号*/
	opFlowUpixelsLC306->OpFlowData.version = rxFrame->payload[9];
	
	/*标记光流数据为更新状态*/
	opFlowUpixelsLC306->UPDATE_STATUS = OPFLOW_UPIXELSLC306_UPDATE_SUCC;
	
	return SYS_RET_SUCC;	
}

/*构建指令*/
u8 bsp_OPFLOW_UpixelsLC306_Make_Command(OpFlowUpixelsLC306TxFrame *txFrame, OPFLOW_UPIXELSLC306_CFG_CMD CMD_TYPE, u8 *payload, u8 cmdLen, u8 *sendBuff)
{
	u8 i, txLenth = 0;	/*发送buff长度*/
	
	/*判断配置指令类型*/
	switch(CMD_TYPE)
	{
		case OPFLOW_UPIXELSLC306_STATR_CFG:	 /*开启配置指令*/
		{
			/*赋值txFrame->command*/				
			txFrame->command = OPFLOW_UPIXELSLC306_STATR_CFG;	/*指令*/
			
			/*计算发送字节数据*/	
			txLenth = 1 + cmdLen + 0;
			
			/*赋值txbuff*/
			memcpy(g_OpFlowUpixelsLC306TxBuff, (u8 *)&g_sOpFlowUpixelsLC306TxFrame.command, 1);									
		}break;	

		case OPFLOW_UPIXELSLC306_MODULE_CFG: /*模块内部参数配置指令*/
		{
			/*赋值txFrame->command*/			
			txFrame->command = OPFLOW_UPIXELSLC306_MODULE_CFG;	/*指令*/
			
			/*赋值txFrame->payload*/
			for (i = 0; i < cmdLen; i++)
			{
				txFrame->payload[i] = payload[i];
			}	
		
			/*计算校验位*/			
			txFrame->XOR = (txFrame->payload[0] ^ txFrame->payload[1] ^ txFrame->payload[2] ^ txFrame->payload[3]);
			
			/*计算发送字节数据*/	
			txLenth = 1 + cmdLen + 1;

			/*赋值txbuff*/
			memcpy(g_OpFlowUpixelsLC306TxBuff, (u8 *)&g_sOpFlowUpixelsLC306TxFrame.command, 1);
			memcpy(g_OpFlowUpixelsLC306TxBuff + 1, (u8 *)&g_sOpFlowUpixelsLC306TxFrame.payload, 4);	
			memcpy(g_OpFlowUpixelsLC306TxBuff + 5, (u8 *)&g_sOpFlowUpixelsLC306TxFrame.XOR, 1);			
			
		}break;

		case OPFLOW_UPIXELSLC306_SENSOR_CFG: /*传感器参数配置指令*/
		{
			/*赋值txFrame->command*/			
			txFrame->command = OPFLOW_UPIXELSLC306_SENSOR_CFG;
			
			/*赋值txFrame->payload*/
			txFrame->payload[0] = OPFLOW_UPIXELSLC306_SENSOR_IIC_ADDR;	/*传感器地址*/
			
			for (i = 0 + 1; i < cmdLen + 1; i++)
			{
				txFrame->payload[i] = payload[i - 1];
			}		
		
			/*计算校验位*/			
			txFrame->XOR = (txFrame->payload[0] ^ txFrame->payload[1] ^ txFrame->payload[2]);
			
			/*计算发送字节数据*/	
			txLenth = 1 + (1 + cmdLen) + 1;

			/*赋值txbuff*/
			memcpy(g_OpFlowUpixelsLC306TxBuff, (u8 *)&g_sOpFlowUpixelsLC306TxFrame.command, 1);
			memcpy(g_OpFlowUpixelsLC306TxBuff + 1, (u8 *)&g_sOpFlowUpixelsLC306TxFrame.payload, 3);	
			memcpy(g_OpFlowUpixelsLC306TxBuff + 4, (u8 *)&g_sOpFlowUpixelsLC306TxFrame.XOR, 1);	
		}break;

		case OPFLOW_UPIXELSLC306_FINISH_CFG: /*关闭配置指令*/
		{
			/*赋值txFrame->command*/				
			txFrame->command = OPFLOW_UPIXELSLC306_FINISH_CFG;	/*指令*/
			
			/*计算发送字节数据*/	
			txLenth = 1 + cmdLen + 0;
			
			/*赋值txbuff*/
			memcpy(g_OpFlowUpixelsLC306TxBuff, (u8 *)&g_sOpFlowUpixelsLC306TxFrame.command, 1);				
		}break;

		default:break;
	}
	
	return txLenth;
}

/*数据偏移重组*/
SYS_RETSTATUS bsp_OPFLOW_UpixelsLC306_Reform_Data(OpFlowUpixelsLC306DataReform *DataReform, u8 *rxBuff)
{
	/*接收数组内存中的起始地址*/
	DataReform->pStart = g_OpFlowUpixelsLC306RxBuff;
	
	/*拼接点即起始点*/
	DataReform->pStitch = DataReform->pStart;
	
	/*head和fixLenth默认起始点*/
	DataReform->pHead    = DataReform->pStart;
	DataReform->fixLenth = DataReform->pHead + 1;
	
	/*计量归零*/
	DataReform->frontByteNbr = 0;
	DataReform->backByteNbr  = 0;
	
	/*扫描直到截取有效位(一个帧头,一个固定长度值)*/
	while((*(DataReform->pHead) != 0xFE) || (*(DataReform->fixLenth) != 0x0A))
	{	
		/*截取点前面的byte++*/
		DataReform->frontByteNbr++;
		
		/*到越界时还未找到,表明该帧中没有找到,就停止*/
		if (DataReform->frontByteNbr > OPFLOW_UPIXELSLC306_RX_BUFF_LENTH - 2)
		{
			DataReform->pHead        = NULL;
			DataReform->fixLenth     = NULL;
			DataReform->frontByteNbr = 0;	
		    DataReform->backByteNbr  = 0;
			
			return SYS_RET_FAIL;
		}		
		
		/*head指针偏移++*/
		DataReform->pHead++;

		/*fixLenth指针偏移++*/
		DataReform->fixLenth++;
	}
	
	/*计算截取点后面还有多少字节数据*/
	DataReform->backByteNbr = OPFLOW_UPIXELSLC306_RX_BUFF_LENTH - DataReform->frontByteNbr;
	
	return SYS_RET_SUCC;	
}

/*从机应答查询*/
SYS_RETSTATUS bsp_OPFLOW_UpixelsLC306_Slave_Response(OpFlowUpixelsLC306ResponseCmpReform *ResponseCmp, u8 cmpBuff[])
{	
	SYS_RETSTATUS CMP_RET_STATUS = SYS_RET_SUCC;
	
	/*最后剩3个空间*/
	if ((ResponseCmp->pCmpData1 - g_OpFlowUpixelsLC306RxBuff) == OPFLOW_UPIXELSLC306_RX_BUFF_LENTH - 3)
	{
		if ((*(ResponseCmp->pCmpData1) == cmpBuff[0]) && \
			(*(ResponseCmp->pCmpData2) == cmpBuff[1]) && \
			(*(ResponseCmp->pCmpData3) == cmpBuff[2]))
		{	
			CMP_RET_STATUS = SYS_RET_FAIL;
		}
		
		ResponseCmp->pCmpData1 = ResponseCmp->pStart + 0;
		ResponseCmp->pCmpData2 = ResponseCmp->pStart + 1;
		ResponseCmp->pCmpData3 = ResponseCmp->pStart + 2;
		
		return CMP_RET_STATUS;
	}
	/*最后剩2个空间*/	
	else if ((ResponseCmp->pCmpData1 - g_OpFlowUpixelsLC306RxBuff) == OPFLOW_UPIXELSLC306_RX_BUFF_LENTH - 2)
	{
		if ((*(ResponseCmp->pCmpData1) == cmpBuff[0]) && \
			(*(ResponseCmp->pCmpData2) == cmpBuff[1]) && \
			(*(ResponseCmp->pCmpData3) == cmpBuff[2]))
		{	
			CMP_RET_STATUS = SYS_RET_FAIL;
		}
		
		ResponseCmp->pCmpData1 = ResponseCmp->pStart + 1;
		ResponseCmp->pCmpData2 = ResponseCmp->pStart + 2;
		ResponseCmp->pCmpData3 = ResponseCmp->pStart + 3;

		return CMP_RET_STATUS;		
	}
	/*最后剩1个空间*/
	else if ((ResponseCmp->pCmpData1 - g_OpFlowUpixelsLC306RxBuff) == OPFLOW_UPIXELSLC306_RX_BUFF_LENTH - 1)
	{
		if ((*(ResponseCmp->pCmpData1) == cmpBuff[0]) && \
			(*(ResponseCmp->pCmpData2) == cmpBuff[1]) && \
			(*(ResponseCmp->pCmpData3) == cmpBuff[2]))
		{
			CMP_RET_STATUS = SYS_RET_FAIL;
		}

		ResponseCmp->pCmpData1 = ResponseCmp->pStart + 2;
		ResponseCmp->pCmpData2 = ResponseCmp->pStart + 3;
		ResponseCmp->pCmpData3 = ResponseCmp->pStart + 4;		
		
		return CMP_RET_STATUS;
	}
	/*位于开头*/
	else if ((ResponseCmp->pCmpData1 - g_OpFlowUpixelsLC306RxBuff) == 0)
	{
		if ((*(ResponseCmp->pCmpData1) == cmpBuff[0]) && \
			(*(ResponseCmp->pCmpData2) == cmpBuff[1]) && \
			(*(ResponseCmp->pCmpData3) == cmpBuff[2]))
		{	
			CMP_RET_STATUS = SYS_RET_FAIL;				
		}

		ResponseCmp->pCmpData1 = ResponseCmp->pStart + 3;
		ResponseCmp->pCmpData2 = ResponseCmp->pStart + 4;
		ResponseCmp->pCmpData3 = ResponseCmp->pStart + 5;
		
		return CMP_RET_STATUS;
	}
	/*位于中间*/
	else
	{
		if ((*(ResponseCmp->pCmpData1) == cmpBuff[0]) && \
			(*(ResponseCmp->pCmpData2) == cmpBuff[1]) && \
			(*(ResponseCmp->pCmpData3) == cmpBuff[2]))
		{
			CMP_RET_STATUS = SYS_RET_FAIL;			
		}

		ResponseCmp->pCmpData1++;
		ResponseCmp->pCmpData2++;
		ResponseCmp->pCmpData3++;		
		
		return CMP_RET_STATUS;
	}
}
