#include "bsp_ULTR_RCWL1603.h"

BSP_RCWL1603 g_sRCWL1603;

/*超声波模块初始化*/
SYS_RETSTATUS bSP_ULTR_RCWL1603_Init(BSP_RCWL1603 *rcwl1603)
{
	SYS_RETSTATUS retStatus;
	
	
	return retStatus;
}

///*超声波模块初始化*/
//SYS_RETSTATUS bSP_ULTR_US100_Init(BSP_US100 *us100)
//{
//	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
//	u8 ackData;
//	
//	if ((ultrasonic->WorkMode == BSP_ULTRASONIC_UART) || \
//		(ultrasonic->WorkMode == BSP_ULTRASONIC_AUTO_UART))	/*串口通信方式*/
//	{
//		/*串口特征初始化*/
//		ultrasonic->UartMaster.Uart               = USART6;
//		ultrasonic->UartMaster.Resource 		  = MSP_SRC_UART6;
//		ultrasonic->UartMaster.BaudRate           = 9600;
//		ultrasonic->UartMaster.Mode               = USART_Mode_Tx | USART_Mode_Rx;
//		
//		/*GPIO管脚设置*/
//		ultrasonic->UartMaster.Tx_GPIO            = GPIOC;
//		ultrasonic->UartMaster.Tx_Pin             = GPIO_Pin_6;
//		ultrasonic->UartMaster.Tx_PinSource		  = GPIO_PinSource6;
//		ultrasonic->UartMaster.RCC_Periph_Tx_GPIO = RCC_AHB1Periph_GPIOC;
//		ultrasonic->UartMaster.Rx_GPIO            = GPIOC;
//		ultrasonic->UartMaster.Rx_Pin   		  = GPIO_Pin_7;
//		ultrasonic->UartMaster.Tx_PinSource		  = GPIO_PinSource7;		
//		ultrasonic->UartMaster.RCC_Periph_Rx_GPIO = RCC_AHB1Periph_GPIOC;
//		
//		/*复用目标*/
//		ultrasonic->UartMaster.GPIO_AF			  = GPIO_AF_USART6;
//		
//		/*收发buff*/
//		ultrasonic->UartMaster.pRxBuff            = ultrasonic->RxDataBuff;	/*数据接收*/
//		ultrasonic->UartMaster.pTxBuff            = &(ultrasonic->TxCmd);	/*指令发送*/
//		
//		/*设置串口收发工作模式*/
//		ultrasonic->UartMaster.TxMode             = MSP_UART_POLL;
//		ultrasonic->UartMaster.RxMode 			  = MSP_UART_DMA;
//		
//		/*选择配置DMA通道*/
//		ultrasonic->UartMaster.RxDma.Stream		  = DMA2_Stream1;
//		ultrasonic->UartMaster.RxDma.Channel	  = DMA_Channel_5;
//		ultrasonic->UartMaster.RxDma.PerAlignByte = DMA_PeripheralDataSize_Byte;		
//		ultrasonic->UartMaster.RxDma.MemAlignByte = DMA_MemoryDataSize_Byte;
//		ultrasonic->UartMaster.RxDma.BuffSize     = sizeof(ultrasonic->RxDataBuff) / sizeof(u8);
//		ultrasonic->UartMaster.RxDma.Mode         = DMA_Mode_Circular;	/*循环模式*/
//		ultrasonic->UartMaster.RxDma.Priority     = DMA_Priority_High;
//		
//		/*串口初始化*/
//		msp_Uart_Init(&(ultrasonic->UartMaster));		
//	}
//	
//	/*根据超声波模块设置指定工作模式*/
//	if (ultrasonic->Targ == BSP_ULTRASONIC_RCWL_1603)	/*RCWL_1603*/
//	{	
//		/*RCWL_1603设置工作模式*/
//		if (ultrasonic->WorkMode == BSP_ULTRASONIC_UART)
//		{
//			ultrasonic->TxCmd  = RCWL_1603_SET_SINGLE_UART_MODE_CMD;
//		}
//		else if (ultrasonic->WorkMode == BSP_ULTRASONIC_AUTO_UART)
//		{
//			ultrasonic->TxCmd  = RCWL_1603_SET_AUTO_UART_MODE_CMD;
//		}
//		else if (ultrasonic->WorkMode == BSP_ULTRASONIC_GPIO)	/*GPIO:Trig和Echo模式*/
//		{
//			ultrasonic->TxCmd = RCWL_1603_SET_GPIO_MODE_CMD;		
//		}
//		else if (ultrasonic->WorkMode == BSP_ULTRASONIC_PWM)	/*从机PWM输出模式*/
//		{
//			ultrasonic->TxCmd = RCWL_1603_SET_PWM_OUT_MODE_CMD;	
//		}

//		BSP_Ultrasonic_Send_Command(ultrasonic);
//		
//		sys_DelayMs(100);	/*等待从机超声波应答*/
//		
//		/*接收超声波应答数据*/
//		ackData = BSP_Ultrasonic_Recv_AckData(ultrasonic);
//		
//		if ((ackData == RCWL_1603_SET_SINGLE_UART_MODE_RCV) || \
//			(ackData == RCWL_1603_SET_GPIO_MODE_RCV)        || \
//			(ackData == RCWL_1603_SET_AUTO_UART_MODE_RCV)   || \
//			 ackData == RCWL_1603_SET_PWM_OUT_MODE_RCV)
//		{
//			statusRet = SYS_RET_SUCC;		/*RCWL_1603模块设置成功*/
//		}
//		
//		/*获取RCWL_1603的固件版本信息*/
//		ultrasonic->TxCmd = RCWL_1603_GET_MODULE_FIRMWARE_VERSION_CMD;
//		BSP_Ultrasonic_Send_Command(ultrasonic);
//		
//		ultrasonic->FirmwareVersion = BSP_Ultrasonic_Recv_AckData(ultrasonic);		
//		
//		/*设置获取温度值命令*/
//		ultrasonic->GetTempCmd = RCWL_1603_GET_MODULE_TEMPERATURE_CMD;
//		
//		/*设置获取测量距离值命令*/
//		ultrasonic->GetDistanceCmd = RCWL_1603_GET_MODULE_DISTANCE_CMD;
//	}
//	else if (ultrasonic->Targ == BSP_ULTRASONIC_US100)	/*US100*/
//	{
//		
//		/*设置获取温度值命令*/
//		ultrasonic->GetTempCmd = US100_GET_MODULE_TEMPERATURE_CMD;
//		
//		/*设置获取测量距离值命令*/
//		ultrasonic->GetDistanceCmd = US100_GET_MODULE_DISTANCE_CMD;
//	}
//	
//	return statusRet;
//}

///*向超声波模块发送命令*/
//void BSP_Ultrasonic_Send_Command(BSP_Ultrasonic *ultrasonic)
//{
//	msp_Uart_Send_Data(&(ultrasonic->UartMaster));	/*ultrasonic->UartMaster.pTxBuff*/
//}

///*从超声波模块接收应答数据(DMA / interrupt / POLL接收,都赋值到该数组)*/
//u8 BSP_Ultrasonic_Recv_AckData(BSP_Ultrasonic *ultrasonic)
//{
//	u8 ackData;
//	
//	if (ultrasonic->RxDataBuff[0] != 0)
//	{
//		ackData = ultrasonic->RxDataBuff[0];
//	}
//	else if (ultrasonic->RxDataBuff[1] != 0)
//	{
//		ackData = ultrasonic->RxDataBuff[1];
//	}
//	else
//	{
//		ackData = 0xFF;
//	}
//	
//	/*接收Buff清0,确保数据都是本次指令后的数据*/
//	memset(ultrasonic->RxDataBuff, 0, sizeof(ultrasonic->RxDataBuff) / sizeof(u8));
//	
//	return ackData;
//}

///*命令超声波开始测量模块温度*/
//void BSP_Ultrasonic_Start_Meas_Temperature(BSP_Ultrasonic *ultrasonic)
//{
//	/*获取温度命令*/
//	ultrasonic->TxCmd = ultrasonic->GetTempCmd;
//	
//	/*发送命令*/
//	BSP_Ultrasonic_Send_Command(ultrasonic);
//}

///*命令超声波开始测量距离*/
//void BSP_Ultrasonic_Start_Meas_Distance(BSP_Ultrasonic *ultrasonic)
//{
//	/*获取距离命令*/
//	ultrasonic->TxCmd = ultrasonic->GetDistanceCmd;
//	
//	/*发送命令*/
//	BSP_Ultrasonic_Send_Command(ultrasonic);	
//}

///*获取超声波模块温度(DMA / interrupt / POLL接收,都赋值到该数组)*/
//u16 BSP_Ultrasonic_Get_Temperature(BSP_Ultrasonic *ultrasonic)
//{
//	u8 temperature;
//	
//	if (ultrasonic->RxDataBuff[0] != 0)
//	{
//		temperature = ultrasonic->RxDataBuff[0];
//	}
//	else if (ultrasonic->RxDataBuff[1] != 0)
//	{
//		temperature = ultrasonic->RxDataBuff[1];
//	}	
//	
//	/*接收Buff清0,确保数据都是本次指令后的数据*/
//	memset(ultrasonic->RxDataBuff, 0, sizeof(ultrasonic->RxDataBuff) / sizeof(u8));
//	
//	if (ultrasonic->Targ == BSP_ULTRASONIC_US100)
//	{
//		/*TEMP = temperature - 45*/
//		ultrasonic->Temperature = temperature - US100_MODULE_TEMPRATURE_BASE;
//	}
//	else
//	{
//		ultrasonic->Temperature = temperature;
//	}
//	
//	return (ultrasonic->Temperature);
//}

///*获取超声波模块距离(DMA / interrupt / POLL接收,都赋值到该数组)*/
//u16 BSP_Ultrasonic_Get_Distance(BSP_Ultrasonic *ultrasonic)
//{	
//	ultrasonic->Distance = ((u16)ultrasonic->RxDataBuff[0] << 8) + ultrasonic->RxDataBuff[1];
//	
//	/*接收Buff清0,确保数据都是本次指令后的数据*/
//	memset(ultrasonic->RxDataBuff, 0, sizeof(ultrasonic->RxDataBuff) / sizeof(u8));
//	
//	return (ultrasonic->Distance);
//}
