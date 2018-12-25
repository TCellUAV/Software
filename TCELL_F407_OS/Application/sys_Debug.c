#include "sys_Debug.h"
#include "sys_McuInit.h"
#include "hci_show_send_config.h"

u8 g_DebugTxBuff[DEBUG_TX_BUFF_SIZE] = {0};
u8 g_DebugRxBuff[DEBUG_RX_BUFF_SIZE] = {0};

/*Debug UART*/
MSP_Uart g_sDebugUart = 
{
	.Uart 				   = USART1,
#if ((USER_AIRCRAFT_SEND_TO_ANO_HOST == SYS_ENABLE) || \
	 (USER_USE_ANO_HOST_AIRCRAFT_SLAVE_EXCHANGE == SYS_ENABLE) && \
	 (HOST_SLAVE_DATA_TRANSECEIVE_BY_WIRELESS == SYS_ENABLE))     /*匿名上位机 无线传输*/
	.BaudRate 			   = 115200,
	
#elif ((USER_AIRCRAFT_SEND_TO_ANO_HOST == SYS_ENABLE) || \
	   (USER_USE_ANO_HOST_AIRCRAFT_SLAVE_EXCHANGE == SYS_ENABLE) && \
	   (HOST_SLAVE_DATA_TRANSECEIVE_BY_WIRELESS == SYS_DISABLE))  /*匿名上位机 有线传输*/
	   .BaudRate 		   = 256000,	
	
#elif ((USER_AIRCRAFT_SEND_VCAN_HOST == SYS_ENABLE) && \
	   (HOST_SLAVE_DATA_TRANSECEIVE_BY_WIRELESS == SYS_ENABLE))   /*山外上位机 无线传输*/
	.BaudRate 			   = 115200,
	
#elif ((USER_AIRCRAFT_SEND_VCAN_HOST == SYS_ENABLE) && \
	   (HOST_SLAVE_DATA_TRANSECEIVE_BY_WIRELESS == SYS_DISABLE))  /*山外上位机 有线传输*/
	.BaudRate 			   = 256000,	
	
#elif ((USER_AIRCRAFT_SEND_BSK_HOST == SYS_ENABLE) && \
	   (HOST_SLAVE_DATA_TRANSECEIVE_BY_WIRELESS == SYS_ENABLE))	  /*BlueSky上位机 无线传输*/
	.BaudRate 			   = 115200,

#elif ((USER_AIRCRAFT_SEND_BSK_HOST == SYS_ENABLE) && \
	   (HOST_SLAVE_DATA_TRANSECEIVE_BY_WIRELESS == SYS_DISABLE))  /*BlueSky上位机 有线传输*/
	.BaudRate 			   = 115200,
#endif

	.NVIC_IRQChannel	   = USART1_IRQn,
};

/*串口打印初始化*/
void rt_debug_uart_init(void)
{
	SYS_RCC_TREE gpioRccTree;
	SYS_RCC_TREE periphRccTree;
	
/*Debug Uart Init: RCC + GPIO + UART + RxDMA*/
	g_sDebugUart.Tx_GPIO 			= GPIOA;
	g_sDebugUart.Tx_Pin 			= GPIO_Pin_9;
	g_sDebugUart.Tx_PinSource	    = GPIO_PinSource9;
	g_sDebugUart.RCC_Periph_Tx_GPIO = RCC_AHB1Periph_GPIOA;
	
	g_sDebugUart.Rx_GPIO 			= GPIOA;
	g_sDebugUart.Rx_Pin 			= GPIO_Pin_10;
	g_sDebugUart.Rx_PinSource	    = GPIO_PinSource10;
	g_sDebugUart.RCC_Periph_Rx_GPIO = RCC_AHB1Periph_GPIOA;	
	
	g_sDebugUart.GPIO_AF			= GPIO_AF_USART1;			/*GPIO复用*/
	g_sDebugUart.RCC_Periph_UART    = RCC_APB2Periph_USART1;	/*UART时钟*/
	gpioRccTree	                    = SYS_RCC_AHB1;			    /*设定GPIO时钟树*/
	periphRccTree					= SYS_RCC_APB2;			    /*设定Uart时钟树*/
	
	/*RCC Init*/
	sys_Peripheral_RCC_Init(gpioRccTree, g_sDebugUart.RCC_Periph_Tx_GPIO, ENABLE);
	sys_Peripheral_RCC_Init(gpioRccTree, g_sDebugUart.RCC_Periph_Rx_GPIO, ENABLE);
	sys_Peripheral_RCC_Init(periphRccTree, g_sDebugUart.RCC_Periph_UART, ENABLE);
	
	/*USART 配置*/
	g_sDebugUart.Uart     = USART1,
	g_sDebugUart.Mode     = USART_Mode_Rx | USART_Mode_Tx;	/*收发送模式*/
	
	/*USART工作方式, 接收:DMA 发送:DMA*/
	g_sDebugUart.RxMode = MSP_UART_DMA_IDLE;
	g_sDebugUart.TxMode = MSP_UART_DMA;
	
	/*DMA / DMA+IDLE*/
	if ((g_sDebugUart.RxMode == MSP_UART_DMA) || (g_sDebugUart.RxMode == MSP_UART_DMA_IDLE))
	{
		/*配置接收DMA*/
		g_sDebugUart.RxDma.Stream         = DMA2_Stream2;	               /*DMA流*/
		g_sDebugUart.RxDma.Channel        = DMA_Channel_4;		           /*DMA通道*/
		g_sDebugUart.RxDma.RCC_Periph_DMA = RCC_AHB1Periph_DMA2;           /*DMA时钟*/
		g_sDebugUart.RxDma.BuffSize 	  = DEBUG_RX_BUFF_SIZE; 		   /*buff(缓冲区)大小*/
		g_sDebugUart.RxDma.PerAlignByte   = DMA_PeripheralDataSize_Byte;   /*1字节对齐*/
		g_sDebugUart.RxDma.MemAlignByte   = DMA_MemoryDataSize_Byte;       /*1字节对齐*/
		g_sDebugUart.RxDma.Mode           = DMA_Mode_Circular;		       /*有数据才接收*/
		g_sDebugUart.RxDma.Priority       = DMA_Priority_Medium;		   /*优先级*/
		g_sDebugUart.RxDma.dmaFlag        = DMA_FLAG_TCIF2;				   /*Stream传输完毕标志*/
		g_sDebugUart.RxDma.dmaIT		  = DMA_IT_DISABLE;			       /*DMA中断使能状态*/
		
		/*是否使用DMA接收中断,若使用就选择中断类型,并设置中断请求通道*/
		if (g_sDebugUart.RxDma.dmaIT == DMA_IT_ENABLE)
		{
			g_sDebugUart.RxDma.ITType          = DMA_IT_TC;
			g_sDebugUart.RxDma.NVIC_IRQChannel = DMA2_Stream2_IRQn;
		}
		
		/*使能接收DMA时钟*/
		periphRccTree = SYS_RCC_AHB1;  /*设定DMA时钟树*/
		sys_Peripheral_RCC_Init(periphRccTree, g_sDebugUart.RxDma.RCC_Periph_DMA, ENABLE);
	}
	
	/*DMA / DMA+IDLE*/
	if ((g_sDebugUart.TxMode == MSP_UART_DMA) || (g_sDebugUart.TxMode == MSP_UART_DMA_IDLE))
	{	
		/*配置发送DMA*/
		g_sDebugUart.TxDma.Stream         = DMA2_Stream7;	               /*DMA流*/
		g_sDebugUart.TxDma.Channel        = DMA_Channel_4;		           /*DMA通道*/
		g_sDebugUart.TxDma.RCC_Periph_DMA = RCC_AHB1Periph_DMA2;           /*DMA时钟*/
		g_sDebugUart.TxDma.BuffSize 	  = DEBUG_TX_BUFF_SIZE; 		   /*buff(缓冲区)大小*/
		g_sDebugUart.TxDma.PerAlignByte   = DMA_PeripheralDataSize_Byte;   /*1字节对齐*/
		g_sDebugUart.TxDma.MemAlignByte   = DMA_MemoryDataSize_Byte;       /*1字节对齐*/
		g_sDebugUart.TxDma.Mode           = DMA_Mode_Normal;		       /*有数据才发送*/
		g_sDebugUart.TxDma.Priority       = DMA_Priority_Medium;		   /*优先级*/
		g_sDebugUart.TxDma.dmaFlag        = DMA_FLAG_TCIF7;				   /*Stream传输完毕标志*/
		g_sDebugUart.TxDma.dmaIT		  = DMA_IT_DISABLE;			       /*不启用DMA中断*/
		
		/*是否使用DMA发送中断,若使用就选择中断类型,并设置中断请求通道*/
		if (g_sDebugUart.TxDma.dmaIT == DMA_IT_ENABLE)
		{
			g_sDebugUart.TxDma.ITType          = DMA_IT_TC;
			g_sDebugUart.TxDma.NVIC_IRQChannel = DMA2_Stream7_IRQn;			
		}		

		/*使能发送DMA时钟*/
		periphRccTree = SYS_RCC_AHB1; 	 /*设定DMA时钟树*/
		sys_Peripheral_RCC_Init(periphRccTree, g_sDebugUart.TxDma.RCC_Periph_DMA, ENABLE);		
	}		
	
	/*指定发送/接收内存*/
	g_sDebugUart.pTxBuff = g_DebugTxBuff;
	g_sDebugUart.pRxBuff = g_DebugRxBuff;
	
	/*GPIO & UART Init*/	
	msp_Peripheral_GPIO_Init(SYS_GPIO_UART, &g_sDebugUart);	
}
