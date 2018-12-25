#include "msp_UART.h"

/*串口初始化*/
void msp_Uart_Init(MSP_Uart *uart)
{
	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	/*Tx GPIO*/
	GPIO_InitStruct.GPIO_Pin   = uart->Tx_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(uart->Tx_GPIO, &GPIO_InitStruct);
	
	/*Rx GPIO*/
	GPIO_InitStruct.GPIO_Pin   = uart->Rx_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(uart->Rx_GPIO, &GPIO_InitStruct);	
	
	/*Tx/Rx PinAF*/
	GPIO_PinAFConfig(uart->Tx_GPIO, uart->Tx_PinSource, uart->GPIO_AF);
	GPIO_PinAFConfig(uart->Rx_GPIO, uart->Rx_PinSource, uart->GPIO_AF);
	
	/*Uart Init*/
	USART_InitStruct.USART_BaudRate 		   = uart->BaudRate;
	USART_InitStruct.USART_WordLength 		   = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits 		   = USART_StopBits_1;
	USART_InitStruct.USART_Parity      	       = USART_Parity_No;	
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	USART_InitStruct.USART_Mode 			   = uart->Mode;
	USART_Init(uart->Uart, &USART_InitStruct); /*初始化串口*/
	
	/*1.配置UART发送工作模式*/
	/*1.1配置串口发送中断*/
	if (uart->TxMode == MSP_UART_INTERRUPT) /*发送完成中断*/
	{
		USART_ITConfig(uart->Uart, USART_IT_TXE, ENABLE); /*使能接收中断*/		
	}
	
	/*1.2配置串口发送DMA工作方式*/
	if (uart->TxMode == MSP_UART_DMA)	/*配置DMA*/
	{
		/*发送DMA配置及初始化*/
		uart->TxDma.PerBaseAddr = (u32)&(uart->Uart->DR);	    /*外设目标地址*/
		uart->TxDma.MemBaseAddr = (u32)(uart->pTxBuff);		    /*内存目标地址*/	
		uart->TxDma.Direction   = DMA_DIR_MemoryToPeripheral;	/*内存到外设*/		
		
		/*发送DMA初始化*/		
		msp_Dma_Init(&(uart->TxDma));
	}
	
	/*2.配置UART接收工作模式*/	
	/*2.1配置串口接收中断*/
	if ((uart->RxMode == MSP_UART_INTERRUPT) || (uart->RxMode == MSP_UART_DMA_IDLE)) /*接收中断 / 接收空闲中断*/
	{
		/*普通接收中断*/
		if (uart->RxMode == MSP_UART_INTERRUPT)
		{
			USART_ITConfig(uart->Uart, USART_IT_RXNE, ENABLE); /*使能接收中断*/
		}
		/*空闲中断*/
		else if (uart->RxMode == MSP_UART_DMA_IDLE)
		{
			USART_ITConfig(uart->Uart, USART_IT_IDLE, ENABLE); /*使能空闲中断*/
		}
	}	
	
	/*2.2配置串口接收DMA工作方式*/
	if ((uart->RxMode == MSP_UART_DMA) || (uart->RxMode == MSP_UART_DMA_IDLE))	/*配置DMA*/
	{
		/*接收DMA配置及初始化*/
		uart->RxDma.PerBaseAddr = (u32)&(uart->Uart->DR);		/*外设目标地址*/
		uart->RxDma.MemBaseAddr = (u32)(uart->pRxBuff);			/*内存目标地址*/
		uart->RxDma.Direction   = DMA_DIR_PeripheralToMemory;	/*外设到内存*/
		
		/*接收DMA初始化*/
		msp_Dma_Init(&(uart->RxDma));
	}
	
	/*使能UART*/
	USART_Cmd(uart->Uart, ENABLE);
}

/*串口发送数据*/
void msp_Uart_Send_Data(MSP_Uart *uart, u8 *txBuff, u16 lenth, MSP_UART_WORK_MODE UART_TX_MODE)
{
	u8 i;
	
	/*轮询发送模式*/
	if (UART_TX_MODE == MSP_UART_POLL)	
	{
		for (i = 0; i < lenth; i++)
		{
			USART_SendData(uart->Uart, (u16)(txBuff[i]));
			
			while(USART_GetFlagStatus(uart->Uart, USART_FLAG_TXE) != SET);
		}
	}
	
	/*DMA发送模式*/
	if (UART_TX_MODE == MSP_UART_DMA)	
	{	
		/*判断是否传输完毕,完成后清除传输完成标志,才能开启下次传输*/
		if (DMA_GetFlagStatus(uart->TxDma.Stream, uart->TxDma.dmaFlag) != RESET)
		{
			DMA_ClearFlag(uart->TxDma.Stream, uart->TxDma.dmaFlag);
		}
		
		/*判断DMA传输完成标志位为RESET,才能开启传输*/
		if (DMA_GetFlagStatus(uart->TxDma.Stream, uart->TxDma.dmaFlag) == RESET)
		{
			/*使能串口发送DMA*/
			USART_DMACmd(uart->Uart, USART_DMAReq_Tx, ENABLE);
		
			/*开启DMA传送*/
			msp_Dma_Enable(&(uart->TxDma));
		}
	}
}

/*串口接收数据*/
void msp_uart_recv_data(MSP_Uart *uart)
{
//	u8 i;
	
	/*轮询接收模式*/
	if (uart->RxMode == MSP_UART_POLL)	
	{
		
	}
	
	/*DMA接收模式*/
	if (uart->RxMode == MSP_UART_DMA)	
	{
		/*判断是否传输完毕,完成后清除传输完成标志,才能开启下次传输*/
		if (DMA_GetFlagStatus(uart->RxDma.Stream, uart->RxDma.dmaFlag) != RESET)
		{
			DMA_ClearFlag(uart->RxDma.Stream, uart->RxDma.dmaFlag);
		}
		
		/*判断DMA接收完成标志位为RESET,才能开启传输*/
		if (DMA_GetFlagStatus(uart->RxDma.Stream, uart->RxDma.dmaFlag) == RESET)
		{
			uart->Uart->DR = 0;
			
			/*使能串口接收DMA*/
			USART_DMACmd(uart->Uart, USART_DMAReq_Rx, ENABLE);
		
			/*开启DMA接收*/
			msp_Dma_Enable(&(uart->RxDma));
		}
	}

	/*DMA + IDLE模式*/	
	if (uart->RxMode == MSP_UART_DMA_IDLE)
	{
		/*使能串口接收DMA*/
		USART_DMACmd(uart->Uart, USART_DMAReq_Rx, ENABLE);
		
		/*开启DMA接收*/
		msp_Dma_Enable(&(uart->RxDma));	
	}
}

#pragma import(__use_no_semihosting) 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;   

void _sys_exit(int x) 
{ 
	x = x; 
}

int fputc(int ch, FILE *f)
{	
	while((USART1->SR & 0X40) == 0){};//循环发送,直到发送完毕	
    USART1->DR = (u8)ch;
		
	return ch;
}
