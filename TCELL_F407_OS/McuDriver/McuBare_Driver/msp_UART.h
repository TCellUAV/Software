#ifndef _MSP_UART_H_
#define _MSP_UART_H_

#include "sys_Platform.h"
#include "msp_DMA.h"

#define UART_TX_BUFF_LENTH_1	(1)
#define UART_TX_BUFF_LENTH_2	(2)
#define UART_TX_BUFF_LENTH_10	(10)

#define UART_RX_BUFF_LENTH_1	(1)
#define UART_RX_BUFF_LENTH_2	(2)
#define UART_RX_BUFF_LENTH_10	(10)
#define UART_RX_BUFF_LENTH_30	(30)

/*UART发送/接收工作方式(调试方式)*/
typedef enum
{
	MSP_UART_POLL      = 1,	   /*CPU轮询*/
	MSP_UART_DMA       = 2,	   /*DMA*/
	MSP_UART_INTERRUPT = 3,	   /*中断*/
	MSP_UART_DMA_IDLE  = 4,    /*DMA+空闲中断*/
}MSP_UART_WORK_MODE;

typedef struct
{
	/*Uart*/
	USART_TypeDef 	    *Uart;
	u32  	            BaudRate;	
	u16 	            Mode;
	u32 				RCC_Periph_UART;
	
	/*Gpio*/
	GPIO_TypeDef 	    *Tx_GPIO;
	u16 		 	    Tx_Pin;
	u32 		        RCC_Periph_Tx_GPIO;
	u16 				Tx_PinSource;
	GPIO_TypeDef        *Rx_GPIO;
	u16 		        Rx_Pin;	
	u32 		        RCC_Periph_Rx_GPIO;	
	u16 				Rx_PinSource;	
	
	u8 					GPIO_AF;
	
	u8 				    NVIC_IRQChannel; /*中断通道*/	
	/*Tx/Rx工作方式*/
	MSP_UART_WORK_MODE  TxMode;
	MSP_UART_WORK_MODE  RxMode;
	
	/*DMA*/
	Msp_Dma			    TxDma;
	Msp_Dma			    RxDma;	
	
	/*Tx/Rx Buff*/
	u8  			    *pTxBuff;
	u8  			    *pRxBuff;
}MSP_Uart;

/*串口初始化*/
void msp_Uart_Init(MSP_Uart *uart);

/*串口发送数据*/
void msp_Uart_Send_Data(MSP_Uart *uart, u8 *txBuff, u16 lenth, MSP_UART_WORK_MODE UART_TX_MODE);

/*串口接收数据*/
void msp_uart_recv_data(MSP_Uart *uart);

#endif
