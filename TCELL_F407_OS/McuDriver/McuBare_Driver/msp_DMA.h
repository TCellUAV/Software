#ifndef _MSP_DMA_H_
#define _MSP_DMA_H_

#include "sys_Platform.h"

typedef enum
{
	DMA_IT_DISABLE = 0,	
	DMA_IT_ENABLE  = 1,
}DMA_IT_ENABLE_STATUS;

typedef struct
{
	DMA_Stream_TypeDef*  	 Stream;
	u32					 	 Channel;
	u32 				 	 PerBaseAddr;
	u32 				 	 MemBaseAddr;
	u32					 	 Direction;
	u32 			 	 	 PerAlignByte;
	u32 			 	 	 MemAlignByte;	
	u32 				 	 Mode;	
	u32 				 	 Priority;
	u32 				 	 BuffSize;
	u32					 	 RCC_Periph_DMA;
	u32					     dmaFlag;	/*DMA传输标志*/
	DMA_IT_ENABLE_STATUS     dmaIT;	    /*DMA是否使用中断*/
	volatile FunctionalState dmaStatus;	/*DMA使能状态,第一次使能是在解析数据时判断再使能,main函数while(1)前不可使能*/
	u32 				 	 ITType;    /*DMA中断类型*/
	u8 				     	 NVIC_IRQChannel; /*中断通道*/
}Msp_Dma;

/*DMA初始化*/
void msp_Dma_Init(Msp_Dma * msp_Dma);

/*DMA使能*/
void msp_Dma_Enable(Msp_Dma * msp_Dma);

/*DMA失能*/
void msp_Dma_Disable(Msp_Dma * msp_Dma);
	
#endif
