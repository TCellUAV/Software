#include "msp_DMA.h"

/*DMA初始化*/
void msp_Dma_Init(Msp_Dma * msp_Dma)
{
	DMA_InitTypeDef DMA_InitStruct;
	
	DMA_DeInit(msp_Dma->Stream);	/*将DMA_Stream寄存器设置为缺省值*/
	
	while (DMA_GetCmdStatus(msp_Dma->Stream) != DISABLE){} /*等待DMA可配置*/
	
	DMA_InitStruct.DMA_Channel 		       = msp_Dma->Channel;
	DMA_InitStruct.DMA_PeripheralBaseAddr  = msp_Dma->PerBaseAddr;
	DMA_InitStruct.DMA_Memory0BaseAddr     = msp_Dma->MemBaseAddr;
	DMA_InitStruct.DMA_DIR                 = msp_Dma->Direction;
	DMA_InitStruct.DMA_BufferSize          = msp_Dma->BuffSize;
	DMA_InitStruct.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;	/*禁止外设地址增长*/
	DMA_InitStruct.DMA_MemoryInc           = DMA_MemoryInc_Enable;		/*使能内存地址增长*/
	DMA_InitStruct.DMA_PeripheralDataSize  = msp_Dma->PerAlignByte;
	DMA_InitStruct.DMA_MemoryDataSize      = msp_Dma->MemAlignByte;
	DMA_InitStruct.DMA_Mode                = msp_Dma->Mode;
	DMA_InitStruct.DMA_Priority            = msp_Dma->Priority;
	DMA_InitStruct.DMA_FIFOMode 		   = DMA_FIFOMode_Disable;       /*禁用Fifo*/  
	DMA_InitStruct.DMA_FIFOThreshold       = DMA_FIFOThreshold_Full;	 /*满*/
	DMA_InitStruct.DMA_MemoryBurst         = DMA_MemoryBurst_Single;     /*存储器突发单次传输*/
	DMA_InitStruct.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single; /*外设突发单次传输*/		
	
	DMA_Init(msp_Dma->Stream, &DMA_InitStruct);
		
	/*判断是否启用DMA中断,并配置对应对应中断*/
	if (msp_Dma->dmaIT == DMA_IT_ENABLE)
	{
		/*预清除中断标志*/
		DMA_ClearFlag(msp_Dma->Stream, msp_Dma->ITType);
		
		/*配置使能中断*/
		DMA_ITConfig(msp_Dma->Stream, msp_Dma->ITType, ENABLE);			
	}		
}

/*DMA使能*/
void msp_Dma_Enable(Msp_Dma * msp_Dma)
{
	/*关闭DMA指示通道*/
	DMA_Cmd(msp_Dma->Stream, DISABLE);
	
	/*确保DMA可以被设置*/
	while(DMA_GetCmdStatus(msp_Dma->Stream) != DISABLE){}	
	
	/*DMA通道的数据传输量设置*/
	DMA_SetCurrDataCounter(msp_Dma->Stream, msp_Dma->BuffSize);
	
	/*使能DMA指示通道*/
	DMA_Cmd(msp_Dma->Stream, ENABLE);	
}

/*DMA失能*/
void msp_Dma_Disable(Msp_Dma * msp_Dma)
{
	/*关闭DMA指示通道*/
	DMA_Cmd(msp_Dma->Stream, DISABLE);
}
