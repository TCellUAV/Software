#include "ssp_SimSPI.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

void ssp_SimSPI_Init(SSP_SimSPI *SimSPI)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*主出,从入*/
	GPIO_InitStruct.GPIO_Pin   = SimSPI->MOSI_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(SimSPI->MOSI_GPIO, &GPIO_InitStruct);
	
	/*主入,从出*/	
	GPIO_InitStruct.GPIO_Pin   = SimSPI->MISO_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(SimSPI->MISO_GPIO, &GPIO_InitStruct);

	/*时钟*/
	GPIO_InitStruct.GPIO_Pin   = SimSPI->CLK_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(SimSPI->CLK_GPIO, &GPIO_InitStruct);
	
	/*片选*/
	GPIO_InitStruct.GPIO_Pin   = SimSPI->CS_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(SimSPI->CS_GPIO, &GPIO_InitStruct);
	
	/*复位*/
	GPIO_InitStruct.GPIO_Pin   = SimSPI->RST_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(SimSPI->RST_GPIO, &GPIO_InitStruct);
	
	/*DC*/
	GPIO_InitStruct.GPIO_Pin   = SimSPI->DC_Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(SimSPI->DC_GPIO, &GPIO_InitStruct);
}


void ssp_SimSPI_WriteByte(SSP_SimSPI *SimSPI, u8 data, uint8_t command)
{
	uint8_t i;			  
	
#ifdef PLATFORM_RTOS__RT_THREAD
	/*SPI互斥锁获取*/
//	rt_mutex_take(&spi_mutex, 5);
#endif	
	
	/*判断执行command / data*/
	if(command != NO_CMD)
	{
		SIM_SPI_DC_SET(SimSPI->DC_GPIO, SimSPI->DC_Pin);
	}
	else
	{		
		SIM_SPI_DC_RESET(SimSPI->DC_GPIO, SimSPI->DC_Pin);		
	}
	
	for(i = 0; i < 8; i++)
	{			  
		SIM_SPI_CLK_RESET(SimSPI->CLK_GPIO, SimSPI->CLK_Pin);		
		
		if(data & 0x80)
		{
			SIM_SPI_SDIN_SET(SimSPI->MOSI_GPIO, SimSPI->MOSI_Pin);
		}
		else
		{			
			SIM_SPI_SDIN_RESET(SimSPI->MOSI_GPIO, SimSPI->MOSI_Pin);
		}
		
		SIM_SPI_CLK_SET(SimSPI->CLK_GPIO, SimSPI->CLK_Pin);	
		
		data <<= 1;   
	}			
	
	SIM_SPI_DC_SET(SimSPI->DC_GPIO, SimSPI->DC_Pin);
	
#ifdef PLATFORM_RTOS__RT_THREAD
	/*SPI互斥锁释放*/
//	rt_mutex_release(&spi_mutex);
#endif	
}
