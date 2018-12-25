#ifndef _MSP_ADC_H_
#define _MSP_ADC_H_

#include "sys_Platform.h"

/********** ADC1 **********/
#define MSP_ADC1_MODE				 ADC_Mode_Independent
#define MSP_ADC1_SCANCONVMODE   	 ENABLE
#define MSP_ADC1_EXTERNALTRIGCONV	 ADC_ExternalTrigConv_None
#define MSP_ADC1_NBROFCHANNEL 	     (2)
#define MSP_ADC1_DATA_MODE			 ADC_DMA//ADC_DMA_NORMAL /*ADC_DMA*/

#define MSP_ADC1_BUFF_SIZE			 ((MSP_ADC1_NBROFCHANNEL * sizeof(short)) / sizeof(short))

/*Time resource*/
typedef enum
{
	MSP_SRC_ADC1 = 0x01,
	MSP_SRC_ADC2 = 0x02,
	MSP_SRC_ADC3 = 0x04,
}MSP_ADC_RESOURCE;

typedef enum
{
	MSP_ADC_CH0  = ADC_Channel_0,
	MSP_ADC_CH1  = ADC_Channel_1,
	MSP_ADC_CH2  = ADC_Channel_2,
	MSP_ADC_CH3  = ADC_Channel_3,
	MSP_ADC_CH4  = ADC_Channel_4,
	MSP_ADC_CH5  = ADC_Channel_5,
	MSP_ADC_CH6  = ADC_Channel_6,
	MSP_ADC_CH7  = ADC_Channel_7,
	MSP_ADC_CH8  = ADC_Channel_8,
	MSP_ADC_CH9  = ADC_Channel_9,
	MSP_ADC_CH10 = ADC_Channel_10,
	MSP_ADC_CH11 = ADC_Channel_11,
	MSP_ADC_CH12 = ADC_Channel_12,
	MSP_ADC_CH13 = ADC_Channel_13,
	MSP_ADC_CH14 = ADC_Channel_14,	
	MSP_ADC_CH15 = ADC_Channel_15,	
	MSP_ADC_CH16 = ADC_Channel_16,	
	MSP_ADC_CH17 = ADC_Channel_17,		
}MSP_ADC_CHANNLE;

typedef enum
{
	MSP_ADC_RANK1 = 1,
	MSP_ADC_RANK2 = 2, 
	MSP_ADC_RANK3 = 3, 
	MSP_ADC_RANK4 = 4,
	MSP_ADC_RANK5 = 5,
	MSP_ADC_RANK6 = 6, 
	MSP_ADC_RANK7 = 7, 
	MSP_ADC_RANK8 = 8, 
}MSP_ADC_RANK;

void msp_Adc_Init(MSP_ADC_RESOURCE mspAdcResource);
void msp_Adc_Start(MSP_ADC_RESOURCE mspAdcResource);
void msp_Adc_Stop(MSP_ADC_RESOURCE mspAdcResource);

extern u16 g_u16Adc1Buff[MSP_ADC1_BUFF_SIZE];
extern u16* g_pu16Adc1Buff;

#endif

