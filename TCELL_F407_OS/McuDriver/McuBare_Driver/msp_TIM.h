#ifndef _MSP_TIM_H_
#define _MSP_TIM_H_

#include "sys_Platform.h"

/*Time resource*/
typedef enum
{
	MSP_SRC_TIM1  = 1,
	MSP_SRC_TIM2  = 2,
	MSP_SRC_TIM3  = 3,
	MSP_SRC_TIM4  = 4,
	MSP_SRC_TIM5  = 5,
	MSP_SRC_TIM6  = 6,
	MSP_SRC_TIM7  = 7,
	MSP_SRC_TIM8  = 8,
	MSP_SRC_TIM9  = 9,
	MSP_SRC_TIM10 = 5,
	MSP_SRC_TIM11 = 6,
	MSP_SRC_TIM12 = 7,
	MSP_SRC_TIM13 = 8,
	MSP_SRC_TIM14 = 9,	
}MSP_TIM_RESOURCE;

typedef enum
{
	MSP_TIM_CH1 = 1,
	MSP_TIM_CH2 = 2,
	MSP_TIM_CH3 = 4,
	MSP_TIM_CH4 = 8,
}MSP_TIM_CHANNLE;

/**** TIM PWM_Out(脉冲宽度调制输出) ****/
typedef struct
{
	TIM_TypeDef 	 *Tim;
	MSP_TIM_RESOURCE Resource;
	u8 				 NVIC_IRQChannel;
	u32              RCC_Periph_Tim;
	u32 			 RCC_Tree_Mhz;
	u32 			 Prescaler;
	u32 			 Period;
	u32 			 ClockDivision;
	
	/*GPIO*/
	GPIO_TypeDef 	 *CH1_GPIO;
	u16 		 	 CH1_Pin;
	u32 		     RCC_Periph_CH1_GPIO;
	u16 		     CH1_PinSource;
	
	GPIO_TypeDef 	 *CH2_GPIO;
	u16 		 	 CH2_Pin;
	u32 		     RCC_Periph_CH2_GPIO;
	u16 		     CH2_PinSource;

	GPIO_TypeDef 	 *CH3_GPIO;
	u16 		 	 CH3_Pin;
	u32 		     RCC_Periph_CH3_GPIO;
	u16 		     CH3_PinSource;

	GPIO_TypeDef 	 *CH4_GPIO;
	u16 		 	 CH4_Pin;
	u32 		     RCC_Periph_CH4_GPIO;
	u16 		     CH4_PinSource;		
	
	u8 				 GPIO_AF;		
}TimMultiPwmOut;

typedef struct
{
	TIM_TypeDef 	 *Tim;
	MSP_TIM_RESOURCE Resource;
	u8 				 NVIC_IRQChannel;
	u32              RCC_Periph_Tim;
	u32 			 RCC_Tree_Mhz;
	u32 			 Prescaler;
	u32 			 Period;
	u32 			 ClockDivision;
	
	/*GPIO*/
	GPIO_TypeDef 	 *GPIO;
	u16 		 	 Pin;
	u32 		     RCC_Periph_GPIO;
	u16 		     PinSource;	
	
	u8 				 GPIO_AF;	

	MSP_TIM_CHANNLE  CHANNLE;
}TimSinglePwmOut;

/*MultiPwm*/
void msp_TimMultiPwmOut_Init(TimMultiPwmOut *timPwmOut);
void msp_TimMultiPwmOut_SetPluse(TimMultiPwmOut *timPwmOut, MSP_TIM_CHANNLE TIM_CHANNLE, u16 Pulse);

/*SinglePwm*/
void msp_TimSinglePwmOut_Init(TimSinglePwmOut *timPwmOut);
void msp_TimSinglePwmOut_SetPluse(TimSinglePwmOut *timPwmOut, MSP_TIM_CHANNLE TIM_CHANNLE, u16 Pulse);

/**** TIM PWM_In(脉冲宽度调制输入) ****/
typedef struct
{
	TIM_TypeDef 	 *Tim;
	MSP_TIM_RESOURCE Resource;
	u8 				 NVIC_IRQChannel;
	u32              RCC_Periph_Tim;
	u32 			 RCC_Tree_Mhz;
	u32 			 Prescaler;
	u32 			 Period;
	u32 			 ClockDivision;
	
	/*GPIO*/
	GPIO_TypeDef 	 *CH1_GPIO;
	u16 		 	 CH1_Pin;
	u32 		     RCC_Periph_CH1_GPIO;
	u16 		     CH1_PinSource;
	
	GPIO_TypeDef 	 *CH2_GPIO;
	u16 		 	 CH2_Pin;
	u32 		     RCC_Periph_CH2_GPIO;
	u16 		     CH2_PinSource;

	GPIO_TypeDef 	 *CH3_GPIO;
	u16 		 	 CH3_Pin;
	u32 		     RCC_Periph_CH3_GPIO;
	u16 		     CH3_PinSource;

	GPIO_TypeDef 	 *CH4_GPIO;
	u16 		 	 CH4_Pin;
	u32 		     RCC_Periph_CH4_GPIO;
	u16 		     CH4_PinSource;		
	
	u8 				 GPIO_AF;	
}TimPwmIn;

void msp_TimPwmIn_Init(TimPwmIn *timPwmIn);

/**** TIM ALARM(定时器) ****/
typedef struct
{
	TIM_TypeDef 	 *Tim;
	MSP_TIM_RESOURCE Resource;
	u8 				 NVIC_IRQChannel;
	u32              RCC_Periph_Tim;
	u32 			 RCC_Tree_Mhz;
	u32 			 Prescaler;
	u32 			 Period;
	u32 			 ClockDivision;
}TimAlarm;

void msp_TimAlarm_Init(TimAlarm *timAlarm);
/*******************************/

/***** TIM ENCODER(编码器) *****/
typedef struct
{
	s16 TargANbr;	/*对象A的计数值*/
	s16 TargBNbr;	/*对象B的计数值*/
}MSP_EncoderNbr;

void msp_TimEncoder_Init(MSP_TIM_RESOURCE mspTimResource);
MSP_EncoderNbr* msp_TimEncoder_GetEncoderNbr(MSP_TIM_RESOURCE mspTimResource);
/*******************************/

#endif
