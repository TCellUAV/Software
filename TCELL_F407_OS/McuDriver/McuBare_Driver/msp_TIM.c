#include "msp_TIM.h"
#include "msp_GPIO.h"

/**** TIM PWM(脉冲宽度调制) ****/
/*MultiPwm*/
void msp_TimMultiPwmOut_Init(TimMultiPwmOut *timPwmOut)
{
	GPIO_InitTypeDef 	    GPIO_InitStruct	;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef       TIM_OCInitStruct;	
	
	/*GPIO & TIM Channel config*/
	if (timPwmOut->CH1_Pin != 0)
	{
		/*GPIO config*/
		GPIO_InitStruct.GPIO_Pin   = timPwmOut->CH1_Pin;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;	
		GPIO_Init(timPwmOut->CH1_GPIO, &GPIO_InitStruct);
		
		/*AF*/
		GPIO_PinAFConfig(timPwmOut->CH1_GPIO, timPwmOut->CH1_PinSource, timPwmOut->GPIO_AF);
		
		/*Tim Channle config*/
		TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;   /*选择定时器模式:TIM脉冲宽度调制模式2*/
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; /*比较输出使能*/
		TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_High; /*TIM输出比较极性高*/
		TIM_OCInitStruct.TIM_Pulse       = 0;					  /*初始化0*/			
		TIM_OC1Init(timPwmOut->Tim, &TIM_OCInitStruct); /*根据T指定的参数初始化外设TIMxOC1*/
		
		/*使能TIMx在CCR1上的预装载寄存器*/
		TIM_OC1PreloadConfig(timPwmOut->Tim, TIM_OCPreload_Enable);		
	}
	
	if (timPwmOut->CH2_Pin != 0)
	{
		/*GPIO config*/
		GPIO_InitStruct.GPIO_Pin   = timPwmOut->CH2_Pin;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;	
		GPIO_Init(timPwmOut->CH2_GPIO, &GPIO_InitStruct);
		
		/*AF*/
		GPIO_PinAFConfig(timPwmOut->CH2_GPIO, timPwmOut->CH2_PinSource, timPwmOut->GPIO_AF);
		
		/*Tim Channle config*/
		TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;   /*选择定时器模式:TIM脉冲宽度调制模式2*/
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; /*比较输出使能*/
		TIM_OCInitStruct.TIM_Pulse       = 0;					  /*初始化0*/		
		TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_High; /*TIM输出比较极性高*/
		TIM_OC2Init(timPwmOut->Tim, &TIM_OCInitStruct); /*根据T指定的参数初始化外设TIMxOC2*/
		
		/*使能TIMx在CCR1上的预装载寄存器*/
		TIM_OC2PreloadConfig(timPwmOut->Tim, TIM_OCPreload_Enable);		
	}

	if (timPwmOut->CH3_Pin != 0)
	{
		/*GPIO config*/
		GPIO_InitStruct.GPIO_Pin   = timPwmOut->CH3_Pin;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;	
		GPIO_Init(timPwmOut->CH3_GPIO, &GPIO_InitStruct);
		
		/*AF*/
		GPIO_PinAFConfig(timPwmOut->CH3_GPIO, timPwmOut->CH3_PinSource, timPwmOut->GPIO_AF);
		
		/*Tim Channle config*/
		TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;   /*选择定时器模式:TIM脉冲宽度调制模式2*/
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; /*比较输出使能*/
		TIM_OCInitStruct.TIM_Pulse       = 0;					  /*初始化0*/		
		TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_High; /*TIM输出比较极性高*/
		TIM_OC3Init(timPwmOut->Tim, &TIM_OCInitStruct); /*根据T指定的参数初始化外设TIMxOC3*/
		
		/*使能TIMx在CCR1上的预装载寄存器*/
		TIM_OC3PreloadConfig(timPwmOut->Tim, TIM_OCPreload_Enable);			
	}

	if (timPwmOut->CH4_Pin != 0)
	{
		/*GPIO config*/
		GPIO_InitStruct.GPIO_Pin   = timPwmOut->CH4_Pin;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;	
		GPIO_Init(timPwmOut->CH4_GPIO, &GPIO_InitStruct);
		
		/*AF*/
		GPIO_PinAFConfig(timPwmOut->CH4_GPIO, timPwmOut->CH4_PinSource, timPwmOut->GPIO_AF);
		
		/*Tim Channle config*/
		TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;   /*选择定时器模式:TIM脉冲宽度调制模式2*/
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; /*比较输出使能*/
		TIM_OCInitStruct.TIM_Pulse       = 0;					  /*初始化0*/
		TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_High; /*TIM输出比较极性高*/
		TIM_OC4Init(timPwmOut->Tim, &TIM_OCInitStruct); /*根据T指定的参数初始化外设TIMxOC3*/
		
		/*使能TIMx在CCR1上的预装载寄存器*/
		TIM_OC4PreloadConfig(timPwmOut->Tim, TIM_OCPreload_Enable);			
	}	
	
	/*Tim config*/
	TIM_TimeBaseInitStruct.TIM_Period            = timPwmOut->Period;		/*设定计数器自动重装值*/
	TIM_TimeBaseInitStruct.TIM_Prescaler         = timPwmOut->Prescaler;	/*预分频器*/
	TIM_TimeBaseInitStruct.TIM_ClockDivision     = timPwmOut->ClockDivision;/*置时钟分割:TDTS = Tck_tim*/
	TIM_TimeBaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up;		/*TIM向上计数模式*/
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;				        /*重复比较次数更新事件*/
	TIM_TimeBaseInit(timPwmOut->Tim, &TIM_TimeBaseInitStruct);
	
	/*使能TIMx_CHx在CCR2上的预装载寄存器*/
	TIM_ARRPreloadConfig(timPwmOut->Tim, ENABLE);
	
	/*使能定时器*/
	TIM_Cmd(timPwmOut->Tim, ENABLE);

	/*开始启动定时器输出pwm,这句是高级定时器才有的,输出pwm必须打开*/
//	TIM_CtrlPWMOutputs(timPwmOut->Tim, ENABLE);
	
	/*初始化PWM输出脉冲宽度,电调有效行程最小值(停转值)*/
	msp_TimMultiPwmOut_SetPluse(timPwmOut, MSP_TIM_CH1, ESC_MIN_PULSE_ZERO_SPEED_VALUE);
	msp_TimMultiPwmOut_SetPluse(timPwmOut, MSP_TIM_CH2, ESC_MIN_PULSE_ZERO_SPEED_VALUE);
	msp_TimMultiPwmOut_SetPluse(timPwmOut, MSP_TIM_CH3, ESC_MIN_PULSE_ZERO_SPEED_VALUE);
	msp_TimMultiPwmOut_SetPluse(timPwmOut, MSP_TIM_CH4, ESC_MIN_PULSE_ZERO_SPEED_VALUE);	
}

/*PWM施加*/
void msp_TimMultiPwmOut_SetPluse(TimMultiPwmOut *timPwmOut, MSP_TIM_CHANNLE TIM_CHANNLE, u16 Pulse)
{
	switch(TIM_CHANNLE)
	{
		case MSP_TIM_CH1:
		{
			TIM_SetCompare1(timPwmOut->Tim, Pulse);
		}break;
		
		case MSP_TIM_CH2:
		{
			TIM_SetCompare2(timPwmOut->Tim, Pulse);		
		}break;

		case MSP_TIM_CH3:
		{
			TIM_SetCompare3(timPwmOut->Tim, Pulse);		
		}break;
		
		case MSP_TIM_CH4:
		{
			TIM_SetCompare4(timPwmOut->Tim, Pulse);		
		}break;

		default:break;
	}
}

/*SinglePwm*/
void msp_TimSinglePwmOut_Init(TimSinglePwmOut *timPwmOut)
{
	GPIO_InitTypeDef 	    GPIO_InitStruct	;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef       TIM_OCInitStruct;	
	
	/*GPIO & TIM Channel config*/
	/*GPIO config*/
	GPIO_InitStruct.GPIO_Pin   = timPwmOut->Pin;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;	
	GPIO_Init(timPwmOut->GPIO, &GPIO_InitStruct);
		
	/*AF*/
	GPIO_PinAFConfig(timPwmOut->GPIO, timPwmOut->PinSource, timPwmOut->GPIO_AF);
		
	/*Tim Channle config*/
	TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;   /*选择定时器模式:TIM脉冲宽度调制模式2*/
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; /*比较输出使能*/
	TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_High; /*TIM输出比较极性高*/
	TIM_OCInitStruct.TIM_Pulse       = 0;					  /*初始化0*/			
	TIM_OC1Init(timPwmOut->Tim, &TIM_OCInitStruct); /*根据T指定的参数初始化外设TIMxOC1*/
		
	/*使能TIMx在CCR1上的预装载寄存器*/
	TIM_OC1PreloadConfig(timPwmOut->Tim, TIM_OCPreload_Enable);		
	
	/*Tim config*/
	TIM_TimeBaseInitStruct.TIM_Period            = timPwmOut->Period;		/*设定计数器自动重装值*/
	TIM_TimeBaseInitStruct.TIM_Prescaler         = timPwmOut->Prescaler;	/*预分频器*/
	TIM_TimeBaseInitStruct.TIM_ClockDivision     = timPwmOut->ClockDivision;/*置时钟分割:TDTS = Tck_tim*/
	TIM_TimeBaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up;		/*TIM向上计数模式*/
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;				        /*重复比较次数更新事件*/
	TIM_TimeBaseInit(timPwmOut->Tim, &TIM_TimeBaseInitStruct);
	
	/*使能TIMx_CHx在CCR2上的预装载寄存器*/
	TIM_ARRPreloadConfig(timPwmOut->Tim, ENABLE);
	
	/*使能定时器*/
	TIM_Cmd(timPwmOut->Tim, ENABLE);

	/*开始启动定时器输出pwm,这句是高级定时器才有的,输出pwm必须打开*/
//	TIM_CtrlPWMOutputs(timPwmOut->Tim, ENABLE);
	
	/*初始化PWM输出脉冲宽度,电调有效行程最小值(停转值)*/
	msp_TimSinglePwmOut_SetPluse(timPwmOut, timPwmOut->CHANNLE, 0);
}

void msp_TimSinglePwmOut_SetPluse(TimSinglePwmOut *timPwmOut, MSP_TIM_CHANNLE TIM_CHANNLE, u16 Pulse)
{
	switch(TIM_CHANNLE)
	{
		case MSP_TIM_CH1:
		{
			TIM_SetCompare1(timPwmOut->Tim, Pulse);
		}break;
		
		case MSP_TIM_CH2:
		{
			TIM_SetCompare2(timPwmOut->Tim, Pulse);		
		}break;

		case MSP_TIM_CH3:
		{
			TIM_SetCompare3(timPwmOut->Tim, Pulse);		
		}break;
		
		case MSP_TIM_CH4:
		{
			TIM_SetCompare4(timPwmOut->Tim, Pulse);		
		}break;

		default:break;
	}
}
/*******************************/

/*=== PWM_In输入捕获 ===*/
void msp_TimPwmIn_Init(TimPwmIn *timPwmIn)
{
	GPIO_InitTypeDef 	    GPIO_InitStruct	;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef       TIM_ICInitStruct;
	
	/*GPIO & TIM Channel config*/
	if (timPwmIn->CH1_Pin != 0)
	{
		/*GPIO config*/
		GPIO_InitStruct.GPIO_Pin   = timPwmIn->CH1_Pin;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;	
		GPIO_Init(timPwmIn->CH1_GPIO, &GPIO_InitStruct);
		
		/*AF*/
		GPIO_PinAFConfig(timPwmIn->CH1_GPIO, timPwmIn->CH1_PinSource, timPwmIn->GPIO_AF);
		
		/*Tim Channle config*/
		TIM_ICInitStruct.TIM_Channel     = TIM_Channel_1;			 /*选择输入端 IC1映射到TI1上*/
		TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Rising;	 /*上升沿捕获*/
		TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; /*映射到TI1上*/
		TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 /*配置输入分频,不分频*/
		TIM_ICInitStruct.TIM_ICFilter    = 0x00;				  	 /*配置输入滤波器 不滤波*/
		TIM_ICInit(timPwmIn->Tim, &TIM_ICInitStruct);
		
		/*允许更新中断,允许CC1IE捕获中断*/
		TIM_ITConfig(timPwmIn->Tim, TIM_IT_Update | TIM_IT_CC1, ENABLE);		
	}
	
	if (timPwmIn->CH2_Pin != 0)
	{
		/*GPIO config*/		
		GPIO_InitStruct.GPIO_Pin  = timPwmIn->CH2_Pin;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;			
		GPIO_Init(timPwmIn->CH2_GPIO, &GPIO_InitStruct);

		/*AF*/
		GPIO_PinAFConfig(timPwmIn->CH2_GPIO, timPwmIn->CH2_PinSource, timPwmIn->GPIO_AF);		

		/*Tim Channle config*/
		TIM_ICInitStruct.TIM_Channel     = TIM_Channel_2;			 /*选择输入端 IC1映射到TI1上*/
		TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Rising;	 /*上升沿捕获*/
		TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; /*映射到TI1上*/
		TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 /*配置输入分频,不分频*/
		TIM_ICInitStruct.TIM_ICFilter    = 0x00;				  	 /*配置输入滤波器 不滤波*/
		TIM_ICInit(timPwmIn->Tim, &TIM_ICInitStruct);	

		/*允许更新中断,允许CC1IE捕获中断*/
		TIM_ITConfig(timPwmIn->Tim, TIM_IT_Update | TIM_IT_CC2, ENABLE);
	}

	if (timPwmIn->CH3_Pin != 0)
	{
		/*GPIO config*/		
		GPIO_InitStruct.GPIO_Pin  = timPwmIn->CH3_Pin;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;			
		GPIO_Init(timPwmIn->CH3_GPIO, &GPIO_InitStruct);

		/*AF*/
		GPIO_PinAFConfig(timPwmIn->CH3_GPIO, timPwmIn->CH3_PinSource, timPwmIn->GPIO_AF);		

		/*Tim Channle config*/
		TIM_ICInitStruct.TIM_Channel     = TIM_Channel_3;			 /*选择输入端 IC1映射到TI1上*/
		TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Rising;	 /*上升沿捕获*/
		TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; /*映射到TI1上*/
		TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 /*配置输入分频,不分频*/
		TIM_ICInitStruct.TIM_ICFilter    = 0x00;				  	 /*配置输入滤波器 不滤波*/
		TIM_ICInit(timPwmIn->Tim, &TIM_ICInitStruct);		

		/*允许更新中断,允许CC1IE捕获中断*/
		TIM_ITConfig(timPwmIn->Tim, TIM_IT_Update | TIM_IT_CC3, ENABLE);		
	}

	if (timPwmIn->CH4_Pin != 0)
	{
		/*GPIO config*/		
		GPIO_InitStruct.GPIO_Pin   = timPwmIn->CH4_Pin;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;		
		GPIO_Init(timPwmIn->CH4_GPIO, &GPIO_InitStruct);

		/*AF*/
		GPIO_PinAFConfig(timPwmIn->CH4_GPIO, timPwmIn->CH4_PinSource, timPwmIn->GPIO_AF);		

		/*Tim Channle config*/
		TIM_ICInitStruct.TIM_Channel     = TIM_Channel_4;			 /*选择输入端 IC1映射到TI1上*/
		TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Rising;	 /*上升沿捕获*/
		TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; /*映射到TI1上*/
		TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 /*配置输入分频,不分频*/
		TIM_ICInitStruct.TIM_ICFilter    = 0x00;				  	 /*配置输入滤波器 不滤波*/
		TIM_ICInit(timPwmIn->Tim, &TIM_ICInitStruct);
		
		/*允许更新中断,允许CC1IE捕获中断*/
		TIM_ITConfig(timPwmIn->Tim, TIM_IT_Update | TIM_IT_CC4, ENABLE);		
	}	
	
	/*Tim config*/
	TIM_TimeBaseInitStruct.TIM_Period        = timPwmIn->Period;		/*设定计数器自动重装值*/
	TIM_TimeBaseInitStruct.TIM_Prescaler     = timPwmIn->Prescaler;		/*预分频器*/
	TIM_TimeBaseInitStruct.TIM_ClockDivision = timPwmIn->ClockDivision;	/*置时钟分割:TDTS = Tck_tim*/
	TIM_TimeBaseInitStruct.TIM_CounterMode   = TIM_CounterMode_Up;		/*TIM向上计数模式*/
	TIM_TimeBaseInit(timPwmIn->Tim, &TIM_TimeBaseInitStruct);
	
	/*使能定时器*/
	TIM_Cmd(timPwmIn->Tim, ENABLE);
}

/*=== TIM ALARM(定时器) ===*/
void msp_TimAlarm_Init(TimAlarm *timAlarm)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	/*TIM定时器功能配置*/
	TIM_TimeBaseStructure.TIM_Prescaler     = timAlarm->Prescaler;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period        = timAlarm->Period;
	TIM_TimeBaseStructure.TIM_ClockDivision = timAlarm->ClockDivision;
	TIM_TimeBaseInit(timAlarm->Tim, &TIM_TimeBaseStructure);
	
	/*清除中断更新标志位*/
	TIM_ClearFlag(timAlarm->Tim, TIM_FLAG_Update);

	/*使能指定的TIM中断,允许更新中断*/
	TIM_ITConfig(timAlarm->Tim, TIM_IT_Update, ENABLE );  
	
	/*使能TIM*/
	TIM_Cmd(timAlarm->Tim, ENABLE);
}
/*******************************/

/***** TIM ENCODER(编码器) *****/
void msp_TimEncoder_Init(MSP_TIM_RESOURCE mspTimResource)
{	
//	GPIO_InitTypeDef         GPIO_InitStruct;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_ICInitTypeDef        TIM_ICInitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
//	
//	/***** MOTOR B ENCODER *****/
//	GPIO_InitStruct.GPIO_Pin   = MSP_ENCODER_HB_1PIN | MSP_ENCODER_HB_2PIN;
//	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStruct);
//	
//	/*Tim Base Init*/
//	TIM_TimeBaseStructure.TIM_Prescaler     = MSP_TIM2_ENCODER_PRESCALER;
//	TIM_TimeBaseStructure.TIM_Period        = MSP_TIM2_ENCODER_PERIOD;  
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  
//	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up; 
//	TIM_TimeBaseInit(MSP_TIM2_ENCODER_TIMX, &TIM_TimeBaseStructure);
//	
//	/*Tim 编码器:TIM_ICPolarity_Rising上升沿捕获*/
//	TIM_EncoderInterfaceConfig(MSP_TIM2_ENCODER_TIMX, TIM_EncoderMode_TI12, \
//						       TIM_ICPolarity_Falling, TIM_ICPolarity_Falling); 
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_ICFilter = MSP_TIM2_ENCODER_ICFILTER; /*无滤波器*/
//	TIM_ICInit(MSP_TIM2_ENCODER_TIMX, &TIM_ICInitStructure);

//	/***** MOTOR A ENCODER *****/
//	GPIO_InitStruct.GPIO_Pin   = MSP_ENCODER_HA_1PIN | MSP_ENCODER_HA_2PIN;
//	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStruct);
// 
//	/*Tim Base Init*/
//	TIM_TimeBaseStructure.TIM_Prescaler     = MSP_TIM4_ENCODER_PRESCALER;
//	TIM_TimeBaseStructure.TIM_Period        = MSP_TIM4_ENCODER_PERIOD;  
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //设置时钟分频系数：不分频
//	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;  //向上计数模式 
//	TIM_TimeBaseInit(MSP_TIM4_ENCODER_TIMX, &TIM_TimeBaseStructure);
//	
//	/*Tim 编码器:TIM_ICPolarity_Rising上升沿捕获*/
//	TIM_EncoderInterfaceConfig(MSP_TIM4_ENCODER_TIMX, TIM_EncoderMode_TI12, \
//							   TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_ICFilter = MSP_TIM4_ENCODER_ICFILTER; //无滤波器
//	TIM_ICInit(MSP_TIM4_ENCODER_TIMX, &TIM_ICInitStructure);	
//	
//	/*清除定时器TIM2/4中断标志位*/
//	TIM_ClearFlag(MSP_TIM2_ENCODER_TIMX, TIM_FLAG_Update);
//	TIM_ClearFlag(MSP_TIM4_ENCODER_TIMX, TIM_FLAG_Update);
//	
//	/*使能计数器TIM2/TIM4*/
//	TIM_Cmd(MSP_TIM2_ENCODER_TIMX, ENABLE);
//	TIM_Cmd(MSP_TIM4_ENCODER_TIMX, ENABLE);	
}

MSP_EncoderNbr* msp_TimEncoder_GetEncoderNbr(MSP_TIM_RESOURCE mspTimResource)
{
	MSP_EncoderNbr *psEncoderNbr;
//	
//	psEncoderNbr->TargANbr = TIM_GetCounter(MSP_TIM2_ENCODER_TIMX);
//	psEncoderNbr->TargBNbr = TIM_GetCounter(MSP_TIM4_ENCODER_TIMX);
//	
//	TIM_SetCounter(MSP_TIM2_ENCODER_TIMX, 0);
//	TIM_SetCounter(MSP_TIM4_ENCODER_TIMX, 0);	
	
	return psEncoderNbr;
}
/*******************************/
