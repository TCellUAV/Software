#ifndef _SYS_MCUMAP_H_
#define _SYS_MCUMAP_H_

/*Just ReadME*/
/*=== TIM ===*/
/*1.MCU RESOURCE
   ¢ÙAdvanced-control timers:TIM1 and TIM8;
   ¢ÚGeneral-purpose timers :TIM2 to TIM5;
   ¢ÛGeneral-purpose timers :TIM9 to TIM14;
   ¢ÜBase timers 		   :TIM6 and TIM7;
  
  2.USER HAVE USED
	TOTAL: TIM1¡¢TIM2¡¢TIM3¡¢TIM4¡¢TIM5¡¢TIM7¡¢TIM8¡¢TIM9;
	
	TIM1
	{
		PE9 : TIM1_CH1   -------->   PWM_IN_5  (PWM_IN)
		PE11: TIM1_CH2   -------->   PWM_IN_6  (PWM_IN)	
		PE13: TIM1_CH3   -------->   PWM_IN_7  (PWM_IN)
		PE14: TIM1_CH4   -------->   PWM_IN_8  (PWM_IN)			
	} 
	
	TIM2
	{
	
	}	
  
	TIM3
	{
		PA6: TIM3_CH1   -------->   PWM_OUT_4  (PWM_OUT)
		PA7: TIM3_CH2   -------->   PWM_OUT_3  (PWM_OUT)	
		PB0: TIM3_CH3   -------->   PWM_OUT_2  (PWM_OUT)
		PB1: TIM3_CH4   -------->   PWM_OUT_1  (PWM_OUT)	
	} 	

	TIM4
	{
		PE9 : TIM4_CH1   -------->   PWM_IN_1  (PWM_IN)
		PE11: TIM4_CH2   -------->   PWM_IN_2  (PWM_IN)	
		PE13: TIM4_CH3   -------->   PWM_IN_3  (PWM_IN)
		PE14: TIM4_CH4   -------->   PWM_IN_4  (PWM_IN)		
	}	
  
	TIM5
	{
		PA2: TIM5_CH3   -------->   PWM_OUT_5  (PWM_OUT)
		PA3: TIM5_CH4   -------->   PWM_OUT_6  (PWM_OUT)
	}

	TIM7
	{
		Alarm	 -------->	 Task Scheduing(5ms)

	}
	
	TIM8
	{
		PC8: TIM8_CH3   -------->   PWM_OUT_7  (PWM_OUT)
		PC9: TIM8_CH4   -------->   PWM_OUT_8  (PWM_OUT)	
	}	
	
    TIM9   
    {
		Alarm	 -------->	 Test Period(10000us)
    }
	
  3.RESERVED
*/

#endif
