#include "bsp_BASE_BEEP.h"

BSP_BEEP g_sBeep = {0};

/*³õÊ¼»¯*/
SYS_RETSTATUS bsp_BEEP_Init(BSP_BEEP *beep)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;

	/*beep gpio init*/
	beep->SinglePWM = &g_sBeepPwmOut;
	
	RGB_DELAY_MS(1);
	
	return statusRet;	
}

