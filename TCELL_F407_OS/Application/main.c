#include "sys_OsTask.h"

/*注:因硬件的外部晶振是8Mhz,而官方固件标准是25Mhz;
	 因此使用本套程序时,需要做以下两点修改:
  1.HSE晶振频率修改:
  stm32f4xx.h文件中,第144行:
  #define HSE_VALUE    ((uint32_t)8000000) 修改为
  #define HSE_VALUE    ((uint32_t)25000000) 正确的外部晶振(25Mhz)
  
  2.时钟源倍频分频修改:
	system_stm32f4xx.c文件中

	第371行 PLL_M (4)   修改为   PLL_M (25) 
	第384行 PLL_Q (4)   修改为   PLL_Q (7)
	第401行 PLL_N (168) 修改为   PLL_N (336)
	第403行 PLL_P (2)   修改为   PLL_P (2)   
	
   3.本程序I2C采用模拟I2C,延时采用__NOP(),
   因此先测得while(1--){__NOP();}的时间
   在 mcu_driver_init() 中的 nop_delay_init()
   测得前取消注释,测得后加上注释
  */

int main(void)
{
	/*MCU中断管理初始化*/
	mcu_nvic_init();
	
	/*MCU驱动初始化*/
	mcu_driver_init();
	
	/*RTOS 组件初始化(必须放在硬件初始化前面)*/
	rtos_unit_init();
	
	/*硬件初始化*/
	hardware_init();
	
	/*系统参数初始化/读取*/
	uav_system_init();
	
	/*创建多任务线程*/
	rtos_thread_create();
	
	/*启用CPU使用率检测线程(空闲线程的钩子函数中执行)*/
	cpu_usage_init();
	
	/*系统初始化后工作*/
	work_after_system_init();
	
	return 0;
}

