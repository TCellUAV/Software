#ifndef _BSP_ULTR_HCSR04_CMD_H_
#define _BSP_ULTR_HCSR04_CMD_H_

///*===RCWL_1603超声波模块指令===*/
///*单次串口模式*/
//#define RCWL_1603_SET_SINGLE_UART_MODE_CMD			(0x01)	/*发送指令值*/
//#define RCWL_1603_SET_SINGLE_UART_MODE_RCV			(0xA1)	/*从机应答值*/
//		
///*GPIO模式(同HS-SR06)*/		
//#define RCWL_1603_SET_GPIO_MODE_CMD					(0x02)	/*发送指令值*/
//#define RCWL_1603_SET_GPIO_MODE_RCV					(0xA2)	/*从机应答值*/
//		
///*PWM输出模式*/		
//#define RCWL_1603_SET_PWM_OUT_MODE_CMD				(0x03)	/*发送指令值*/
//#define RCWL_1603_SET_PWM_OUT_MODE_RCV				(0xA3)	/*从机应答值*/
//		
///*自动串口模式*/		
//#define RCWL_1603_SET_AUTO_UART_MODE_CMD		    (0x04)	/*发送指令值*/
//#define RCWL_1603_SET_AUTO_UART_MODE_RCV			(0xA4)	/*从机应答值*/

//#define RCWL_1603_GET_MODULE_DISTANCE_CMD			(0xC1)	/*获取测量距离指令值(BYTE1 * 256 + BYTE2)*/
//#define RCWL_1603_GET_MODULE_TEMPERATURE_CMD		(0xC2)	/*获取模块温度值(BYTE1)*/

//#define RCWL_1603_DISABLE_MEASURE_POWER_CMD			(0xC3)	/*测距部分电源关指令值*/
//#define RCWL_1603_DISABLE_MEASURE_POWER_RCV			(0xC3)	/*测距部分电源关应答值*/
//#define RCWL_1603_ENABLE_MEASURE_POWER_CMD			(0xC4)	/*测距部分电源开指令值*/
//#define RCWL_1603_ENABLE_MEASURE_POWER_RCV			(0xC4)	/*测距部分电源开应答值*/

///*获取模块固件信息*/
//#define RCWL_1603_GET_MODULE_FIRMWARE_VERSION_CMD	(0x0F)	/*发送指令值*/

///*===US100超声波模块指令===*/
///*串口模式*/
//#define US100_SET_UART_MODE_CMD						(0)		/*指令无效,插上2.54mm跳线帽即可*/

///*GPIO模式*/
//#define US100_SET_GPIO_MODE_CMD						(0)		/*指令无效,拔掉2.54mm跳线帽即可*/

//#define US100_GET_MODULE_TEMPERATURE_CMD			(0x50)	/*获取模块温度指令*/
//#define US100_GET_MODULE_DISTANCE_CMD				(0x55)	/*获取模块测量距离(BYTE1 * 256 + BYTE2)*/
//#define US100_MODULE_TEMPRATURE_BASE				(45)	/*RealTemp = TEMP - US100_MODULE_TEMPRATURE_BASE*/

#endif
