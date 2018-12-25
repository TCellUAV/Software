#ifndef _BSP_ULTR_US100_CMD_H_
#define _BSP_ULTR_US100_CMD_H_

/*===US100超声波模块指令===*/
/*串口模式*/
#define US100_SET_UART_MODE_CMD						(0)		/*指令无效,插上2.54mm跳线帽即可*/

/*GPIO电平触发模式*/
#define US100_SET_GPIO_MODE_CMD						(0)		/*指令无效,拔掉2.54mm跳线帽即可*/

#define US100_GET_MODULE_TEMPERATURE_CMD			(0x50)	/*获取模块温度指令*/
#define US100_GET_MODULE_DISTANCE_CMD				(0x55)	/*获取模块测量距离(BYTE1 * 256 + BYTE2)*/
#define US100_MODULE_TEMPRATURE_BASE				(45)	/*RealTemp = TEMP - US100_MODULE_TEMPRATURE_BASE*/

#endif
