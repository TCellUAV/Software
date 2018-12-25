#ifndef _HCI_SHOW_SEND_CONFIG_H_
#define _HCI_SHOW_SEND_CONFIG_H_

/*======================== 下位机和上位机交互配置 ========================*/
/********* 上位机和下位机数据传输方式(任选其一) ********/
#define HOST_SLAVE_DATA_TRANSECEIVE_BY_UART
#undef  HOST_SLAVE_DATA_TRANSECEIVE_BY_USB

/*有线/无线传输方式*/
#define HOST_SLAVE_DATA_TRANSECEIVE_BY_WIRELESS	  	(1)		/*无线传输:1, 有线传输:0*/
/*********************************************/

/*观看数据波形的上位机*/
#define USER_AIRCRAFT_SEND_TO_ANO_HOST 			  	(0) 	/*使用匿名上位机看波形*/

#define USER_AIRCRAFT_SEND_VCAN_HOST   			  	(1) 	/*使用山外上位机看波形*/

#define USER_AIRCRAFT_SEND_BSK_HOST	   			  	(0) 	/*使用天穹(BlueSky)上位机看波形*/	

/*地面站选择*/
#define USER_USE_ANO_HOST_AIRCRAFT_SLAVE_EXCHANGE 	(0) 	/*使用匿名上位机作为地面站*/

#define USER_AIRCRAFT_SEND_BSK_HOST	   			   	(0) 	/*使用天穹(BlueSky)上位机作为地面站*/

#endif

/*======================== OLED显示交互配置 ========================*/
#define HCI_OLED_SHOW_SENSOR_CALIB_PARA_WHEN_BOOT	(0)		/*开机显示传感器(加速度计&磁力计)校准参数*/
#define HCI_OLED_SHOW_PID_CONTROL_PARA_WHEN_BOOT	(0) 	/*开机显示PID控制参数*/

/**********************************************************/
//	.HOST_TARG     = USER_HOST_CHOOSE_ANO, 			/*默认匿名上位机*/
//	.MSG_ID        = USER_ANO_MSG_EXCHANGE,			/*默认作为上位机使用*/
//	.SWITCH_STATUS = USER_HOST_SWITCH_NO,           /*切换状态,默认未切换完成*/

/* 以下信息由 uav_cmd_creater.exe 生成, powered by --- lcx*/

/* cmd type: AAAF AA [DATA_LENTH] [HOST_TARG] [MSG_ID] [PERIOD_5MS] [SUM_CHECK]*/

/* 1.period tick : 20 ms */
/* 2.output type : host type*/


/*------------ 匿名上位机 ------------*/

/*功能:切换至-> 上位机功能(不显示波形)*/
/*  AA AF AA 03 01 FF 04 0A  */
/*功能:切换至-> 传感器原始和滤波值(只显示该波形)*/
/*  AA AF AA 03 01 00 04 0B  */
/*功能:切换至-> 竖直方向融合数据(只显示该波形)*/
/*  AA AF AA 03 01 01 04 0C  */
/*功能:切换至-> 水平X方向融合数据(只显示该波形)*/
/*  AA AF AA 03 01 02 04 0D  */
/*功能:切换至-> 水平Y方向融合数据(只显示该波形)*/
/*  AA AF AA 03 01 03 04 0E  */
/*功能:切换至-> 光流控制数据(只显示该波形)*/
/*  AA AF AA 03 01 04 04 0F  */
/*功能:切换至-> 竖直方向控制(只显示该波形)*/
/*  AA AF AA 03 01 0A 04 15  */
/*功能:切换至-> GPS 水平X方向控制(只显示该波形)*/
/*  AA AF AA 03 01 0B 04 16  */
/*功能:切换至-> GPS 水平Y方向控制(只显示该波形)*/
/*  AA AF AA 03 01 0C 04 17  */
/*功能:切换至-> OPTIC FLOW 水平X方向控制(只显示该波形)*/
/*  AA AF AA 03 01 0D 04 18  */
/*功能:切换至-> OPTIC FLOW 水平Y方向控制(只显示该波形)*/
/*  AA AF AA 03 01 0E 04 19  */
/*功能:切换至-> 水平Z方向控制(只显示该波形)*/
/*  AA AF AA 03 01 0F 04 1A  */


/*------------ 山外上位机 ------------*/

/*功能:切换至-> 上位机功能(不显示波形)*/
/*  AA AF AA 03 02 FF 04 0B  */
/*功能:切换至-> 传感器原始和滤波值(只显示该波形)*/
/*  AA AF AA 03 02 00 04 0C  */
/*功能:切换至-> 竖直方向融合数据(只显示该波形)*/
/*  AA AF AA 03 02 01 04 0D  */
/*功能:切换至-> 水平X方向融合数据(只显示该波形)*/
/*  AA AF AA 03 02 02 04 0E  */
/*功能:切换至-> 水平Y方向融合数据(只显示该波形)*/
/*  AA AF AA 03 02 03 04 0F  */
/*功能:切换至-> 光流控制数据(只显示该波形)*/
/*  AA AF AA 03 02 04 04 10  */
/*功能:切换至-> 竖直方向控制(只显示该波形)*/
/*  AA AF AA 03 02 0A 04 16  */
/*功能:切换至-> GPS 水平X方向控制(只显示该波形)*/
/*  AA AF AA 03 02 0B 04 17  */
/*功能:切换至-> GPS 水平Y方向控制(只显示该波形)*/
/*  AA AF AA 03 02 0C 04 18  */
/*功能:切换至-> OPTIC FLOW 水平X方向控制(只显示该波形)*/
/*  AA AF AA 03 02 0D 04 19  */
/*功能:切换至-> OPTIC FLOW 水平Y方向控制(只显示该波形)*/
/*  AA AF AA 03 02 0E 04 1A  */
/*功能:切换至-> 水平Z方向控制(只显示该波形)*/
/*  AA AF AA 03 02 0F 04 1B  */


/*------------ 蓝天上位机 ------------*/

/*功能:切换至-> 上位机功能(不显示波形)*/
/*  AA AF AA 03 03 FF 04 0C  */
/*功能:切换至-> 传感器原始和滤波值(只显示该波形)*/
/*  AA AF AA 03 03 00 04 0D  */
/*功能:切换至-> 竖直方向融合数据(只显示该波形)*/
/*  AA AF AA 03 03 01 04 0E  */
/*功能:切换至-> 水平X方向融合数据(只显示该波形)*/
/*  AA AF AA 03 03 02 04 0F  */
/*功能:切换至-> 水平Y方向融合数据(只显示该波形)*/
/*  AA AF AA 03 03 03 04 10  */
/*功能:切换至-> 光流控制数据(只显示该波形)*/
/*  AA AF AA 03 03 04 04 11  */
/*功能:切换至-> 竖直方向控制(只显示该波形)*/
/*  AA AF AA 03 03 0A 04 17  */
/*功能:切换至-> GPS 水平X方向控制(只显示该波形)*/
/*  AA AF AA 03 03 0B 04 18  */
/*功能:切换至-> GPS 水平Y方向控制(只显示该波形)*/
/*  AA AF AA 03 03 0C 04 19  */
/*功能:切换至-> OPTIC FLOW 水平X方向控制(只显示该波形)*/
/*  AA AF AA 03 03 0D 04 1A  */
/*功能:切换至-> OPTIC FLOW 水平Y方向控制(只显示该波形)*/
/*  AA AF AA 03 03 0E 04 1B  */
/*功能:切换至-> 水平Z方向控制(只显示该波形)*/
/*  AA AF AA 03 03 0F 04 1C  */
/**********************************************************/
