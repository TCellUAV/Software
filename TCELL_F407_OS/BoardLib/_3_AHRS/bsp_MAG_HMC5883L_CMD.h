#ifndef _BSP_MAG_HMC5883L_CMD_H_
#define _BSP_MAG_HMC5883L_CMD_H_

/*HMC5883L寄存器*/
#define HMC5883L_SLAVEADDR		    (0x1E)	/*7bit地址*/
#define HMC5883L_CONF_REG_A		    (0x00)	/*配置寄存器A(R/W)*/
#define HMC5883L_CONF_REG_B		    (0x01)	/*配置寄存器B(R/W)*/
#define HMC5883L_MODE_REG		    (0x02)	/*模式寄存器(R/W)*/
#define HMC5883L_DATA_X_MSB_REG	    (0x03)	/*X轴数据高8位寄存器(R)*/
#define HMC5883L_DATA_X_LSB_REG	    (0x04)	/*X轴数据低8位寄存器(R)*/
#define HMC5883L_DATA_Z_MSB_REG	    (0x05)	/*Z轴数据高8位寄存器(R)*/
#define HMC5883L_DATA_Z_LSB_REG	    (0x06)	/*Z轴数据低8位寄存器(R)*/
#define HMC5883L_DATA_Y_MSB_REG	    (0x07)	/*Y轴数据高8位寄存器(R)*/
#define HMC5883L_DATA_Y_LSB_REG	    (0x08)	/*Y轴数据低8位寄存器(R)*/
#define HMC5883L_STATUS_REG		    (0x09)	/*状态寄存器(R)*/
#define HMC5883L_IDENT_REG_A	    (0x0A)	/*识别寄存器A(R)*/
#define HMC5883L_IDENT_REG_B		(0x0B)	/*识别寄存器B(R)*/
#define HMC5883L_IDENT_REG_C		(0x0C)	/*识别寄存器C(R)*/

#define HMC5883L_IDENT_A_CONST_VAL	('H')	/*识别A*/
#define HMC5883L_IDENT_B_CONST_VAL	('4')	/*识别B*/
#define HMC5883L_IDENT_C_CONST_VAL	('3')	/*识别C*/


#define HMC5883L_MAG_GAIN_SCALE0 	(1370)	/*0x00   0.88Ga*/
#define HMC5883L_MAG_GAIN_SCALE1 	(1090)	/*0x20   1.30Ga*/
#define HMC5883L_MAG_GAIN_SCALE2 	(820)	/*0x40   1.90Ga*/
#define HMC5883L_MAG_GAIN_SCALE3 	(660)	/*0x60   2.50Ga*/
#define HMC5883L_MAG_GAIN_SCALE4 	(440)	/*0x80   4.00Ga*/
#define HMC5883L_MAG_GAIN_SCALE5 	(390)	/*0xA0   4.70Ga*/
#define HMC5883L_MAG_GAIN_SCALE6 	(330)	/*0xC0   5.66Ga*/
#define HMC5883L_MAG_GAIN_SCALE7 	(230)	/*0xE0   8.10Ga*/

#endif 
