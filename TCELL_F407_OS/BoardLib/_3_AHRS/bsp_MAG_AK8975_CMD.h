#ifndef _BSP_MAG_AK8975_CMD_H_
#define _BSP_MAG_AK8975_CMD_H_

/*AK8975从机地址*/
#define AK8975_SLAVEADDR		    (0x0C)	/*7bit地址*/

/*AK8975寄存器*/
#define AK8975_WIA		    		(0x00)	/*Device ID(R)*/
#define AK8975_INFO		    		(0x01)	/*Device Information for AKM(R)*/
#define AK8975_ST1		    		(0x02)	/*Data Ready(R)*/
#define AK8975_HXL		    		(0x03)	/*X-axis measurement data lower 8bit(R)*/
#define AK8975_HXH		    		(0x04)	/*X-axis measurement data higher 8bit(R)*/
#define AK8975_HYL		    		(0x05)	/*Y-axis measurement data lower 8bit(R)*/
#define AK8975_HYH		    		(0x06)	/*Y-axis measurement data higher 8bit(R)*/
#define AK8975_HZL		    		(0x07)	/*Z-axis measurement data lower 8bit(R)*/
#define AK8975_HZH		    		(0x08)	/*Z-axis measurement data higher 8bit(R)*/
#define AK8975_ST2		    		(0x09)	/*Data Error(R)*/
#define AK8975_CNTL		    		(0x0A)	/*Operation mode setting(R)*/
#define AK8975_RSV		    		(0x0B)	/*RSV register is reserved. Do not use this register(R)*/
#define AK8975_ASTC		    		(0x0C)	/*Self test control(R/W)*/
#define AK8975_TS1					(0x0D)	/*TS1 registers are test registers for shipment test. Do not use these registers(R/W)*/
#define AK8975_TS2					(0x0E)	/*TS2 registers are test registers for shipment test. Do not use these registers(R/W)*/
#define AK8975_I2CDIS		    	(0x0F)	/*This register disables I2C bus interface(R/W)*/
#define AK8975_ASAX					(0x10)	/*Magnetic sensor X-axis sensitivity adjustment value(R)*/
#define AK8975_ASAY					(0x11)	/*Magnetic sensor Y-axis sensitivity adjustment value(R)*/
#define AK8975_ASAZ					(0x12)	/*Magnetic sensor Z-axis sensitivity adjustment value(R)*/

/*AK8975寄存器特征值及对应寄存器*/
#define AK8975_WIA_VALUE		    (0x48)	/*识别码(AK8975_WIA)*/

#define AK8975_DATA_READY			(0x01)	/*Data is ready(AK8975_ST1)*/
#define AK8975_DATA_NOT_READY		(0x00)	/*Normal(AK8975_ST1)*/

#define AK8975_DATA_READ_ERROR		(0x04)	/*Data read error occurred(AK8975_ST2)*/
#define AK8975_DATA_READ_RIGHT		(0x00)	/*Normal(AK8975_ST2)*/
#define AK8975_MAG_SENSOR_OVERFLOW	(0x08)	/*Magnetic sensor overflow occurred(AK8975_ST2)*/
#define AK8975_MAG_SENSOR_NORMAL	(0x00)	/*Normal(AK8975_ST2)*/

#define AK8975_POWER_DOWN_MODE		(0x00)	/*Power-down mode(AK8975_CNTL)*/
#define AK8975_SINGLE_MEAS_MODE		(0x01)	/*Single measurement mode(AK8975_CNTL)*/
#define AK8975_SELF_TEST_MODE		(0x08)	/*Self-test mode(AK8975_CNTL)*/
#define AK8975_FUSE_ROM_ACCESS_MODE	(0x0F)	/*Fuse ROM access mode(AK8975_CNTL)*/

#define AK8975_FIELD_FOR_SELF_TEST	(0x40)  /*Generate magnetic field for self-test(AK8975_ASTC)*/
#define AK8975_SELF_TEST_NOT		(0x00)  /*Self test control Normal(AK8975_ASTC)*/

#define AK8975_I2C_DISABLE_CMD		(0x1B)	/*To disable I2C bus interface,write "00011011" to I2CDIS register(AK8975_I2CDIS)*/
#define AK8975_I2C_DISABLE_CHECK	(0x01)	/*Then I2CDIS bit turns to "1" and I2C bus interface is disabled(AK8975_I2CDIS)*/

/*How to adjust sensitivity
Hadj = H * (((ASA - 128) * 0.5) / 128 + 1)
H    is the measurement data read out from the measurement data register
ASA  is the sensitivity adjustment value
Hadj is the adjusted measurement data.
*/


#endif 
