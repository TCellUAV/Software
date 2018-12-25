#ifndef _BSP_IMU_MPU6050_CMD_H_
#define _BSP_IMU_MPU6050_CMD_H_

/*****************************************************
 MPU6050内部寄存器
 *****************************************************/
#define MPU_SLAVEADDR   	(0x68) 	/*MPU6050从机地址*/
#define MPU_SMPRT_DIV		(0x19)	/*陀螺仪采样频率相关*/		
#define MPU_CONFIG			(0x1A)	/*低通滤波器,典型值*/
#define MPU_GYRO_CONFIG		(0x1B)	/*陀螺仪自检及测量范围,典型值:0x18(不自检,2000deg/s)*/ 
#define MPU_ACCEL_CONFIG	(0x1C)	/*加速度自检,测量范围及高通滤波频率,典型值:0x01(不自检,2G,5hz)*/
#define MPU_FIFO_EN			(0X23)	/*各功能的FIFO的使能或失能*/
#define MPU_I2C_MST_CTRL	(0x24)	/*Master模式的通信速率*/
#define MPU_I2C_SLV0_ADDR   (0x25)	/*从机地址+读写*/
#define MPU_I2C_SLV0_REG    (0x26)	/*从机寄存器*/
#define MPU_I2C_SLV0_CTRL   (0x27)	/*使能操作及操作从机个数*/
#define MPU_INT_PIN_CFG		(0x37)	/*中断/旁路设置*/
#define MPU_INT_ENABLE		(0x38)	/*中断使能寄存器*/
#define MPU_ACCEL_XOUT_H	(0x3B)	/*加速度计输出*/
#define MPU_ACCEL_XOUT_L	(0x3C)	
#define MPU_ACCEL_YOUT_H	(0x3D)
#define MPU_ACCEL_YOUT_L	(0x3E)
#define MPU_ACCEL_ZOUT_H	(0x3F)
#define MPU_ACCEL_ZOUT_L	(0x40)	/*加速度计输出*/
#define MPU_TEMP_OUT_H		(0x41)	/*温度计高*/	
#define MPU_TEMP_OUT_L		(0x42)	/*温度计低*/
#define MPU_GYRO_XOUT_H		(0x43)	/*陀螺仪输出*/
#define MPU_GYRO_XOUT_L		(0x44)
#define MPU_GYRO_YOUT_H		(0x45)
#define MPU_GYRO_YOUT_L		(0x46)
#define MPU_GYRO_ZOUT_H		(0x47)
#define MPU_GYRO_ZOUT_L		(0x48)	/*陀螺仪输出*/
#define MPU_I2C_SLV0_DATA   (0x49)	/*从机0数据存储位置的起始地址*/
#define MPU_I2C_SLV0_DO		(0x63)	/*写给从机的数据*/
#define MPU_USER_CTRL		(0x6A)	/*用户控制寄存器*/
#define MPU_PWR_MGMT_1		(0x6B)	/*电源管理,典型值:0x01(正常启用)*/
#define MPU_PWR_MGMT_2		(0x6C)	/*这只工作模式,还是唤醒模式*/
#define MPU_WHO_AM_I		(0x75)	/*IIC从机地址*/

#endif
