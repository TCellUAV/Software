#ifndef _BSP_BERO_SPL06_CMD_H_
#define _BSP_BERO_SPL06_CMD_H_

#define SPL06_SLAVEADDR						(0x77)		/*SDO Low: 0x77 / SDO High or NC: 0x77*/
			
/* SPL06内部寄存器*/			
/*data register*/			
#define SPL06_PSR_B2						(0x00)		/*Prresure data[23:16](R)*/
#define SPL06_PSR_B1						(0x01)		/*Prresure data[15:8](R)*/
#define SPL06_PSR_B0						(0x02)		/*Prresure data[7:0](R)*/
#define SPL06_TMP_B2						(0x03)		/*temperature data[23:16](R)*/
#define SPL06_TMP_B1						(0x04)		/*temperature data[15:8](R)*/
#define SPL06_TMP_B0						(0x05)		/*temperature data[7:0](R)*/
				
/*Configuration register*/				
#define SPL06_PRS_CFG						(0x06)		/*Config pressure measurement rate(PM_RATE) and resolution(PM_PRC)(R/W)*/
#define SPL06_TMP_CFG						(0x07)		/*Config temperature measurement rate (TMP_RATE) and resolution (TMP_PRC)*/
#define SPL06_MEAS_CFG						(0x08)		/*Setup measurement mode(COEF_RDY / SENSOR_RDY / TMP_RDY / PRS_RDY / MEAS_CTRL)*/
#define SPL06_CFG_REG						(0x09)		/*Configuration of interrupts, measurement data shift, and FIFO enable*/
				
/*status register*/				
#define SPL06_INT_STS						(0x0A)		/*Interrupt status register. The register is cleared on read*/
#define SPL06_FIFO_STS						(0x0B)		/*FIFO status register(FIFO_FULL / FIFO_EMPTY)*/
				
/*system*/				
#define SPL06_RESET							(0x0C)		/*Flush FIFO or generate soft reset(FIFO_FLUSH / FIFO_RST)*/
#define SPL06_ID							(0x0D)		/*Product and Revision ID(PROD_ID / REV_ID)*/

/*Calibration Coefficients register*/
#define SPL06_COEF_C0_H						(0x10)		/*c0[11:4]				  */
#define SPL06_COEF_C0_L_C1_H				(0x11)		/*c0[3:0]     + c1[11:8]  */
#define SPL06_COEF_C1_L						(0x12)		/*c1[7:0]				  */
#define SPL06_COEF_C00_H					(0x13)		/*c00[19:12]			  */
#define SPL06_COEF_C00_M					(0x14)		/*c00[11:4]				  */
#define SPL06_COEF_C00_L_C10_H				(0x15)		/*c00[3:0]    + c10[19:16]*/
#define SPL06_COEF_C10_M					(0x16)		/*c10[15:8]				  */
#define SPL06_COEF_C10_L					(0x17)		/*c10[7:0]				  */
#define SPL06_COEF_C01_H					(0x18)		/*c01[15:8]               */
#define SPL06_COEF_C01_L					(0x19)		/*c01[7:0]                */
#define SPL06_COEF_C11_H					(0x1A)		/*c11[15:8]				  */
#define SPL06_COEF_C11_L					(0x1B)		/*c11[7:0]                */
#define SPL06_COEF_C20_H					(0x1C)		/*c20[15:8]               */
#define SPL06_COEF_C20_L					(0x1D)		/*c20[7:0]                */
#define SPL06_COEF_C21_H					(0x1E)		/*c21[15:8]   			  */
#define SPL06_COEF_C21_L					(0x1F)		/*c21[7:0]				  */
#define SPL06_COEF_C30_H					(0x20)		/*c30[15:8]				  */
#define SPL06_COEF_C30_L					(0x21)		/*c30[7:0]				  */
			
//#define SPL06_RESERVED					(0x22~0x27)
#define SPL06_COEF_SRCE						(0x28)		/*Internal / External temprature sensor(TMP_COEF_SRCE(r))	*/


/*SPL06寄存器特征值及对应寄存器*/
/*SPL06_PRS_CFG*/
/*PM_RATE(Pressure measurement rate)*/
#define SPL06_PRS_1_MEAS_RATE_W		    	(0x00 << 4)	/*1 measurements*/
#define SPL06_PRS_2_MEAS_RATE_W		    	(0x01 << 4)	/*2 measurements*/
#define SPL06_PRS_4_MEAS_RATE_W		    	(0x02 << 4)	/*4 measurements*/
#define SPL06_PRS_8_MEAS_RATE_W		  	    (0x03 << 4)	/*8 measurements*/
#define SPL06_PRS_16_MEAS_RATE_W		    (0x04 << 4)	/*16 measurements*/
#define SPL06_PRS_32_MEAS_RATE_W		    (0x05 << 4)	/*32 measurements*/
#define SPL06_PRS_64_MEAS_RATE_W		    (0x06 << 4)	/*64 measurements*/
#define SPL06_PRS_128_MEAS_RATE_W		    (0x07 << 4)	/*128 measurements*/
/*PM_PRC(Pressure oversampling rate)*/
#define SPL06_PRS_1_OVERSAMP_RATE_W			(0x00)	/*single times  		   kP=524288,  3.6ms*/
#define SPL06_PRS_2_OVERSAMP_RATE_W			(0x01)	/*2 times_LOW_Power 	   kP=1572864, 5.2ms*/
#define SPL06_PRS_4_OVERSAMP_RATE_W			(0x02)	/*4 times 				   kP=3670016, 8.4ms*/
#define SPL06_PRS_8_OVERSAMP_RATE_W			(0x03)	/*8 times 				   kP=7864320, 14.8ms*/
#define SPL06_PRS_16_OVERSAMP_RATE_W		(0x04)	/*16 times_Standard 	   kP=253952,  27.6ms*/
#define SPL06_PRS_32_OVERSAMP_RATE_W		(0x05)	/*32 times 				   kP=516096,  53.2ms*/
#define SPL06_PRS_64_OVERSAMP_RATE_W		(0x06)	/*64 times_High Precision  kP=1040384, 104.4ms*/
#define SPL06_PRS_128_OVERSAMP_RATE_W		(0x07)	/*128 times 			   kP=2088960, 206.8ms*/

/*SPL06_TMP_CFG*/
/*TMP_EXT(Temperature measurement sensor select)*/
#define SPL06_TMP_INT_SENSOR_W				(0x00 << 7) /*Internal sensor (in ASIC)*/
#define SPL06_TMP_EXT_SENSOR_W				(0x01 << 7) /*External sensor (in pressure sensor MEMS element)*/
/*TMP_RATE(Temperature measurement rate)*/
#define SPL06_TMP_1_MEAS_RATE_W		    	(0x00 << 4)	/*1 measurements*/
#define SPL06_TMP_2_MEAS_RATE_W		    	(0x01 << 4)	/*2 measurements*/
#define SPL06_TMP_4_MEAS_RATE_W		    	(0x02 << 4)	/*4 measurements*/
#define SPL06_TMP_8_MEAS_RATE_W		    	(0x03 << 4)	/*8 measurements*/
#define SPL06_TMP_16_MEAS_RATE_W		    (0x04 << 4)	/*16 measurements*/
#define SPL06_TMP_32_MEAS_RATE_W		    (0x05 << 4)	/*32 measurements*/
#define SPL06_TMP_64_MEAS_RATE_W		    (0x06 << 4)	/*64 measurements*/
#define SPL06_TMP_128_MEAS_RATE_W		    (0x07 << 4)	/*128 measurements*/
/*TMP_PRC(Temperature measurement rate)*/
#define SPL06_TMP_1_OVERSAMP_RATE_W			(0x00)	/*single times              kP=524288,  3.6ms*/
#define SPL06_TMP_2_OVERSAMP_RATE_W			(0x01)	/*2 times_LOW_Power         kP=1572864, 5.2ms*/
#define SPL06_TMP_4_OVERSAMP_RATE_W			(0x02)	/*4 times                   kP=3670016, 8.4ms*/
#define SPL06_TMP_8_OVERSAMP_RATE_W			(0x03)	/*8 times                   kP=7864320, 14.8ms*/
#define SPL06_TMP_16_OVERSAMP_RATE_W		(0x04)	/*16 times_Standard         kP=253952,  27.6ms*/
#define SPL06_TMP_32_OVERSAMP_RATE_W		(0x05)	/*32 times                  kP=516096,  53.2ms*/
#define SPL06_TMP_64_OVERSAMP_RATE_W		(0x06)	/*64 times_High Precision   kP=1040384, 104.4ms*/
#define SPL06_TMP_128_OVERSAMP_RATE_W		(0x07)	/*128 times                 kP=2088960, 206.8ms*/

/*SPL06_MEAS_CFG*/
/*COEF_RDY(Coefficients will be read to the Coefficients Registers after start- up)*/
#define SPL06_COEF_DATA_NOT_READY_R			(0x00) /*Coefficients are not available*/
#define SPL06_COEF_DATA_READY_R				(0x80) /*Coefficients are available*/
/*SENSOR_RDY(The pressure sensor is running through self initialization after start-up)*/
#define SPL06_PRS_SENSOR_INIT_NOT_CPLT_R	(0x00) /*Sensor initialization not complete*/
#define SPL06_PRS_SENSOR_INIT_CPLT_R	    (0x40) /*Sensor initialization complete*/
/*TMP_RDY(Temperature measurement ready)*/
#define SPL06_TMP_NEW_DATA_READY_R			(0x20)	/*new tmp data read, cleared when read*/
#define SPL06_PRS_NEW_DATA_READY_R			(0x10) /*new prs data read, cleared when read*/
/*MEAS_CTRL(Set measurement mode and type)*/
#define SPL06_MEAS_STANDBY_MODE_WR			(0x00) /*(000) Idle / Stop background measurement*/
#define SPL06_MEAS_PRS_COMMAND_MODE_WR		(0x01) /*(001) Pressure Command Mode measurement*/
#define SPL06_MEAS_TMP_COMMAND_MODE_WR		(0x02) /*(010) temperature Command Mode measurement*/
#define SPL06_MEAS_PRS_CONTINU_MODE_WR		(0x05) /*(101) Pressure Background Mode measurement*/
#define SPL06_MEAS_TMP_CONTINU_MODE_WR		(0x06) /*(110) temperature Background Mode measurement*/
#define SPL06_MEAS_PRS_TMP_CONTINU_MODE_WR  (0x07) /*(111) Pressure and temperature Background Mode measurement*/

/*SPL06_CFG_REG*/
/*T_SHIFT(Temperature result bit-shift)*/
#define SPL06_CFG_T_SHIFT_WR				(0x08) /*shift result right in data register,  Must be set to '1' when the oversampling rate is >8 times.*/
/*P_SHIFT(Pressure result bit-shif)*/
#define SPL06_CFG_P_SHIFT_WR				(0x04) /*shift result right in data register,  Must be set to '1' when the oversampling rate is >8 times.*/

/*SPL06_RESET*/
/*SOFT_RST(soft reset)*/
#define SPL06_SOFT_RESET_W					(0x89)	/*(1001) Write '1001' to generate a soft reset. A soft reset will run though the
												  same sequences as in power-on reset, 0x80 fifo clear*/

/*SPL06_ID*/												  
#define SPL06_PROD_REV_ID_R					(0x10)	/*Product and revision ID*/
											
#endif
