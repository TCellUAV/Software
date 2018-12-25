#ifndef _BSP_BERO_MS5611_CMD_H_
#define _BSP_BERO_MS5611_CMD_H_                                                          _H_

/*****************************************************
 MS561101BA操作指令
 *****************************************************/
/*addresses of the device*/
//CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
//CBR=0 0x77 I2C address when CSB is connected to LOW (GND) 
#define MS5611_ADDR   		  		 0x77    
/*command for the device*/		
#define MS5611_D1 			  		 0x40	/*Digital pressure value*/
#define MS5611_D2 			  		 0x50	/*Digital temprature value*/
#define MS5611_RESET 		  		 0x1E	/*reset*/
/*ADC Read*/
#define MS5611_ADC_READ				 0x00
/*D1 and D2 result size(bytes)*/     
#define MS5611_D1D2_SIZE 	  		 3
/*OSR (Over Sampling Ratio) constants*/
#define MS5611_OSR_256 		  		 0x00
#define MS5611_OSR_512 		  		 0x02
#define MS5611_OSR_1024 	  		 0x04
#define MS5611_OSR_2048 	  		 0x06
#define MS5611_OSR_4096 	  		 0x08
/*Valid start address can be read*/
#define MS5611_PROM_BASE_ADDR        0xA2     //by adding ints from 0 to 6 we can read all the prom configuration values. 
/*C1 will be at 0xA2 and all the subsequent are multiples of 2*/
#define MS5611_PROM_REG_COUNT        6 	      //number of registers in the PROM
#define MS5611_PROM_REG_SIZE         2 	      //size in bytes of a prom registry.

/*****************************************************
 MS561101BA其他相关宏
 *****************************************************/
#define MS5611_CONVERSION_TIME		 10       //OSR 4096 max:9.04ms
#define MS5611_EXTRA_PRECISION       5		  //trick to add more precision to the pressure and temp readings

#endif
