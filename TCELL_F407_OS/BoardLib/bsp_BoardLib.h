#ifndef _BSP_BOARDLIB_H_
#define _BSP_BOARDLIB_H_

#include "sys_BoardMap.h"

/*所有板载资源状态*/
typedef struct
{
	/*0.基础模块*/
	u32 LED:1;
	u32 RGB:1;
	u32 BEEP:1;	
	
	/*1.数据存储*/
	u32 STOR_FLASH:1;
	u32 STOR_EEPROM:1;
	u32 STOR_TFSD:1;
	
	/*2.IMU + 磁力计(AHRS)*/
	u32 MD_IMU:1;
	u32 MD_MAG:1;
	u32 MD_AHRS:1;
	u32 BD_IMU:1;
	u32 GPS_MAG:1;
	
	/*3.气压计*/
	u32 BERO_1:1;
	u32 BERO_2:1;	
	
	/*4.超声波*/
	u32 ULTR:1;	
	
	/*5.GPS*/
	u32 MD_GPS:1;
	u32 BD_GPS:1;	
	
	/*6.光流*/
	u32 MD_OPFLOW:1;
	u32 BD_OPFLOW:1;	
	

	/*7.人机交互*/
	u32 HCI_MD_OLED:1;
}BSP_BoardStatus;

/*资源初始化*/
void bsp_BoardLib_Init(BSP_BoardStatus *boardStatus);

/*0.基础模块初始化*/
SYS_RETERR bsp_BASE_Module_Init(BSP_BoardStatus *boardStatus);

/*1.数据存储单元初始化*/
SYS_RETERR bsp_STOR_Module_Init(BSP_BoardStatus *boardStatus);

/*2.AHRS(IMU + 磁力计)模块初始化*/
SYS_RETERR bsp_AHRS_Module_Init(BSP_BoardStatus *boardStatus);

/*3.气压计模块初始化*/
SYS_RETERR bsp_BERO_Module_Init(BSP_BoardStatus *boardStatus);

/*4.超声波模块初始化*/
SYS_RETERR bsp_ULTR_Module_Init(BSP_BoardStatus *boardStatus);

/*5.GPS模块初始化*/
SYS_RETERR bsp_GPS_Module_Init(BSP_BoardStatus *boardStatus);

/*6.光流模块初始化*/
SYS_RETERR bsp_OPFLOW_Module_Init(BSP_BoardStatus *boardStatus);

/*7.人机交互模块初始化*/
SYS_RETERR bsp_HCI_Module_Init(BSP_BoardStatus *boardStatus);

/*多模块初始化结果*/
SYS_RETERR bsp_Mut_Module_Init_Result(SYS_RETERR md1, SYS_RETERR md2, SYS_RETERR md3, SYS_RETERR md4);

extern BSP_BoardStatus g_sBoardStatus;

#endif
