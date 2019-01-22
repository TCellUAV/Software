#include "bsp_BoardLib.h"
#include "ahrs_Caculation.h"
#include "hci_oledshow.h"

BSP_BoardStatus g_sBoardStatus = {0}; 

/*资源初始化*/
void bsp_BoardLib_Init(BSP_BoardStatus *boardStatus)
{
	SYS_RETERR errRet = SYS_RETERR_FF;
	
	/*上电延时稳定*/
	sys_DelayMs(500);

	/*0.人机交互模块初始化*/
	errRet = bsp_HCI_Module_Init(boardStatus);

	/*显示飞行器启动logo,并保持3S*/
	hci_Show_AircraftLogoHoldMs(1500);
	
	/*显示初始化进度*/
	hci_Show_InitRateOfProgress(BSP_TOTAL_MODULE_TYPE_NUMBER, HCI_SHOW_INIT_HCI_MODE, errRet);
	
	/*1.基础模块初始化*/
	errRet = bsp_BASE_Module_Init(boardStatus);
	
	/*显示初始化进度*/
	hci_Show_InitRateOfProgress(BSP_TOTAL_MODULE_TYPE_NUMBER, HCI_SHOW_INIT_BASE_MODE, errRet);	

	/*2.数据存储单元初始化*/
	errRet = bsp_STOR_Module_Init(boardStatus);

	/*显示初始化进度*/
	hci_Show_InitRateOfProgress(BSP_TOTAL_MODULE_TYPE_NUMBER, HCI_SHOW_INIT_STOR_MODE, errRet);

	/*3.AHRS(IMU + 磁力计)模块初始化*/
	errRet = bsp_AHRS_Module_Init(boardStatus);

	/*显示初始化进度*/
	hci_Show_InitRateOfProgress(BSP_TOTAL_MODULE_TYPE_NUMBER, HCI_SHOW_INIT_AHRS_MODE, errRet);

	/*4.气压计模块初始化*/
	errRet = bsp_BERO_Module_Init(boardStatus);

	/*显示初始化进度*/
	hci_Show_InitRateOfProgress(BSP_TOTAL_MODULE_TYPE_NUMBER, HCI_SHOW_INIT_BERO_MODE, errRet);

	/*5.超声波模块初始化*/
	errRet = bsp_ULTR_Module_Init(boardStatus);

	/*显示初始化进度*/
	hci_Show_InitRateOfProgress(BSP_TOTAL_MODULE_TYPE_NUMBER, HCI_SHOW_INIT_ULTR_MODE, errRet);

	/*6.GPS模块初始化*/
	errRet = bsp_GPS_Module_Init(boardStatus);

	/*显示初始化进度*/
	hci_Show_InitRateOfProgress(BSP_TOTAL_MODULE_TYPE_NUMBER, HCI_SHOW_INIT_GPS_MODE, errRet);

	/*7.光流模块初始化*/
	errRet = bsp_OPFLOW_Module_Init(boardStatus);
	
	/*显示初始化进度*/
	hci_Show_InitRateOfProgress(BSP_TOTAL_MODULE_TYPE_NUMBER, HCI_SHOW_INIT_OPFLOW_MODE, errRet);	
}



/*0.基础模块初始化*/
SYS_RETERR bsp_BASE_Module_Init(BSP_BoardStatus *boardStatus)
{	
	SYS_RETERR errRet = SYS_RETERR_FF;
	
#ifdef HW_CUT__USE_LED
	/*LED Init*/
	boardStatus->LED = bsp_Led_Init();
#endif

#ifdef HW_CUT__USE_RGB
	/*RGB Init*/
	boardStatus->RGB = bsp_RGB_Init(&g_sRgb);
#endif
	
#ifdef HW_CUT__USE_BEEP
	/*BEEP Init*/
	boardStatus->BEEP = bsp_BEEP_Init(&g_sBeep);
#endif	
	
	errRet = bsp_Mut_Module_Init_Result((SYS_RETERR)boardStatus->LED, (SYS_RETERR)boardStatus->RGB, (SYS_RETERR)boardStatus->BEEP, (SYS_RETERR)0);
	
	return errRet;
}

/*1.数据存储单元初始化*/
SYS_RETERR bsp_STOR_Module_Init(BSP_BoardStatus *boardStatus)
{
	SYS_RETERR errRet = SYS_RETERR_FF;
	
#ifdef STOR_MCU__FLASH
	boardStatus->STOR_FLASH = ;
#endif
	
#ifdef STOR_BOARD__AT24CXX
	/*EEPROM_AT24CXX*/
	boardStatus->STOR_EEPROM = bsp_AT24CXX_Init(&g_sAt24cxx);
#endif
	
#ifdef STOR_BOARD__TFSD
	boardStatus->STOR_TFSD = ;
#endif
	
	errRet = bsp_Mut_Module_Init_Result((SYS_RETERR)boardStatus->STOR_FLASH, (SYS_RETERR)boardStatus->STOR_EEPROM, (SYS_RETERR)boardStatus->STOR_TFSD, (SYS_RETERR)0);
	
	return errRet;
}

/*2.AHRS(IMU + 磁力计)模块初始化*/
SYS_RETERR bsp_AHRS_Module_Init(BSP_BoardStatus *boardStatus)
{
	SYS_RETERR errRet = SYS_RETERR_FF;

/*MD_IMU*/
#ifdef MD_IMU__MPU6050
	/*MPU6050(MD_IMU)*/
	boardStatus->MD_IMU = bsp_MPU6050_Init(&g_sMpu6050);
#endif	
	
#ifdef MD_IMU__MPU6000
	/*MPU6000(MD_IMU)*/
	boardStatus->MD_IMU = bsp_MPU6000_Init(&g_sMpu6000);
#endif
	
/*BD IMU*/
#ifdef BD_IMU__MPU6050
	/*MPU6050(MD_IMU)*/
	boardStatus->BD_IMU = bsp_MPU6050_Init(&g_sMpu6050);
#endif	
	
#ifdef BD_IMU__MPU6000
	/*MPU6000(MD_IMU)*/
	boardStatus->BD_IMU = bsp_MPU6000_Init(&g_sMpu6000);
#endif

/*优先选用GPS磁力计*/
#ifdef HW_CUT__USE_MD_MAG
#ifdef HW_CUT__USE_GPS_MAG
/*GPS MAG*/
#ifdef GPS_MAG__AK8975
	/*AK8975*/
	boardStatus->GPS_MAG = bsp_AK8975_Init(&g_sAk8975);
#endif
#ifdef GPS_MAG__HMC5883L
	/*HMC5883L*/
	boardStatus->GPS_MAG = bsp_HMC5883L_Init(&g_sHmc5883l);
#endif
#ifdef GPS_MAG__IST8310
	/*IST8310*/
	boardStatus->MD_MAG = bsp_IST8310_Init(&g_sIst8310);	
#endif	

#ifdef GPS_MAG__HMC5983
	/*HMC5983*/
	boardStatus->MD_MAG = bsp_HMC5983_Init(&g_sHmc5983);
#endif	

#else 

/*MD_MAG*/
#ifdef MD_MAG__AK8975
	/*AK8975*/
	boardStatus->MD_MAG = bsp_AK8975_Init(&g_sAk8975);
#endif

#ifdef MD_MAG__HMC5883L
	/*HMC5883L*/
	boardStatus->MD_MAG = bsp_HMC5883L_Init(&g_sHmc5883l);
#endif

#ifdef MD_MAG__IST8310
	/*IST8310*/
	boardStatus->MD_MAG = bsp_IST8310_Init(&g_sIst8310);	
#endif
#endif
#endif

	errRet = bsp_Mut_Module_Init_Result((SYS_RETERR)boardStatus->MD_IMU, (SYS_RETERR)boardStatus->MD_MAG, (SYS_RETERR)boardStatus->BD_IMU, (SYS_RETERR)boardStatus->GPS_MAG);
	
	return errRet;
}

/*3.气压计模块初始化*/
SYS_RETERR bsp_BERO_Module_Init(BSP_BoardStatus *boardStatus)
{
	SYS_RETERR errRet = SYS_RETERR_FF;
	
#ifdef MD_BERO__SPL06
	/*SPL06-001 初始化*/
	boardStatus->BERO_1 = bsp_SPL06_Init(&g_sSpl06);
#endif
	
#ifdef MD_BERO__BMP280
	/*BMP280 初始化*/
	boardStatus->BERO_1 = bsp_BMP280_Init(&g_sBmp280);
#endif	
		
#ifdef MD_BERO__MS5611
	/*MS5611 初始化*/
	boardStatus->BERO_1 = bsp_MS5611_Init(&g_sMs5611);
#endif	
	
	errRet = bsp_Mut_Module_Init_Result((SYS_RETERR)boardStatus->BERO_1, (SYS_RETERR)boardStatus->BERO_2, (SYS_RETERR)0, (SYS_RETERR)0);
	
	return errRet;
}

/*4.超声波模块初始化*/
SYS_RETERR bsp_ULTR_Module_Init(BSP_BoardStatus *boardStatus)
{
	SYS_RETERR errRet = SYS_RETERR_FF;	
	
#ifdef ULTR_BD__US100
	/*US100 初始化*/
	boardStatus->ULTR = bsp_US100_Init(&g_sUs100);
#endif
	
#ifdef ULTR_BD__RCWL1603
	/*RCWL1603 初始化*/
	boardStatus->ULTR = bsp_RCWL1603_Init(&g_sRcwl1603);
#endif	

#ifdef ULTR_BD__HCSR04
	/*RCWL1603 初始化*/
	boardStatus->ULTR = bsp_HCSR04_Init(&g_sHcsr04);
#endif		
	
	errRet = bsp_Mut_Module_Init_Result((SYS_RETERR)boardStatus->ULTR, (SYS_RETERR)0, (SYS_RETERR)0, (SYS_RETERR)0);
	
	return errRet;
}

/*5.GPS模块初始化*/
SYS_RETERR bsp_GPS_Module_Init(BSP_BoardStatus *boardStatus)
{
	SYS_RETERR errRet = SYS_RETERR_FF;	
	
#ifdef GPS_MD__M8N
	/*MD GPS_M8N 初始化*/
	/*如果插得是新的GPS,则需要初始化;如果不是,则不需要再重新初始化*/
	#if (HW_THE_GPS_IS_A_NEW_ONE == SYS_DEFINE_IS_TRUE)
	boardStatus->MD_GPS = bsp_GPS_M8N_Init(&g_sGpsM8N);
	#else
	boardStatus->MD_GPS = SYS_RET_SUCC;
	#endif
#endif
	
#ifdef GPS_BD__M8N
	/*BD GPS_M8N 初始化*/
	boardStatus->BD_GPS = bsp_GPS_M8N_Init(&g_sGpsM8N);
#endif	
	
	
	errRet = bsp_Mut_Module_Init_Result((SYS_RETERR)boardStatus->MD_GPS, (SYS_RETERR)boardStatus->BD_GPS, (SYS_RETERR)0, (SYS_RETERR)0);
	
	return errRet;
}

/*6.光流模块初始化*/
SYS_RETERR bsp_OPFLOW_Module_Init(BSP_BoardStatus *boardStatus)
{
	SYS_RETERR errRet = SYS_RETERR_FF;

#ifdef OPTICALFLOW_MD__UPIXELSLC306
	/*MD UpixelsLC306 OpticFlow 初始化*/
	boardStatus->MD_OPFLOW = bsp_OPFLOW_UpixelsLC306_Init(&g_sOpFlowUpixelsLC306);
#endif	

#ifdef OPTICALFLOW_BD__UPIXELSLC306
	/*MD UpixelsLC306 OpticFlow 初始化*/
	boardStatus->BD_OPFLOW = bsp_OPFLOW_UpixelsLC306_Init(&g_sOpFlowUpixelsLC306);
#endif		
	
	
	errRet = bsp_Mut_Module_Init_Result((SYS_RETERR)boardStatus->MD_OPFLOW, (SYS_RETERR)boardStatus->BD_OPFLOW, (SYS_RETERR)0, (SYS_RETERR)0);
	
	return errRet;
}

/*7.人机交互模块初始化*/
SYS_RETERR bsp_HCI_Module_Init(BSP_BoardStatus *boardStatus)
{
	SYS_RETERR errRet = SYS_RETERR_FF;	

#ifdef HCI_MD__OLED0_96
	/*OLED 0.96 初始化*/
	boardStatus->HCI_MD_OLED = bsp_OLED0_96_Init(&g_sOled0_96);
#endif
	
	errRet = bsp_Mut_Module_Init_Result((SYS_RETERR)boardStatus->HCI_MD_OLED, (SYS_RETERR)0, (SYS_RETERR)0, (SYS_RETERR)0);
	
	return errRet;	
}


/*多模块初始化结果*/
SYS_RETERR bsp_Mut_Module_Init_Result(SYS_RETERR md1, SYS_RETERR md2, SYS_RETERR md3, SYS_RETERR md4)
{
	volatile SYS_RETERR errRet = SYS_RETERR_0ZR;
	
	errRet |= (md1 << 0);
	errRet |= (md2 << 1);			
	errRet |= (md3 << 2);
	errRet |= (md4 << 3);
	
	return errRet;
}
