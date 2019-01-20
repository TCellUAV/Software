#include "hci_oledshow.h"
#include "period_Execute.h"
#include "bsp_BoardLib.h"
#include "ahrs_Caculation.h"
#include "sins_Strapdown.h"
#include "status_Aircraft.h"
#include "attitude_Aircraft.h"
#include "remot_DataAnaly.h"
#include "control_Aircraft.h"
#include "safe_Operation.h"

#ifdef PLATFORM_RTOS__RT_THREAD
#include "sys_OsTask.h"
#endif

/*========= 初始化人机交互 ========= */
/*显示飞行器启动logo*/
void hci_Show_AircraftLogoHoldMs(u32 holdMs)
{
	/*显示LOGO*/
	bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 0, 128, 8, (u8 *)&g_LogoFlashWolves128X64);
	
	/*保持时间*/
	sys_DelayMs(holdMs);
	
	/*清屏*/
	bsp_OLED0_96_Clear(&g_sOled0_96);
}

/*显示初始化进度*/
void hci_Show_InitRateOfProgress(u8 totalModuleNbr, HCI_SHOW_INIT_TARG INIT_TARG, SYS_RETERR INIT_STATUS)
{
	static u8 moduleInitSuccCnt = 0;
	static u8 progressBarPer = 0;
	static u8 showOnceFlag = 0;
	static u8 xHanziStart = 5;
	static u8 xLogStart = 0;
	u8 xNumberStart = 0;
	static u8 xProgressBarBlock = 20;
	u8 numberLen = 0;
	
	/*初始化模块成功加1*/
	if (INIT_STATUS == SYS_RETERR_0ZR)
	{
		moduleInitSuccCnt++;
	}
	
	/*算出百分比:0~100*/
	progressBarPer = (u8)((fp32)moduleInitSuccCnt / (fp32)totalModuleNbr * 100);
	
	/*================== 1.汉字+进度百分数显示 ==================*/
	/*显示"系统初始化中"*/
	if (showOnceFlag == 0)
	{ 
		bsp_OLED0_96_ShowCHinese(&g_sOled0_96, xHanziStart, 0, 0, OLED096_HANZI_12X12);
		xHanziStart += OLED096_HANZI_12X12;
		
		bsp_OLED0_96_ShowCHinese(&g_sOled0_96, xHanziStart, 0, 1, OLED096_HANZI_12X12);
		xHanziStart += OLED096_HANZI_12X12;		
		
		bsp_OLED0_96_ShowCHinese(&g_sOled0_96, xHanziStart, 0, 2, OLED096_HANZI_12X12);
		xHanziStart += OLED096_HANZI_12X12;
		
		bsp_OLED0_96_ShowCHinese(&g_sOled0_96, xHanziStart, 0, 3, OLED096_HANZI_12X12);
		xHanziStart += OLED096_HANZI_12X12;
		
		bsp_OLED0_96_ShowCHinese(&g_sOled0_96, xHanziStart, 0, 4, OLED096_HANZI_12X12);
		xHanziStart += OLED096_HANZI_12X12;
		
		bsp_OLED0_96_ShowCHinese(&g_sOled0_96, xHanziStart, 0, 5, OLED096_HANZI_12X12);
		xHanziStart += OLED096_HANZI_12X12;
		
		/*显示进度条外框*/
		bsp_OLED0_96_DrawBMP(&g_sOled0_96, 16, 2, 112, 5, (u8 *)&g_ProgressBar96X24);

		/*显示汉字与进度百分数的分隔号*/
		xHanziStart	+= 8; /*跳过一个空格(' ')字符*/
		bsp_OLED0_96_ShowChar(&g_sOled0_96, xHanziStart, 0, '-', OLED096_ACSII_6X12);

		/*显示百分比号,跳过一个分隔('-')字符 + 一个空格(' ')字符 + 3个数字位*/	
		bsp_OLED0_96_ShowChar(&g_sOled0_96, (xHanziStart + 6 + 8 + 18), 0, '%', OLED096_ACSII_6X12);
		
		/*显示初始化记录LOG*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"LOG : ", OLED096_ACSII_6X8);
		xLogStart += 6 * 6;
		
		/*只显示一次*/
		showOnceFlag = 1;
	}
	
	/*判断数字的位个数*/
	if (progressBarPer <= 99)
	{
		numberLen = 2;
	}
	else
	{
		numberLen = 3;	
	}
	
	/*显示百分比数*/
	xNumberStart += xHanziStart	+ 14; /*跳过一个分隔符('-')6+空格(' ')8字符*/
	if (progressBarPer <= 99) 	/*| |x|x|%|*/
	{
		xNumberStart += 8;
		
		bsp_OLED0_96_ShowNum(&g_sOled0_96, xNumberStart, 0, progressBarPer, numberLen, OLED096_ACSII_6X12);
	}
	else /*|1|0|0|%|*/
	{
		bsp_OLED0_96_ShowNum(&g_sOled0_96, xNumberStart, 0, progressBarPer, numberLen, OLED096_ACSII_6X12);
	}		
	
	/*================== 2.进度框显示+进度更新 ==================*/
	/*初始化成功才加一个进度块*/
	if (INIT_STATUS == SYS_RETERR_0ZR)
	{	
		/*显示进度条进度块*/
		bsp_OLED0_96_DrawBMP(&g_sOled0_96, xProgressBarBlock, 3, xProgressBarBlock + 11, 4, (u8 *)&g_ProgressBarFillBlock11X8);	
		xProgressBarBlock += 11; /*1个块 = 11个点位*/
	}
	
	/*================== 3.初始化对象及初始化结果字符串提示 ==================*/	
	/*初始化对象字符串提示*/
	switch(INIT_TARG)
	{	 
		/*_0_人机交互模块*/
		case HCI_SHOW_INIT_HCI_MODE:
		{
			if (INIT_STATUS == SYS_RETERR_0ZR)
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_0_HCI    = SUCC", OLED096_ACSII_6X8);	
				
				/*显示初始化记录 人机交互模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"0-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;				
			}
			else
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_0_HCI    = FAIL", OLED096_ACSII_6X8);
				
				/*显示初始化记录 人机交互模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"1-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;	
			}
		}break;

		/*_1_基础模块*/
		case HCI_SHOW_INIT_BASE_MODE:
		{
			if (INIT_STATUS == SYS_RETERR_0ZR)
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_1_Base   = SUCC", OLED096_ACSII_6X8);	

				/*显示初始化记录 基础模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"0-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;					
			}
			else
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_1_Base   = FAIL", OLED096_ACSII_6X8);	
				
				/*显示初始化记录 基础模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"1-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;				
			}		
		}break;

		/*_2_数据存储模块*/
		case HCI_SHOW_INIT_STOR_MODE:
		{
			if (INIT_STATUS == SYS_RETERR_0ZR)
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_2_Stor   = SUCC", OLED096_ACSII_6X8);	 

				/*显示初始化记录 数据存储模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"0-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;				
			}
			else
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_2_Stor   = FAIL", OLED096_ACSII_6X8);	
				
				/*显示初始化记录 数据存储模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"1-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;					
			}		
		}break;

		/*_3_AHRS模块*/
		case HCI_SHOW_INIT_AHRS_MODE:
		{
			if (INIT_STATUS == SYS_RETERR_0ZR)
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_3_AHRS   = SUCC", OLED096_ACSII_6X8);

				/*显示初始化记录 AHRS模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"0-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;	
			}
			else
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_3_AHRS   = FAIL", OLED096_ACSII_6X8);

				/*显示初始化记录 AHRS模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"1-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;	
			}		
		}break;

		/*_4_气压计模块*/
		case HCI_SHOW_INIT_BERO_MODE:
		{
			if (INIT_STATUS == SYS_RETERR_0ZR)
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_4_BERO   = SUCC", OLED096_ACSII_6X8);	

				/*显示初始化记录 气压计模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"0-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;					
			}
			else
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_4_BERO   = FAIL", OLED096_ACSII_6X8);	
				
				/*显示初始化记录 气压计模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"1-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;	
			}		
		}break;

		/*_5_超声波模块*/
		case HCI_SHOW_INIT_ULTR_MODE:
		{
			if (INIT_STATUS == SYS_RETERR_0ZR)
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_5_ULTR   = SUCC", OLED096_ACSII_6X8);

				/*显示初始化记录 超声波模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"0-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;	
			}
			else
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_5_ULTR   = FAIL", OLED096_ACSII_6X8);

				/*显示初始化记录 超声波模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"1-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;	
			}		
		}break;

		/*_6_GPS模块*/
		case HCI_SHOW_INIT_GPS_MODE:
		{
			if (INIT_STATUS == SYS_RETERR_0ZR)
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_6_GPS    = SUCC", OLED096_ACSII_6X8);

				/*显示初始化记录 GPS模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"0-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;	
			}
			else
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_6_GPS    = FAIL", OLED096_ACSII_6X8);

				/*显示初始化记录 GPS模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"1-", OLED096_ACSII_6X8);
				xLogStart += 2 * 6;	
			}		
		}break;
		
		/*_7_光流模块*/
		case HCI_SHOW_INIT_OPFLOW_MODE:
		{
			if (INIT_STATUS == SYS_RETERR_0ZR)
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_7_OPFLOW = SUCC", OLED096_ACSII_6X8);	

				/*显示初始化记录 光流模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"0", OLED096_ACSII_6X8);
				xLogStart += 1 * 6;
			}
			else
			{
				bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"_7_OPFLOW = FAIL", OLED096_ACSII_6X8);	

				/*显示初始化记录 光流模块结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, xLogStart, 7, (u8*)&"1", OLED096_ACSII_6X8);
				xLogStart += 1 * 6;	
			}		
		}break;		

		default:break;
	}	
	
	/*最后一个初始化完毕后,延时1.5s后清屏*/
	if (INIT_TARG == HCI_SHOW_INIT_OPFLOW_MODE)
	{
		sys_DelayMs(1000);
		
		/*清屏*/
		bsp_OLED0_96_Clear(&g_sOled0_96);
	}
	else
	{
		/*延时,便于查看*/
		sys_DelayMs(1);
	}
}

/*显示传感器校准结果及参数*/
void hci_Show_Sensor_Calib_Parameter(RESULT_CALIB_STATUS ACC_CALIB_STATUS, RESULT_CALIB_STATUS MAG_CALIB_STATUS, u32 holdMs)
{	
/*1.显示内容*/
/*	         	
		                  |-x
			      |-Scale |-y
	              |       |-z
		    |-Acc |
	        |     |        |-x 
	        |	  |-Offset |-y
	        |              |-z
SensorCalib |      
            |              |-x
			|-Mag |-Offset |-y
	                       |-z
*/  
/*2.简排版
SENSOR_CALIB_RESULT
Acc:Scale&Offset SUCC
SX:±0.9999 SY:±0.9999 
SZ:±0.9999 OX:±0.9999 
OY:±0.9999 OZ:±0.9999 
MAG:Offset       SUCC
OX:±0.9999 OY:±0.9999 
OZ:±0.9999 
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示页面框架*/
	/*第0行*/
	bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"SENSOR_CALIB_RESULT", OLED096_ACSII_6X8);
	
	/*第1行*/
	bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 1, (u8*)&"Acc:Scale&Offset", OLED096_ACSII_6X8);
	
	/*第2行*/
	bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"SX:", OLED096_ACSII_6X8);
	bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 2, (u8*)&"SY:", OLED096_ACSII_6X8);
	
	/*第3行*/
	bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"SZ:", OLED096_ACSII_6X8);
	bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"OX:", OLED096_ACSII_6X8);
	
	/*第4行*/
	bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"OY:", OLED096_ACSII_6X8);	
	bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 4, (u8*)&"OZ:", OLED096_ACSII_6X8);	

	/*第5行*/
	bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"MAG:Offset", OLED096_ACSII_6X8);
	
	/*第6行*/
	bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"OX:", OLED096_ACSII_6X8);
	bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 6, (u8*)&"OY", OLED096_ACSII_6X8);	

	/*第7行*/
	bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"OZ:", OLED096_ACSII_6X8);
	
	/*显示结果*/
	/*显示加速度计校准结果*/
	if (ACC_CALIB_STATUS == RESULT_CALIB_SUCC)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 1, (u8*)&"SUCC", OLED096_ACSII_6X8);		
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 1, (u8*)&"FAIL", OLED096_ACSII_6X8);	
	}
	
	/*显示磁力计校准结果*/
	if (MAG_CALIB_STATUS == RESULT_CALIB_SUCC)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 5, (u8*)&"SUCC", OLED096_ACSII_6X8);		
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 5, (u8*)&"FAIL", OLED096_ACSII_6X8);	
	}

	/*1.显示从存储器读取的加速度校准系数值*/
	/*量度系数*/
	/*1.1显示SensorCalib -> Acc -> Scale -> x*/
	math_Floater_Number_Analy(g_fpAccScaleX, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*1.2显示SensorCalib -> Acc -> Scale -> y*/
	math_Floater_Number_Analy(g_fpAccScaleY, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	/*1.3显示SensorCalib -> Acc -> Scale -> z*/
	math_Floater_Number_Analy(g_fpAccScaleZ, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);		

	/*零偏系数*/
	/*1.4显示SensorCalib -> Acc -> Offset -> x*/
	math_Floater_Number_Analy(g_fpAccOffsetX, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*1.5显示SensorCalib -> Acc -> Offset -> y*/
	math_Floater_Number_Analy(g_fpAccOffsetY, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	/*1.6显示SensorCalib -> Acc -> Offset -> z*/
	math_Floater_Number_Analy(g_fpAccOffsetZ, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	


	/*2.显示从存储器读取的磁力计校准系数值*/
	/*零偏系数*/
	/*2.1显示SensorCalib -> Mag -> Offset -> x*/
	math_Floater_Number_Analy(g_fpMagOffsetX, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*2.2显示SensorCalib -> Mag -> Offset -> y*/
	math_Floater_Number_Analy(g_fpMagOffsetY, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	/*2.3显示SensorCalib -> Mag -> Offset -> z*/
	math_Floater_Number_Analy(g_fpMagOffsetZ, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 7, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	/*保持时间*/
	sys_DelayMs(holdMs);
	
	/*清屏*/
	bsp_OLED0_96_Clear(&g_sOled0_96);
}

/*显示控制系统初始化参数(PID参数)*/
void hci_Show_Control_System_Parameter(SYS_RETSTATUS READ_STATUS, fp32 kP, fp32 kI, fp32 kD, PID_CONTROLER_LINK LINK_TARG, u32 holdMs)
{
	static u8 showOnceFlag  = 0;
	static u8 logStartXPos1 = 0;
	static u8 logStartXPos2 = 0;	
	u8 *pstr;
/*1.显示内容*/
/*	         	
		            |-kP
		|-PitchGyro |-kI
	    |           |-kD
		|
		|          |-kP
	    |-RollGyro |-kI
 	    |          |-kD
        |
        |	      |-kP
	    |-YawGyro |-kI
		|         |-kD
		|
		|            |-kP
		|-PitchAngle |-kI
	    |            |-kD
		|
		|           |-kP
	    |-RollAngle |-kI
 	    |           |-kD
        |
        |	       |-kP
	    |-YawAngle |-kI
		|          |-kD
pidPara	|
		|             |-kP
		|-HeightSpeed |-kI
	    |             |-kD
		|
		|           |-kP
	    |-HeightPos |-kI
 	    |           |-kD
        |
		|                |-kP
        |	       |-Lat |-kI
		|		   |     |-kD
	    |-GpsSpeed |
		|		   |	 |-kP
		|          |-Lon |-kI
        |                |-kD
		|                 
		|              |-kP
		|        |-Lat |-kI
		|		 |     |-kD
		|-GpsPos |      
		|        |	   |-kP
		|        |-Lon |-kI
		|              |-kD
		|
		|
		|              	  |-kP
        |	       	  |-x |-kI
		|		   	  |   |-kD
	    |-OpFlowSpeed |
		|		   	  |	  |-kP
		|          	  |-y |-kI
        |                 |-kD
		|                 
		|               |-kP
		|           |-x |-kI
		|		    |   |-kD
		|-OpFlowPos |      
		|           |	|-kP
		|           |-y |-kI
		|               |-kD
		|
		|		
		|           |-kP
	    |-HeightAcc |-kI
 	    |           |-kD
		|
        |
		|              |-kP
        |	     |-Lat |-kI
		|		 |     |-kD
	    |-GpsAcc |
		|		 |	   |-kP
		|        |-Lon |-kI
        |              |-kD
		|            
		|
		|                 |-kP
        |	        |-Lat |-kI
		|		    |     |-kD
	    |-OpFlowAcc |
	 		        |	  |-kP
	                |-Lon |-kI
                          |-kD
*/  
/*2.简排版
PID_SYSTEM_PARAMETER 
READ LINK "01" - SUCC
Link: PitchGyro      
kP: ±123.50
kI: ±123.50
kD: ±123.50
0-0-0-0-0-0-0-0-0-0-0
0-0-0-0-0-0-0-0-0-0-0
*/	
	/*显示页面框架*/
	if (showOnceFlag == 0)
	{
		/*禁止显示提示界面*/
		g_sHciShowPage.SHOW_HINT_STATUS = UAV_HCI_SHOW_DISABLE;
		
		/*标记显示任务忙碌*/
		g_sHciShowPage.SHOW_TASK_STATUS = HCI_SHOW_TASK_BUSY;
		
		/*页面清屏*/
		bsp_OLED0_96_Clear(&g_sOled0_96);
		
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"PID_SYSTEM_PARAMETER", OLED096_ACSII_6X8);
	
		/*第1行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 1, (u8*)&"READ LINK "  " -", OLED096_ACSII_6X8);
	
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"Link: ", OLED096_ACSII_6X8);
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"kP: ", OLED096_ACSII_6X8);
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"kI: ", OLED096_ACSII_6X8);	

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"kD: ", OLED096_ACSII_6X8);	
			
		/*框架只显示一次*/
		showOnceFlag = 1;
	}
	
	/*显示link序号*/
	math_Integer_Number_Analy(LINK_TARG, 2, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 66, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*显示linkTarg*/
	switch (LINK_TARG)
	{
		case PID_CONTROLER_PITCH_GYRO:
		{
			pstr = (u8*)&"PitchGyro      ";
		}break;
		
		case PID_CONTROLER_ROLL_GYRO:
		{
			pstr = (u8*)&"RollGyro       ";
		}break;	
		
		case PID_CONTROLER_YAW_GYRO:
		{
			pstr = (u8*)&"YawGyro        ";
		}break;	

		case PID_CONTROLER_PITCH_ANGLE:
		{
			pstr = (u8*)&"PitchAngle     ";
		}break;	

		case PID_CONTROLER_ROLL_ANGLE:
		{
			pstr = (u8*)&"RollAngle      ";
		}break;	
		
		case PID_CONTROLER_YAW_ANGLE:
		{
			pstr = (u8*)&"YawAngle       ";
		}break;	

		case PID_CONTROLER_HIGH_SPEED:
		{
			pstr = (u8*)&"HighSpeed      ";
		}break;	

		case PID_CONTROLER_HIGH_POSITION:
		{
			pstr = (u8*)&"HighPosition   ";
		}break;	
		
		case PID_CONTROLER_LATITUDE_SPEED:
		{
			pstr = (u8*)&"LatitudeSpeed  ";
		}break;	

		case PID_CONTROLER_LATITUDE_POSITION:
		{
			pstr = (u8*)&"LatitudePos    ";
		}break;	

		case PID_CONTROLER_LONGITUDE_SPEED:
		{
			pstr = (u8*)&"LongitudeSpeed ";
		}break;		
		
		case PID_CONTROLER_LONGITUDE_POSITION:
		{
			pstr = (u8*)&"LongitudePos   ";
		}break;	
		
		case PID_CONTROLER_OPTICFLOW_X_SPEED:
		{
			pstr = (u8*)&"OpFlowXSpeed   ";
		}break;		
		
		case PID_CONTROLER_OPTICFLOW_X_POSITION:
		{
			pstr = (u8*)&"OpFlowXPosSpeed";
		}break;	

		case PID_CONTROLER_OPTICFLOW_Y_SPEED:
		{
			pstr = (u8*)&"OpFlowYSpeed   ";
		}break;	

		case PID_CONTROLER_OPTICFLOW_Y_POSITION:
		{
			pstr = (u8*)&"OpFlowYPosSpeed";
		}break;	

		case PID_CONTROLER_HIGH_ACC:
		{
			pstr = (u8*)&"HighAcc        ";
		}break;	

		case PID_CONTROLER_LONGITUDE_ACC:
		{
			pstr = (u8*)&"LongitudeAcc   ";
		}break;		

		case PID_CONTROLER_LATITUDE_ACC:
		{
			pstr = (u8*)&"LatitudeAcc    ";
		}break;	

		case PID_CONTROLER_OPTICFLOW_X_ACC:
		{
			pstr = (u8*)&"OpFlowXAcc     ";
		}break;	

		case PID_CONTROLER_OPTICFLOW_Y_ACC:
		{
			pstr = (u8*)&"OpFlowYAcc     ";
		}break;			
		
		default:break;
	}		
	
	bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 2, pstr, OLED096_ACSII_6X8);
	
	/*显示PID参数*/
	math_Floater_Number_Analy(kP, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 24, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	math_Floater_Number_Analy(kI, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 24, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	math_Floater_Number_Analy(kD, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 24, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*显示所有PID参数初始化状态*/
	if (READ_STATUS == SYS_RET_SUCC)
	{
		/*当前环状态为成功*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 1, (u8*)&"SUCC", OLED096_ACSII_6X8);
		
		/*历史LOG标记成功*/		
		if (LINK_TARG <= 10) /*未换行*/
		{
			bsp_OLED0_96_ShowString(&g_sOled0_96, logStartXPos1, 6, (u8*)&"0", OLED096_ACSII_6X8);
			logStartXPos1 += 6;		
		}
		else /*换行*/
		{
			bsp_OLED0_96_ShowString(&g_sOled0_96, logStartXPos2, 7, (u8*)&"0", OLED096_ACSII_6X8);
			logStartXPos2 += 6;			
		}			
		
		/*分隔符*/
		if (LINK_TARG <= 9) /*未换行*/
		{			
			bsp_OLED0_96_ShowString(&g_sOled0_96, logStartXPos1, 6, (u8*)&"-", OLED096_ACSII_6X8);				
			logStartXPos1 += 6;
		}
		else if ((11 <= LINK_TARG) && (LINK_TARG <= 20)) /*换行*/
		{
			bsp_OLED0_96_ShowString(&g_sOled0_96, logStartXPos2, 7, (u8*)&"-", OLED096_ACSII_6X8);				
			logStartXPos2 += 6;
		}
	}
	else if (READ_STATUS == SYS_RET_FAIL)
	{
		/*当前环状态为失败*/		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 1, (u8*)&"FAIL", OLED096_ACSII_6X8);		
			
		/*历史LOG标记失败*/
		if (LINK_TARG <= 10) /*未换行*/
		{
			bsp_OLED0_96_ShowString(&g_sOled0_96, logStartXPos1, 6, (u8*)&"1", OLED096_ACSII_6X8);
			logStartXPos1 += 6;		
		}
		else /*换行*/
		{
			bsp_OLED0_96_ShowString(&g_sOled0_96, logStartXPos2, 7, (u8*)&"1", OLED096_ACSII_6X8);
			logStartXPos2 += 6;			
		}
		
		/*分隔符*/		
		if (LINK_TARG <= 9) /*未换行*/
		{			
			bsp_OLED0_96_ShowString(&g_sOled0_96, logStartXPos1, 6, (u8*)&"-", OLED096_ACSII_6X8);				
			logStartXPos1 += 6;
		}
		else if ((11 <= LINK_TARG) && (LINK_TARG <= 20)) /*换行*/
		{
			bsp_OLED0_96_ShowString(&g_sOled0_96, logStartXPos2, 7, (u8*)&"-", OLED096_ACSII_6X8);				
			logStartXPos2 += 6;
		}			
	}	
	
	/*所有控制环参数读取完毕*/
	if (LINK_TARG >= (PID_PARAMETER_SETTING_NBR - 1))
	{
		/*状态机复位*/
		showOnceFlag  = 0;
		
		/*座标复位*/
	    logStartXPos1 = 90;
	    logStartXPos2 = 0;			
		
		sys_DelayMs(holdMs);
		
		/*判断接下来显示内容*/
		if (g_sHciShowPage.SHOW_DATA_STATUS == UAV_HCI_SHOW_ENABLE) /*已经允许显示,则切换到默认首页*/
		{				
			/*恢复默认显示页序号*/
			g_sHciShowPage.curPageIndex  = HCI_SHOW_PAGE_0;
			g_sHciShowPage.lastPageIndex = (HCI_SHOW_PAGE_INDEX)(g_sHciShowPage.curPageIndex + 1);
		}
		else		
		{
			/*允许显示提示界面*/
			g_sHciShowPage.SHOW_HINT_STATUS = UAV_HCI_SHOW_ENABLE;
			
			/*重新显示进入菜单显示提示页面*/
			g_sHciShowPage.EXIT_SHOW_STATUS = HCI_EXIT_SHOW_OP_FIRST;
		}		
		
		/*标记显示任务空闲*/
		g_sHciShowPage.SHOW_TASK_STATUS = HCI_SHOW_TASK_IDLE;			
	}
	else 
	{
		sys_DelayMs(300);		
	}
}

/*使能OLED显示提示*/
void hci_Show_Enable_Hint(void)
{
	/*显示操作提示*/
	bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 0, 128, 8, (u8 *)&g_EnableShowHint128X64);	
}

/*=========  程序运行过程中显示 =========*/
void hci_show_on_run_progress(void)
{
	/*HCI(OLED):用遥控器获取当前显示页面序号*/
	hci_remot_switch_show_status(&g_sHciShowPage);
	
	/*首先判断是否允许显示,显示任务空闲(即无其他显示任务占用)*/
	if ((g_sHciShowPage.SHOW_DATA_STATUS == UAV_HCI_SHOW_ENABLE) && \
		(g_sHciShowPage.SHOW_TASK_STATUS != HCI_SHOW_TASK_BUSY))
	{
		/*开启显示后,退出显示操作复位*/
		g_sHciShowPage.EXIT_SHOW_STATUS = HCI_EXIT_SHOW_OP_FIRST;
		
		/*判断是否是新页*/
		if (g_sHciShowPage.curPageIndex != g_sHciShowPage.lastPageIndex)
		{
			g_sHciShowPage.lastPageIndex = g_sHciShowPage.curPageIndex;
			
			/*清屏*/
			bsp_OLED0_96_Clear(&g_sOled0_96);
			
			/*页面模板显示状态复位*/
			g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_FIRST;
		}
		
		/*显示序号*/
		switch(g_sHciShowPage.curPageIndex)
		{
			case HCI_SHOW_PAGE_0:
			{
				/*NO0.AHRS数据显示*/
				hci_Show_Cur_Ahrs_Data();
			}break;

			case HCI_SHOW_PAGE_1:
			{
				/*NO1.高度传感器(超声波+气压计)数据显示*/
				hci_Show_High_Sensor_Data();			
			}break;

			case HCI_SHOW_PAGE_2:
			{
				/*NO2.GPS数据显示*/
				hci_Show_Gps_Data();
			}break;

			case HCI_SHOW_PAGE_3:
			{
				/*NO3.光流数据显示*/
				hci_Show_Opticflow_Data();			
			}break;

			case HCI_SHOW_PAGE_4:
			{
				/*NO4.竖直高度惯导融合显示*/
				hci_Show_Height_SINS_Data();			
			}break;

			case HCI_SHOW_PAGE_5:
			{
				/*NO5.水平沿PITCH方向惯导融合显示*/
				hci_Show_HorizontalPitch_SINS_Data();			
			}break;

			case HCI_SHOW_PAGE_6:
			{
				/*NO6.水平沿ROLL方向惯导融合显示*/
				hci_Show_HorizontalRoll_SINS_Data();			
			}break;

			case HCI_SHOW_PAGE_7:
			{
				/*NO7.程序执行周期显示*/
				hci_Show_Execute_Period();				
			}break;

			case HCI_SHOW_PAGE_8:
			{
				/*NO8.飞行器本身状态*/
				hci_Show_Aircraft_Status();			
			}break;

			case HCI_SHOW_PAGE_9:
			{
				/*NO9.遥控状态显示*/
				hci_Show_Remot_Status();
			}break;	

			case HCI_SHOW_PAGE_10:
			{
				/*NO10.传感器校准结果*/
				hci_Show_Sensor_Calib_Result();
			}break;
			
			case HCI_SHOW_PAGE_11:
			{
				/*N11.Gps home点数据*/
				hci_Show_Gps_Home_Data();
			}break;	

			case HCI_SHOW_PAGE_12:
			{
				/*No12.控制模式和任务显示*/
				hci_Show_Ctrl_Mission_Data();
			}break;				

			default:break;
		}
	}
	else if (g_sHciShowPage.SHOW_HINT_STATUS == UAV_HCI_SHOW_ENABLE)
	{
		if (g_sHciShowPage.EXIT_SHOW_STATUS == HCI_EXIT_SHOW_OP_FIRST)
		{
			/*显示页面序号清0*/
			g_sHciShowPage.curPageIndex  = HCI_SHOW_PAGE_0;
			
			/*上次页面≠当前页面,不然使能显示后不能清屏*/
			g_sHciShowPage.lastPageIndex = (HCI_SHOW_PAGE_INDEX)(g_sHciShowPage.curPageIndex + 1);
		
			/*页面清屏*/
			bsp_OLED0_96_Clear(&g_sOled0_96);
			
			/*操作提示显示*/
			hci_Show_Enable_Hint();
			
			/*标记已经退出处理过了*/
			g_sHciShowPage.EXIT_SHOW_STATUS = HCI_EXIT_SHOW_OP_NOTFIRST;
		}
		
		/*在待机页面,显示CPU利用率*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 38, 2, (u8*)&"CPU:", OLED096_ACSII_6X8);
		math_Integer_Number_Analy(g_psUav_Status->UavProgrameStatus.CpuUse.major, 2, &g_sMathIntegerAnaly);				
		bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 66, 2, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 78, 2, '%', OLED096_ACSII_6X8);
		
		/*在待机页面,显示RTOS调度状态*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 38, 3, (u8*)&"TASK:", OLED096_ACSII_6X8);
		if (gps_SafeOperation->Task_Check_Status.TASK_CHECK_STATUS == SAFE_TASK_CHECK_NORMAL)
		{
			/*任务调度正常*/
			bsp_OLED0_96_ShowString(&g_sOled0_96, 68, 3, (u8*)&"OK", OLED096_ACSII_6X8);
		}
		else
		{
			/*任务调度异常*/
			bsp_OLED0_96_ShowString(&g_sOled0_96, 68, 3, (u8*)&"NO", OLED096_ACSII_6X8);
		}
	}
}

/*==== 1.实时数据显示 ====*/
/*NO0.AHRS数据显示*/
void hci_Show_Cur_Ahrs_Data(void)
{
	u8 xNbr1Pos = 0;
	u8 xNbr2Pos = 0;	
	xNbr1Pos    = 18;	
	xNbr2Pos    = 84;

/*1.显示内容*/
/*	         				 |- x
			   	      |- ACC |- y
		              |      |- z
	           |- IMU |
			   |      |       |- x
			   |	  |- GYRO |- y
			   |			  |- z
	 |- Sensor |
     |         |	  |- x
	 |	   	   |- MAG |- y
AHRS |                |- z
     |	
	 |        |- pitch       
	 |- Euler |- roll
	          |- yaw
*/  
/*2.简排版
NO0: AHRS_SENSOR    *
	
ax:±0.9999 ay:±0.9999
az:±0.9999 gx:±0.9999
gy:±0.9999 gz:±0.9999
mx:±0.9999 my:±0.9999
mz:±0.9999 Er:±1.0011
Ep:±11.411 Ey:±111.11
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NO0: AHRS_SENSOR", OLED096_ACSII_6X8);		
		
		/*第1行分割不显示*/
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"ax:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 2, (u8*)&"ay:", OLED096_ACSII_6X8);
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"az:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"gx:", OLED096_ACSII_6X8);

		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"gy:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 4, (u8*)&"gz:", OLED096_ACSII_6X8);	

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"mx:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 5, (u8*)&"my:", OLED096_ACSII_6X8);		
		
		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"mz:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 6, (u8*)&"Er:", OLED096_ACSII_6X8);

		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"Ep:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 7, (u8*)&"Ey:", OLED096_ACSII_6X8);

		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*=== 1.ACC ===*/
	/*1.1显示AHRS -> Sensor -> IMU -> Acc -> x*/
	math_Floater_Number_Analy(g_psAccAttitude->x, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr1Pos, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*1.2显示AHRS -> Sensor -> IMU -> Acc -> y*/
	math_Floater_Number_Analy(g_psAccAttitude->y, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr2Pos, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*1.3显示AHRS -> Sensor -> IMU -> Acc -> z*/
	math_Floater_Number_Analy(g_psAccAttitude->z, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr1Pos, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	
	/*=== 2.GYRO ===*/	
	/*2.1显示AHRS -> Sensor -> IMU -> Gyro -> x*/
	math_Floater_Number_Analy(g_psGyroAttitude->x, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr2Pos, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*2.2显示AHRS -> Sensor -> IMU -> Gyro -> y*/
	math_Floater_Number_Analy(g_psGyroAttitude->y, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr1Pos, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*2.3显示AHRS -> Sensor -> IMU -> Gyro -> z*/
	math_Floater_Number_Analy(g_psGyroAttitude->z, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr2Pos, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	
	/*=== 3.MAG ===*/	
	/*3.1显示AHRS -> Sensor -> Mag -> x*/
	math_Floater_Number_Analy(g_psMagFilter->x, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr1Pos, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*3.2显示AHRS -> Sensor -> Mag -> y*/
	math_Floater_Number_Analy(g_psMagFilter->y, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr2Pos, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*3.3显示AHRS -> Sensor -> Mag -> z*/
	math_Floater_Number_Analy(g_psMagFilter->z, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr1Pos, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	
	/*=== 4.Euler ===*/	
	/*4.1显示AHRS -> Euler -> roll*/
	math_Floater_Number_Analy(g_psAhrsAttitude->roll, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr2Pos, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*4.1显示AHRS -> Euler -> pitch*/
	math_Floater_Number_Analy(g_psAhrsAttitude->pitch, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr1Pos, 7, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*4.3显示AHRS -> Euler -> yaw*/
	math_Floater_Number_Analy(g_psAhrsAttitude->yaw, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr2Pos, 7, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
}

/*NO1.高度传感器(超声波+气压计)数据显示*/
void hci_Show_High_Sensor_Data(void)
{
	u8 xNbr1Pos1    = 12;	
	u8 xNbr2Pos1    = 78;	
	u8 xNbr1Pos2    = 24;	
	u8 xNbr2Pos2    = 90;	
	
/*1.显示内容*/
/*	         		     |- zeroPressure
	             |- BERO |
			     |       |- curPressure
			     |	
			     |		
	   |- Sensor |
       |         |	
	   |	   	 |- ULTR 
Height |                
       |			     |- zeroHeight
	   |         |- BERO |
	   |- Height |       |- curHeight
			     |
			     |		 |- zeroHeight
				 |- ULTR |
					     |- curHeight
*/  				
/*2.简排版
NO1: HEIGHT_SENSOR  *
BeroData:
ObOffset:±99999.0
z:±99999.0 c:±99999.0
BeroHeight(cm):
PzH:±6666  PcH:±6666
UltrHeight(cm):
UzH:±6666  UcH:±6666
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}
	
	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NO1: HEIGHT_SENSOR", OLED096_ACSII_6X8);		
		
		/*第1行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 1, (u8*)&"BeroData:", OLED096_ACSII_6X8);			
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"ObOffset:", OLED096_ACSII_6X8);	
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"z:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"c:", OLED096_ACSII_6X8);

		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"BeroHeight(cm):", OLED096_ACSII_6X8);

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"PzH:", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 5, (u8*)&"PcH:", OLED096_ACSII_6X8);
		
		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"UltrHeight(cm):", OLED096_ACSII_6X8);

		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"UzH:", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 7, (u8*)&"UcH:", OLED096_ACSII_6X8);

		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*=== 1.BeroData ===*/
	/*1.1显示气压计观测补偿偏移*/
	math_Floater_Number_Analy(g_psAttitudeAll->BaroData.obAltitudeOffset, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr2Pos1, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	/*1.2显示High_Sensor -> Sensor -> BERO -> zeroPressure*/
	math_Floater_Number_Analy(g_psAttitudeAll->BaroData.zeroPressure, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr1Pos1, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	/*1.3显示High_Sensor -> Sensor -> BERO -> curPressure*/
	math_Floater_Number_Analy(g_psAttitudeAll->BaroData.filterPressure, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, xNbr2Pos1, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	
	/*=== 2.BeroHeight ===*/
	/*2.1显示BeroHeight -> Height -> BERO -> zeroHeight*/
	math_Integer_Number_Analy(g_psAttitudeAll->BaroData.zeroHeight, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, xNbr1Pos2, 5, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*2.1显示BeroHeight -> Height -> BERO -> curHeight*/
	math_Integer_Number_Analy(g_psAttitudeAll->BaroData.curAltitude, 4, &g_sMathIntegerAnaly);	
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, xNbr2Pos2, 5, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	
	
	
	/*=== 3.UltrHeight ===*/
	/*2.1显示BeroHeight -> Height -> ULTR -> zeroHeight*/
	math_Integer_Number_Analy(g_psAttitudeAll->UltrData.zeroHeight, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, xNbr1Pos2, 7, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*2.1显示BeroHeight -> Height -> ULTR -> curHeight*/
	math_Integer_Number_Analy(g_psAttitudeAll->UltrData.curAltitude, 4, &g_sMathIntegerAnaly);	
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, xNbr2Pos2, 7, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
}

/*NO2.GPS数据显示*/
void hci_Show_Gps_Data(void)
{
/*1.显示内容*/
/*
			   |-year
			   |-month
		|-Date |-day
		|      |-hour
        |      |-minute
		|	   |-second
	    |
	    |     |-fixType
	    |-Fix |
	    |     |-sateNbr
	    |
	    |                |-latitude
GpsDate	|-_3_dimensional |-longitude
	    |                |-height
	    |
	    |- dec_yaw //根据GPS定位查找的地磁偏角
	    |
	    |          |-north
	    |-NavSpeed |-east
		|          |-up
		|
		|          |-pos
		|-Accuracy |
		           |-speed
*/  				
/*2.简排版
NO2: GPS_DATA	    *
D:2018/09/25 11:13:00
Fix: 2D   SateNbr: 13//Fix: 3D    SateNbr: 9//Fix: NA    SateNbr: 9
Lat:±101.1681770
Lon:±101.1681770
Hei±9999.0 dYaw±206.7//m °
E:±0.99999 N:±9999.99
U:±0.99999 PDOP±999.0
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NO2: GPS_DATA", OLED096_ACSII_6X8);		
		
		/*第1行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 1, (u8*)&"D:1970/01/01 00:00:00", OLED096_ACSII_6X8);		
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"Fix: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 60, 2, (u8*)&"SateNbr: ", OLED096_ACSII_6X8);				
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"Lat: ", OLED096_ACSII_6X8);	
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"Lon: ", OLED096_ACSII_6X8);

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"Hei: ", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 5, (u8*)&"dYaw", OLED096_ACSII_6X8);		

		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"E:", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 6, (u8*)&"N:", OLED096_ACSII_6X8);					
		
		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"U:", OLED096_ACSII_6X8);			
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 7, (u8*)&"PDOP:", OLED096_ACSII_6X8);		
		
		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*=== 1.Date ===*/
	/*1.1显示GpsDate -> Date -> year*/
	math_Integer_Number_Analy(g_psAttitudeAll->GpsData.LocalTime.year, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 12, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*1.1显示GpsDate -> Date -> month*/
	math_Integer_Number_Analy(g_psAttitudeAll->GpsData.LocalTime.month, 2, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 42, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*1.1显示GpsDate -> Date -> day*/
	math_Integer_Number_Analy(g_psAttitudeAll->GpsData.LocalTime.day, 2, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 60, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*1.1显示GpsDate -> Date -> hour*/
	math_Integer_Number_Analy(g_psAttitudeAll->GpsData.LocalTime.hour, 2, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 78, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*1.1显示GpsDate -> Date -> minute*/
	math_Integer_Number_Analy(g_psAttitudeAll->GpsData.LocalTime.minute, 2, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 96, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*1.1显示GpsDate -> Date -> second*/
	math_Integer_Number_Analy(g_psAttitudeAll->GpsData.LocalTime.second, 2, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 114, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	
	/*=== 2.Fix ===*/
	/*2.1显示GpsDate -> Fix -> fixType*/
	if (g_psAttitudeAll->GpsData.POS_FIX_TYPE == M8N_POS_FIX_3D)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 30, 2, (u8*)&"3D", OLED096_ACSII_6X8);		
	}
	else if (g_psAttitudeAll->GpsData.POS_FIX_TYPE == M8N_POS_FIX_2D)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 30, 2, (u8*)&"2D", OLED096_ACSII_6X8);		
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 30, 2, (u8*)&"NA", OLED096_ACSII_6X8);			
	}

	/*2.2显示GpsDate -> Fix -> sateNbr*/
	math_Integer_Number_Analy(g_psAttitudeAll->GpsData.satelliteNbr, 2, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, 108, 2, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	
	/*=== 3._3_dimensional===*/
	/*3.1显示GpsDate -> _3_dimensional -> latitude*/
	math_Floater_Number_Analy(g_psAttitudeAll->HomePos.Coordinate_f8.lat, 12, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 24, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*3.2显示GpsDate -> _3_dimensional -> longitude*/
	math_Floater_Number_Analy(g_psAttitudeAll->HomePos.Coordinate_f8.lon, 12, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 24, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	/*3.3显示GpsDate -> _3_dimensional -> height*/
	math_Floater_Number_Analy(g_psAttitudeAll->GpsData.hMSL, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	
	/*=== 4.dec_yaw===*/
	/*4.1显示g_psAttitudeAll->declination*/
	math_Floater_Number_Analy(g_psAttitudeAll->declination, 6, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 90, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	
	/*=== 5.NavSpeed ===*/
	/*5.1显示GpsDate -> NavSpeed -> east*/
	math_Floater_Number_Analy(g_psAttitudeAll->GpsData.CurSpeed.east, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 12, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*5.2显示GpsDate -> NavSpeed -> north*/	
	math_Floater_Number_Analy(g_psAttitudeAll->GpsData.CurSpeed.north, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 78, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);		
		
	/*5.3显示GpsDate -> NavSpeed -> up*/			
	math_Floater_Number_Analy(g_psAttitudeAll->GpsData.CurSpeed.up, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 12, 7, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	/*=== 6.精度因子 ===*/
	/*6.1速度精度因子*/
	
	/*6.2位置精度因子*/	
	math_Floater_Number_Analy(g_psAttitudeAll->GpsData.quality, 6, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 90, 7, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
}

/*NO3.光流数据显示*/
void hci_Show_Opticflow_Data(void)
{
/*1.显示内容*/
/*
			   |-year
			   |-month
		|-Date |-day
		|      |-hour
        |      |-minute
		|	   |-second
	    |
	    |     |-fixType
	    |-Fix |
	    |     |-sateNbr
	    |
	    |                |-latitude
GpsDate	|-_3_dimensional |-longitude
	    |                |-height
	    |
	    |- yaw
	    |
	    |          |-north
	    |-NavSpeed |-east
		|          |-up
		|
		|          |-pos
		|-Accuracy |
		           |-speed
*/ 

/*2.简排版
NO3: OPFLOW_DATA    *
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}					
	
	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NO3: OPFLOW_DATA", OLED096_ACSII_6X8);			

		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;		
	}
}

/*==== 2.惯导融合数据显示 ====*/
/*NO4.竖直高度惯导融合显示*/
void hci_Show_Height_SINS_Data(void)
{
/*1.显示内容*/
/*
	             |-origionAcc
	       |-ACC |
	       |     |-sinsAcc
           |
	       |       |-origionSpeed
	       |-Speed |
	       |       |-sinsSpeed
HeightSINS |  
	       |     |-estimateHeight
	       |-Pos |
	       |     |-sinsHeight
		   |
		   |            |-acc
		   |-TOCCorrect |-speed
	                    |-pos
*/  				
/*2.简排版
NO4: VER_SINS_DATA  *

OA:±0.9999 SA:±0.9999
OS:±0.9999 SS:±0.9999
OP:±0.9999 SP:±0.9999
TOC_CA: ±0.99999
TOC_CS: ±0.99999
TOC_CP: ±0.99999
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NO4: VER_SINS_DATA", OLED096_ACSII_6X8);		
		
		/*第1行隔行*/	
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"OA:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 2, (u8*)&"SA", OLED096_ACSII_6X8);				
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"OS:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"SS", OLED096_ACSII_6X8);	
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"OP:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 4, (u8*)&"SP", OLED096_ACSII_6X8);

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"TOC_CA: ", OLED096_ACSII_6X8);

		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"TOC_CS: ", OLED096_ACSII_6X8);					
		
		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"TOC_CP: ", OLED096_ACSII_6X8);	

		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*=== 1.ACC===*/
	/*1.1显示HeightSINS -> ACC -> origionAcc*/
	math_Floater_Number_Analy(g_psSinsOrigion->curAcc[EARTH_FRAME_Z], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*1.2显示HeightSINS -> ACC -> origionAcc*/
	math_Floater_Number_Analy(g_psSinsReal->curAcc[EARTH_FRAME_Z], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	
	/*=== 2.Speed===*/
	/*2.1显示HeightSINS -> Speed -> origionSpeed*/
	math_Floater_Number_Analy(g_psSinsOrigion->curSpeed[EARTH_FRAME_Z], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*2.2显示HeightSINS -> Speed -> sinsSpeed*/
	math_Floater_Number_Analy(g_psSinsReal->curSpeed[EARTH_FRAME_Z], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);		
	
	
	/*=== 3.Pos===*/
	/*3.1显示HeightSINS -> Pos -> estimateHeight*/
	math_Floater_Number_Analy(g_psSinsReal->estimatePos[EARTH_FRAME_Z], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*3.2显示HeightSINS -> Pos -> estimateHeight*/
	math_Floater_Number_Analy(g_psSinsReal->curPosition[EARTH_FRAME_Z], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	
	/*=== 4.TOCCorrect ===*/
	/*4.1显示TOCCorrect -> acc*/
	math_Floater_Number_Analy(g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].acc, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 48, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*4.2显示TOCCorrect -> speed*/
	math_Floater_Number_Analy(g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].speed, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 48, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	/*4.1显示TOCCorrect -> pos*/
	math_Floater_Number_Analy(g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].pos, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 48, 7, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
}

/*NO5.水平沿PITCH方向惯导融合显示*/
void hci_Show_HorizontalPitch_SINS_Data(void)
{
/*1.显示内容*/
/*
					      |-origionAcc
					|-ACC |
					|     |-sinsAcc
					|
					|       |-origionSpeed
					|-Speed |
	                |       |-sinsSpeed
HorizontalPitchSINS |  
					|     |-estimateHorizontalPitch//y轴
			        |-Pos |
					|     |-sinsHorizontalPitch//y轴
				    |
				    |            |-acc
					|-TOCCorrect |-speed
							     |-pos
*/  				
/*2.简排版
NO5: PIH_SINS_DATA  *

OA:±0.9999 SA:±0.9999
OS:±0.9999 SS:±0.9999
OP:±0.9999 SP:±0.9999
TOC_CA: ±0.99999
TOC_CS: ±0.99999
TOC_CP: ±0.99999
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NO5: PIH_SINS_DATA", OLED096_ACSII_6X8);		
		
		/*第1行隔行*/	
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"OA:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 2, (u8*)&"SA", OLED096_ACSII_6X8);				
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"OS:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"SS", OLED096_ACSII_6X8);	
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"OP:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 4, (u8*)&"SP", OLED096_ACSII_6X8);

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"TOC_CA: ", OLED096_ACSII_6X8);

		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"TOC_CS: ", OLED096_ACSII_6X8);					
		
		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"TOC_CP: ", OLED096_ACSII_6X8);	

		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*=== 1.ACC===*/
	/*1.1显示HorizontalPitchSINS -> ACC -> origionAcc*/
	math_Floater_Number_Analy(g_psSinsOrigion->curAcc[EARTH_FRAME_X], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*1.2显示HorizontalPitchSINS -> ACC -> origionAcc*/
	math_Floater_Number_Analy(g_psSinsReal->curAcc[EARTH_FRAME_X], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	
	/*=== 2.Speed===*/
	/*2.1显示HorizontalPitchSINS -> Speed -> origionSpeed*/
	math_Floater_Number_Analy(g_psSinsOrigion->curSpeed[EARTH_FRAME_X], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*2.2显示HorizontalPitchSINS -> Speed -> sinsSpeed*/
	math_Floater_Number_Analy(g_psSinsReal->curSpeed[EARTH_FRAME_X], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);		
	
	
	/*=== 3.Pos===*/
	/*3.1显示HorizontalPitchSINS -> Pos -> estimateHorizontalPitch*/
	math_Floater_Number_Analy(g_psSinsReal->estimatePos[EARTH_FRAME_X], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*3.2显示HorizontalPitchSINS -> Pos -> sinsHorizontalPitch*/
	math_Floater_Number_Analy(g_psSinsReal->curPosition[EARTH_FRAME_X], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	
	/*=== 4.TOCCorrect ===*/
	/*4.1显示TOCCorrect -> acc*/
	math_Floater_Number_Analy(g_psTOCSystem->BackCorrect[EARTH_FRAME_X].acc, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 48, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*4.2显示TOCCorrect -> speed*/
	math_Floater_Number_Analy(g_psTOCSystem->BackCorrect[EARTH_FRAME_X].speed, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 48, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	/*4.1显示TOCCorrect -> pos*/
	math_Floater_Number_Analy(g_psTOCSystem->BackCorrect[EARTH_FRAME_X].pos, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 48, 7, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
}

/*NO6.水平沿ROLL方向惯导融合显示*/
void hci_Show_HorizontalRoll_SINS_Data(void)
{
/*1.显示内容*/
/*
					     |-origionAcc
				   |-ACC |
				   |     |-sinsAcc
				   |
				   |       |-origionSpeed
				   |-Speed |
	               |       |-sinsSpeed
HorizontalRollSINS |  
				   |     |-estimateHorizontalRoll//y轴
			       |-Pos |
				   |     |-sinsHorizontalRoll//y轴
				   |
				   |            |-acc
				   |-TOCCorrect |-speed
							    |-pos
*/  				
/*6.简排版
NO6: ROL_SINS_DATA  *

OA:±0.9999 SA:±0.9999
OS:±0.9999 SS:±0.9999
OP:±0.9999 SP:±0.9999
TOC_CA: ±0.99999
TOC_CS: ±0.99999
TOC_CP: ±0.99999
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NO6: ROL_SINS_DATA", OLED096_ACSII_6X8);		
		
		/*第1行隔行*/	
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"OA:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 2, (u8*)&"SA", OLED096_ACSII_6X8);				
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"OS:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"SS", OLED096_ACSII_6X8);	
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"OP:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 4, (u8*)&"SP", OLED096_ACSII_6X8);

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"TOC_CA: ", OLED096_ACSII_6X8);

		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"TOC_CS: ", OLED096_ACSII_6X8);					
		
		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"TOC_CP: ", OLED096_ACSII_6X8);	

		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*=== 1.ACC===*/
	/*1.1显示HorizontalRollSINS -> ACC -> origionAcc*/
	math_Floater_Number_Analy(g_psSinsOrigion->curAcc[EARTH_FRAME_Y], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*1.2显示HorizontalRollSINS -> ACC -> origionAcc*/
	math_Floater_Number_Analy(g_psSinsReal->curAcc[EARTH_FRAME_Y], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	
	/*=== 2.Speed===*/
	/*2.1显示HorizontalRollSINS -> Speed -> origionSpeed*/
	math_Floater_Number_Analy(g_psSinsOrigion->curSpeed[EARTH_FRAME_Y], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*2.2显示HorizontalRollSINS -> Speed -> sinsSpeed*/
	math_Floater_Number_Analy(g_psSinsReal->curSpeed[EARTH_FRAME_Y], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);		
	
	
	/*=== 3.Pos===*/
	/*3.1显示HorizontalRollSINS -> Pos -> estimateHorizontalRoll*/
	math_Floater_Number_Analy(g_psSinsReal->estimatePos[EARTH_FRAME_Y], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*3.2显示HorizontalRollSINS -> Pos -> sinsHorizontalRoll*/
	math_Floater_Number_Analy(g_psSinsReal->curPosition[EARTH_FRAME_Y], 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	
	/*=== 4.TOCCorrect ===*/
	/*4.1显示TOCCorrect -> acc*/
	math_Floater_Number_Analy(g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].acc, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 48, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*4.2显示TOCCorrect -> speed*/
	math_Floater_Number_Analy(g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].speed, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 48, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	/*4.1显示TOCCorrect -> pos*/
	math_Floater_Number_Analy(g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].pos, 8, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 48, 7, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
}

/*==== 3.飞行器当前状态显示 ====*/
/*NO7.程序执行周期显示*/
void hci_Show_Execute_Period(void)
{
/*1.显示内容*/
/*
			  |-Process |-task
			  |         
			  |
			  |      |-height
		      |-SINS |
	          |      |-horizontal
              | 
              |        |-beroAboveAltitude	
	          |-Sensor |-berobeneathAltitude
	          |        |-ultrAltitude
			  |
ExecutePeriod |-ahrsAttitude
	          |
			  |
	          |							 |-ctrlMainLeading(包含latitudePos,longitudePos)
			  |        			         |-heightPos
			  |		   |-CtrlMainLeading |-heightAcc
			  |		   |			     |-heightSpeed
			  |		   |			     |-latitudeSpeed
			  |		   |				 |-longitudeSpeed
			  |        |
			  |		   |						 |-pitch
			  |Control |		         |-Angle |-roll
	                   |                 |       |-yaw
					   |-AttitudeControl |
					                     |      |-pitch
 				                         |-Gyro |-roll
										        |-yaw			 
*/ 									 
/*2.简排版
NO7: EXE_PERIOD_MS  *
task999 Sih999 Sio999  //task(task) Sh(SINS_height) Sho(SINS_horizontal)
SsBerA:999 SsBerB:999  //SiberA(beroAboveAltitude) SiberB(berobeneathAltitude)
SsUltr:999   Ahrs:999  //SsUltr(ultrAltitude) Ahrs(ahrsAttitude)
CMa999 Clp999 Cls999 //CMa(ctrlMainLeading(latitudePos,longitudePos)) Clp(latitudePos,longitudePos) Cls(latitudeSpeed,longitudeSpeed)
Cha999 Chs999 Chp999   //Cha(heightAcc)   Chs(heightSpeed) Chp(heightPos)
CAp999 CAr999 CAy999   //CAp(Angle_pitch) CAr(Angle_roll)  CAy(Angle_yaw)
CGp999 CGr999 CGy999   //CGp(Angle_pitch) CGr(Angle_roll)  CGy(Angle_yaw)
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NO7: EXE_PERIOD_MS", OLED096_ACSII_6X8);		
		
		/*第1行隔行*/	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 1, (u8*)&"task", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 1, (u8*)&"Sih", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 90, 1, (u8*)&"Sio", OLED096_ACSII_6X8);
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"SsBerA:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 2, (u8*)&"SsBerB:", OLED096_ACSII_6X8);				
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"SsUltr:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 78, 3, (u8*)&"Ahrs:", OLED096_ACSII_6X8);	
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"CMa", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 4, (u8*)&"Clp", OLED096_ACSII_6X8);		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 4, (u8*)&"Cls", OLED096_ACSII_6X8);

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"Cha", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 5, (u8*)&"Chs", OLED096_ACSII_6X8);		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 5, (u8*)&"Chp", OLED096_ACSII_6X8);

		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"CAp", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 6, (u8*)&"CAr", OLED096_ACSII_6X8);		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 6, (u8*)&"CAy", OLED096_ACSII_6X8);					
		
		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"CGp", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 7, (u8*)&"CGr", OLED096_ACSII_6X8);		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 7, (u8*)&"CGy", OLED096_ACSII_6X8);	

		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*=== 1.Process ===*/
	/*1.1显示ExecutePeriod -> Process -> task*/
	math_Integer_Number_Analy(g_psSystemPeriodExecuteTime->UavCtrl.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 24, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	
	/*=== 2.SINS ===*/
	/*2.1显示ExecutePeriod -> SINS -> height*/	
	math_Integer_Number_Analy(g_psSystemPeriodExecuteTime->SINS_High.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 66, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);

	/*2.2显示ExecutePeriod -> SINS -> horizontal*/	
	math_Integer_Number_Analy(g_psSystemPeriodExecuteTime->SINS_Horizontal.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 108, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	
	
	
	/*=== 3.Sensor ===*/
	/*3.1显示ExecutePeriod -> Sensor -> beroAboveAltitude*/
	math_Integer_Number_Analy(g_psSystemPeriodExecuteTime->BeroAboveAltitude.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 42, 2, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*3.2显示ExecutePeriod -> Sensor -> berobeneathAltitude*/	
	math_Integer_Number_Analy(g_psSystemPeriodExecuteTime->BeroBeneathAltitude.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 108, 2, OLED096_ACSII_6X8, g_sMathIntegerAnaly);

	/*3.3显示ExecutePeriod -> Sensor -> ultrAltitude*/	
	math_Integer_Number_Analy(g_psSystemPeriodExecuteTime->UltrAltitude.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 42, 3, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	
	/*=== 4.ahrsAttitude ===*/
	/*4.1显示ExecutePeriod -> ahrsAttitude*/	
	math_Integer_Number_Analy(g_psSystemPeriodExecuteTime->AhrsAttitude.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 108, 3, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	
	
	
	/*=== 5.Control ===*/	
	/*5.1显示ExecutePeriod -> Control -> CtrlMainLeading -> ctrlMainLeading*/	
	math_Integer_Number_Analy(g_psSystemPeriodExecuteTime->CTRL_MainLeading.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 12, 4, OLED096_ACSII_6X8, g_sMathIntegerAnaly);

	/*5.2显示ExecutePeriod -> Control -> CtrlMainLeading -> latitudePos & lontitudePos*/	
	math_Integer_Number_Analy(g_psSystemPeriodExecuteTime->CTRL_MainLeading.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 60, 4, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*5.3显示ExecutePeriod -> Control -> CtrlMainLeading -> latitudeSpeed & lontitudeSpeed*/		
	math_Integer_Number_Analy(g_psSystemPeriodExecuteTime->CTRL_MainLeading.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 102, 4, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*5.4显示ExecutePeriod -> Control -> CtrlMainLeading -> heightAcc*/	
	math_Integer_Number_Analy(g_psPidSystem->HighAcc.PidControlDT.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 12, 5, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*5.5显示ExecutePeriod -> Control -> CtrlMainLeading -> heightSpeed*/	
	math_Integer_Number_Analy(g_psPidSystem->HighSpeed.PidControlDT.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 60, 5, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*5.6显示ExecutePeriod -> Control -> CtrlMainLeading -> heightPos*/	
	math_Integer_Number_Analy(g_psPidSystem->HighPosition.PidControlDT.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 108, 5, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

	/*5.7显示ExecutePeriod -> Control -> AttitudeControl -> Angle -> pitch*/	
	math_Integer_Number_Analy(g_psPidSystem->PitchAngle.PidControlDT.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 12, 6, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*5.8显示ExecutePeriod -> Control -> AttitudeControl -> Angle -> roll*/	
	math_Integer_Number_Analy(g_psPidSystem->PitchGyro.PidControlDT.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 60, 6, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*5.9显示ExecutePeriod -> Control -> AttitudeControl -> Angle -> yaw*/	
	math_Integer_Number_Analy(g_psPidSystem->YawAngle.PidControlDT.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 108, 6, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

	/*5.10显示ExecutePeriod -> Control -> AttitudeControl -> Gyro -> pitch*/	
	math_Integer_Number_Analy(g_psPidSystem->PitchGyro.PidControlDT.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 12, 7, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*5.11显示ExecutePeriod -> Control -> AttitudeControl -> Gyro -> roll*/	
	math_Integer_Number_Analy(g_psPidSystem->RollGyro.PidControlDT.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 60, 7, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*5.12显示ExecutePeriod -> Control -> AttitudeControl -> Gyro -> yaw*/	
	math_Integer_Number_Analy(g_psPidSystem->YawGyro.PidControlDT.DeltaTime, 3, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 108, 7, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	
}

/*NO8.飞行器本身状态*/
void hci_Show_Aircraft_Status(void)
{
/*1.显示内容*/
/*
					  |-lock
					  |-flyStatus
	           |-Self |-gpshomeSet        
		       |	  |-cmcStatus
			   |
	           |		              |-lastHeightControlMode
	           |             |-Height |-curHeightControlMode
			   |-ControlType |        |-heightControlMode 
			   |             |
			   |             |			  |-lastHorizontalControlMode
			   |			 |-Horizontal |-curHorizontalControlMode
AircraftStatus |                          |-HorizontalControlMode			 
			   |
			   |		             |-beroEstimateAltitude
               |          |-Altitude |-ultrEstimateAltitude 
			   |-Estimate |         
			   |          |-Horizontal |-gpsEstimateHorizontal
			   |                       |-opflowEstimateHorizontal
			   |
	           |-Sensor |-estimateAltitude
			            |-estimateHorizontal
*/
/*2.简排版
NO8: UAV_STATUS     *

locS: UNL  flyS: LAD
gpsH: SET  cmcS: SUC
LV: SA CV: FX RV: FX //LV(lastHeightControlMode) CV(curHeightControlMode) RV(heightControlMode)
LH: SA CH: FX RH: FX //LH(lastHorizontalControlMode) CH(curHorizontalControlMode) RH(HorizontalControlMode)
bE: OK uE: OK gE: OK //bE(beroEstimateAltitude) uE(ultrEstimateAltitude) gE(gpsEstimateHorizontal)
oE: OK eV:B+U eH:G+O //oE(opflowEstimateHorizontal) eV(estimateAltitude) eH(estimateHorizontal)
*/	
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NO8: UAV_STATUS", OLED096_ACSII_6X8);		
		
		/*第1行隔行*/
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"locS: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 2, (u8*)&"flyS: ", OLED096_ACSII_6X8);				
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"gpsH: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"cmcS: ", OLED096_ACSII_6X8);	
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"LV: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 4, (u8*)&"CV: ", OLED096_ACSII_6X8);		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 4, (u8*)&"RV:", OLED096_ACSII_6X8);

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"LH: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 5, (u8*)&"CH: ", OLED096_ACSII_6X8);		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 5, (u8*)&"RH:", OLED096_ACSII_6X8);

		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"bE: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 6, (u8*)&"uE: ", OLED096_ACSII_6X8);		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 6, (u8*)&"gE: ", OLED096_ACSII_6X8);					
		
		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"oE: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 7, (u8*)&"eV:", OLED096_ACSII_6X8);		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 7, (u8*)&"eH:", OLED096_ACSII_6X8);	

		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*=== 1.飞行器自身状态显示 ===*/
	/*1.1显示AircraftStatus -> Self -> lock*/
	if (g_sUav_Status.LOCK_STATUS == UAV_LOCK_NOT)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 2, (u8*)&"UNL", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.LOCK_STATUS == UAV_LOCK_YES)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 2, (u8*)&"LOC", OLED096_ACSII_6X8);			
	}
	
	/*1.2显示AircraftStatus -> Self -> flyStatus*/
	if (g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_NOT)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 2, (u8*)&"FLY", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_YES)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 2, (u8*)&"LAD", OLED096_ACSII_6X8);			
	}	
	
	/*1.3显示AircraftStatus -> Self -> gpshomeSet*/
	if (g_sUav_Status.HOME_SET_STATUS == UAV_HOME_SET_YES)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 3, (u8*)&"SET", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.HOME_SET_STATUS == UAV_HOME_SET_NOT)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 3, (u8*)&"NST", OLED096_ACSII_6X8);			
	}	

	/*1.4显示AircraftStatus -> Self -> cmcStatus*/
	if (g_sUav_Status.WIRELESS_CMC_STATUS == UAV_WIRELESS_CMC_SUCC)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 3, (u8*)&"SUC", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.WIRELESS_CMC_STATUS == UAV_WIRELESS_CMC_FAIL)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 3, (u8*)&"FAL", OLED096_ACSII_6X8);			
	}

	/*=== 2.控制模式 ===*/
	/*2.1显示AircraftStatus -> ControlType -> Height -> lastHeightControlMode*/
	if (g_sUav_Status.UavControlMode.Vertical.Mode_Switch.LastTime == UAV_VERTICAL_CONTROL_SELFAUTO)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 4, (u8*)&"SA", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavControlMode.Vertical.Mode_Switch.LastTime == UAV_VERTICAL_CONTROL_FIX_HEIGHT)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 4, (u8*)&"FX", OLED096_ACSII_6X8);			
	}	
	
	/*2.2显示AircraftStatus -> ControlType -> Height -> curHeightControlMode*/
	if (g_sUav_Status.UavControlMode.Vertical.Mode_Switch.ThisTime == UAV_VERTICAL_CONTROL_SELFAUTO)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 4, (u8*)&"SA", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavControlMode.Vertical.Mode_Switch.ThisTime == UAV_VERTICAL_CONTROL_FIX_HEIGHT)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 4, (u8*)&"FX", OLED096_ACSII_6X8);			
	}	
	
	/*2.3显示AircraftStatus -> ControlType -> Height -> heightControlMode*/
	if (g_sUav_Status.UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_SELFAUTO)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 4, (u8*)&"F2S", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_FIX_HEIGHT)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 4, (u8*)&"S2F", OLED096_ACSII_6X8);			
	}	

	/*2.4显示AircraftStatus -> ControlType -> Horizontal -> lastHorizontalControlMode*/
	if (g_sUav_Status.UavControlMode.Horizontal.Mode_Switch.LastTime == UAV_HORIZONTAL_CONTROL_SELFAUTO)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 5, (u8*)&"SA", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavControlMode.Horizontal.Mode_Switch.LastTime == UAV_HORIZONTAL_CONTROL_FIX_POS)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 5, (u8*)&"FX", OLED096_ACSII_6X8);			
	}	
	
	/*2.5显示AircraftStatus -> ControlType -> Horizontal -> curHorizontalControlMode*/
	if (g_sUav_Status.UavControlMode.Horizontal.Mode_Switch.ThisTime == UAV_HORIZONTAL_CONTROL_SELFAUTO)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 5, (u8*)&"SA", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavControlMode.Horizontal.Mode_Switch.ThisTime == UAV_HORIZONTAL_CONTROL_FIX_POS)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 5, (u8*)&"FX", OLED096_ACSII_6X8);			
	}	
	
	/*2.6显示AircraftStatus -> ControlType -> Horizontal -> HorizontalControlMode*/
	if (g_sUav_Status.UavControlMode.Horizontal.CONTROL_MODE == UAV_HORIZONTAL_CONTROL_SELFAUTO)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 5, (u8*)&"F2S", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavControlMode.Horizontal.CONTROL_MODE == UAV_HORIZONTAL_CONTROL_FIX_POS)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 5, (u8*)&"S2F", OLED096_ACSII_6X8);			
	}
	
	/*=== 3.观测有效状态 ===*/
	/*3.1显示AircraftStatus -> Estimate -> Altitude -> beroEstimateAltitude*/
	if (g_sUav_Status.UavSenmodStatus.Vertical.Bero.DATA_STATUS == UAV_SENMOD_DATA_OK)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 6, (u8*)&"OK", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavSenmodStatus.Vertical.Bero.DATA_STATUS == UAV_SENMOD_DATA_NO)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 6, (u8*)&"NO", OLED096_ACSII_6X8);			
	}
	
	/*3.2显示AircraftStatus -> Estimate -> Altitude -> ultrEstimateAltitude*/
	if (g_sUav_Status.UavSenmodStatus.Vertical.Ultr.DATA_STATUS == UAV_SENMOD_DATA_OK)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 6, (u8*)&"OK", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavSenmodStatus.Vertical.Ultr.DATA_STATUS == UAV_SENMOD_DATA_NO)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 6, (u8*)&"NO", OLED096_ACSII_6X8);			
	}
	
	/*3.3显示AircraftStatus -> Estimate -> Horizontal -> gpsEstimateHorizontal*/
	if (g_sUav_Status.UavSenmodStatus.Horizontal.Gps.DATA_STATUS == UAV_SENMOD_DATA_OK)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 108, 6, (u8*)&"OK", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavSenmodStatus.Horizontal.Gps.DATA_STATUS == UAV_SENMOD_DATA_NO)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 108, 6, (u8*)&"NO", OLED096_ACSII_6X8);			
	}	
	
	/*3.4显示AircraftStatus -> Estimate -> Horizontal -> opflowEstimateHorizontal*/
	if (g_sUav_Status.UavSenmodStatus.Horizontal.Opticflow.DATA_STATUS == UAV_SENMOD_DATA_OK)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 7, (u8*)&"OK", OLED096_ACSII_6X8);			
	}	
	else if (g_sUav_Status.UavSenmodStatus.Horizontal.Opticflow.DATA_STATUS == UAV_SENMOD_DATA_NO)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 7, (u8*)&"NO", OLED096_ACSII_6X8);			
	}
	
	/*=== 4.竖直和水平传感器观测状态 ===*/
	/*4.1显示AircraftStatus -> Sensor -> estimateAltitude*/
	if ((g_sUav_Status.UavSenmodStatus.Vertical.Bero.DATA_STATUS == UAV_SENMOD_DATA_NO) && \
		(g_sUav_Status.UavSenmodStatus.Vertical.Ultr.DATA_STATUS == UAV_SENMOD_DATA_NO))
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 60, 7, (u8*)&"X+X", OLED096_ACSII_6X8);		
	}
	else if ((g_sUav_Status.UavSenmodStatus.Vertical.Bero.DATA_STATUS == UAV_SENMOD_DATA_OK) && \
			 (g_sUav_Status.UavSenmodStatus.Vertical.Ultr.DATA_STATUS == UAV_SENMOD_DATA_NO))
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 60, 7, (u8*)&"B+X", OLED096_ACSII_6X8);			
	}
	else if ((g_sUav_Status.UavSenmodStatus.Vertical.Bero.DATA_STATUS == UAV_SENMOD_DATA_NO) && \
			 (g_sUav_Status.UavSenmodStatus.Vertical.Ultr.DATA_STATUS == UAV_SENMOD_DATA_OK))
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 60, 7, (u8*)&"X+U", OLED096_ACSII_6X8);			
	}
	else if ((g_sUav_Status.UavSenmodStatus.Vertical.Bero.DATA_STATUS == UAV_SENMOD_DATA_OK) && \
			 (g_sUav_Status.UavSenmodStatus.Vertical.Ultr.DATA_STATUS == UAV_SENMOD_DATA_OK))
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 60, 7, (u8*)&"B+U", OLED096_ACSII_6X8);			
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 60, 7, (u8*)&"N+N", OLED096_ACSII_6X8);
	}	

	/*4.2显示AircraftStatus -> Sensor -> estimateHorizontal*/
	if ((g_sUav_Status.UavSenmodStatus.Horizontal.Gps.DATA_STATUS == UAV_SENMOD_DATA_NO) && \
		(g_sUav_Status.UavSenmodStatus.Horizontal.Opticflow.DATA_STATUS == UAV_SENMOD_DATA_NO))
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 7, (u8*)&"X+X", OLED096_ACSII_6X8);			
	}	
	if ((g_sUav_Status.UavSenmodStatus.Horizontal.Gps.DATA_STATUS == UAV_SENMOD_DATA_OK) && \
		(g_sUav_Status.UavSenmodStatus.Horizontal.Opticflow.DATA_STATUS == UAV_SENMOD_DATA_NO))
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 7, (u8*)&"G+X", OLED096_ACSII_6X8);			
	}	
	if ((g_sUav_Status.UavSenmodStatus.Horizontal.Gps.DATA_STATUS == UAV_SENMOD_DATA_NO) && \
		(g_sUav_Status.UavSenmodStatus.Horizontal.Opticflow.DATA_STATUS == UAV_SENMOD_DATA_OK))
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 7, (u8*)&"X+O", OLED096_ACSII_6X8);			
	}
	if ((g_sUav_Status.UavSenmodStatus.Horizontal.Gps.DATA_STATUS == UAV_SENMOD_DATA_OK) && \
		(g_sUav_Status.UavSenmodStatus.Horizontal.Opticflow.DATA_STATUS == UAV_SENMOD_DATA_OK))
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 7, (u8*)&"G+O", OLED096_ACSII_6X8);			
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 7, (u8*)&"N+N", OLED096_ACSII_6X8);			
	}
}

/*NO9.遥控状态显示*/
void hci_Show_Remot_Status(void)
{
/*1.显示内容*/
/*    
	                           |-roll
	                           |-pitch
	                |-Attitude |-throttle
	                |          |-yaw
	  |-ChannleData |        
	  |				|       |-vra
	  |             |Gimbal |-vrb
	  |             |
	  |             |       |-swa
	  |             |Switch |-swb
      |                     |-swc
	  |                     |-swd
      |
	  |                        |-roll
Romot |             |-Attitude |-pitch
	  |             |          |-throttle
	  |-ExpectAngle |          |-yaw
	  |             |
	  |             |       |-pitch
      |	            |Gimbal |
      |                     |-roll
      |
	  |                 |-roll
	  |-ExpectAutoAngle |
	                    |-pitch
*/
/*2.简排版
NO9: REMOT_DATA     *
1h2000 2h2000 3h2000
4h2000 5h2000 6h2000
7h2000 8h2000 9h2000
EArol: ±90 EApit: ±90	//pitch & roll期望角度范围-45°~+45°
EAthr:1000 EAyaw:±123	//油门期望1000~1900, yaw期望角度范围-150°~+150°
EGpit:±123 EGyaw:±123
EAArol:±90 EAApit:±90
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NO9: REMOT_DATA", OLED096_ACSII_6X8);		
		
		/*第1行*/	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 1, (u8*)&"1h", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 1, (u8*)&"2h", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 1, (u8*)&"3h", OLED096_ACSII_6X8);
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"4h", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 2, (u8*)&"5h", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 2, (u8*)&"6h", OLED096_ACSII_6X8);				
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"7h", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 3, (u8*)&"8h", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 3, (u8*)&"9h", OLED096_ACSII_6X8);	
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"EArol: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 4, (u8*)&"EApit: ", OLED096_ACSII_6X8);

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"EAthr:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 5, (u8*)&"EAyaw:", OLED096_ACSII_6X8);

		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"EGpit:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 6, (u8*)&"EGyaw:", OLED096_ACSII_6X8);		
		
		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"EAArol:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 7, (u8*)&"EAApit:", OLED096_ACSII_6X8);	
		
		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*=== 1.ChannleData ===*/
	/*1.1显示Romot -> ChannleData -> Attitude -> roll*/
	math_Integer_Number_Analy(g_psRemotData->AttRoll, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 12, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

	/*1.2显示Romot -> ChannleData -> Attitude -> pitch*/
	math_Integer_Number_Analy(g_psRemotData->AttPitch, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 54, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

	/*1.3显示Romot -> ChannleData -> Attitude -> throttle*/
	math_Integer_Number_Analy(g_psRemotData->AttThrottle, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 96, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

	/*1.4显示Romot -> ChannleData -> Attitude -> yaw*/
	math_Integer_Number_Analy(g_psRemotData->AttYaw, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 12, 2, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

	/*1.5显示Romot -> ChannleData -> Switch -> swa*/
	math_Integer_Number_Analy(g_psRemotData->SWA, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 54, 2, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

	/*1.6显示Romot -> ChannleData -> Switch -> swb*/
	math_Integer_Number_Analy(g_psRemotData->SWB, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 96, 2, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	
	
	/*1.7显示Romot -> ChannleData -> Switch -> swc*/
	math_Integer_Number_Analy(g_psRemotData->SWC, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 12, 3, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

	/*1.8显示Romot -> ChannleData -> Switch -> swd*/
	math_Integer_Number_Analy(g_psRemotData->SWD, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 54, 3, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

	/*1.9显示Romot -> ChannleData -> gimbal -> vra*/
	math_Integer_Number_Analy(g_psRemotData->GimPitch, 4, &g_sMathIntegerAnaly);
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 96, 3, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	
	
	
	/*=== 2.ExpectAngle ===*/
	/*2.1显示Romot -> ExpectAngle -> Attitude -> roll*/
	math_Integer_Number_Analy(g_psControlAircraft->RemotExpectAngle.roll, 2, &g_sMathIntegerAnaly);	
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, 42, 4, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*2.2显示Romot -> ExpectAngle -> Attitude -> pitch*/
	math_Integer_Number_Analy(g_psControlAircraft->RemotExpectAngle.pitch, 2, &g_sMathIntegerAnaly);	
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, 108, 4, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*2.3显示Romot -> ExpectAngle -> Attitude -> throttle*/
	math_Integer_Number_Analy(g_psControlAircraft->RemotExpectAngle.throttle, 4, &g_sMathIntegerAnaly);	
	bsp_OLED0_96_Show_Integer_No_Sign(&g_sOled0_96, 36, 5, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	
	
	/*2.4显示Romot -> ExpectAngle -> Attitude -> yaw*/
	math_Integer_Number_Analy(g_psControlAircraft->RemotExpectAngle.yaw, 3, &g_sMathIntegerAnaly);	
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, 102, 5, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

	/*2.5显示Romot -> ExpectAngle -> Gimbal -> pitch*/
	math_Integer_Number_Analy(g_psControlAircraft->GimbalExpectAngle.pitch, 3, &g_sMathIntegerAnaly);	
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, 36, 6, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
	/*2.6显示Romot -> ExpectAngle -> Gimbal -> yaw*/
	math_Integer_Number_Analy(g_psControlAircraft->GimbalExpectAngle.yaw, 3, &g_sMathIntegerAnaly);	
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, 102, 6, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	
	
	
	/*=== 3.ExpectAutoAngle ===*/
	/*3.1显示Romot -> ExpectAutoAngle -> roll*/
	math_Integer_Number_Analy(g_psControlAircraft->RemotExpectAutoAngle.roll, 2, &g_sMathIntegerAnaly);	
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, 42, 7, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	
	
	/*3.2显示Romot -> ExpectAutoAngle -> roll*/
	math_Integer_Number_Analy(g_psControlAircraft->RemotExpectAutoAngle.pitch, 2, &g_sMathIntegerAnaly);	
	bsp_OLED0_96_Show_Integer(&g_sOled0_96, 108, 7, OLED096_ACSII_6X8, g_sMathIntegerAnaly);		
}

/*NO10.传感器校准结果*/
void hci_Show_Sensor_Calib_Result(void)
{
/*1.显示内容*/
/*	         	
		                  |-x
			      |-Scale |-y
	              |       |-z
		    |-Acc |
	        |     |        |-x 
	        |	  |-Offset |-y
	        |              |-z
SensorCalib |      
            |              |-x
			|-Mag |-Offset |-y
	                       |-z
*/  
/*2.简排版
NOA: SS_CALIB_RESULT *
Acc:Scale&Offset SUCC
SX:±0.9999 SY:±0.9999 
SZ:±0.9999 OX:±0.9999 
OY:±0.9999 OZ:±0.9999 
MAG:Offset       SUCC
OX:±0.9999 OY:±0.9999 
OZ:±0.9999 
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NOA: SS_CALIB_RESULT", OLED096_ACSII_6X8);
	
		/*第1行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 1, (u8*)&"Acc:Scale&Offset", OLED096_ACSII_6X8);
	
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"SX:", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 2, (u8*)&"SY:", OLED096_ACSII_6X8);
	
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"SZ:", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"OX:", OLED096_ACSII_6X8);
	
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"OY:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 4, (u8*)&"OZ:", OLED096_ACSII_6X8);	

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"MAG:Offset", OLED096_ACSII_6X8);
	
		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"OX:", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 6, (u8*)&"OY", OLED096_ACSII_6X8);	

		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"OZ:", OLED096_ACSII_6X8);
		
		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;		
	}
	
	/*显示结果*/
	/*显示加速度计校准结果*/
	if (g_psAccCalibSystem->RESULT_STATUS == RESULT_CALIB_SUCC)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 1, (u8*)&"SUCC", OLED096_ACSII_6X8);		
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 1, (u8*)&"FAIL", OLED096_ACSII_6X8);	
	}
	
	/*显示磁力计校准结果*/
	if (g_psMagCalibSystem->RESULT_STATUS == RESULT_CALIB_SUCC)
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 5, (u8*)&"SUCC", OLED096_ACSII_6X8);		
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 5, (u8*)&"FAIL", OLED096_ACSII_6X8);	
	}

	/*1.显示从存储器读取的加速度校准系数值*/
	/*量度系数*/
	/*1.1显示SensorCalib -> Acc -> Scale -> x*/
	math_Floater_Number_Analy(g_fpAccScaleX, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*1.2显示SensorCalib -> Acc -> Scale -> y*/
	math_Floater_Number_Analy(g_fpAccScaleY, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	/*1.3显示SensorCalib -> Acc -> Scale -> z*/
	math_Floater_Number_Analy(g_fpAccScaleZ, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);		

	/*零偏系数*/
	/*1.4显示SensorCalib -> Acc -> Offset -> x*/
	math_Floater_Number_Analy(g_fpAccOffsetX, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*1.5显示SensorCalib -> Acc -> Offset -> y*/
	math_Floater_Number_Analy(g_fpAccOffsetY, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	/*1.6显示SensorCalib -> Acc -> Offset -> z*/
	math_Floater_Number_Analy(g_fpAccOffsetZ, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	


	/*2.显示从存储器读取的磁力计校准系数值*/
	/*零偏系数*/
	/*2.1显示SensorCalib -> Mag -> Offset -> x*/
	math_Floater_Number_Analy(g_fpMagOffsetX, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*2.2显示SensorCalib -> Mag -> Offset -> y*/
	math_Floater_Number_Analy(g_fpMagOffsetY, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

	/*2.3显示SensorCalib -> Mag -> Offset -> z*/
	math_Floater_Number_Analy(g_fpMagOffsetZ, 7, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 7, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
}

/*==== 4.飞行器大数据显示 ====*/
/*N11.Gps home点数据*/
void hci_Show_Gps_Home_Data(void)
{
/*1.显示内容*/
/*
	          |-lat
	    |curr |
	    |     |-lon
GpsData |  
	    |     |-lat
	    |curr |
	    |     |-lon
*/  				
/*6.简排版
NOB: GPS_POS_DATA  *
c_lat: 36.00000000
c_lon: 360.0000000
epp: ±30.0 epr: ±30.0
EF_Lat_cm: ±1232222.0    cm   
EF_Lon_cm: ±1232222.0    cm
BF_Lat_cm: ±1232222.0    cm
BF_Lon_cm: ±1232222.0    cm
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NOB: GPS_POS_DATA", OLED096_ACSII_6X8);		
		
		/*第1行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 1, (u8*)&"c_lat: ", OLED096_ACSII_6X8);		
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"c_lon: ", OLED096_ACSII_6X8);					
		
		/*第3行隔行*/	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"epp: ", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"epr: ", OLED096_ACSII_6X8);		
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"EF_Lat_cm: ", OLED096_ACSII_6X8);	

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"EF_Lon_cm: ", OLED096_ACSII_6X8);

		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"BF_Lat_cm: ", OLED096_ACSII_6X8);					
		
		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"BF_Lon_cm: ", OLED096_ACSII_6X8);	

		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*=== 1.CURRENT POS===*/
	/*1.1显示当前座标Lat*/
	math_Floater_Number_Analy(g_psAttitudeAll->GpsData.Coordinate_f8.lat, 12, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 42, 1, OLED096_ACSII_6X8, g_sMathFloaterAnaly);

	/*1.2显示当前座标Lon*/
	math_Floater_Number_Analy(g_psAttitudeAll->GpsData.Coordinate_f8.lon, 12, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 42, 2, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	/*=== 2.USE_CTRL STATUS ===*/
	/*gps_fix: pitch_angle_expect*/
	math_Floater_Number_Analy(g_sHorizontalExpectAngle.y, 5, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 30, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
	/*gps_fix: roll_angle_expect*/
	math_Floater_Number_Analy(g_sHorizontalExpectAngle.x, 5, &g_sMathFloaterAnaly);
	bsp_OLED0_96_Show_Floater(&g_sOled0_96, 96, 3, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	
	/*=== 3.Distamce ===*/
	/*有效才显示*/
	if (g_psUav_Status->UavSenmodStatus.Horizontal.Gps.USE_CONTROL_STATUS == UAV_SENMOD_USE_CONTROL_ALLOW)
	{
		/*3.1 EarthFrame_Lat_cm*/	
	    math_Floater_Number_Analy(g_psAttitudeAll->EarthFrameRelativeHome.north, 10, &g_sMathFloaterAnaly);		
		bsp_OLED0_96_Show_Floater(&g_sOled0_96, 66, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);		
	
		/*3.2 EarthFrame_Lon_cm*/
	    math_Floater_Number_Analy(g_psAttitudeAll->EarthFrameRelativeHome.east, 10, &g_sMathFloaterAnaly);		
		bsp_OLED0_96_Show_Floater(&g_sOled0_96, 66, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
		/*3.3 BodyFrame_Lat_cm*/
        math_Floater_Number_Analy(g_psAttitudeAll->BodyFrameRelativeHome.y, 10, &g_sMathFloaterAnaly);		
		bsp_OLED0_96_Show_Floater(&g_sOled0_96, 66, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
		/*3.4 BodyFrame_Lon_cm*/
        math_Floater_Number_Analy(g_psAttitudeAll->BodyFrameRelativeHome.x, 10, &g_sMathFloaterAnaly);		
		bsp_OLED0_96_Show_Floater(&g_sOled0_96, 66, 7, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	
	}
	/*无效清除*/
	else if (g_psUav_Status->UavSenmodStatus.Horizontal.Gps.USE_CONTROL_STATUS == UAV_SENMOD_USE_CONTROL_DISALLOW)
	{
		bsp_OLED0_96_Display_Part(&g_sOled0_96, 66, 4, 127, 4, 0x00);
		bsp_OLED0_96_Display_Part(&g_sOled0_96, 66, 5, 127, 5, 0x00);
		bsp_OLED0_96_Display_Part(&g_sOled0_96, 66, 6, 127, 6, 0x00);
		bsp_OLED0_96_Display_Part(&g_sOled0_96, 66, 7, 127, 7, 0x00);		
	}
}

/*No12.控制模式和任务显示*/
void hci_Show_Ctrl_Mission_Data(void)
{			
/*6.简排版
NOC: CTRL_MISSION  *

FlyType: ATTITUDE    //飞行模式
ec_V: AUTO ec_H: AUTO//期望控制模式
rc_V: FIXH rc_H: FIXP//真实控制模式
su_V: BERO su_H: GPS //传感器使用对象
FlyMission:			 //飞行任务
ONEKEY_LAND_HOME
*/
	/*判断是否锁定当前页*/
	if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, '*', OLED096_ACSII_6X8);
	}
	else if (g_sHciShowPage.PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
	{
		bsp_OLED0_96_ShowChar(&g_sOled0_96, 120, 0, ' ', OLED096_ACSII_6X8);	
	}

	/*显示布局框架,然后再填数字*/
	if (g_sHciShowPage.MOULD_STATUS == HCI_SHOW_MOULD_FIRST)
	{
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 0, (u8*)&"NOC: CTRL_MISSION", OLED096_ACSII_6X8);		
		
		/*第1行空行*/	
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"FlyType: ", OLED096_ACSII_6X8);					
		
		/*第3行*/	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"ec_V: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"ec_H: ", OLED096_ACSII_6X8);			
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"rc_V: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 4, (u8*)&"rc_H: ", OLED096_ACSII_6X8);	

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"su_V: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 5, (u8*)&"su_H: ", OLED096_ACSII_6X8);

		/*第6行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"FlyMission:", OLED096_ACSII_6X8);					
		
		/*第7行*/

		/*标记模板框显示过了*/
		g_sHciShowPage.MOULD_STATUS = HCI_SHOW_MOULD_NOTFIRST;
	}
	
	/*1.FlyType*/
	if (g_psUav_Status->UavFlyType.CURRENT == UAV_FLY_TYPE_ATTITUDE) /*姿态*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 54, 2, (u8*)&"ATTITUDE  ", OLED096_ACSII_6X8);
	}
	else if (g_psUav_Status->UavFlyType.CURRENT == UAV_FLY_TYPE_FIX_HEIGHT) /*定高*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 54, 2, (u8*)&"FIX_HEIGHT", OLED096_ACSII_6X8);	
	}
	else if (g_psUav_Status->UavFlyType.CURRENT == UAV_FLY_TYPE_FIX_POS) /*定点*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 54, 2, (u8*)&"FIX_POS   ", OLED096_ACSII_6X8);		
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 54, 2, (u8*)&"UNKNOW    ", OLED096_ACSII_6X8);	
	}
	
	/*2.Control Mode*/
	/*2.1 竖直期望控制*/
	if (g_psUav_Status->UavControlMode.Vertical.EXPECT_CONTROL_MODE == UAV_VERTICAL_CONTROL_SELFAUTO) /*竖直期望控制:自稳*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 3, (u8*)&"AUTO", OLED096_ACSII_6X8);
	}
	else if (g_psUav_Status->UavControlMode.Vertical.EXPECT_CONTROL_MODE == UAV_VERTICAL_CONTROL_FIX_HEIGHT) /*竖直期望控制:定高*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 3, (u8*)&"FIXH", OLED096_ACSII_6X8);	
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 3, (u8*)&"UKNW", OLED096_ACSII_6X8);	 /*竖直期望控制:未知*/
	}	
	
	/*2.2 竖真实际控制*/
	if (g_psUav_Status->UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_SELFAUTO) /*竖直真实控制:自稳*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 4, (u8*)&"AUTO", OLED096_ACSII_6X8);
	}
	else if (g_psUav_Status->UavControlMode.Vertical.CONTROL_MODE == UAV_VERTICAL_CONTROL_FIX_HEIGHT) /*竖直真实控制:定高*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 4, (u8*)&"FIXH", OLED096_ACSII_6X8);	
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 4, (u8*)&"UKNW", OLED096_ACSII_6X8);	 /*竖直真实控制:未知*/
	}	
	
	/*2.3 水平期望控制*/
	if (g_psUav_Status->UavControlMode.Horizontal.EXPECT_CONTROL_MODE == UAV_VERTICAL_CONTROL_SELFAUTO) /*水平期望控制:自稳*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 3, (u8*)&"AUTO", OLED096_ACSII_6X8);
	}
	else if (g_psUav_Status->UavControlMode.Horizontal.EXPECT_CONTROL_MODE == UAV_HORIZONTAL_CONTROL_FIX_POS) /*水平期望控制:定点*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 3, (u8*)&"FIXP", OLED096_ACSII_6X8);	
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 3, (u8*)&"UKNW", OLED096_ACSII_6X8);	 /*水平期望控制:未知*/
	}	
	
	/*2.4 水平实际控制*/
	if (g_psUav_Status->UavControlMode.Horizontal.CONTROL_MODE == UAV_VERTICAL_CONTROL_SELFAUTO) /*水平真实控制:自稳*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 4, (u8*)&"AUTO", OLED096_ACSII_6X8);
	}
	else if (g_psUav_Status->UavControlMode.Horizontal.CONTROL_MODE == UAV_HORIZONTAL_CONTROL_FIX_POS) /*水平真实控制:定点*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 4, (u8*)&"FIXP", OLED096_ACSII_6X8);	
	}
	else
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 4, (u8*)&"UKNW", OLED096_ACSII_6X8);	 /*水平真实控制:未知*/
	}	
	
	/*3.Sensor use*/
	/*3.1 竖直方向当前使用的传感器*/
	if (g_psUav_Status->UavSenmodStatus.Vertical.CURRENT_USE == UAV_VERTICAL_SENMOD_CURRENT_BERO) /*气压计*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 5, (u8*)&"BERO", OLED096_ACSII_6X8);
	}
	else if (g_psUav_Status->UavSenmodStatus.Vertical.CURRENT_USE == UAV_VERTICAL_SENMOD_CURRENT_ULTR) /*超声波*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 5, (u8*)&"ULTR", OLED096_ACSII_6X8);	
	}
	else if (g_psUav_Status->UavSenmodStatus.Vertical.CURRENT_USE == UAV_VERTICAL_SENMOD_CURRENT_NULL) /*未知*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 36, 5, (u8*)&"UKNW", OLED096_ACSII_6X8);	
	}

	/*3.2 水平方向当前使用的传感器*/
	if (g_psUav_Status->UavSenmodStatus.Horizontal.CURRENT_USE == UAV_HORIZONTAL_SENMOD_CURRENT_GPS) /*GPS*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 5, (u8*)&"GPS ", OLED096_ACSII_6X8);
	}
	else if (g_psUav_Status->UavSenmodStatus.Horizontal.CURRENT_USE == UAV_HORIZONTAL_SENMOD_CURRENT_OPTICFLOW) /*光流*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 5, (u8*)&"OPTC", OLED096_ACSII_6X8);	
	}
	else if (g_psUav_Status->UavSenmodStatus.Horizontal.CURRENT_USE == UAV_HORIZONTAL_SENMOD_CURRENT_OPTICFLOW) /*未知*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 102, 5, (u8*)&"UKNW", OLED096_ACSII_6X8);	
	}	
	
	/*3.FlyMission*/
	if (gs_Uav_Fly_Mission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_NULL) /*没有任务*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"NULL               ", OLED096_ACSII_6X8);
	}
	else if (gs_Uav_Fly_Mission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_ONEKEY_FLY) /*一键起飞*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"ONEKEY_FLY         ", OLED096_ACSII_6X8);	
	}
	else if (gs_Uav_Fly_Mission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_ONEKEY_LAND_HOME) /*一键返航/降落*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"ONEKEY_LAND_HOME   ", OLED096_ACSII_6X8);	
	}	
	else if (gs_Uav_Fly_Mission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_OPFLOW_FOLLOW_POINT) /*光流追点*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"OPFLOW_FOLLOW_POINT", OLED096_ACSII_6X8);	
	}	
	else if (gs_Uav_Fly_Mission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_OPFLOW_FOLLOW_LINE) /*光流巡线*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"OPFLOW_FOLLOW_LINE ", OLED096_ACSII_6X8);	
	}
	else if (gs_Uav_Fly_Mission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_GPS_WRITE_POS) 	 /*GPS写入当前点位*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"GPS_WRITE_POS      ", OLED096_ACSII_6X8);	
	}	
	else if (gs_Uav_Fly_Mission.CURRENT_FLY_MISSION == UAV_FLY_MISSION_GPS_PATROL_SKY) /*GPS巡天*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"GPS_PATROL_SKY     ", OLED096_ACSII_6X8);	
	}
	else /*UNKNOW*/
	{
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"UNKNOW             ", OLED096_ACSII_6X8);	
	}
}

/*==== 5.传感器校准交互 ====*/
/*1.进入/退出加速度计校准提示*/
void hci_Show_Acc_Calib_Status(ENTRY_CALIB_STATUS ENTRY_STATUS, CALIB_SIDE_INDEX GONNA_CALIB_SIDE)
{ 
	static U8_Bit_Flag accShowBitFlag = {BIT_FLAG_RESET};
/*2.简排版
      ACC_CALIB
	                *
 **** ****  **** ****
 *  * *  *  *  * * **
 ** * *  *  * ** ** *
 * ** *  *  ** * *  *
 **** ****  **** ****
	 *     *
   START       EXIT
*/	
	/*进入加速度计校准提示*/
	if (ENTRY_STATUS == ENTRY_CALIB_INTO)
	{
		/*禁止显示提示界面*/
		g_sHciShowPage.SHOW_HINT_STATUS = UAV_HCI_SHOW_DISABLE;
		
		/*标记显示任务忙碌*/
		g_sHciShowPage.SHOW_TASK_STATUS = HCI_SHOW_TASK_BUSY;
			
		/*显示操作提示*/
		switch (GONNA_CALIB_SIDE)
		{
			case SIDE_INDEX_1ST:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit0 == BIT_FLAG_RESET)
				{
					/*清屏*/
					bsp_OLED0_96_Clear(&g_sOled0_96);					
				
					/*显示提示画面*/
					bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 1, 128, 6, (u8 *)&g_AccCalib_1st_Side_48X128);
	
					/*显示提示语*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 17, 7, (u8*)&"S:1ST", OLED096_ACSII_6X8);
					
					/*标记已显示*/
					accShowBitFlag.bit0 = BIT_FLAG_SET;
				}
			}break;

			case SIDE_INDEX_2ND:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit1 == BIT_FLAG_RESET)
				{				
					/*清屏*/
					bsp_OLED0_96_Clear(&g_sOled0_96);
				
					/*显示提示画面*/
					bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 1, 128, 6, (u8 *)&g_AccCalib_2nd_Side_48X128);
	
					/*显示提示语*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 17, 7, (u8*)&"S:2ND", OLED096_ACSII_6X8);		
					
					/*标记已显示*/
					accShowBitFlag.bit1 = BIT_FLAG_SET;
				}					
			}break;

			case SIDE_INDEX_3RD:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit2 == BIT_FLAG_RESET)
				{					
					/*清屏*/
					bsp_OLED0_96_Clear(&g_sOled0_96);
				
					/*显示提示画面*/
					bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 1, 128, 6, (u8 *)&g_AccCalib_3rd_Side_48X128);
	
					/*显示提示语*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 17, 7, (u8*)&"S:3RD", OLED096_ACSII_6X8);	
					
					/*标记已显示*/
					accShowBitFlag.bit2 = BIT_FLAG_SET;
				}						
			}break;

			case SIDE_INDEX_4TH:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit3 == BIT_FLAG_RESET)
				{					
					/*清屏*/
					bsp_OLED0_96_Clear(&g_sOled0_96);
				
					/*显示提示画面*/
					bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 1, 128, 6, (u8 *)&g_AccCalib_4th_Side_48X128);
	
					/*显示提示语*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 17, 7, (u8*)&"S:4TH", OLED096_ACSII_6X8);		
					
					/*标记已显示*/
					accShowBitFlag.bit3 = BIT_FLAG_SET;
				}						
			}break;

			case SIDE_INDEX_5TH:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit4 == BIT_FLAG_RESET)
				{					
					/*清屏*/
					bsp_OLED0_96_Clear(&g_sOled0_96);
				
					/*显示提示画面*/
					bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 1, 128, 6, (u8 *)&g_AccCalib_5th_Side_48X128);
	
					/*显示提示语*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 17, 7, (u8*)&"S:5TH", OLED096_ACSII_6X8);	
					
					/*标记已显示*/
					accShowBitFlag.bit4 = BIT_FLAG_SET;
				}						
			}break;		
			
			case SIDE_INDEX_6TH:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit5 == BIT_FLAG_RESET)
				{					
					/*清屏*/
					bsp_OLED0_96_Clear(&g_sOled0_96);
				
					/*显示提示画面*/
					bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 1, 128, 6, (u8 *)&g_AccCalib_6th_Side_48X128);
	
					/*显示提示语*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 17, 7, (u8*)&"S:6TH", OLED096_ACSII_6X8);	
					
					/*标记已显示*/
					accShowBitFlag.bit5 = BIT_FLAG_SET;
				}						
			}break;		

			default:break;
		}
		
		/*显示标题*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 37, 0, (u8*)&"ACC_CALIB", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 7, (u8*)&"EXIT", OLED096_ACSII_6X8);	
	}
	/*退出加速度计校准*/	
	else if (ENTRY_STATUS == ENTRY_CALIB_EXIT)
	{
		/*状态机复位*/
		accShowBitFlag.bit0 = BIT_FLAG_RESET;
		accShowBitFlag.bit1 = BIT_FLAG_RESET;
		accShowBitFlag.bit2 = BIT_FLAG_RESET;
		accShowBitFlag.bit3 = BIT_FLAG_RESET;
		accShowBitFlag.bit4 = BIT_FLAG_RESET;		
		accShowBitFlag.bit5 = BIT_FLAG_RESET;
		
		/*判断接下来显示内容*/
		if (g_sHciShowPage.SHOW_DATA_STATUS == UAV_HCI_SHOW_ENABLE) /*已经允许显示,则切换到默认首页*/
		{				
			/*恢复默认显示页序号*/
			g_sHciShowPage.curPageIndex  = HCI_SHOW_PAGE_0;
			g_sHciShowPage.lastPageIndex = (HCI_SHOW_PAGE_INDEX)(g_sHciShowPage.curPageIndex + 1);
		}	
		else		
		{
			/*允许显示提示界面*/
			g_sHciShowPage.SHOW_HINT_STATUS = UAV_HCI_SHOW_ENABLE;
			
			/*重新显示进入菜单显示提示页面*/
			g_sHciShowPage.EXIT_SHOW_STATUS = HCI_EXIT_SHOW_OP_FIRST;
		}
		
		/*标记显示任务空闲*/
		g_sHciShowPage.SHOW_TASK_STATUS = HCI_SHOW_TASK_IDLE;			
	}
}

/*2.进入/退出磁力计校准提示*/
void hci_Show_Mag_Calib_Status(ENTRY_CALIB_STATUS ENTRY_STATUS, CALIB_SIDE_INDEX GONNA_CALIB_SIDE)
{
	static U8_Bit_Flag magShowBitFlag = {BIT_FLAG_RESET};		
/*2.简排版
      MAG_CALIB
	                *
 **** ****  **** ****
 *  * *  *  *  * * **
 ** * *  *  * ** ** *
 * ** *  *  ** * *  *
 **** ****  **** ****
	 *     *
   START       EXIT
*/	
	/*进入磁力计校准提示*/
	if (ENTRY_STATUS == ENTRY_CALIB_INTO)
	{
		/*禁止显示提示界面*/
		g_sHciShowPage.SHOW_HINT_STATUS = UAV_HCI_SHOW_DISABLE;
		
		/*标记显示任务忙碌*/
		g_sHciShowPage.SHOW_TASK_STATUS = HCI_SHOW_TASK_BUSY;
			
		/*显示操作提示*/
		switch (GONNA_CALIB_SIDE)
		{
			case SIDE_INDEX_1ST:
			{
				/*只显示一次*/
				if (magShowBitFlag.bit0 == BIT_FLAG_RESET)
				{					
					/*清屏*/
					bsp_OLED0_96_Clear(&g_sOled0_96);					
				
					/*显示提示画面*/
					bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 1, 128, 6, (u8 *)&g_MagCalib_1st_Side_48X128);
	
					/*显示提示语*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 17, 7, (u8*)&"S:1ST", OLED096_ACSII_6X8);
					
					/*标记已显示*/
					magShowBitFlag.bit0 = BIT_FLAG_SET;
				}
			}break;

			case SIDE_INDEX_2ND:
			{
				/*只显示一次*/
				if (magShowBitFlag.bit1 == BIT_FLAG_RESET)
				{					
					/*清屏*/
					bsp_OLED0_96_Clear(&g_sOled0_96);
				
					/*显示提示画面*/
					bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 1, 128, 6, (u8 *)&g_MagCalib_2nd_Side_48X128);
	
					/*显示提示语*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 17, 7, (u8*)&"S:2ND", OLED096_ACSII_6X8);	

					/*标记已显示*/
					magShowBitFlag.bit1 = BIT_FLAG_SET;
				}
			}break;

			case SIDE_INDEX_3RD:
			{
				/*只显示一次*/
				if (magShowBitFlag.bit2 == BIT_FLAG_RESET)
				{				
					/*清屏*/
					bsp_OLED0_96_Clear(&g_sOled0_96);
				
					/*显示提示画面*/
					bsp_OLED0_96_DrawBMP(&g_sOled0_96, 0, 1, 128, 6, (u8 *)&g_MagCalib_3rd_Side_48X128);
	
					/*显示提示语*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 17, 7, (u8*)&"S:3RD", OLED096_ACSII_6X8);		

					/*标记已显示*/
					magShowBitFlag.bit2 = BIT_FLAG_SET;					
				}
			}break;	

			default:break;
		}			
			
		/*显示标题*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 37, 0, (u8*)&"MAG_CALIB", OLED096_ACSII_6X8);		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 84, 7, (u8*)&"EXIT", OLED096_ACSII_6X8);			
	}
	/*退出磁力计校准*/	
	else if (ENTRY_STATUS == ENTRY_CALIB_EXIT)
	{
		/*状态机复位*/
		magShowBitFlag.bit0 = BIT_FLAG_RESET;
		magShowBitFlag.bit1 = BIT_FLAG_RESET;
		magShowBitFlag.bit2 = BIT_FLAG_RESET;		
		
		/*判断接下来显示内容*/
		if (g_sHciShowPage.SHOW_DATA_STATUS == UAV_HCI_SHOW_ENABLE) /*已经允许显示,则切换到默认首页*/
		{				
			/*恢复默认显示页序号*/			
			g_sHciShowPage.curPageIndex  = HCI_SHOW_PAGE_0;
			g_sHciShowPage.lastPageIndex = (HCI_SHOW_PAGE_INDEX)(g_sHciShowPage.curPageIndex + 1);
		}	
		else		
		{
			/*允许显示提示界面*/
			g_sHciShowPage.SHOW_HINT_STATUS = UAV_HCI_SHOW_ENABLE;
			
			/*重新显示进入菜单显示提示页面*/
			g_sHciShowPage.EXIT_SHOW_STATUS = HCI_EXIT_SHOW_OP_FIRST;
		}
		
		/*标记显示任务空闲*/
		g_sHciShowPage.SHOW_TASK_STATUS = HCI_SHOW_TASK_IDLE;
	}	
}

/*3.加速度计校准显示*/
void hci_Show_Acc_Calib_Process(CALIB_SIDE_INDEX CUR_SIDE_INDEX, SAMPLE_PROCESS_TYPE PROCESS_TYPE, SINGLE_SAMPLE_STATUS SIGSAMPLE_STATUS, fp32 gyroLenth, fp32 xg, fp32 yg, fp32 zg, u32 holdMs)
{
	static volatile CALIB_SIDE_INDEX LAST_SIDE_INDEX = SIDE_INDEX_NULL;
	static U8_Bit_Flag accShowBitFlag = {BIT_FLAG_RESET};
	static u8 showLogXPos  = 30;
	u8 *pstr1, *pstr2;
/*1.显示内容*/
/*

*/ 
/*2.简排版
    ACC_CALIB_SHOW	
	
SideNbr:04 PitchFront
nextNbr:05 PitchBack
GyroLenth: ±20.000
xg:±0.999  yg:±0.999 
zg:±0.999  RST: SUCC
Log: -0-0-0-0-0-0
*/	
	/*显示布局框架,然后再填数字*/
	if (CUR_SIDE_INDEX != LAST_SIDE_INDEX)
	{
		/*标记模板框显示过了*/
		LAST_SIDE_INDEX = CUR_SIDE_INDEX;		
		
		/*禁止显示提示界面*/
		g_sHciShowPage.SHOW_HINT_STATUS = UAV_HCI_SHOW_DISABLE;
		
		/*标记显示任务忙碌*/
		g_sHciShowPage.SHOW_TASK_STATUS = HCI_SHOW_TASK_BUSY;
		
		/*清屏*/
		bsp_OLED0_96_Clear(&g_sOled0_96);
		
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 37, 0, (u8*)&"ACC_CALIB", OLED096_ACSII_6X8);
		
		/*第1行*/	
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"SideNbr:", OLED096_ACSII_6X8);				
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"nextNbr:", OLED096_ACSII_6X8);		
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"GyroLenth: ", OLED096_ACSII_6X8);	

		/*第5行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 5, (u8*)&"xg:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 5, (u8*)&"yg:", OLED096_ACSII_6X8);

		/*第6行*/	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 6, (u8*)&"zg:", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 6, (u8*)&"RST:", OLED096_ACSII_6X8);		
		
		/*第7行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 7, (u8*)&"Log: ", OLED096_ACSII_6X8);	
	}
	
	/*显示本次采样过程中的动态数据*/
	if (PROCESS_TYPE == SAMPLE_PROCESS_SAMPLEING)
	{
		/*显示采样页序号*/
		switch(CUR_SIDE_INDEX)
		{
			case SIDE_INDEX_1ST:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit0 == BIT_FLAG_RESET)
				{
					pstr1 = (u8*)&"01 Top       ";
					pstr2 = (u8*)&"02 RollLeft  ";

					/*显示当前页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 2, pstr1, OLED096_ACSII_6X8);
		
					/*显示下一页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 3, pstr2, OLED096_ACSII_6X8);			
					
					/*标记已显示*/
					accShowBitFlag.bit0 = BIT_FLAG_SET;
				}
			}break;

			case SIDE_INDEX_2ND:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit1 == BIT_FLAG_RESET)
				{				
					pstr1 = (u8*)&"02 RollLeft  ";
					pstr2 = (u8*)&"03 RollRight ";	
			
					/*显示当前页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 2, pstr1, OLED096_ACSII_6X8);
		
					/*显示下一页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 3, pstr2, OLED096_ACSII_6X8);

					/*标记已显示*/
					accShowBitFlag.bit1 = BIT_FLAG_SET;
				}
			}break;

			case SIDE_INDEX_3RD:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit2 == BIT_FLAG_RESET)
				{				
					pstr1 = (u8*)&"03 RollRight ";
					pstr2 = (u8*)&"04 PitchFront";

					/*显示当前页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 2, pstr1, OLED096_ACSII_6X8);
		
					/*显示下一页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 3, pstr2, OLED096_ACSII_6X8);	
					
					/*标记已显示*/
					accShowBitFlag.bit2 = BIT_FLAG_SET;					
				}
			}break;

			case SIDE_INDEX_4TH:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit3 == BIT_FLAG_RESET)
				{				
					pstr1 = (u8*)&"04 PitchFront";
					pstr2 = (u8*)&"05 PitchBack ";	

					/*显示当前页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 2, pstr1, OLED096_ACSII_6X8);
		
					/*显示下一页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 3, pstr2, OLED096_ACSII_6X8);	
					
					/*标记已显示*/
					accShowBitFlag.bit3 = BIT_FLAG_SET;					
				}					
			}break;

			case SIDE_INDEX_5TH:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit4 == BIT_FLAG_RESET)
				{					
					pstr1 = (u8*)&"05 PitchBack ";
					pstr2 = (u8*)&"06 Under     ";	

					/*显示当前页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 2, pstr1, OLED096_ACSII_6X8);
		
					/*显示下一页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 3, pstr2, OLED096_ACSII_6X8);
					
					/*标记已显示*/
					accShowBitFlag.bit4 = BIT_FLAG_SET;											
				}
			}break;

			case SIDE_INDEX_6TH:
			{
				/*只显示一次*/
				if (accShowBitFlag.bit5 == BIT_FLAG_RESET)
				{					
					pstr1 = (u8*)&"06 Under     ";
			
					/*显示当前页序号*/
					bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 2, pstr1, OLED096_ACSII_6X8);			
					
					/*标记已显示*/
					accShowBitFlag.bit5 = BIT_FLAG_SET;						
				}
			}break;

			default:break;
		}
	
		/*显示陀螺仪步长(判断静止)*/
		math_Floater_Number_Analy(gyroLenth, 7, &g_sMathFloaterAnaly);
		bsp_OLED0_96_Show_Floater(&g_sOled0_96, 66, 4, OLED096_ACSII_6X8, g_sMathFloaterAnaly);
	
		/*显示x y z加速度值*/
		math_Floater_Number_Analy(xg, 6, &g_sMathFloaterAnaly);
		bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);	

		math_Floater_Number_Analy(yg, 6, &g_sMathFloaterAnaly);
		bsp_OLED0_96_Show_Floater(&g_sOled0_96, 84, 5, OLED096_ACSII_6X8, g_sMathFloaterAnaly);		

		math_Floater_Number_Analy(zg, 6, &g_sMathFloaterAnaly);
		bsp_OLED0_96_Show_Floater(&g_sOled0_96, 18, 6, OLED096_ACSII_6X8, g_sMathFloaterAnaly);		
	}
	/*显示本次采样的结果*/
	else if (PROCESS_TYPE == SAMPLE_PROCESS_RESULT)
	{
		/*显示结果*/
		if (SIGSAMPLE_STATUS == SINGLE_SAMPLE_SUCC)
		{
			/*显示本次结果*/
			bsp_OLED0_96_ShowString(&g_sOled0_96, 96, 6, (u8*)&"SUCC", OLED096_ACSII_6X8);	
		
			/*显示历史结果*/		
			bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos, 7, (u8*)&"-0", OLED096_ACSII_6X8);
			showLogXPos += 12;
		}
		else
		{
			/*显示本次结果*/
			bsp_OLED0_96_ShowString(&g_sOled0_96, 96, 6, (u8*)&"FAIL", OLED096_ACSII_6X8);		
		
			/*显示历史结果*/	
			bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos, 7, (u8*)&"-1", OLED096_ACSII_6X8);
			showLogXPos += 12;		
		}
	}
	
	/*延时*/
	sys_DelayMs(holdMs);
	
	/*六面采样完毕*/
	if ((CUR_SIDE_INDEX == SIDE_INDEX_6TH) && \
		(PROCESS_TYPE == SAMPLE_PROCESS_RESULT))
	{
		/*状态机复位*/
		LAST_SIDE_INDEX     = SIDE_INDEX_NULL;
		accShowBitFlag.bit0 = BIT_FLAG_RESET;
		accShowBitFlag.bit1 = BIT_FLAG_RESET;
		accShowBitFlag.bit2 = BIT_FLAG_RESET;
		accShowBitFlag.bit3 = BIT_FLAG_RESET;
		accShowBitFlag.bit4 = BIT_FLAG_RESET;		
		accShowBitFlag.bit5 = BIT_FLAG_RESET;
		
		/*坐标点复位*/
		showLogXPos = 30;
		
		/*清屏*/
		bsp_OLED0_96_Clear(&g_sOled0_96);
		
		/*判断接下来显示内容*/
		if (g_sHciShowPage.SHOW_DATA_STATUS == UAV_HCI_SHOW_ENABLE) /*已经允许显示,则切换到默认首页*/
		{
			/*恢复默认显示页序号*/			
			g_sHciShowPage.curPageIndex  = HCI_SHOW_PAGE_0;
			g_sHciShowPage.lastPageIndex = (HCI_SHOW_PAGE_INDEX)(g_sHciShowPage.curPageIndex + 1);
		}
		else		
		{		
			/*允许显示提示界面*/
			g_sHciShowPage.SHOW_HINT_STATUS = UAV_HCI_SHOW_ENABLE;
			
			/*重新显示进入菜单显示提示页面*/
			g_sHciShowPage.EXIT_SHOW_STATUS = HCI_EXIT_SHOW_OP_FIRST;		
		}
		
		/*标记显示任务空闲*/
		g_sHciShowPage.SHOW_TASK_STATUS = HCI_SHOW_TASK_IDLE;		
	}
}

/*4.磁力计校准显示*/
void hci_Show_Mag_Calib_Process(CALIB_SIDE_INDEX CUR_SIDE_INDEX, CALIB_POSITION_INDEX POSITION_INDEX, POSITION_SAMPLE_STATUS POS_SAMP_STATUS, SAMPLE_PROCESS_TYPE PROCESS_TYPE, u16 curYaw, u16 targetYaw, u8 errorMaxYaw, s16 targAccAxis, u32 holdMs)
{
	static volatile CALIB_SIDE_INDEX LAST_SIDE_INDEX = SIDE_INDEX_NULL;
	static U8_Bit_Flag magShowBitFlag = {BIT_FLAG_RESET};
	static u8 showLogXPos1 = 48;
	static u8 showLogXPos2 = 0;
	static u8 showLogXPos3 = 0;
	static u8 showLogXPos4 = 0;
/*1.显示内容*/
/*

*/ 	
/*2.简排版
	MAG_CALIB_SHOW
cPos: 1-03 nPos: 1-04
cYaw:360 tYaw:360 r10
AccAxis:+x Val: ±8192 
LOG:SUCC-0-0-0-0-0-0
-0-0-0-0-0-0-0-0-0-0
-0-0-0-0-0-0-0-0-0-0
-0-0-0-0-0-0-0-0-0-0

*/
	/*显示布局框架,然后再填数字*/
	if (CUR_SIDE_INDEX != LAST_SIDE_INDEX)
	{
		/*标记模板框显示过了*/			
		LAST_SIDE_INDEX = CUR_SIDE_INDEX;
		
		/*禁止显示提示界面*/
		g_sHciShowPage.SHOW_HINT_STATUS = UAV_HCI_SHOW_DISABLE;
		
		/*标记显示任务忙碌*/
		g_sHciShowPage.SHOW_TASK_STATUS = HCI_SHOW_TASK_BUSY;
		
		/*清屏*/
		bsp_OLED0_96_Clear(&g_sOled0_96);
		
		/*第0行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 37, 0, (u8*)&"MAG_CALIB", OLED096_ACSII_6X8);				

		/*第1行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 1, (u8*)&"cPos: ", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 42, 1, (u8*)&"-", OLED096_ACSII_6X8);		
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 1, (u8*)&"nPos: ", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 108, 1, (u8*)&"-", OLED096_ACSII_6X8);			
		
		/*第2行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 2, (u8*)&"cYaw:", OLED096_ACSII_6X8);
		bsp_OLED0_96_ShowString(&g_sOled0_96, 54, 2, (u8*)&"tYaw:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 108, 2, (u8*)&"r", OLED096_ACSII_6X8);			
		
		/*第3行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 3, (u8*)&"AccAxis:", OLED096_ACSII_6X8);	
		bsp_OLED0_96_ShowString(&g_sOled0_96, 66, 3, (u8*)&"Val: ", OLED096_ACSII_6X8);
		
		/*第4行*/
		bsp_OLED0_96_ShowString(&g_sOled0_96, 0, 4, (u8*)&"LOG:", OLED096_ACSII_6X8);				
	}
	
	/*显示本次采样过程中的动态数据*/
	if (PROCESS_TYPE == SAMPLE_PROCESS_SAMPLEING)
	{
		/*显示采样页序号*/
		switch(CUR_SIDE_INDEX)
		{
			case SIDE_INDEX_1ST:
			{
				/*只显示一次*/				
				if (magShowBitFlag.bit0 == BIT_FLAG_RESET)
				{
					/*本次面序号*/
					bsp_OLED0_96_ShowNum(&g_sOled0_96, 36, 1, SIDE_INDEX_1ST + 1, 1, OLED096_ACSII_6X8);
			
					/*本次角点序号*/
					math_Integer_Number_Analy(POSITION_INDEX + 1, 2, &g_sMathIntegerAnaly);
					bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 48, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);			
			
					/*本次角点序号是1~35,则下次是2~36*/
					if (POSITION_INDEX <= POSITION_INDEX_36TH)
					{
						/*下次面序号*/
						bsp_OLED0_96_ShowNum(&g_sOled0_96, 102, 1, SIDE_INDEX_1ST + 1, 1, OLED096_ACSII_6X8);
			
						/*下次角点序号*/
						math_Integer_Number_Analy(POSITION_INDEX + 1 + 1, 2, &g_sMathIntegerAnaly);				
						bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 114, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);				
					}
			
					/*本次是1-36,下次是2-1*/
					else if (POSITION_INDEX >= POSITION_INDEX_36TH)
					{
						/*下次面序号*/
						bsp_OLED0_96_ShowNum(&g_sOled0_96, 102, 1, SIDE_INDEX_2ND + 1, 1, OLED096_ACSII_6X8);
			
						/*下次角点序号*/
						math_Integer_Number_Analy(POSITION_INDEX_1ST + 1, 2, &g_sMathIntegerAnaly);				
						bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 114, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);				
					}
					
					/*标记已显示*/
					magShowBitFlag.bit0 = BIT_FLAG_SET;		
				}
			}break;

			case SIDE_INDEX_2ND:
			{
				/*只显示一次*/				
				if (magShowBitFlag.bit1 == BIT_FLAG_RESET)
				{				
					/*本次面序号*/
					bsp_OLED0_96_ShowNum(&g_sOled0_96, 36, 1, SIDE_INDEX_2ND + 1, 1, OLED096_ACSII_6X8);
					
					/*本次角点序号*/
					math_Integer_Number_Analy(POSITION_INDEX + 1, 2, &g_sMathIntegerAnaly);
					bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 48, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
					
					/*本次角点序号是1~35,则下次是2~36*/
					if (POSITION_INDEX <= POSITION_INDEX_35TH)
					{
						/*下次面序号*/
						bsp_OLED0_96_ShowNum(&g_sOled0_96, 102, 1, SIDE_INDEX_2ND + 1, 1, OLED096_ACSII_6X8);
					
						/*下次角点序号*/
						math_Integer_Number_Analy(POSITION_INDEX + 1 + 1, 2, &g_sMathIntegerAnaly);				
						bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 114, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);				
					}
					/*本次是2-36,下次是3-1*/
					else if (POSITION_INDEX >= POSITION_INDEX_36TH)
					{			
						/*下次面序号*/
						bsp_OLED0_96_ShowNum(&g_sOled0_96, 102, 1, SIDE_INDEX_3RD + 1, 1, OLED096_ACSII_6X8);
				
						/*下次角点序号*/
						math_Integer_Number_Analy(POSITION_INDEX_1ST + 1, 2, &g_sMathIntegerAnaly);				
						bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 114, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);						
					}
					
					/*标记已显示*/
					magShowBitFlag.bit1 = BIT_FLAG_SET;						
				}
			}break;
			
			case SIDE_INDEX_3RD:
			{
				/*只显示一次*/				
				if (magShowBitFlag.bit2 == BIT_FLAG_RESET)
				{					
					/*本次面序号*/
					bsp_OLED0_96_ShowNum(&g_sOled0_96, 36, 1, SIDE_INDEX_3RD + 1, 1, OLED096_ACSII_6X8);
				
					/*本次角点序号*/
					math_Integer_Number_Analy(POSITION_INDEX + 1, 2, &g_sMathIntegerAnaly);
					bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 48, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);
	
					/*本次角点序号是1~35,则下次是2~36*/
					if (POSITION_INDEX <= POSITION_INDEX_35TH)
					{
						/*下次面序号*/
						bsp_OLED0_96_ShowNum(&g_sOled0_96, 102, 1, SIDE_INDEX_3RD + 1, 1, OLED096_ACSII_6X8);
				
						/*下次角点序号*/
						math_Integer_Number_Analy(POSITION_INDEX + 1 + 1, 2, &g_sMathIntegerAnaly);				
						bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 114, 1, OLED096_ACSII_6X8, g_sMathIntegerAnaly);				
					}
					/*本次是3-36,下次是N-N*/
					else if (POSITION_INDEX >= POSITION_INDEX_36TH)
					{
						/*下次面序号*/
						bsp_OLED0_96_ShowChar(&g_sOled0_96, 102, 1, 'N', OLED096_ACSII_6X8);				
					
						/*下次角点序号*/
						bsp_OLED0_96_ShowChar(&g_sOled0_96, 114, 1, 'N', OLED096_ACSII_6X8);					
					}
					
					/*标记已显示*/
					magShowBitFlag.bit2 = BIT_FLAG_SET;						
				}
			}break;			
	
			default:break;
		}
	
		/*当前YAW角,目标Yaw角,有效范围r*/
		/*当前YAW角*/
		math_Integer_Number_Analy(curYaw, 3, &g_sMathIntegerAnaly);
		bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 30, 2, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	

		/*目标YAW角*/
		math_Integer_Number_Analy(targetYaw, 3, &g_sMathIntegerAnaly);
		bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 84, 2, OLED096_ACSII_6X8, g_sMathIntegerAnaly);		
	
		/*有效范围*/
		math_Integer_Number_Analy(errorMaxYaw, 2, &g_sMathIntegerAnaly);
		bsp_OLED0_96_Show_Calendar(&g_sOled0_96, 114, 2, OLED096_ACSII_6X8, g_sMathIntegerAnaly);		
	
		/*校准时竖直朝上的轴*/
		if (CUR_SIDE_INDEX == SIDE_INDEX_1ST)
		{
			bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 3, (u8*)&"+Z", OLED096_ACSII_6X8);	
		}
		else if (CUR_SIDE_INDEX == SIDE_INDEX_2ND)
		{
			bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 3, (u8*)&"+Y", OLED096_ACSII_6X8);			
		}
		else if (CUR_SIDE_INDEX == SIDE_INDEX_3RD)
		{
			bsp_OLED0_96_ShowString(&g_sOled0_96, 48, 3, (u8*)&"+X", OLED096_ACSII_6X8);			
		}		
	
		/*该轴加速度值*/
		math_Integer_Number_Analy(targAccAxis, 4, &g_sMathIntegerAnaly);
		bsp_OLED0_96_Show_Integer(&g_sOled0_96, 96, 3, OLED096_ACSII_6X8, g_sMathIntegerAnaly);	
	}
	/*显示本次采样的结果*/
	else if (PROCESS_TYPE == SAMPLE_PROCESS_RESULT)
	{
		/*本此(本面本角点采样结果)*/
		if (CUR_SIDE_INDEX == SIDE_INDEX_1ST)
		{
			if (POS_SAMP_STATUS == POSITION_SAMPLE_SUCC)
			{
				/*显示本次结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 4, (u8*)&"SUCC", OLED096_ACSII_6X8);	
		
				/*显示历史结果*/
				if (POSITION_INDEX <= POSITION_INDEX_6TH)
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos1, 4, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos1 += 12;					
				}
				else if ((POSITION_INDEX_7TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_16TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos2, 5, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos2 += 12;					
				}
				else if ((POSITION_INDEX_17TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_26TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos3, 6, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos3 += 12;					
				}
				else if ((POSITION_INDEX_27TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_36TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos4, 7, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos4 += 12;					
				}
			}
			else if (POS_SAMP_STATUS == POSITION_SAMPLE_FAIL)
			{
				/*显示本次结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 4, (u8*)&"FAIL", OLED096_ACSII_6X8);	
		
				/*显示历史结果*/		
				if (POSITION_INDEX <= POSITION_INDEX_6TH)
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos1, 4, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos1 += 12;					
				}
				else if ((POSITION_INDEX_7TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_16TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos2, 5, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos2 += 12;					
				}
				else if ((POSITION_INDEX_17TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_26TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos3, 6, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos3 += 12;					
				}
				else if ((POSITION_INDEX_27TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_36TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos4, 7, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos4 += 12;
				}			
			}
		}
		else if (CUR_SIDE_INDEX == SIDE_INDEX_2ND)
		{
			if (POS_SAMP_STATUS == POSITION_SAMPLE_SUCC)
			{	
				/*显示本次结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 4, (u8*)&"SUCC", OLED096_ACSII_6X8);	
		
				/*显示历史结果*/		
				if (POSITION_INDEX <= POSITION_INDEX_6TH)
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos1, 4, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos1 += 12;					
				}
				else if ((POSITION_INDEX_7TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_16TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos2, 5, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos2 += 12;					
				}
				else if ((POSITION_INDEX_17TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_26TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos3, 6, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos3 += 12;					
				}
				else if ((POSITION_INDEX_27TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_36TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos4, 7, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos4 += 12;					
				}
			}
			else if (POS_SAMP_STATUS == POSITION_SAMPLE_FAIL)
			{
				/*显示本次结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 4, (u8*)&"FAIL", OLED096_ACSII_6X8);	
		
				/*显示历史结果*/		
				if (POSITION_INDEX <= POSITION_INDEX_6TH)
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos1, 4, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos1 += 12;					
				}
				else if ((POSITION_INDEX_7TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_16TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos2, 5, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos2 += 12;					
				}
				else if ((POSITION_INDEX_17TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_26TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos3, 6, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos3 += 12;					
				}
				else if ((POSITION_INDEX_27TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_36TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos4, 7, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos4 += 12;
				}
			}		
		}
		else if (CUR_SIDE_INDEX == SIDE_INDEX_3RD)
		{
			if (POS_SAMP_STATUS == POSITION_SAMPLE_SUCC)
			{
				/*显示本次结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 4, (u8*)&"SUCC", OLED096_ACSII_6X8);	
		
				/*显示历史结果*/		
				if (POSITION_INDEX <= POSITION_INDEX_6TH)
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos1, 4, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos1 += 12;					
				}
				else if ((POSITION_INDEX_7TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_16TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos2, 5, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos2 += 12;					
				}
				else if ((POSITION_INDEX_17TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_26TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos3, 6, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos3 += 12;					
				}
				else if ((POSITION_INDEX_27TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_36TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos4, 7, (u8*)&"-0", OLED096_ACSII_6X8);
					showLogXPos4 += 12;					
				}	
			}
			else if (POS_SAMP_STATUS == POSITION_SAMPLE_FAIL)
			{
				/*显示本次结果*/
				bsp_OLED0_96_ShowString(&g_sOled0_96, 24, 4, (u8*)&"FAIL", OLED096_ACSII_6X8);	
		
				/*显示历史结果*/		
				if (POSITION_INDEX <= POSITION_INDEX_6TH)
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos1, 4, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos1 += 12;					
				}
				else if ((POSITION_INDEX_7TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_16TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos2, 5, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos2 += 12;					
				}
				else if ((POSITION_INDEX_17TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_26TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos3, 6, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos3 += 12;					
				}
				else if ((POSITION_INDEX_27TH <= POSITION_INDEX) && \
						 (POSITION_INDEX <= POSITION_INDEX_36TH))
				{
					bsp_OLED0_96_ShowString(&g_sOled0_96, showLogXPos4, 7, (u8*)&"-1", OLED096_ACSII_6X8);
					showLogXPos4 += 12;
				}
			}
		}
	}
	
	/*单面校准完毕,座标复位*/
	if (PROCESS_TYPE == SAMPLE_PROCESS_RESULT)
	{
		showLogXPos1 = 48;
		showLogXPos2 = 0;	
		showLogXPos3 = 0;
		showLogXPos4 = 0;
	}
		
	/*延时*/
	sys_DelayMs(holdMs);
		
	/*校准流程完毕*/
	if ((CUR_SIDE_INDEX == SIDE_INDEX_3RD) && \
		(POSITION_INDEX == POSITION_INDEX_36TH) && \
		(PROCESS_TYPE == SAMPLE_PROCESS_RESULT))
	{
		/*状态机复位*/
		LAST_SIDE_INDEX     = SIDE_INDEX_NULL;
		magShowBitFlag.bit0 = BIT_FLAG_RESET;		
		magShowBitFlag.bit1 = BIT_FLAG_RESET;
		magShowBitFlag.bit2 = BIT_FLAG_RESET;			
		
		/*清屏*/
		bsp_OLED0_96_Clear(&g_sOled0_96);
		
		/*判断接下来显示内容*/
		if (g_sHciShowPage.SHOW_DATA_STATUS == UAV_HCI_SHOW_ENABLE) /*已经允许显示,则切换到默认首页*/
		{							
			/*恢复默认显示页序号*/			
			g_sHciShowPage.curPageIndex  = HCI_SHOW_PAGE_0;
			g_sHciShowPage.lastPageIndex = (HCI_SHOW_PAGE_INDEX)(g_sHciShowPage.curPageIndex + 1);
		}
		else
		{
			/*允许显示提示界面*/
			g_sHciShowPage.SHOW_HINT_STATUS = UAV_HCI_SHOW_ENABLE;	
			
			/*重新显示进入菜单显示提示页面*/
			g_sHciShowPage.EXIT_SHOW_STATUS = HCI_EXIT_SHOW_OP_FIRST;
		}
		
		/*标记显示任务空闲*/
		g_sHciShowPage.SHOW_TASK_STATUS = HCI_SHOW_TASK_IDLE;		
	}	
}

/*=== 遥控配合HCI ===*/
HciShowPage g_sHciShowPage = 
{
	.curPageIndex     = HCI_SHOW_PAGE_0,	         /*从第0页开始显示*/
	.lastPageIndex    = HCI_SHOW_PAGE_0,             /*上次显示页面*/
	.MOULD_STATUS     = HCI_SHOW_MOULD_FIRST,        /*第一次显示*/
	.SHOW_DATA_STATUS = UAV_HCI_SHOW_DISABLE,   	 /*默认不显示*/
	.PAGE_STATUS      = UAV_HCI_SHOW_FREE_PAGE,      /*默认不锁定数据显示页*/
	.SHOW_HINT_STATUS = UAV_HCI_SHOW_ENABLE,         /*默认显示提示页*/
	.EXIT_SHOW_STATUS = HCI_EXIT_SHOW_OP_FIRST,      /*退出操作状态*/
	.SHOW_TASK_STATUS = HCI_SHOW_TASK_IDLE,          /*默认空闲*/
};

vu16 g_vu16ShowEnableContinueTicks     = 0;
vu16 g_vu16ShowSwitchNextContinueTicks = 0;
vu16 g_vu16ShowSwitchLastContinueTicks = 0;
vu16 g_vu16ShowHoldContinueTicks       = 0;

/*=== 遥控配合HCI ===*/
/*HCI(OLED):允许显示/禁止显示/显示上一页/显示下一页*/
void hci_remot_switch_show_status(HciShowPage *hciShowPage)
{
	/*1.开启/关闭OLED显示*/
	/*
	       *  	      *	
		*******       *******
		*  *  *       * *   *
		*  *  *       *  *  * 
		*  *  *       *     *
		*******       *******		                        
	       *
	*/
	/*锁定状态 + 摇杆位置(不再关心油门摇杆位置)*/
	if ((g_sUav_Status.LOCK_STATUS == UAV_LOCK_YES) && \
		(remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MIN)     == REMOT_DATA_MIN) && \
		(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MAX)    == REMOT_DATA_MAX) && \
		(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MID)      == REMOT_DATA_MID))
	{
		g_vu16ShowEnableContinueTicks++;
		
		/*非目标Ticks清0*/
		g_vu16ShowSwitchNextContinueTicks = 0;
		g_vu16ShowSwitchLastContinueTicks = 0;
		g_vu16ShowHoldContinueTicks       = 0;
		
		if (g_vu16ShowEnableContinueTicks > (HCI_PAGE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_HCI_OLED_SHOW_FOC_MS))
		{
			/*完成一次检测,目标Ticks清0*/
			g_vu16ShowEnableContinueTicks = 0;
			
			/*显示使能与不使能 状态切换*/
			if (hciShowPage->SHOW_DATA_STATUS == UAV_HCI_SHOW_DISABLE)
			{
				hciShowPage->SHOW_DATA_STATUS = UAV_HCI_SHOW_ENABLE;
			}
			else if (hciShowPage->SHOW_DATA_STATUS == UAV_HCI_SHOW_ENABLE)
			{
				hciShowPage->SHOW_DATA_STATUS = UAV_HCI_SHOW_DISABLE;
			}
		}
	}

	/*2.允许显示后,才能显示页面*/	
	/*锁定状态 + 摇杆位置(不再关心油门摇杆位置)*/
	/*1.判断切换到下一页动作*/
	/*	   
		   *
		*******       *******
		*  *  *       *     *
		*  *  *       *  ****** 
		*  *  *       *     *
		*******       *******
	       *
	*/		
	if (hciShowPage->SHOW_DATA_STATUS == UAV_HCI_SHOW_ENABLE)
	{
	
		if ((g_sUav_Status.LOCK_STATUS == UAV_LOCK_YES) && \
			(remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MAX)     == REMOT_DATA_MAX) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MID)    == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MID)      == REMOT_DATA_MID))
		{	
			g_vu16ShowSwitchNextContinueTicks++;
			
			/*其它Tick清0*/
			g_vu16ShowEnableContinueTicks     = 0;
			g_vu16ShowSwitchLastContinueTicks = 0;
			g_vu16ShowHoldContinueTicks       = 0;
			
			if (g_vu16ShowSwitchNextContinueTicks >= (HCI_PAGE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_HCI_OLED_SHOW_FOC_MS))
			{
				/*完成一次检测,目标Ticks清0*/
				g_vu16ShowSwitchNextContinueTicks = 0;	
				
				/*判断当前页是否锁定*/
				if (hciShowPage->PAGE_STATUS != UAV_HCI_SHOW_HOLD_PAGE)
				{
					/*显示号越界调整*/
					if (hciShowPage->curPageIndex != (HCI_SHOW_PAGE_INDEX)(HCI_TOTAL_SHOW_PAGE_NUMBER - 1))
					{
						
						hciShowPage->curPageIndex++; /*没有锁定才允许页面序号增加*/						
					}
					else
					{
						hciShowPage->curPageIndex = HCI_SHOW_PAGE_0; /*显示号加到极限从0页开始*/
					}	
				}
				else
				{
					hciShowPage->curPageIndex = hciShowPage->lastPageIndex; /*锁定本页后,本次显示号等于上次显示号*/
				}
			}
		}	

		/*2.判断切换到上一页动作*/
		/*锁定状态 + 摇杆位置(不再关心油门摇杆位置)*/		
		/*	   	
               *		
			*******       *******
			*  *  *       *     *
			*  *  *     ******  *
			*  *  *       *     *
			*******       *******
		       *
		*/		
		if ((g_sUav_Status.LOCK_STATUS == UAV_LOCK_YES) && \
			(remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MIN)     == REMOT_DATA_MIN) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MID)    == REMOT_DATA_MID) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MID)      == REMOT_DATA_MID))
		{	
			g_vu16ShowSwitchLastContinueTicks++;				
				
			/*其它Tick清0*/
			g_vu16ShowEnableContinueTicks     = 0;
			g_vu16ShowSwitchNextContinueTicks = 0;
			g_vu16ShowHoldContinueTicks       = 0;
			
			if (g_vu16ShowSwitchLastContinueTicks >= (HCI_PAGE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_HCI_OLED_SHOW_FOC_MS))
			{
				/*完成一次检测,目标Ticks清0*/
				g_vu16ShowSwitchLastContinueTicks = 0;			
				
				/*判断当前页是否锁定*/
				if (hciShowPage->PAGE_STATUS != UAV_HCI_SHOW_HOLD_PAGE)
				{				
					/*显示号越界调整*/
					if (hciShowPage->curPageIndex != HCI_SHOW_PAGE_0)
					{

						hciShowPage->curPageIndex--; /*没有锁定才允许页面序号减少*/
					}	
					else
					{
						hciShowPage->curPageIndex = (HCI_SHOW_PAGE_INDEX)(HCI_TOTAL_SHOW_PAGE_NUMBER - 1); /*显示号减到极限从最后一页开始*/
					}					
				}
				else
				{
					hciShowPage->curPageIndex = hciShowPage->lastPageIndex; /*锁定本页后,本次显示号等于上次显示号*/
				}
			}
		}		
		
		/*3.当前页面锁定和解锁*/
		/*锁定状态 + 摇杆位置(不再关心油门摇杆位置)*/			
		/*	   *  	 					
			*******       *******
			*  *  *       *     *
			*  *  *       *  *  *
			*  *  *       * *   *
			*******       *******
		       *          *
		*/		
		if ((g_sUav_Status.LOCK_STATUS == UAV_LOCK_YES) && \
			(remot_Data_Range(g_sRemotData.AttRoll, REMOT_DATA_MIN)     == REMOT_DATA_MIN) && \
			(remot_Data_Range(g_sRemotData.AttPitch, REMOT_DATA_MIN)    == REMOT_DATA_MIN) && \
			(remot_Data_Range(g_sRemotData.AttYaw, REMOT_DATA_MID)      == REMOT_DATA_MID))
		{	
			g_vu16ShowHoldContinueTicks++;
			
			/*其它Tick清0*/
			g_vu16ShowEnableContinueTicks     = 0;
			g_vu16ShowSwitchNextContinueTicks = 0;
			g_vu16ShowSwitchLastContinueTicks = 0;
			
			if (g_vu16ShowHoldContinueTicks >= (HCI_PAGE_CHANGE_MIN_HOLD_TIME_MS / RTOS_WAKE_UP_HCI_OLED_SHOW_FOC_MS))
			{
				/*完成一次检测,目标Ticks清0*/
				g_vu16ShowHoldContinueTicks = 0;			

				/*显示使能与不使能 状态切换*/
				if (hciShowPage->PAGE_STATUS == UAV_HCI_SHOW_FREE_PAGE)
				{
					hciShowPage->PAGE_STATUS = UAV_HCI_SHOW_HOLD_PAGE;
				}
				else if (hciShowPage->PAGE_STATUS == UAV_HCI_SHOW_HOLD_PAGE)
				{
					hciShowPage->PAGE_STATUS = UAV_HCI_SHOW_FREE_PAGE;
				}				
			}
		}	
	}
}
