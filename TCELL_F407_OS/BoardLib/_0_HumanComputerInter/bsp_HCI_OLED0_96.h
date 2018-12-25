#ifndef _BSP_HCI_OLED0_96_H_
#define _BSP_HCI_OLED0_96_H_

#include "sys_Platform.h"
#include "math_Function.h"
#include "bsp_HCI_OLED0_96_CMD.h"
#include "msp_GPIO.h"
#include "ssp_SimSPI.h"
#include "sys_McuInit.h"

/*汉字字体*/
typedef enum
{
	OLED096_HANZI_16X16 = 16,
	OLED096_HANZI_12X12 = 12,
}OLED096_HANZI_FONT;

/*acsii码字体*/
typedef enum
{
	OLED096_ACSII_8X16 = 16,
	OLED096_ACSII_6X12 = 12,
	OLED096_ACSII_6X8  = 8,
}OLED096_ACSII_FONT;

typedef struct
{
	SSP_SimSPI *SimSpiMaster;
}BSP_OLED0_96;

/******************OLED基础功能函数******************/
/*OLED初始化*/
SYS_RETSTATUS bsp_OLED0_96_Init(BSP_OLED0_96 *oled0_96);	

/*写命令*/	
void bsp_OLED0_96_Write_Command(BSP_OLED0_96 *oled0_96, u8 cmd);

/*写数据*/	
void bsp_OLED0_96_Write_Data(BSP_OLED0_96 *oled0_96, u8 dat);

/*OLED开启显示*/   
void bsp_OLED0_96_Display_On(BSP_OLED0_96 *oled0_96);

/*OLED关闭显示*/
void bsp_OLED0_96_Display_Off(BSP_OLED0_96 *oled0_96);	

/*局部操作*/
void bsp_OLED0_96_Display_Part(BSP_OLED0_96 *oled0_96, u8 x0, u8 y0, u8 x1, u8 y1, u8 dat);

/*清屏*/   							   		    
void bsp_OLED0_96_Clear(BSP_OLED0_96 *oled0_96);

/*显示字符*/
void bsp_OLED0_96_ShowChar(BSP_OLED0_96 *oled0_96, u8 x, u8 y, u8 chr, OLED096_ACSII_FONT ACSII_FONT);

/*显示数字*/
void bsp_OLED0_96_ShowNum(BSP_OLED0_96 *oled0_96, u8 x, u8 y, uint32_t num, u8 len, OLED096_ACSII_FONT ACSII_FONT);

/*显示字符串*/
void bsp_OLED0_96_ShowString(BSP_OLED0_96 *oled0_96, u8 x, u8 y, u8 *str, OLED096_ACSII_FONT ACSII_FONT);	 

/*设置坐标*/
void bsp_OLED0_96_Set_Pos(BSP_OLED0_96 *oled0_96, u8 x, u8 y);

/*显示中文*/
void bsp_OLED0_96_ShowCHinese(BSP_OLED0_96 *oled0_96, u8 x, u8 y, u8 no, OLED096_HANZI_FONT HANZI_FONT);

/*画图*/
void bsp_OLED0_96_DrawBMP(BSP_OLED0_96 *oled0_96, u8 x0, u8 y0, u8 x1, u8 y1, u8 BMP[]);

/*=== 拓展显示函数 ===*/
/*显示整数*/
void bsp_OLED0_96_Show_Integer(BSP_OLED0_96 *oled0_96, u8 x, u8 y, OLED096_ACSII_FONT ACSII_FONT, MATH_Integer integer);

/*显示浮点数*/
void bsp_OLED0_96_Show_Floater(BSP_OLED0_96 *oled0_96, u8 x, u8 y, OLED096_ACSII_FONT ACSII_FONT, MATH_Floater floater);

/*显示需要补零数(9->09)*/
void bsp_OLED0_96_Show_Calendar(BSP_OLED0_96 *oled0_96, u8 x, u8 y, OLED096_ACSII_FONT ACSII_FONT, MATH_Integer integer);

/*显示正数(没有符号位)*/
void bsp_OLED0_96_Show_Integer_No_Sign(BSP_OLED0_96 *oled0_96, u8 x, u8 y, OLED096_ACSII_FONT ACSII_FONT, MATH_Integer integer);

extern BSP_OLED0_96 g_sOled0_96;

#endif
