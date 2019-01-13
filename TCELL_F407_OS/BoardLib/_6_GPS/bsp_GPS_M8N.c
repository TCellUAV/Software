#include "bsp_GPS_M8N.h"

GpsM8N g_sGpsM8N = 
{
	.UPDATE_STATUS = GPS_DATA_UPDATE_FAIL,
};

/*GPS M8N 发送协议帧*/
GpsM8nCmdTxFrame g_sGpsM8nCmdTxFrame;

/*GPS M8N PVT 接收协议帧*/
GpsM8nRxPvtFrame g_sGpsM8nRxPvtFrame;

/*串口发送buff*/
u8 g_GpsM8nTxBuff[GPS_M8N_TX_BUFF_LENTH] = {0};

/*串口接收buff*/
u8 g_GpsM8nRxBuff[GPS_M8N_RX_BUFF_LENTH] = {0};

/*PVT数据buff*/
u8 g_GpsM8nPvtBuff[GPS_M8N_RX_PVT_ALL_LENTH] = {0};

/*数组重组*/
GpsDataReform g_sGpsDataReform = 
{
	.backByteNbr  = 0,
	.frontByteNbr = 0,
};

GpsDataReform *g_psGpsDataReform = &g_sGpsDataReform;

/*GPS初始化指令*/
/*1.CLASS_ID: CFG, MESSAGE_ID: MSG, Disable NEMA*/
const u8 NEMA_Disable_Buff[20][8] = 
{
	0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/*F0-00 NMEA-GxGGA*/
	0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-01 NMEA-GxGLL*/
	0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-02 NMEA-GxGSA*/
	0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-03 NMEA-GxGSV*/
	0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-04 NMEA-GxRMC*/
	0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-05 NMEA-GxVTG*/
	0xF0, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-06 NMEA-GxGRS*/
	0xF0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-07 NMEA-GxGST*/
	0xF0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-08 NMEA-GxZDA*/
	0xF0, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-09 NMEA-GxGBS*/
	0xF0, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-0A NMEA-GxDTM*/
	0xF0, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-0D NMEA-GxGNS*/
	0xF0, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-0E NMEA-GxTHS*/
	0xF0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F0-0F NMEA-GxVLW*/
	
	0xF1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F1-00 NMEA-PUBX,00*/	
	0xF1, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*F1-01 NMEA-PUBX,01*/
	0xF1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/*F1-03 NMEA-PUBX,03*/
	0xF1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/*F1-04 NMEA-PUBX,04*/	
	0xF2, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/*F2-0B NMEA-PSARS*/
	0xF2, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/*F2-0C NMEA-PSARL*/	
};

/*2.CLASS_ID: CFG, MESSAGE_ID: MSG, Disable UBLOX*/
const u8 UBLOX_Disable_Buff[59][8] = 
{
	0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-01 NAV-POSECEF*/
	0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-02 NAV-POSLLH*/
	0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-03 NAV-STATUS*/
	0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-04 NAV-DOP*/
	0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-06 NAV-SOL*/
	0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-07 NAV-PVT*/
	0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-08 NAV-POSUTM*/
	0x01, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-09 NAV-ODO*/
	0x01, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-11 NAV-VELECEF*/
	0x01, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-12 NAV-VELNED*/
	0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-20 NAV-TIMEGPS*/
	0x01, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-21 NAV-TIMEUTC*/
	0x01, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-22 NAV-CLOCK*/	
	0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-30 NAV-SVINFO*/
	0x01, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-31 NAV-DGPS*/
	0x01, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-32 NAV-SBAS*/
	0x01, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-34 NAV-ORB*/
	0x01, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-35 NAV-SAT*/	
	0x01, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-37 NAV-HNR*/
	0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*01-40 NAV-EKFSTATUS*/
	
	0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*02-10 RXM-RAW*/
	0x02, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*02-11 RXM-SFRB*/	
	0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*02-15 RXM-RAWX*/
	0x02, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*02-20 RXM-SVSI*/
	0x02, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*02-23 RXM-RTC5*/
	0x02, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*02-30 RXM-ALM*/
	0x02, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*02-31 RXM-EPH*/
	
	0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-01 MON-SCHD*/
	0x0A, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-02 MON-IO*/	
	0x0A, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-03 MON-IPC*/
	0x0A, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-05 MON-EXCEPT*/
	0x0A, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-06 MON-MSGPP*/
	0x0A, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-07 MON-RXBUF*/
	0x0A, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-08 MON-TXBUF*/
	0x0A, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-09 MON-HW*/
	0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-0A MON-USB*/
	0x0A, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-20 MON-PT*/
	0x0A, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-21 MON-RXR*/
	0x0A, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0A-2E MON-SMGR*/

	0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0B-00 AID-REQ*/
	0x0B, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0B-05 AID-MAPMATCH*/
	0x0B, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0B-30 AID-ALM*/
	0x0B, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0B-31 AID-EPH*/	
	0x0B, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0B-32 AID-ALPSRV*/
	0x0B, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0B-33 AID-AOP*/

	0x0D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0D-01 TIM-TP*/
	0x0D, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0D-02 TIM-TM*/
	0x0D, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0D-03 TIM-TM2*/
	0x0D, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0D-04 TIM-VRFY*/
	0x0D, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0D-06 TIM-SVIN*/
	0x0D, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0D-11 TIM-DOSC*/
	0x0D, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0D-12 TIM-TOS*/
	0x0D, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0D-13 TIM-SMEAS*/
	0x0D, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*0D-16 TIM-FCHG*/	
	
	0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*10-01 ESF-WT*/	
	0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*10-02 ESF-MEAS*/
	0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*10-10 ESF-STATUS*/	
	
	0x21, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*21-02 LOG-STA*/

	0x24, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*24-01 ODO-DST*/
};

/*3.CLASS_ID: CFG, MESSAGE_ID: MSG, Enable PVT*/
const u8 PVT_Enable_Buff[8] = 
{
	0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
};

/*4.CLASS_ID: CFG, MESSAGE_ID: RATE, Set Output Rate*/
const u8 Output_Rate_Buff[6] = 
{
	0x64, 0x00, 0x01, 0x00, 0x00, 0x00,
}; 

/*5.CLASS_ID: CFG, MESSAGE_ID: PRT, Set Uart BaudRate*/
const u8 Uart_BaudRate_Buff[20] = 
{
	/*38400*/
//	0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96,
//	0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
	
	/*115200*/
	0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2,
	0x01, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
}; 

/*6.CLASS_ID: CFG, MESSAGE_ID: CFG, Save Config*/
const u8 Save_Config_Buff[13] = 
{
	0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x17,
};

/*GPS初始化:PVT, 10Hz, 38400*/
#if defined(HW_CUT__USE_GPS)

SYS_RETSTATUS bsp_GPS_M8N_Init(GpsM8N *gpsM8N)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	u8 i, j;
	
	/*1.Uart Init*/
	gpsM8N->UartMaster = &g_sGpsUart;
	
	/*2.禁掉所有NEMA输出语句*/
	for (i = 0; i < 20; i++)
	{
		for (j = 0; j < 3; j++)
		{
			/*创建发送协议帧*/
			bsp_GPS_M8N_Make_Command(&g_sGpsM8nCmdTxFrame, GPS_M8N_TX_CLASS_CFG, GPS_M8N_TX_CFG_MSG, (u8*)&NEMA_Disable_Buff[i], 8, g_GpsM8nTxBuff);
		
			/*发送给GPS模块*/
			msp_Uart_Send_Data(gpsM8N->UartMaster, (g_GpsM8nTxBuff + 2), g_GpsM8nTxBuff[0] + ((u16)g_GpsM8nTxBuff[1] << 8), MSP_UART_POLL);
			
			sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MIN_MS);			
		}
	}
	
	sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MAX_MS);	
	
	/*3.禁掉所有UBLOX输出语句*/
	for (i = 0; i < 59; i++)
	{
		for (j = 0; j < 3; j++)
		{		
			/*创建发送协议帧*/
			bsp_GPS_M8N_Make_Command(&g_sGpsM8nCmdTxFrame, GPS_M8N_TX_CLASS_CFG, GPS_M8N_TX_CFG_MSG, (u8*)&UBLOX_Disable_Buff[i], 8, g_GpsM8nTxBuff);
		
			/*发送给GPS模块*/
			msp_Uart_Send_Data(gpsM8N->UartMaster, (g_GpsM8nTxBuff + 2), g_GpsM8nTxBuff[0] + ((u16)g_GpsM8nTxBuff[1] << 8), MSP_UART_POLL);
			
			sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MIN_MS);			
		}			
	}	
	
	sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MAX_MS);	
	
	/*4.打开PVT语句*/
	/*创建发送协议帧*/
	bsp_GPS_M8N_Make_Command(&g_sGpsM8nCmdTxFrame, GPS_M8N_TX_CLASS_CFG, GPS_M8N_TX_CFG_MSG, (u8*)&PVT_Enable_Buff, 8, g_GpsM8nTxBuff);	
	

	for (j = 0; j < 3; j++)
	{	
		/*发送给GPS模块*/
		msp_Uart_Send_Data(gpsM8N->UartMaster, (g_GpsM8nTxBuff + 2), g_GpsM8nTxBuff[0] + ((u16)g_GpsM8nTxBuff[1] << 8), MSP_UART_POLL);	
		
		sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MIN_MS);		
	}
	
	sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MAX_MS);
	
	/*5.设置10hz刷新*/
	/*创建发送协议帧*/
	bsp_GPS_M8N_Make_Command(&g_sGpsM8nCmdTxFrame, GPS_M8N_TX_CLASS_CFG, GPS_M8N_TX_CFG_RATE, (u8*)&Output_Rate_Buff, 6, g_GpsM8nTxBuff);	
	
	for (j = 0; j < 3; j++)
	{
		/*发送给GPS模块*/
		msp_Uart_Send_Data(gpsM8N->UartMaster, (g_GpsM8nTxBuff + 2), g_GpsM8nTxBuff[0] + ((u16)g_GpsM8nTxBuff[1] << 8), MSP_UART_POLL);		
		
		sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MIN_MS);			
	}
	
	sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MAX_MS);
	
	/*6.设置波特率*/
	/*创建发送协议帧*/
	bsp_GPS_M8N_Make_Command(&g_sGpsM8nCmdTxFrame, GPS_M8N_TX_CLASS_CFG, GPS_M8N_TX_CFG_PRT, (u8*)&Uart_BaudRate_Buff, 20, g_GpsM8nTxBuff);	
	
	for (j = 0; j < 3; j++)
	{	
		/*发送给GPS模块*/
		msp_Uart_Send_Data(gpsM8N->UartMaster, (g_GpsM8nTxBuff + 2), g_GpsM8nTxBuff[0] + ((u16)g_GpsM8nTxBuff[1] << 8), MSP_UART_POLL);	
		
		sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MIN_MS);		
	}		
	
	sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MAX_MS);	
	
	/*7.保存所有参数到模块内部掉电非易失存储单元*/
	/*创建发送协议帧*/
	bsp_GPS_M8N_Make_Command(&g_sGpsM8nCmdTxFrame, GPS_M8N_TX_CLASS_CFG, GPS_M8N_TX_CFG_CFG, (u8*)&Save_Config_Buff, 13, g_GpsM8nTxBuff);	
	
	for (j = 0; j < 3; j++)
	{	
		/*发送给GPS模块*/
		msp_Uart_Send_Data(gpsM8N->UartMaster, (g_GpsM8nTxBuff + 2), g_GpsM8nTxBuff[0] + ((u16)g_GpsM8nTxBuff[1] << 8), MSP_UART_POLL);		
		
		sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MIN_MS);		
	}
	
	sys_DelayMs(GPS_M8N_INIT_CMC_PERIOD_TICKS_MAX_MS);	
	
	return statusRet;
}
#endif

/*解析更新GPS数据*/
SYS_RETSTATUS bsp_GPS_M8N_PVT_Parse(GpsM8N *gpsM8N, GpsM8nRxPvtFrame *pvtFrame, u8 *rxBuff)
{
	u8 i, checkA = 0, checkB = 0;
	
	/*0.对数据进行重新截取拼装成一帧数据*/
	if (bsp_GPS_M8N_Reform_PVT_Data(&g_sGpsDataReform, rxBuff) != SYS_RET_SUCC)
	{
		/*标记GPS数据更新失败*/
		gpsM8N->UPDATE_STATUS = GPS_DATA_UPDATE_FAIL;
		
		return SYS_RET_FAIL;	/*未找到1帧数据*/
	}
	
	/*先拼凑截取点往后的数据*/
	memcpy(g_GpsM8nPvtBuff, g_psGpsDataReform->pHead1, g_psGpsDataReform->backByteNbr);

	/*再拼凑截取点往前的数据*/	
	memcpy((g_GpsM8nPvtBuff + g_psGpsDataReform->backByteNbr), g_psGpsDataReform->pStitch, g_psGpsDataReform->frontByteNbr);
	
	/*将重组后的数据赋值给PVT协议帧*/
	memcpy((u8 *)&g_sGpsM8nRxPvtFrame, g_GpsM8nPvtBuff, GPS_M8N_RX_PVT_ALL_LENTH);
	
	/*PVT buff清0*/
	memset(g_GpsM8nPvtBuff, 0, GPS_M8N_RX_PVT_ALL_LENTH);
	
	/*1.对本帧数据有效性进行判断*/
	/*对帧头有效性进行检测*/
	if ((pvtFrame->SY_NC_CHAR1 != 0xB5) || \
		(pvtFrame->SY_NC_CHAR2 != 0x62))
	{
		/*标记GPS数据更新失败*/
		gpsM8N->UPDATE_STATUS = GPS_DATA_UPDATE_FAIL;			
		
		return SYS_RET_FAIL;
	}

	/*对ID有效性进行检测*/
	if ((pvtFrame->CLASS_ID   != GPS_M8N_TX_CLASS_NAV) || \
		(pvtFrame->MESSAGE_ID != GPS_M8N_TX_NAV_PVT))
	{
		/*标记GPS数据更新失败*/
		gpsM8N->UPDATE_STATUS = GPS_DATA_UPDATE_FAIL;			
		
		return SYS_RET_FAIL;
	}	
	
	/*对有效位长度进行检测*/
	if (pvtFrame->LENTH_LOW + ((u16)pvtFrame->LENTH_HIGH << 8) \
		!= GPS_M8N_RX_PVT_PAYLOAD_LENTH)
	{
		/*标记GPS数据更新失败*/
		gpsM8N->UPDATE_STATUS = GPS_DATA_UPDATE_FAIL;			
		
		return SYS_RET_FAIL;
	}
	
	/*对数据校验有效性进行检测*/
	/*校验位计算(id + lenth + payload)*/
	for (i = 0; i < (2 + 2 + GPS_M8N_RX_PVT_PAYLOAD_LENTH); i++)
	{
		checkA += *(((u8 *)&g_sGpsM8nRxPvtFrame) + 2 + i);
		checkB += checkA;
	}
	
	if ((pvtFrame->checkA != checkA) && \
		(pvtFrame->checkB != checkB))
	{
		/*标记GPS数据更新失败*/
		gpsM8N->UPDATE_STATUS = GPS_DATA_UPDATE_FAIL;		
		
		return SYS_RET_FAIL;		
	}
	
	/*2.数据为有效数据,进行拼装*/
	/*UTC时间提取*/
	gpsM8N->PvtData.UtcTime.year   = (pvtFrame->PAYLOAD[4] + (pvtFrame->PAYLOAD[5] << 8));	         /*year*/
	gpsM8N->PvtData.UtcTime.month  = (pvtFrame->PAYLOAD[6]); 									     /*month*/
	gpsM8N->PvtData.UtcTime.day    = (pvtFrame->PAYLOAD[7]); 									     /*day*/
	gpsM8N->PvtData.UtcTime.hour   = (pvtFrame->PAYLOAD[8]); 									     /*hour*/
	gpsM8N->PvtData.UtcTime.minute = (pvtFrame->PAYLOAD[9]); 									     /*minute*/	
	gpsM8N->PvtData.UtcTime.second = (pvtFrame->PAYLOAD[10]); 									     /*second*/	
	
	/*定位类型*/
	gpsM8N->PvtData.POS_FIX_TYPE = (GPS_M8N_POS_FIX_TYPE)pvtFrame->PAYLOAD[20];
	
	/*Fix Status Flags*/
	gpsM8N->PvtData.FixStatusFlag.gnssFixOK    = (pvtFrame->PAYLOAD[21]) & 0x01;      /*是否有效定位*/
	gpsM8N->PvtData.FixStatusFlag.diffSoln     = (pvtFrame->PAYLOAD[21] >> 1) & 0x01; /*差分模式是否开启(DGPS)*/	
	gpsM8N->PvtData.FixStatusFlag.psmState     = (pvtFrame->PAYLOAD[21] >> 2) & 0x07; /*省电模式(未用)*/	
	gpsM8N->PvtData.FixStatusFlag.headVehValid = (pvtFrame->PAYLOAD[21] >> 5) & 0x01; /*航向是否有效*/	
	
	/*定位卫星个数*/
	gpsM8N->PvtData.satelliteNbr = pvtFrame->PAYLOAD[23];
	
	/*经度*10^7*/
	gpsM8N->PvtData.Coordinate.lon = (pvtFrame->PAYLOAD[24])       + \
							         (pvtFrame->PAYLOAD[25] << 8)  + \
							         (pvtFrame->PAYLOAD[26] << 16) + \
							         (pvtFrame->PAYLOAD[27] << 24);							

	/*纬度*10^7*/
	gpsM8N->PvtData.Coordinate.lat = (pvtFrame->PAYLOAD[28])       + \
							         (pvtFrame->PAYLOAD[29] << 8)  + \
							         (pvtFrame->PAYLOAD[30] << 16) + \
							         (pvtFrame->PAYLOAD[31] << 24);								
	
	/*获取海拔高度*/
	gpsM8N->PvtData.hMSL = (pvtFrame->PAYLOAD[36])       + \
						   (pvtFrame->PAYLOAD[37] << 8)  + \
						   (pvtFrame->PAYLOAD[38] << 16) + \
						   (pvtFrame->PAYLOAD[39] << 24); /*mm*/
	
	/*水平位置估计精度*/
	gpsM8N->PvtData.HV_Accuracy.hAcc = (pvtFrame->PAYLOAD[40])       + \
									   (pvtFrame->PAYLOAD[41] << 8)  + \
									   (pvtFrame->PAYLOAD[42] << 16) + \
									   (pvtFrame->PAYLOAD[43] << 24); /*mm*/												
	
	/*垂直位置估计精度*/
	gpsM8N->PvtData.HV_Accuracy.vAcc = (pvtFrame->PAYLOAD[44])       + \
									   (pvtFrame->PAYLOAD[45] << 8)  + \
									   (pvtFrame->PAYLOAD[46] << 16) + \
									   (pvtFrame->PAYLOAD[47] << 24); /*mm*/  												
							 
	/*GPS获取的沿导航系正北速度(Y Axis)*/
	gpsM8N->PvtData.NED_Velocity.velN = (pvtFrame->PAYLOAD[48])       + \
										(pvtFrame->PAYLOAD[49] << 8)  + \
										(pvtFrame->PAYLOAD[50] << 16) + \
									    (pvtFrame->PAYLOAD[51] << 24); /*velN: mm/s*/

	/*GPS获取的沿导航系正东向速度(/X Axis)*/
	gpsM8N->PvtData.NED_Velocity.velE = (pvtFrame->PAYLOAD[52])       + \
										(pvtFrame->PAYLOAD[53] << 8)  + \
										(pvtFrame->PAYLOAD[54] << 16) + \
										(pvtFrame->PAYLOAD[55] << 24); /*velE: mm/s*/										 								 

	/*GPS获取的沿导航系'天'向速度*/
	gpsM8N->PvtData.NED_Velocity.velD = (pvtFrame->PAYLOAD[56])       + \
										(pvtFrame->PAYLOAD[57] << 8)  + \
										(pvtFrame->PAYLOAD[58] << 16) + \
										(pvtFrame->PAYLOAD[59] << 24); /*velD: mm/s*/
	
	/*载体对地速度*/
	gpsM8N->PvtData.gSpeed = (pvtFrame->PAYLOAD[60])       + \
							 (pvtFrame->PAYLOAD[61] << 8)  + \
							 (pvtFrame->PAYLOAD[62] << 16) + \
							 (pvtFrame->PAYLOAD[63] << 24); /*gSpeed: mm/s*/
	
	/*载体运动航向角*/
	gpsM8N->PvtData.headMot =(pvtFrame->PAYLOAD[64])       + \
							 (pvtFrame->PAYLOAD[65] << 8)  + \
						     (pvtFrame->PAYLOAD[66] << 16) + \
							 (pvtFrame->PAYLOAD[67] << 24); /*deg 1e-5*/
	
	/*速度估计精度*/
	gpsM8N->PvtData.sAcc = (pvtFrame->PAYLOAD[68])       + \
						   (pvtFrame->PAYLOAD[69] << 8)  + \
						   (pvtFrame->PAYLOAD[70] << 16) + \
						   (pvtFrame->PAYLOAD[71] << 24); /*mm/s*/
	
	/*位置估计精度*/
	gpsM8N->PvtData.pDOP = (pvtFrame->PAYLOAD[76]) + \
						   (pvtFrame->PAYLOAD[77] << 8);
	
	/*标记GPS数据更新成功*/
	gpsM8N->UPDATE_STATUS = GPS_DATA_UPDATE_SUCC;
	
	return SYS_RET_SUCC;
}

/*构建指令*/
void bsp_GPS_M8N_Make_Command(GpsM8nCmdTxFrame *txFrame, GPS_M8N_TX_CLASS_ID classId, GPS_M8N_TX_MESSAGE_ID messageId, u8 *payload, u8 cmdLen, u8 *sendBuff)
{
	u8 i, cnt = 0;
	u16 payloadLen;
	
	/*帧头*/
	txFrame->SY_NC_CHAR1 = 0xB5;
	txFrame->SY_NC_CHAR2 = 0x62;
	
	/*指令类ID*/
	txFrame->CLASS_ID   = classId;
	
	/*服务类ID*/
	txFrame->MESSAGE_ID = messageId;
	
	/*有效数据位长度*/
	txFrame->LENTH_LOW  = cmdLen & 0xff;
	txFrame->LENTH_HIGH = (cmdLen & 0xff00) >> 8;
	
	/*有效指令*/
	for (i = 0; i < cmdLen; i++)
	{
		txFrame->PAYLOAD[i] = payload[i];
	}
	
	/*校验位计算(id + lenth + payload)*/
	for (i = 0; i < (2 + 2 + cmdLen); i++)
	{
		txFrame->checkA += *(((u8 *)&g_sGpsM8nCmdTxFrame) + 2 + i);
		txFrame->checkB += txFrame->checkA;
	}	
	
	/*发送buff清0*/
	memset(g_GpsM8nTxBuff, 0, sizeof(g_GpsM8nTxBuff));
	
	/*将帧赋值给发送Buff*/
	cnt += 2;	/*预留两个buff装数组长度*/
	sendBuff[cnt++] = txFrame->SY_NC_CHAR1;
	sendBuff[cnt++] = txFrame->SY_NC_CHAR2;	
	sendBuff[cnt++] = txFrame->CLASS_ID;		
	sendBuff[cnt++] = txFrame->MESSAGE_ID;
	sendBuff[cnt++] = txFrame->LENTH_LOW;		
	sendBuff[cnt++] = txFrame->LENTH_HIGH;	
	
	/*计算有效数据长度*/
	payloadLen = ((u16)txFrame->LENTH_HIGH << 8) + txFrame->LENTH_LOW;
	
	/*有效指令赋值*/
	for (i = 0; i < payloadLen; i++)
	{
		sendBuff[cnt++] = txFrame->PAYLOAD[i];
	}
	
	/*校验位*/
	sendBuff[cnt++] = txFrame->checkA;
	sendBuff[cnt]   = txFrame->checkB;	
	
	/*帧校验位清0*/
	txFrame->checkA = 0;
	txFrame->checkB = 0;
	
	/*数组第一和第二个空间,存储发送buff长度*/
	sendBuff[0] = (cnt - 1) & 0xff;
	sendBuff[1] = ((cnt - 1) & 0xff00) >> 8;	
}

/*数据偏移重组*/
SYS_RETSTATUS bsp_GPS_M8N_Reform_PVT_Data(GpsDataReform *DataReform, u8 *rxBuff)
{
	/*接收数组内存中的起始地址*/
	DataReform->pStart = g_GpsM8nRxBuff;
	
	/*拼接点即起始点*/
	DataReform->pStitch = DataReform->pStart;
	
	/*head1和head2默认起始点*/
	DataReform->pHead1 = DataReform->pStart;
	DataReform->pHead2 = DataReform->pHead1 + 1;
	
	/*计量归零*/
	DataReform->frontByteNbr = 0;
	DataReform->backByteNbr  = 0;
	
	/*扫描直到截取有效位(两个帧头)*/
	while((*(DataReform->pHead1) != 0xB5) || (*(DataReform->pHead2) != 0x62))
	{	
		/*截取点前面的byte++*/
		DataReform->frontByteNbr++;
		
		/*到越界时还未找到,表明该帧中没有找到,就停止*/
		if (DataReform->frontByteNbr > GPS_M8N_RX_BUFF_LENTH - 2)
		{
			DataReform->pHead1       = NULL;
			DataReform->pHead2       = NULL;
			DataReform->frontByteNbr = 0;	
		    DataReform->backByteNbr  = 0;
			
			return SYS_RET_FAIL;
		}		
		
		/*head1指针偏移++*/
		DataReform->pHead1++;

		/*head2指针偏移++*/
		DataReform->pHead2++;
	}
	
	/*计算截取点后面还有多少字节数据*/
	DataReform->backByteNbr = GPS_M8N_RX_BUFF_LENTH - DataReform->frontByteNbr;
	
	return SYS_RET_SUCC;
}
