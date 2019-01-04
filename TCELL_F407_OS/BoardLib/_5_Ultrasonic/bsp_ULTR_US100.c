#include "bsp_ULTR_US100.h"

BSP_US100 g_sUs100 = 
{
	.UPDATE_STATUS = US100_UPDATE_FAIL,	/*默认没有数据*/
};

/*串口发送Buff*/
u8 	g_Us100TxBuff[ULTR_US100_TX_BUFF_LENTH] = {0};

/*串口接收Buff*/
u8 	g_Us100RxBuff[ULTR_US100_RX_BUFF_LENTH] = {0};	

/*超声波模块初始化*/
#if defined(HW_CUT__USE_ULTR)
SYS_RETSTATUS bsp_US100_Init(BSP_US100 *us100)
{
	SYS_RETSTATUS statusRet = SYS_RET_SUCC;
	u8 i = 0;
	
	/*1.Uart Init*/	
	us100->UartMaster = &g_sUltrUart;
	
	/*距离测量测试*/
	bsp_US100_Start_Meas_Distance(us100);
	
	/*读取距离测试:内含启动串口接受,并判断数据是否有效*/
	while(bsp_US100_Get_Distance(us100) == SYS_NO_AVA_MARK)
	{
		sys_DelayMs(100);
		
		/*再次重发测量指令*/
		bsp_US100_Start_Meas_Distance(us100);
		
		/*重复次数++*/
		i++;
		
		if (i >= 10)
		{
			/*返回失败*/
			statusRet = SYS_RET_FAIL;
			
			break;
		}
	}
	
	return statusRet;
}
#endif

/*从超声波模块接收数据并解析*/
SYS_RETSTATUS bsp_US100_Data_Parse(BSP_US100 *us100)
{
	SYS_RETSTATUS retStaus;
	fp32 distance = 0;
	fp32 temperature = 0;
	
	/*通过指令判断接收到的内容是距离*/
	if (us100->UartMaster->pTxBuff[0] == US100_GET_MODULE_DISTANCE_CMD)
	{
		/*有效的距离不应为0*/
		if ((us100->UartMaster->pRxBuff[0] != 0) || \
			(us100->UartMaster->pRxBuff[1] != 0))
		{
			/*拼凑距离数据(mm)*/
			distance = ((u16)us100->UartMaster->pRxBuff[0] << 8) + us100->UartMaster->pRxBuff[1];
			
			/*转化成cm*/
			distance /= 10.0f;
			
			/*赋值距离值*/
			us100->distance = (s16)distance;
			
			/*标记数据更新成功*/
			us100->UPDATE_STATUS = US100_UPDATE_SUCC;
			
			/*标记返回成功*/
			retStaus = SYS_RET_SUCC;
		}
		else	/*数据无效,返回失败*/
		{
			/*标记数据更新失败*/
			us100->UPDATE_STATUS = US100_UPDATE_FAIL;
			
			/*返回失败*/
			retStaus = SYS_RET_FAIL;			
		}
	}
	/*通过指令判断接收到的内容是温度*/	
	else if (us100->UartMaster->pTxBuff[0] == US100_GET_MODULE_TEMPERATURE_CMD)
	{
		if (us100->UartMaster->pRxBuff[0] != 0)
		{
			/*拼凑温度数据(mm)*/			
			temperature = us100->UartMaster->pRxBuff[0] - US100_MODULE_TEMPRATURE_BASE;
			
			/*赋值温度值*/
			us100->temperature = (s16)temperature;	

			/*标记数据更新成功*/
			us100->UPDATE_STATUS = US100_UPDATE_SUCC;			
		
			/*标记返回成功*/			
			retStaus = SYS_RET_SUCC;
		}
		else
		{
			/*标记数据更新失败*/
			us100->UPDATE_STATUS = US100_UPDATE_FAIL;
			
			/*返回失败*/
			retStaus = SYS_RET_FAIL;			
		}
	}	
	
	return retStaus;
}

/*命令超声波开始测量距离*/
void bsp_US100_Start_Meas_Distance(BSP_US100 *us100)
{
	/*获取距离命令*/
	g_Us100TxBuff[0] = US100_GET_MODULE_DISTANCE_CMD;
	
	/*发送命令*/
	msp_Uart_Send_Data(us100->UartMaster, g_Us100TxBuff, 1, MSP_UART_POLL);	
}

/*获取超声波模块距离(cm)*/
s16 bsp_US100_Get_Distance(BSP_US100 *us100)
{
	/*判断数据是否更新*/
	if (us100->UPDATE_STATUS != US100_UPDATE_SUCC)
	{
		return SYS_NO_AVA_MARK; /*无效标记*/
	}
	
	/*本次数据已用,标记无效*/
	us100->UPDATE_STATUS = US100_UPDATE_FAIL;
	
	/*开始新一次距离测量*/
	bsp_US100_Start_Meas_Distance(us100);
	
	return (us100->distance);
}
