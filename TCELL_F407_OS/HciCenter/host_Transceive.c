#include "host_Transceive.h"
#include "sys_Debug.h"
#include "attitude_Aircraft.h"
#include "sins_Strapdown.h"
#include "pid_System.h"

/*数据上传上位机系统*/
User_Send_Host_System g_sUserSendHostSystem = 
{
	.HOST_TARG     = USER_HOST_CHOOSE_ANO, 			/*默认匿名上位机*/
	.MSG_ID        = USER_ANO_MSG_EXCHANGE,			/*默认作为上位机使用*/
	.SWITCH_STATUS = USER_HOST_SWITCH_NO,           /*切换状态,默认未切换完成*/
};

/*传感器数据原始值和滤波值*/
SendSensorData g_sSendSensorData 		       = {0}; 
/*竖直+水平惯导(三阶互补)值*/                  
SendSINSData g_sSendSinsDataHeight             = {0};	/*Z轴竖直方向的惯导*/
SendSINSData g_sSendSinsDataHorizontalX        = {0};	/*X轴(正东)水平方向的惯导*/
SendSINSData g_sSendSinsDataHorizontalY        = {0};	/*Y轴(正北)水平方向的惯导*/
/*PID调试波形数据*/                            
SendPIDPara  g_sSendPidPara                    = {0};
/*光流控制数据*/
SendOpticFlowCtrlData g_sSendOpticFlowCtrlData = {0};

#define ano_user_data_to_send  g_DebugTxBuff	 /*匿名上位机发送数据缓存*/
#define ano_user_rx_buff       g_DebugRxBuff  	 /*匿名上位机接收数据缓存*/

#define vcan_user_data_to_send g_DebugTxBuff 	 /*山外上位机发送数据缓存*/
#define vcan_user_rx_buff      g_DebugRxBuff 	 /*山外上位机接收数据缓存*/

#define user_data_recv_buff	   g_DebugRxBuff     /*上位机通用接收buff*/

/*====== 飞控设置指令接收及解析(上位机通用) ======*/
/*接收初步解析帕判断*/
void user_Host_Data_Receive_Prepare(u8 data)
{
	static vu8 _data_len = 0,_data_cnt = 0;
	static vu8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		user_data_recv_buff[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		user_data_recv_buff[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		user_data_recv_buff[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		user_data_recv_buff[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		user_data_recv_buff[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		user_data_recv_buff[4+_data_cnt]=data;
		user_Host_Data_Receive_Anl(user_data_recv_buff,_data_cnt+5);
	}
	else
		state = 0;
}

/*飞控设置指令解析*/
void user_Host_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	/*1.传感器校准*/
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
//			g_psAccCalibSystem->ENTRY_STATUS = ENTRY_CALIB_SUCC; /*加速度校准*/
		}
		if(*(data_buf+4)==0X02)
		{
			/*陀螺仪校准*/
		}
		if(*(data_buf+4)==0X04)
		{
//			g_psMagCalibSystem->ENTRY_STATUS = ENTRY_CALIB_SUCC; /*磁力计校准*/
		}
		if(*(data_buf+4)==0X05)
		{
			/*气压计校准*/
		}
		
		/*加速度计六面校准退出*/
		if(*(data_buf+4)==0X20)
		{
			/*加速度计六面校准*/
		}
		/*6面校准第1步*/
		if(*(data_buf+4)==0X21)
		{
			/*加速度计六面校准*/
		}
		/*6面校准第2步*/
		if(*(data_buf+4)==0X22)
		{
			/*加速度计六面校准*/
		}
		/*6面校准第3步*/
		if(*(data_buf+4)==0X23)
		{
			/*加速度计六面校准*/
		}
		/*6面校准第4步*/
		if(*(data_buf+4)==0X24)
		{
			/*加速度计六面校准*/
		}
		/*6面校准第5步*/
		if(*(data_buf+4)==0X25)
		{
			/*加速度计六面校准*/
		}
		/*6面校准第6步*/
		if(*(data_buf+4)==0X26)
		{
			/*加速度计六面校准*/
		}			
	}
	
	/*2.上传飞行器信息*/	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
			
			/*允许数据上传*/
			g_psPidSystem->PidSettingSystem.PARA_DOWNLOAD_STATUS = PID_PARA_DOWNLOAD_FINISH;
		}
		if(*(data_buf+4)==0X02) /*未知*/
		{
			
		}
		if(*(data_buf+4)==0XA0) /*读取下位机版本信息*/
		{
			f.send_version = 1;

			/*允许数据上传*/
			g_psPidSystem->PidSettingSystem.PARA_DOWNLOAD_STATUS = PID_PARA_DOWNLOAD_FINISH;
		}
		if(*(data_buf+4)==0XA1)		/*恢复默认PID参数*/
		{
			g_psPidSystem->PidSettingSystem.DO_STATUS = PID_PARAMETER_DO_RESET;

			/*允许数据上传*/
			g_psPidSystem->PidSettingSystem.PARA_DOWNLOAD_STATUS = PID_PARA_DOWNLOAD_FINISH;
		}
	}

	/*3.上位机的PID参数配置写入*/
	if(*(data_buf+2)==0X10)	 /*=== PID1 角速度环 ===*/
    {
		/*着陆状态才允许调试控制参数*/
		if (g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_YES)
		{		
			g_psPidSystem->RollGyro.PID.kP = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
			g_psPidSystem->RollGyro.PID.kI = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
			g_psPidSystem->RollGyro.PID.kD = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);

			g_psPidSystem->PitchGyro.PID.kP  = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
			g_psPidSystem->PitchGyro.PID.kI  = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
			g_psPidSystem->PitchGyro.PID.kD  = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);

			g_psPidSystem->YawGyro.PID.kP   = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
			g_psPidSystem->YawGyro.PID.kI   = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
			g_psPidSystem->YawGyro.PID.kD   = (vs16)(*(data_buf+20)<<8)|*(data_buf+21);
		
			ANO_DT_Send_Check(*(data_buf+2), sum);
		}
    }
    if(*(data_buf+2)==0X11)	/*=== PID2 角度环 ===*/
    {
		/*着陆状态才允许调试控制参数*/
		if (g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_YES)
		{		
			g_psPidSystem->RollAngle.PID.kP = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
			g_psPidSystem->RollAngle.PID.kI = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
			g_psPidSystem->RollAngle.PID.kD = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);

			g_psPidSystem->PitchAngle.PID.kP  = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
			g_psPidSystem->PitchAngle.PID.kI  = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
			g_psPidSystem->PitchAngle.PID.kD  = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);

			g_psPidSystem->YawAngle.PID.kP   = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
			g_psPidSystem->YawAngle.PID.kI   = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
			g_psPidSystem->YawAngle.PID.kD   = (vs16)(*(data_buf+20)<<8)|*(data_buf+21);
		
			/*应答*/
			ANO_DT_Send_Check(*(data_buf+2), sum);
		}
    }
    if(*(data_buf+2)==0X12)	/*=== PID3 竖直速度环 & 位置环 + 水平速度环 ===*/
    {	
		/*着陆状态才允许调试控制参数*/
		if (g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_YES)
		{
			g_psPidSystem->HighSpeed.PID.kP      = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
			g_psPidSystem->HighSpeed.PID.kI      = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
			g_psPidSystem->HighSpeed.PID.kD      = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);
											    
			g_psPidSystem->HighPosition.PID.kP   = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
			g_psPidSystem->HighPosition.PID.kI   = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
			g_psPidSystem->HighPosition.PID.kD   = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);
											    
			g_psPidSystem->LatitudeSpeed.PID.kP  = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
			g_psPidSystem->LatitudeSpeed.PID.kI  = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
			g_psPidSystem->LatitudeSpeed.PID.kD  = (vs16)(*(data_buf+20)<<8)|*(data_buf+21);
								
			/***********************位置控制：位置、速度参数共用一组PID参数*************************/
			g_psPidSystem->LongitudeSpeed.PID.kP = g_psPidSystem->LatitudeSpeed.PID.kP;
			g_psPidSystem->LongitudeSpeed.PID.kI = g_psPidSystem->LatitudeSpeed.PID.kI;	
			g_psPidSystem->LongitudeSpeed.PID.kD = g_psPidSystem->LatitudeSpeed.PID.kD;
		
			/*应答*/
			ANO_DT_Send_Check(*(data_buf+2), sum);
		}
    }
	if(*(data_buf+2)==0X13)	/*=== PID4 水平位置环 & 竖直加速度环 ===*/
	{
		/*着陆状态才允许调试控制参数*/
		if (g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_YES)
		{
			g_psPidSystem->LatitudePosition.PID.kP  = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
			g_psPidSystem->LatitudePosition.PID.kI  = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
			g_psPidSystem->LatitudePosition.PID.kD  = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);
		
			g_psPidSystem->HighAcc.PID.kP           = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
			g_psPidSystem->HighAcc.PID.kI           = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
			g_psPidSystem->HighAcc.PID.kD           = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);	
			
			g_psPidSystem->LatitudeAcc.PID.kP       = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
			g_psPidSystem->LatitudeAcc.PID.kI       = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
			g_psPidSystem->LatitudeAcc.PID.kD       = (vs16)(*(data_buf+20)<<8)|*(data_buf+21);	
		
			/***********************位置控制：位置、速度参数共用一组PID参数*************************/		
			g_psPidSystem->LongitudePosition.PID.kP = g_psPidSystem->LatitudePosition.PID.kP;
			g_psPidSystem->LongitudePosition.PID.kI = g_psPidSystem->LatitudePosition.PID.kI;	
			g_psPidSystem->LongitudePosition.PID.kD = g_psPidSystem->LatitudePosition.PID.kD;	
			
			g_psPidSystem->LongitudeAcc.PID.kP      = g_psPidSystem->LatitudeAcc.PID.kP;
			g_psPidSystem->LongitudeAcc.PID.kI      = g_psPidSystem->LatitudeAcc.PID.kI;
			g_psPidSystem->LongitudeAcc.PID.kD      = g_psPidSystem->LatitudeAcc.PID.kD;			
		
			/*应答*/
			ANO_DT_Send_Check(*(data_buf+2), sum);
		}
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		/*着陆状态才允许调试控制参数*/
		if (g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_YES)
		{	
			g_psPidSystem->OpticFlowXSpeed.PID.kP  = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
			g_psPidSystem->OpticFlowXSpeed.PID.kI  = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
			g_psPidSystem->OpticFlowXSpeed.PID.kD  = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);
		
			g_psPidSystem->OpticFlowXPosition.PID.kP  = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
			g_psPidSystem->OpticFlowXPosition.PID.kI  = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
			g_psPidSystem->OpticFlowXPosition.PID.kD  = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);	
			
			g_psPidSystem->OpticFlowXAcc.PID.kP       = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
			g_psPidSystem->OpticFlowXAcc.PID.kI       = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
			g_psPidSystem->OpticFlowXAcc.PID.kD       = (vs16)(*(data_buf+20)<<8)|*(data_buf+21);	
		
			/***********************位置控制：位置、速度参数共用一组PID参数*************************/		
			g_psPidSystem->OpticFlowYSpeed.PID.kP     = g_psPidSystem->OpticFlowXSpeed.PID.kP;
			g_psPidSystem->OpticFlowYSpeed.PID.kI     = g_psPidSystem->OpticFlowXSpeed.PID.kI;
			g_psPidSystem->OpticFlowYSpeed.PID.kD     = g_psPidSystem->OpticFlowXSpeed.PID.kD;
			
			g_psPidSystem->OpticFlowYPosition.PID.kP  = g_psPidSystem->OpticFlowXPosition.PID.kP;
			g_psPidSystem->OpticFlowYPosition.PID.kI  = g_psPidSystem->OpticFlowXPosition.PID.kI;	
			g_psPidSystem->OpticFlowYPosition.PID.kD  = g_psPidSystem->OpticFlowXPosition.PID.kD;				
			
			g_psPidSystem->OpticFlowYAcc.PID.kP       = g_psPidSystem->OpticFlowXAcc.PID.kP;
			g_psPidSystem->OpticFlowYAcc.PID.kI       = g_psPidSystem->OpticFlowXAcc.PID.kI;
			g_psPidSystem->OpticFlowYAcc.PID.kD       = g_psPidSystem->OpticFlowXAcc.PID.kD;			
		
			/*应答*/
			ANO_DT_Send_Check(*(data_buf+2), sum);
		}
	}
	if(*(data_buf+2)==0X15)								//PID6
	{	
		/*着陆状态才允许调试控制参数*/
		if (g_sUav_Status.UavLandStatus.ThisTime == UAV_LAND_YES)
		{		
			ANO_DT_Send_Check(*(data_buf+2), sum);
			
			/*调参结束,允许数据上传*/
			g_psPidSystem->PidSettingSystem.PARA_DOWNLOAD_STATUS = PID_PARA_DOWNLOAD_FINISH;

			/*PID参数下传完毕后,保存参数到存储器单元(EEPROM)*/
			g_psPidSystem->PidSettingSystem.DO_STATUS = PID_PARAMETER_DO_SAVE;
		}		
	}
	
	/*4.上位机选择及上传数据切换*/
	if(*(data_buf+2)==USER_HOST_CMD_SW_RS)
	{
		/*上位机选择，数据帧对象*/
		g_sUserSendHostSystem.HOST_TARG  = (USER_HOST_CHOOSE_TARG)(*(data_buf+4)); /*上位机选择*/
		g_sUserSendHostSystem.MSG_ID     = (USER_HOST_MSG_ID)(*(data_buf+5));	   /*数据帧内容*/
		
		/*发送间隔,基于5ms*/
		g_sUserSendHostSystem.period_5MS = (u16)(*(data_buf+6));
		
		/*标记上传任务未切换*/
		g_sUserSendHostSystem.SWITCH_STATUS = USER_HOST_SWITCH_NO;
		
		/*解析结束,允许数据上传*/
		g_psPidSystem->PidSettingSystem.PARA_DOWNLOAD_STATUS = PID_PARA_DOWNLOAD_FINISH;
	}
}

void user_ANO_Send_Host_Wave_Data(USER_HOST_MSG_ID USER_WAVE_TARG, u32 periodTaskMs)
{
	static vu8 cnt      = 0;
	static vu8 sendFlag = 0;
	
	/*发送数据前,发送buff清0*/
	memset(ano_user_data_to_send, 0, DEBUG_TX_BUFF_SIZE);
	
	/*发送惯导传感器数据*/
	if((cnt % periodTaskMs) == (periodTaskMs-1))
	{
		sendFlag = 1;	/*标记允许发送*/
	}
	
	cnt++;
	
	/*每次切换清空一次buff*/
	if (g_sUserSendHostSystem.SWITCH_STATUS == USER_HOST_SWITCH_NO)
	{
		memset((u8*)&g_sSendSensorData, 0, sizeof(SendSensorData));
		memset((u8*)&g_sSendSinsDataHeight, 0, sizeof(SendSINSData));
		memset((u8*)&g_sSendSinsDataHorizontalX, 0, sizeof(SendSINSData));	
		memset((u8*)&g_sSendSinsDataHorizontalY, 0, sizeof(SendSINSData));
		memset((u8*)&g_sSendOpticFlowCtrlData, 0, sizeof(SendOpticFlowCtrlData));	
		memset((u8*)&g_sSendPidPara, 0, sizeof(SendPIDPara));		
		
		/*标记已切换,避免重复清0,降低效率*/
		g_sUserSendHostSystem.SWITCH_STATUS = USER_HOST_SWITCH_OK;
	}

	if(sendFlag == 1)
	{
		switch (USER_WAVE_TARG)
		{
			/*传感器原始和滤波值*/
			case USER_HOST_MSG_SENSOR_RAW_FILTER:
			{
				/*acc raw & filter*/
				g_sSendSensorData.accX       = g_psAccRaw->x;
				g_sSendSensorData.accXFilter = g_psAccAttitude->x;
				g_sSendSensorData.accY       = g_psAccRaw->y;
				g_sSendSensorData.accYFilter = g_psAccAttitude->y;
				g_sSendSensorData.accZ  	 = g_psAccRaw->z;	
				g_sSendSensorData.accZFilter = g_psAccAttitude->z;
				
				/*gyro raw & filter*/
				g_sSendSensorData.gyroX 	  = g_psGyroRaw->x;
				g_sSendSensorData.gyroXFilter = g_psGyroAttitude->x;		
				g_sSendSensorData.gyroY 	  = g_psGyroRaw->y;
				g_sSendSensorData.gyroYFilter = g_psGyroAttitude->y;	
				g_sSendSensorData.gyroZ 	  = g_psGyroRaw->z;	
				g_sSendSensorData.gyroZFilter = g_psGyroAttitude->z;		

				/*mag raw & filter*/
				g_sSendSensorData.magX 		 = g_psMagRaw->x;
				g_sSendSensorData.magXFilter = g_psMagFilter->x;	
				g_sSendSensorData.magY 		 = g_psMagRaw->y;
				g_sSendSensorData.magYFilter = g_psMagFilter->y;	
				g_sSendSensorData.magZ 		 = g_psMagRaw->z;		
				g_sSendSensorData.magZFilter = g_psMagFilter->z;
				
				user_ANO_Send_Sensor_RawFilter_Data(g_sSendSensorData);
			}break;
			
			/*竖直方向融合数据*/
			case USER_HOST_MSG_SINS_HEIGHT:
			{
				g_sSendSinsDataHeight.sinsPosition = g_psSinsReal->curPosition[EARTH_FRAME_Z];	     /*惯导估算位置*/
				g_sSendSinsDataHeight.sinsSpeed    = g_psSinsReal->curSpeed[EARTH_FRAME_Z];		     /*惯导估算速度*/
				g_sSendSinsDataHeight.sinsAcc      = g_psSinsReal->curAcc[EARTH_FRAME_Z];		     /*惯导估算加速度*/
				g_sSendSinsDataHeight.obPosition   = g_psSinsReal->estimatePos[EARTH_FRAME_Z];	     /*观测位置*/
				g_sSendSinsDataHeight.obAcc        = g_psSinsOrigion->curAcc[EARTH_FRAME_Z];		     /*观测加速度*/
				g_sSendSinsDataHeight.corAcc       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].acc;   /*三阶互补加速度反馈修正量*/
				g_sSendSinsDataHeight.corSpd       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].speed; /*三阶互补速度反馈修正量*/
				g_sSendSinsDataHeight.corPos       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].pos;	 /*三阶互补位置反馈修正量*/				
				
				user_ANO_Send_Sins_Height_Data(g_sSendSinsDataHeight);
			}break;

			/*水平X方向融合数据(导航系)*/
			case USER_HOST_MSG_SINS_HORIZONTAL_X:
			{
				g_sSendSinsDataHorizontalX.sinsPosition = g_psSinsReal->curPosition[EARTH_FRAME_X];	        /*惯导估算位置*/
				g_sSendSinsDataHorizontalX.sinsSpeed    = g_psSinsReal->curSpeed[EARTH_FRAME_X];		        /*惯导估算速度*/
				g_sSendSinsDataHorizontalX.sinsAcc      = g_psSinsReal->curAcc[EARTH_FRAME_X];		            /*惯导估算加速度*/
				g_sSendSinsDataHorizontalX.obPosition   = g_psSinsReal->estimatePos[EARTH_FRAME_X];			/*观测位置*/
				g_sSendSinsDataHorizontalX.obAcc        = g_psSinsOrigion->curAcc[EARTH_FRAME_X];		        /*观测加速度*/
				g_sSendSinsDataHorizontalX.corAcc       = g_psTOCSystem->BackCorrect[EARTH_FRAME_X].acc;       /*三阶互补加速度反馈修正量*/
				g_sSendSinsDataHorizontalX.corSpd       = g_psTOCSystem->BackCorrect[EARTH_FRAME_X].speed;		/*三阶互补速度反馈修正量*/
				g_sSendSinsDataHorizontalX.corPos       = g_psTOCSystem->BackCorrect[EARTH_FRAME_X].pos;	    /*三阶互补位置反馈修正量*/				
				
				user_ANO_Send_Sins_Horizontal_X_Data(g_sSendSinsDataHorizontalX);
			}break;

			/*水平Y方向融合数据(导航系)*/
			case USER_HOST_MSG_SINS_HORIZONTAL_Y:
			{
				g_sSendSinsDataHorizontalY.sinsPosition = g_psSinsReal->curPosition[EARTH_FRAME_Y];	       	    /*惯导估算位置*/
				g_sSendSinsDataHorizontalY.sinsSpeed    = g_psSinsReal->curSpeed[EARTH_FRAME_Y];		        /*惯导估算速度*/
				g_sSendSinsDataHorizontalY.sinsAcc      = g_psSinsReal->curAcc[EARTH_FRAME_Y];		            /*惯导估算加速度*/
				g_sSendSinsDataHorizontalY.obPosition   = g_psSinsReal->estimatePos[EARTH_FRAME_Y];   			/*观测位置*/
				g_sSendSinsDataHorizontalY.obAcc        = g_psSinsOrigion->curAcc[EARTH_FRAME_Y];		        /*观测加速度*/
				g_sSendSinsDataHorizontalY.corAcc       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].acc;   		/*三阶互补加速度反馈修正量*/
				g_sSendSinsDataHorizontalY.corSpd       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].speed;		/*三阶互补速度反馈修正量*/
				g_sSendSinsDataHorizontalY.corPos       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].pos;		/*三阶互补位置反馈修正量*/
				
				user_ANO_Send_Sins_Horizontal_Y_Data(g_sSendSinsDataHorizontalY);
			}break;
			
			/*发送光流控制数据到上位机*/
			case USER_HOST_MSG_OPTICFLOW_CTRL_DATA:
			{
				g_sSendOpticFlowCtrlData.intPosition     = g_sOpFlowUpixelsLC306.OpFlowData.xIntegral;
				g_sSendOpticFlowCtrlData.intPositionLPF  = g_psAttitudeAll->OpticFlowData.IntPixLPF.x;
				g_sSendOpticFlowCtrlData.angleCompensate = g_psAttitudeAll->OpticFlowData.AngleCompensate.x;
				g_sSendOpticFlowCtrlData.curRawPosition  = g_psAttitudeAll->OpticFlowData.CurRawPosition.x;
				g_sSendOpticFlowCtrlData.diffSpeed       = g_psAttitudeAll->OpticFlowData.DiffSpeed.x;
				g_sSendOpticFlowCtrlData.diffSpeedLPF    = g_psAttitudeAll->OpticFlowData.DiffSpeedLPF.x;
				g_sSendOpticFlowCtrlData.realPosition    = g_psAttitudeAll->OpticFlowData.RealPosition.x;
				g_sSendOpticFlowCtrlData.realSpeed       = g_psAttitudeAll->OpticFlowData.RealSpeed.x;
				
				user_ANO_Send_OpticFlow_Ctrl_Data(g_sSendOpticFlowCtrlData);
			}break;			

			/*竖直方向控制*/
			case USER_HOST_MSG_VER_LINK_CONTROL:
			{				
				/*ver pos*/
				g_sSendPidPara.data1 = g_psPidSystem->HighPosition.expect;
				g_sSendPidPara.data2 = g_psPidSystem->HighPosition.feedback;
				
				/*ver speed*/
				g_sSendPidPara.data3 = g_psPidSystem->HighSpeed.expect;
				g_sSendPidPara.data4 = g_psPidSystem->HighSpeed.feedback;		

				/*ver acc*/
				g_sSendPidPara.data5 = g_psPidSystem->HighAcc.expect;
				g_sSendPidPara.data6 = g_psPidSystem->HighAcc.feedback;
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 6;

				/*上传*/
				user_ANO_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;
			
			/*GPS 水平X方向控制(机体系)*/
			case USER_HOST_MSG_GPS_HOR_X_LINK_CONTROL:
			{
				/*hor_x pos*/
				g_sSendPidPara.data1 = g_psAttitudeAll->BodyFramePosError.roll; /*pos error*/
				
				/*hor_x speed*/
				g_sSendPidPara.data2 = g_psPidSystem->LongitudeSpeed.expect;
				g_sSendPidPara.data3 = g_psPidSystem->LongitudeSpeed.feedback;		

				/*hor_x angle*/
				g_sSendPidPara.data4 = g_psPidSystem->RollAngle.expect;
				g_sSendPidPara.data5 = g_psPidSystem->RollAngle.feedback;
				
				/*hor_x gyro*/
				g_sSendPidPara.data6 = g_psPidSystem->RollGyro.expect;
				g_sSendPidPara.data7 = g_psPidSystem->RollGyro.feedback;				
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 7;

				/*上传*/
				user_ANO_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;		

			/*GPS 水平Y方向控制(机体系)*/
			case USER_HOST_MSG_GPS_HOR_Y_LINK_CONTROL:
			{
				/*hor_y pos*/
				g_sSendPidPara.data1 = g_psAttitudeAll->BodyFramePosError.pitch; /*pos error*/
				
				/*hor_y speed*/
				g_sSendPidPara.data2 = g_psPidSystem->LatitudeSpeed.expect;
				g_sSendPidPara.data3 = g_psPidSystem->LatitudeSpeed.feedback;		

				/*hor_y angle*/
				g_sSendPidPara.data4 = g_psPidSystem->PitchAngle.expect;
				g_sSendPidPara.data5 = g_psPidSystem->PitchAngle.feedback;
				
				/*hor_y gyro*/
				g_sSendPidPara.data6 = g_psPidSystem->PitchGyro.expect;
				g_sSendPidPara.data7 = g_psPidSystem->PitchGyro.feedback;				
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 7;

				/*上传*/
				user_ANO_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;	
			
			/*OPTIC FLOW 水平X方向*/
			case USER_HOST_MSG_OPFLOW_HOR_X_LINK_CONTROL:
			{
				/*hor_x pos*/
//				g_sSendPidPara.Link1.expect   = g_psPidSystem->LongitudePosition.expect;
//				g_sSendPidPara.Link1.feedBack = g_psPidSystem->LongitudePosition.feedback;
				
				/*hor_x speed*/
//				g_sSendPidPara.Link2.expect   = g_psPidSystem->LongitudeSpeed.expect;
//				g_sSendPidPara.Link2.feedBack = g_psPidSystem->LongitudeSpeed.feedback;		

//				/*hor_x angle*/
//				g_sSendPidPara.Link3.expect   = g_psPidSystem->RollAngle.expect;
//				g_sSendPidPara.Link3.feedBack = g_psPidSystem->RollAngle.feedback;
//				
//				/*hor_x gyro*/
//				g_sSendPidPara.Link4.expect   = g_psPidSystem->RollGyro.expect;
//				g_sSendPidPara.Link4.feedBack = g_psPidSystem->RollGyro.feedback;				
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 2 * 4;

				/*上传*/
				user_ANO_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;		

			/*OPTIC FLOW 水平Y方向*/
			case USER_HOST_MSG_OPFLOW_HOR_Y_LINK_CONTROL:
			{
				/*hor_y pos*/
//				g_sSendPidPara.Link1.expect   = g_psPidSystem->LatitudePosition.expect;
//				g_sSendPidPara.Link1.feedBack = g_psPidSystem->LatitudePosition.feedback;
//				
//				/*hor_y speed*/
//				g_sSendPidPara.Link2.expect   = g_psPidSystem->LatitudeSpeed.expect;
//				g_sSendPidPara.Link2.feedBack = g_psPidSystem->LatitudeSpeed.feedback;		

//				/*hor_y angle*/
//				g_sSendPidPara.Link3.expect   = g_psPidSystem->PitchAngle.expect;
//				g_sSendPidPara.Link3.feedBack = g_psPidSystem->PitchAngle.feedback;
//				
//				/*hor_y gyro*/
//				g_sSendPidPara.Link4.expect   = g_psPidSystem->PitchGyro.expect;
//				g_sSendPidPara.Link4.feedBack = g_psPidSystem->PitchGyro.feedback;				
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 2 * 4;

				/*上传*/
				user_ANO_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;			

			/*水平Z方向*/
			case USER_HOST_MSG_HOR_Z_LINK_CONTROL:
			{
				/*hor_z angle*/
				g_sSendPidPara.data1 = g_psPidSystem->YawAngle.expect;
				g_sSendPidPara.data2 = g_psPidSystem->YawAngle.feedback;
				
				/*hor_z gyro*/
				g_sSendPidPara.data3 = g_psPidSystem->YawGyro.expect;
				g_sSendPidPara.data4 = g_psPidSystem->YawGyro.feedback;				
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 4;

				/*上传*/
				user_ANO_Send_Pid_Link_Data(g_sSendPidPara);
			}break;
			
			default:break;
		}
		
		sendFlag = 0;
	}
}

/*1.发送传感器滤波前后数据到上位机*/
void user_ANO_Send_Sensor_RawFilter_Data(SendSensorData sensorData)
{
	u8 _cnt=0;
	vs16 temp1;
	vs32 temp2;
	
	/*上位机帧协议*/
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xF1;
	ano_user_data_to_send[_cnt++] = 0;

	/*加速度3个轴原始和滤波后*/
	temp1 = sensorData.accX;
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.accXFilter;
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.accY;	
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.accYFilter;
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.accZ;	
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.accZFilter;	
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);

	/*角速度3个轴原始和滤波后*/	
	temp1 = sensorData.gyroX;
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.gyroXFilter;
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.gyroY;	
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.gyroYFilter;
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.gyroZ;	
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.gyroZFilter;	
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);	

	/*磁力计3个轴原始和滤波后*/	
	temp1 = sensorData.magX;
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.magXFilter;
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.magY;	
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.magYFilter;
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.magZ;	
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);
	temp1 = sensorData.magZFilter;	
	ano_user_data_to_send[_cnt++] = BYTE1(temp1);
	ano_user_data_to_send[_cnt++] = BYTE0(temp1);

	/*气压计原始和滤波后*/	
	temp2 = (s32)sensorData.bero;
	ano_user_data_to_send[_cnt++] = BYTE3(temp2);
	ano_user_data_to_send[_cnt++] = BYTE2(temp2);
	ano_user_data_to_send[_cnt++] = BYTE1(temp2);
	ano_user_data_to_send[_cnt++] = BYTE0(temp2);
	temp2 = (s32)sensorData.beroFilter;
	ano_user_data_to_send[_cnt++] = BYTE3(temp2);
	ano_user_data_to_send[_cnt++] = BYTE2(temp2);
	ano_user_data_to_send[_cnt++] = BYTE1(temp2);
	ano_user_data_to_send[_cnt++] = BYTE0(temp2);	

	/*有效数据长度*/
	ano_user_data_to_send[3] = _cnt - 4;
	
	u8 sum = 0;
	
	/*校验位前所有数据求和*/
	for(u8 i = 0; i < _cnt; i++)
	{
		sum += ano_user_data_to_send[i];
	}
	
	ano_user_data_to_send[_cnt++] = sum;
	
	/*发送*/
	ANO_DT_Send_Data(ano_user_data_to_send, _cnt, MSP_UART_DMA);		
}


/*2.发送竖直惯导数据到上位机*/
void user_ANO_Send_Sins_Height_Data(SendSINSData sendSinsData)
{
	u8 _cnt = 0;
	vs32 temp;	
	
	/*帧头,帧类型*/
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xF1;
	ano_user_data_to_send[_cnt++] = 0;	
	
	/*惯导估计位置*/
	temp = (s32)sendSinsData.sinsPosition;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*惯导估计速度*/
	temp = (s32)sendSinsData.sinsSpeed;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);		
	
	/*惯导估计加速度*/
	temp = (s32)sendSinsData.sinsAcc;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	

	/*传感器观测位置*/
	temp = (s32)sendSinsData.obPosition;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);

	/*传感器观测加速度*/
	temp = (s32)sendSinsData.obAcc;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*加速度修正量*/
	temp = (s32)sendSinsData.corAcc;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*速度修正量*/
	temp = (s32)sendSinsData.corSpd;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*位置修正量*/
	temp = (s32)sendSinsData.corPos;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*数据长度*/
	ano_user_data_to_send[3] = _cnt - 4;
	
	u8 sum = 0;
	
	/*校验位前所有数据求和*/
	for(u8 i = 0; i < _cnt; i++)
	{
		sum += ano_user_data_to_send[i];
	}
	
	ano_user_data_to_send[_cnt++] = sum;
	
	/*发送*/
	ANO_DT_Send_Data(ano_user_data_to_send, _cnt, MSP_UART_DMA);		
}

/*发送水平X方向惯导数据到上位机*/
void user_ANO_Send_Sins_Horizontal_X_Data(SendSINSData sendSinsData)
{
	u8 _cnt = 0;
	vs32 temp;	
	
	/*帧头,帧类型*/
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xF1;
	ano_user_data_to_send[_cnt++] = 0;	
	
	/*惯导估计位置*/
	temp = (s32)sendSinsData.sinsPosition;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*惯导估计速度*/
	temp = (s32)sendSinsData.sinsSpeed;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);		
	
	/*惯导估计加速度*/
	temp = (s32)sendSinsData.sinsAcc;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	

	/*传感器观测位置*/
	temp = (s32)sendSinsData.obPosition;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);

	/*传感器观测加速度*/
	temp = (s32)sendSinsData.obAcc;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*加速度修正量*/
	temp = (s32)sendSinsData.corAcc;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*速度修正量*/
	temp = (s32)sendSinsData.corSpd;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*位置修正量*/
	temp = (s32)sendSinsData.corPos;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*数据长度*/
	ano_user_data_to_send[3] = _cnt - 4;
	
	u8 sum = 0;
	
	/*校验位前所有数据求和*/
	for(u8 i = 0; i < _cnt; i++)
	{
		sum += ano_user_data_to_send[i];
	}
	
	ano_user_data_to_send[_cnt++] = sum;
	
	/*发送*/
	ANO_DT_Send_Data(ano_user_data_to_send, _cnt, MSP_UART_DMA);
}

/*发送水平Y方向惯导数据到上位机*/
void user_ANO_Send_Sins_Horizontal_Y_Data(SendSINSData sendSinsData)
{
	u8 _cnt = 0;
	vs32 temp;	
	
	/*帧头,帧类型*/
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xF1;
	ano_user_data_to_send[_cnt++] = 0;	
	
	/*惯导估计位置*/
	temp = (s32)sendSinsData.sinsPosition;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*惯导估计速度*/
	temp = (s32)sendSinsData.sinsSpeed;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);		
	
	/*惯导估计加速度*/
	temp = (s32)sendSinsData.sinsAcc;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	

	/*传感器观测位置*/
	temp = (s32)sendSinsData.obPosition;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);

	/*传感器观测加速度*/
	temp = (s32)sendSinsData.obAcc;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*加速度修正量*/
	temp = (s32)sendSinsData.corAcc;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*速度修正量*/
	temp = (s32)sendSinsData.corSpd;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*位置修正量*/
	temp = (s32)sendSinsData.corPos;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*数据长度*/
	ano_user_data_to_send[3] = _cnt - 4;
	
	u8 sum = 0;
	
	/*校验位前所有数据求和*/
	for(u8 i = 0; i < _cnt; i++)
	{
		sum += ano_user_data_to_send[i];
	}
	
	ano_user_data_to_send[_cnt++] = sum;
	
	/*发送*/
	ANO_DT_Send_Data(ano_user_data_to_send, _cnt, MSP_UART_DMA);
}

/*3.发送PID控制系统数据到上位机*/
void user_ANO_Send_Pid_Link_Data(SendPIDPara sendPidPara)
{
	u8 _cnt = 0;
	vs32 temp;	
	
	/*帧头,帧类型*/
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xF1;	
	ano_user_data_to_send[_cnt++] = 0;
	
	/*LINK1 期望值*/
	temp = (s32)sendPidPara.data1;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*LINK1 反馈值*/
	temp = (s32)sendPidPara.data2;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);

	/*LINK2 期望值*/
	temp = (s32)sendPidPara.data3;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*LINK2 反馈值*/
	temp = (s32)sendPidPara.data4;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*LINK3 期望值*/
	temp = (s32)sendPidPara.data5;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*LINK3 反馈值*/
	temp = (s32)sendPidPara.data6;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*LINK4 期望值*/
	temp = (s32)sendPidPara.data7;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*LINK4 反馈值*/
	temp = (s32)sendPidPara.data8;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*数据长度*/
	ano_user_data_to_send[3] = _cnt - 4;
	
	u8 sum = 0;
	
	/*校验位前所有数据求和*/
	for(u8 i = 0; i < _cnt; i++)
	{
		sum += ano_user_data_to_send[i];
	}
	
	ano_user_data_to_send[_cnt++] = sum;
	
	/*发送*/
	ANO_DT_Send_Data(ano_user_data_to_send, _cnt, MSP_UART_DMA);	
}

/*4.发送光流控制数据到上位机*/
void user_ANO_Send_OpticFlow_Ctrl_Data(SendOpticFlowCtrlData sendOpticFlowCtrlData)
{
	u8 _cnt = 0;
	vs32 temp;	
	
	/*帧头,帧类型*/
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xAA;
	ano_user_data_to_send[_cnt++] = 0xF1;	
	ano_user_data_to_send[_cnt++] = 0;
	
	/*积分位移*/
	temp = (s32)sendOpticFlowCtrlData.intPosition;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*积分位移低通滤波*/
	temp = (s32)sendOpticFlowCtrlData.intPositionLPF;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*姿态角补偿积分位移*/
	temp = (s32)sendOpticFlowCtrlData.angleCompensate;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*本次处理后的积分位移*/
	temp = (s32)sendOpticFlowCtrlData.curRawPosition;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*微分速度*/
	temp = (s32)sendOpticFlowCtrlData.diffSpeed;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*微分速度低通滤波*/
	temp = (s32)sendOpticFlowCtrlData.diffSpeedLPF;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*转化后的位移*/
	temp = (s32)sendOpticFlowCtrlData.realPosition;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);
	
	/*转化后的速度*/
	temp = (s32)sendOpticFlowCtrlData.realSpeed;
	ano_user_data_to_send[_cnt++] = BYTE3(temp);
	ano_user_data_to_send[_cnt++] = BYTE2(temp);
	ano_user_data_to_send[_cnt++] = BYTE1(temp);
	ano_user_data_to_send[_cnt++] = BYTE0(temp);	
	
	/*数据长度*/
	ano_user_data_to_send[3] = _cnt - 4;
	
	u8 sum = 0;
	
	/*校验位前所有数据求和*/
	for(u8 i = 0; i < _cnt; i++)
	{
		sum += ano_user_data_to_send[i];
	}
	
	ano_user_data_to_send[_cnt++] = sum;
	
	/*发送*/
	ANO_DT_Send_Data(ano_user_data_to_send, _cnt, MSP_UART_DMA);	
}


/*====== VCAN山外上位机 ======*/
void user_VCAN_Send_Host_Wave_Data(USER_HOST_MSG_ID USER_WAVE_TARG, u32 periodTaskMs)
{
	static vu8 cnt      = 0;
	static vu8 sendFlag = 0;
	
	/*发送数据前,发送buff清0*/
	memset(vcan_user_data_to_send, 0, DEBUG_TX_BUFF_SIZE);
	
	/*发送惯导传感器数据*/
	if((cnt % periodTaskMs) == (periodTaskMs-1))
	{
		sendFlag = 1;	/*标记允许发送*/
	}	
	
	cnt++;
	
	/*每次切换清空一次buff*/
	if (g_sUserSendHostSystem.SWITCH_STATUS == USER_HOST_SWITCH_NO)
	{
		memset((u8*)&g_sSendSensorData, 0, sizeof(SendSensorData));
		memset((u8*)&g_sSendSinsDataHeight, 0, sizeof(SendSINSData));
		memset((u8*)&g_sSendSinsDataHorizontalX, 0, sizeof(SendSINSData));	
		memset((u8*)&g_sSendSinsDataHorizontalY, 0, sizeof(SendSINSData));
		memset((u8*)&g_sSendOpticFlowCtrlData, 0, sizeof(SendOpticFlowCtrlData));	
		memset((u8*)&g_sSendPidPara, 0, sizeof(SendPIDPara));		
		
		/*标记已切换,避免重复清0,降低效率*/
		g_sUserSendHostSystem.SWITCH_STATUS = USER_HOST_SWITCH_OK;
	}	

	if(sendFlag == 1)
	{
		switch (USER_WAVE_TARG)
		{
			/*传感器原始和滤波值*/
			case USER_HOST_MSG_SENSOR_RAW_FILTER:
			{
				user_VCAN_Send_Sensor_RawFilter_Data(g_sSendSensorData);
			}break;
			
			/*竖直方向融合数据*/
			case USER_HOST_MSG_SINS_HEIGHT:
			{
				g_sSendSinsDataHeight.sinsPosition = g_psSinsReal->curPosition[EARTH_FRAME_Z];	     /*惯导估算位置*/
				g_sSendSinsDataHeight.sinsSpeed    = g_psSinsReal->curSpeed[EARTH_FRAME_Z];		     /*惯导估算速度*/
				g_sSendSinsDataHeight.sinsAcc      = g_psSinsReal->curAcc[EARTH_FRAME_Z];		     /*惯导估算加速度*/
				g_sSendSinsDataHeight.obPosition   = g_psSinsReal->estimatePos[EARTH_FRAME_Z];	     /*观测位置*/
				g_sSendSinsDataHeight.obAcc        = g_psSinsOrigion->curAcc[EARTH_FRAME_Z];		     /*观测加速度*/
				g_sSendSinsDataHeight.corAcc       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].acc;   /*三阶互补加速度反馈修正量*/
				g_sSendSinsDataHeight.corSpd       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].speed; /*三阶互补速度反馈修正量*/
				g_sSendSinsDataHeight.corPos       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Z].pos;	 /*三阶互补位置反馈修正量*/				
				
				user_VCAN_Send_Sins_Height_Data(g_sSendSinsDataHeight);				
			}break;

			/*水平X方向融合数据(导航系)*/
			case USER_HOST_MSG_SINS_HORIZONTAL_X:
			{
				g_sSendSinsDataHorizontalX.sinsPosition = g_psSinsReal->curPosition[EARTH_FRAME_X];	        /*惯导估算位置*/
				g_sSendSinsDataHorizontalX.sinsSpeed    = g_psSinsReal->curSpeed[EARTH_FRAME_X];		        /*惯导估算速度*/
				g_sSendSinsDataHorizontalX.sinsAcc      = g_psSinsReal->curAcc[EARTH_FRAME_X];		            /*惯导估算加速度*/
				g_sSendSinsDataHorizontalX.obPosition   = g_psSinsReal->estimatePos[EARTH_FRAME_X];			/*观测位置*/
				g_sSendSinsDataHorizontalX.obAcc        = g_psSinsOrigion->curAcc[EARTH_FRAME_X];		        /*观测加速度*/
				g_sSendSinsDataHorizontalX.corAcc       = g_psTOCSystem->BackCorrect[EARTH_FRAME_X].acc;       /*三阶互补加速度反馈修正量*/
				g_sSendSinsDataHorizontalX.corSpd       = g_psTOCSystem->BackCorrect[EARTH_FRAME_X].speed;		/*三阶互补速度反馈修正量*/
				g_sSendSinsDataHorizontalX.corPos       = g_psTOCSystem->BackCorrect[EARTH_FRAME_X].pos;	    /*三阶互补位置反馈修正量*/				
				
				user_VCAN_Send_Sins_Horizontal_X_Data(g_sSendSinsDataHorizontalX);
			}break;

			/*水平Y方向融合数据(导航系)*/
			case USER_HOST_MSG_SINS_HORIZONTAL_Y:
			{
				g_sSendSinsDataHorizontalY.sinsPosition = g_psSinsReal->curPosition[EARTH_FRAME_Y];	       	    /*惯导估算位置*/
				g_sSendSinsDataHorizontalY.sinsSpeed    = g_psSinsReal->curSpeed[EARTH_FRAME_Y];		        /*惯导估算速度*/
				g_sSendSinsDataHorizontalY.sinsAcc      = g_psSinsReal->curAcc[EARTH_FRAME_Y];		            /*惯导估算加速度*/
				g_sSendSinsDataHorizontalY.obPosition   = g_psSinsReal->estimatePos[EARTH_FRAME_Y];   			/*观测位置*/
				g_sSendSinsDataHorizontalY.obAcc        = g_psSinsOrigion->curAcc[EARTH_FRAME_Y];		        /*观测加速度*/
				g_sSendSinsDataHorizontalY.corAcc       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].acc;   		/*三阶互补加速度反馈修正量*/
				g_sSendSinsDataHorizontalY.corSpd       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].speed;		/*三阶互补速度反馈修正量*/
				g_sSendSinsDataHorizontalY.corPos       = g_psTOCSystem->BackCorrect[EARTH_FRAME_Y].pos;		/*三阶互补位置反馈修正量*/				
				
				user_VCAN_Send_Sins_Horizontal_Y_Data(g_sSendSinsDataHorizontalY);
			}break;
			
			/*光流控制数据*/
			case USER_HOST_MSG_OPTICFLOW_CTRL_DATA:
			{
				g_sSendOpticFlowCtrlData.intPosition     = g_sOpFlowUpixelsLC306.OpFlowData.xIntegral;
				g_sSendOpticFlowCtrlData.intPositionLPF  = g_psAttitudeAll->OpticFlowData.IntPixLPF.x;
				g_sSendOpticFlowCtrlData.angleCompensate = g_psAttitudeAll->OpticFlowData.AngleCompensate.x;
				g_sSendOpticFlowCtrlData.curRawPosition  = g_psAttitudeAll->OpticFlowData.CurRawPosition.x;
				g_sSendOpticFlowCtrlData.diffSpeed       = g_psAttitudeAll->OpticFlowData.DiffSpeed.x;
				g_sSendOpticFlowCtrlData.diffSpeedLPF    = g_psAttitudeAll->OpticFlowData.DiffSpeedLPF.x;
				g_sSendOpticFlowCtrlData.realPosition    = g_psAttitudeAll->OpticFlowData.RealPosition.x;
				g_sSendOpticFlowCtrlData.realSpeed       = g_psAttitudeAll->OpticFlowData.RealSpeed.x;
				
				user_VCAN_Send_OpticFlow_Ctrl_Data(g_sSendOpticFlowCtrlData);
			}break;			

			/*竖直方向控制*/
			case USER_HOST_MSG_VER_LINK_CONTROL:
			{
				/*ver pos*/
				g_sSendPidPara.data1 = g_psPidSystem->HighPosition.expect;
				g_sSendPidPara.data2 = g_psPidSystem->HighPosition.feedback;
				
				/*ver speed*/
				g_sSendPidPara.data3 = g_psPidSystem->HighSpeed.expect;
				g_sSendPidPara.data4 = g_psPidSystem->HighSpeed.feedback;		

				/*ver acc*/
				g_sSendPidPara.data5 = g_psPidSystem->HighAcc.expect;
				g_sSendPidPara.data6 = g_psPidSystem->HighAcc.feedback;
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 6;

				/*上传*/
				user_VCAN_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;
			
			/*GPS 水平X方向控制(机体系)*/
			case USER_HOST_MSG_GPS_HOR_X_LINK_CONTROL:
			{
				/*hor_x pos*/
				g_sSendPidPara.data1 = g_psAttitudeAll->BodyFramePosError.roll; /*pos error*/
				
				/*hor_x speed*/
				g_sSendPidPara.data2 = g_psPidSystem->LongitudeSpeed.expect;
				g_sSendPidPara.data3 = g_psPidSystem->LongitudeSpeed.feedback;		

				/*hor_x angle*/
				g_sSendPidPara.data4 = g_psPidSystem->RollAngle.expect;
				g_sSendPidPara.data5 = g_psPidSystem->RollAngle.feedback;
				
				/*hor_x gyro*/
				g_sSendPidPara.data6 = g_psPidSystem->RollGyro.expect;
				g_sSendPidPara.data7 = g_psPidSystem->RollGyro.feedback;				
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 7;

				/*上传*/
				user_VCAN_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;		

			/*GPS 水平Y方向控制(机体系)*/
			case USER_HOST_MSG_GPS_HOR_Y_LINK_CONTROL:
			{
				/*hor_y pos*/
				g_sSendPidPara.data1 = g_psAttitudeAll->BodyFramePosError.pitch; /*pos error*/
				
				/*hor_y speed*/
				g_sSendPidPara.data2 = g_psPidSystem->LatitudeSpeed.expect;
				g_sSendPidPara.data3 = g_psPidSystem->LatitudeSpeed.feedback;		

				/*hor_y angle*/
				g_sSendPidPara.data4 = g_psPidSystem->PitchAngle.expect;
				g_sSendPidPara.data5 = g_psPidSystem->PitchAngle.feedback;
				
				/*hor_y gyro*/
				g_sSendPidPara.data6 = g_psPidSystem->PitchGyro.expect;
				g_sSendPidPara.data7 = g_psPidSystem->PitchGyro.feedback;				
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 7;

				/*上传*/
				user_VCAN_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;	
			
			/*OPTIC FLOW 水平X方向控制*/
			case USER_HOST_MSG_OPFLOW_HOR_X_LINK_CONTROL:
			{
				/*hor_x pos*/
//				g_sSendPidPara.Link1.expect   = g_psPidSystem->LongitudePosition.expect;
//				g_sSendPidPara.Link1.feedBack = g_psPidSystem->LongitudePosition.feedback;
				
				/*hor_x speed*/
//				g_sSendPidPara.Link2.expect   = g_psPidSystem->LongitudeSpeed.expect;
//				g_sSendPidPara.Link2.feedBack = g_psPidSystem->LongitudeSpeed.feedback;		

//				/*hor_x angle*/
//				g_sSendPidPara.Link3.expect   = g_psPidSystem->RollAngle.expect;
//				g_sSendPidPara.Link3.feedBack = g_psPidSystem->RollAngle.feedback;
//				
//				/*hor_x gyro*/
//				g_sSendPidPara.Link4.expect   = g_psPidSystem->RollGyro.expect;
//				g_sSendPidPara.Link4.feedBack = g_psPidSystem->RollGyro.feedback;				
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 2 * 4;

				/*上传*/
				user_VCAN_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;		

			/*OPTIC FLOW 水平Y方向控制*/
			case USER_HOST_MSG_OPFLOW_HOR_Y_LINK_CONTROL:
			{
				/*hor_y pos*/
//				g_sSendPidPara.Link1.expect   = g_psPidSystem->LatitudePosition.expect;
//				g_sSendPidPara.Link1.feedBack = g_psPidSystem->LatitudePosition.feedback;
//				
//				/*hor_y speed*/
//				g_sSendPidPara.Link2.expect   = g_psPidSystem->LatitudeSpeed.expect;
//				g_sSendPidPara.Link2.feedBack = g_psPidSystem->LatitudeSpeed.feedback;		

//				/*hor_y angle*/
//				g_sSendPidPara.Link3.expect   = g_psPidSystem->PitchAngle.expect;
//				g_sSendPidPara.Link3.feedBack = g_psPidSystem->PitchAngle.feedback;
//				
//				/*hor_y gyro*/
//				g_sSendPidPara.Link4.expect   = g_psPidSystem->PitchGyro.expect;
//				g_sSendPidPara.Link4.feedBack = g_psPidSystem->PitchGyro.feedback;				
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 2 * 4;

				/*上传*/
				user_VCAN_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;			

			/*水平Z方向控制*/
			case USER_HOST_MSG_HOR_Z_LINK_CONTROL:
			{
				/*hor_z angle*/
				g_sSendPidPara.data1   = g_psPidSystem->YawAngle.expect;
				g_sSendPidPara.data2 = g_psPidSystem->YawAngle.feedback;
				
				/*hor_z gyro*/
				g_sSendPidPara.data3   = g_psPidSystem->YawGyro.expect;
				g_sSendPidPara.data4 = g_psPidSystem->YawGyro.feedback;				
				
				/*控制环总计个数*/
				g_sSendPidPara.waveChNbr = 4;

				/*上传*/
				user_VCAN_Send_Pid_Link_Data(g_sSendPidPara);				
			}break;				
			
			default:break;
		}
		
		sendFlag = 0;
	}
}

/*1.发送传感器滤波前后数据到上位机*/
void user_VCAN_Send_Sensor_RawFilter_Data(SendSensorData sensorData)
{

}

/*2.发送竖直惯导数据到上位机*/
void user_VCAN_Send_Sins_Height_Data(SendSINSData sendSinsData)
{
	u16 txBuffLenth;
	
	/*填充发送Buff*/
	txBuffLenth = VCAN_DT_Wave_Data_Fill_TxBuff(VCAN_WAVE_DATA_S32, 8, (void*)&sendSinsData, vcan_user_data_to_send);
	
	/*发送*/	
	VCAN_DT_Send_Data(vcan_user_data_to_send, txBuffLenth, MSP_UART_DMA);
}

/*发送水平X方向惯导数据到上位机*/
void user_VCAN_Send_Sins_Horizontal_X_Data(SendSINSData sendSinsData)
{
	u16 txBuffLenth;
	
	/*填充发送Buff*/
	txBuffLenth = VCAN_DT_Wave_Data_Fill_TxBuff(VCAN_WAVE_DATA_S32, 8, (void*)&sendSinsData, vcan_user_data_to_send);
	
	/*发送*/	
	VCAN_DT_Send_Data(vcan_user_data_to_send , txBuffLenth, MSP_UART_DMA);
}

/*发送水平Y方向惯导数据到上位机*/
void user_VCAN_Send_Sins_Horizontal_Y_Data(SendSINSData sendSinsData)
{
	u16 txBuffLenth;
	
	/*填充发送Buff*/
	txBuffLenth = VCAN_DT_Wave_Data_Fill_TxBuff(VCAN_WAVE_DATA_S32, 8, (void*)&sendSinsData, vcan_user_data_to_send);
	
	/*发送*/	
	VCAN_DT_Send_Data(vcan_user_data_to_send , txBuffLenth, MSP_UART_DMA);
}

/*3.发送PID控制系统数据到上位机*/
void user_VCAN_Send_Pid_Link_Data(SendPIDPara sendPidPara)
{
	u16 txBuffLenth;
	
	/*填充发送Buff*/
	txBuffLenth = VCAN_DT_Wave_Data_Fill_TxBuff(VCAN_WAVE_DATA_S32, sendPidPara.waveChNbr, (void*)&sendPidPara, vcan_user_data_to_send);
	
	/*发送*/	
	VCAN_DT_Send_Data(vcan_user_data_to_send , txBuffLenth, MSP_UART_DMA);
}

/*4.发送光流控制数据到上位机*/
void user_VCAN_Send_OpticFlow_Ctrl_Data(SendOpticFlowCtrlData sendOpticFlowCtrlData)
{
	u16 txBuffLenth;
	
	/*填充发送Buff*/
	txBuffLenth = VCAN_DT_Wave_Data_Fill_TxBuff(VCAN_WAVE_DATA_S32, 8, (void*)&sendOpticFlowCtrlData, vcan_user_data_to_send);
	
	/*发送*/	
	VCAN_DT_Send_Data(vcan_user_data_to_send , txBuffLenth, MSP_UART_DMA);	
}
