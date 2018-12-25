#ifndef _AND_DT_H_
#define	_AND_DT_H_

#include "sys_Platform.h"
#include "msp_Uart.h"

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、fp32等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

typedef struct 
{
	vu8 send_version;
	vu8 send_status;
	vu8 send_senser;
	vu8 send_pid1;
	vu8 send_pid2;
	vu8 send_pid3;
	vu8 send_pid4;
	vu8 send_pid5;
	vu8 send_pid6;
	vu8 send_rcdata;
	vu8 send_offset;
	vu8 send_motopwm;
	vu8 send_power;
	
	/*以下为用户补充*/
	vu8 send_height;
	vu8 send_gps;
	vu8 send_xyzspeed;
}dt_flag_t;

/*=== 匿名上位机官方协议 ===*/
/*发送:1.Euler角,加速度计,陀螺仪,磁力计值; 2.发送遥控解析值(1000~2000); 3.电机PWM值; 4.电池电压值; 5.下位机PID值*/
void ANO_DT_Data_Exchange(void);

/*上传硬件版本信息,软件版本信息,协议版本信息,启动引导信息,*/
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);

/*上传欧拉角等姿态信息*/
void ANO_DT_Send_Status(fp32 angle_rol, fp32 angle_pit, fp32 angle_yaw, s32 alt, u8 fly_model, u8 armed);

/*上传惯导传感器数据*/
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar);

/*上传遥控接收原始值数据*/
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);

/*上传电量信息*/
void ANO_DT_Send_Power(u16 votage, u16 current);

/*上传电机PWM信息*/
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);

/*上传PID参数*/
void ANO_DT_Send_PID(u8 group,fp32 p1_p,fp32 p1_i,fp32 p1_d,fp32 p2_p,fp32 p2_i,fp32 p2_d,fp32 p3_p,fp32 p3_i,fp32 p3_d);

/*下位机应答*/
void ANO_DT_Send_Check(u8 head, u8 check_sum);

/*=== 匿名上位机用户补充协议 ===*/
/*上传气压计和超声波测量高度*/
void ANO_DT_Send_Height(s32 altBaro, u16 altUltr);

/*上传GPS信息*/
void ANO_DT_Send_GPS(u8 FIX_TYPE, u8 SateNbr, s32  longitudeE7, s32  latitudeE7, s16 GpsYaw);

/*上传X,Y,Z轴向速度信息*/
void ANO_DT_Send_XYZ_Speed(s16 speedRoll, s16 speedPitch, s16 speedYaw);


/*=== 下位机数据上传函数 ===*/
/*下位机上传上位机*/
void ANO_DT_Send_Data(u8 *dataToSend , u8 length, MSP_UART_WORK_MODE UART_TX_MODE);

extern dt_flag_t f;

#endif

