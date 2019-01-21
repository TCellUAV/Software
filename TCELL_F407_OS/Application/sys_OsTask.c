#include "sys_OsTask.h"

/*安全操作*/
#include "safe_Operation.h"

/*线程唤醒计数器*/
rtos_thread_wake_up gs_rtos_thread_wake_up = 
{
	/*基础外设*/
	.base_module       = 1,
	/*传感器数据获取及姿态融合*/		
	.euler_angle       = 1,
	.ver_fusion        = 1,
	.gps_hor_fusion    = 1,
	.opflow_hor_fusion = 1,
	/*控制系统*/	
	.uav_ctrl   	   = 1,
	/*校准系统*/	
	.uav_calib 		   = 1,
	/*飞行相关*/	
	.tfsd_fly_log 	   = 1,
	/*人机交互*/	
	.hci_oled_show 	   = 1,
	.hci_slave_host    = 1,	
	/*任务调度状态*/	
	.task_status_check = 1,
};

#define RTOS_UPDATE_TOTAL_THREAD_NUMBER		(01 + 01 + 06 + 01 + 01 + 01 + 02 + 01 = 14)
/*0********** 基础模块控制线程堆栈分配(01/01) ***********/
/*1*************** 参数读写系统(01/01) ******************/
/*2****** 传感器数据获取及处理线程堆栈分配(06/06) *******/
/*3************ 飞控控制线程堆栈分配(01/01) *************/
/*4************ 系统校准线程堆栈分配(01/01) *************/
/*5************ 飞行日志线程堆栈分配(01/01) *************/
/*6************ 人机交互线程堆栈分配(02/02) *************/
/*7************ 调度检测线程堆栈分配(01/01) *************/

/*==================== 线程堆栈分配及调度函数初始化 ====================*/
/*0********** 基础模块控制线程堆栈分配(01/01) ***********/
/*0.基础模块*/
#define RTOS_THREAD_STACK_BASE_MODULE			  (2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_base_module_stack[RTOS_THREAD_STACK_BASE_MODULE];
static struct rt_thread base_module_thread;	

void rt_entry_thread_base_module(void* parameter)
{	
	while(1)
	{
		/*等待信号量被释放*/
		rt_sem_take(&base_module_sem, RT_WAITING_FOREVER);

		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->BaseModule));		
		
		/*LED*/
		#ifdef HW_CUT__USE_LED
		thread_led_ctrl(parameter);
		#endif
		
		/*KEY*/
		#ifdef HW_CUT__USE_KEY  
		thread_key_check(parameter);
		#endif
		
		/*RGB*/
		#ifdef HW_CUT__USE_RGB
		bsp_rgb_work_list_show(&g_sRgb);
		#endif		
		
		/*BEEP*/
		#ifdef HW_CUT__USE_BEEP
	
		#endif		
	}
}                                     

/*1*************** 参数读写系统(01/01) ******************/
/* 0.参数读写 */
#define RTOS_THREAD_STACK_SIZE_STOR_PARA_RDWR 	  (2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_stor_para_rdwr_stack[RTOS_THREAD_STACK_SIZE_STOR_PARA_RDWR];
static struct rt_thread stor_para_rdwr_thread;

void rt_entry_thread_stor_para_rdwr(void* parameter)
{
	rt_uint32_t event;
	
	while(1)
	{	
		/*flash rdwr*/		
		#ifdef STOR_MCU__FLASH
		#endif
		
		/*eeprom rdwr*/
		#ifdef STOR_BOARD__AT24CXX
		/*PID参数保存或重置(在线PID调参)*/
		if (rt_event_recv(&para_rdwr_event, ((1 << RTOS_EVENT_PID_PARA_UPDATE_BY_HOST)),
						  RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                          5, &event) == RT_EOK)
		{
			pid_parameter_save_or_reset(&g_sPidSystem);
		}
		#endif
	}
}	

/*2****** 传感器数据获取及处理线程堆栈分配(04/04) *******/
/* 0.加速度计和陀螺仪(IMU)+磁力计(MAG)数据获取并计算欧拉角 */
#define RTOS_THREAD_STACK_SIZE_EULER_ANGLE_CALCULATE	      (2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_euler_angle_calculate_stack[RTOS_THREAD_STACK_SIZE_EULER_ANGLE_CALCULATE];
static struct rt_thread euler_angle_calculate_thread;

void rt_entry_thread_euler_angle_calculate(void* parameter)
{
	while(1) 
	{
		/*等待信号量被释放*/
		rt_sem_take(&euler_angle_sem, RT_WAITING_FOREVER);
		
		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->EulerAngle));
		
		/*IMU数据获取及处理*/
		#ifdef HW_CUT__USE_MD_IMU	/*MD IMU*/	
		ahrs_imu_data_get_and_dp();
		#endif
		
		#ifdef HW_CUT__USE_BD_IMU	/*BD IMU*/	
		ahrs_imu_data_get_and_dp();
		#endif
		
		/*MAG数据获取及处理*/
		ahrs_mag_data_get_and_dp();
		
		/*欧拉角计算: 96us*/
		ahrs_grades_calculat_attitude(g_psAhrsQuater, \
								  	  g_psAccAttitude, \
								      g_psGyroAttitude, \
									  g_psGyroBwLP, \
								      g_psMagAttitude->magYaw, \
								      g_psAhrsAttitude);																		 
			
		/*更新方向余弦矩阵*/
		ahrs_compute_rotation_matrix();
	}
}

/* 1.竖直传感器数据获取及数据融合 */
#define RTOS_THREAD_STACK_SIZE_VERTICAL_DATA_FUSION   (2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_vertical_fusion_stack[RTOS_THREAD_STACK_SIZE_VERTICAL_DATA_FUSION];
static struct rt_thread vertical_fusion_thread;

void rt_entry_thread_vertical_fusion(void* parameter)
{
	while(1)
	{
		/*等待信号量被释放*/
		rt_sem_take(&ver_fusion_sem, RT_WAITING_FOREVER);
		
		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->VerFusion));
		
		/*气压计数据更新及处理*/
		#ifdef HW_CUT__USE_MD_BERO
		#ifdef HW_CUT__USE_DB_BERO
		bero_altitude_data_get_and_dp(g_psUav_Status);
		#else
		bero_altitude_data_get_and_dp(g_psUav_Status);
		#endif
		#endif
		
		/*超声波数据更新及处理*/		
		#if defined(HW_CUT__USE_ULTR)
		ultr_altitude_data_get_and_dp(g_psUav_Status);
		#endif
		
		/*机体系->导航系 加速度*/
		sins_get_body_relative_earth_acc(g_psAccSINS);
		
		/*竖直零参考点已经设定,且竖直传感器可用,即可进行数据融合*/
		if ((g_sUav_Status.UavSenmodStatus.Vertical.Ultr.DATA_STATUS == UAV_SENMOD_DATA_OK) || \
			(g_sUav_Status.UavSenmodStatus.Vertical.Bero.DATA_STATUS == UAV_SENMOD_DATA_OK))
		{
			#ifdef SINS_DATA_FUSION__VER_THIRDORDER
			/*三阶互补求竖直方向上的加速度、速度、位置(Z竖直)*/
			sins_thirdorder_complement_vertical();
			#endif
			
			#ifdef SINS_DATA_FUSION__VER_KALMAN
			/*卡尔曼滤波求竖直方向上的加速度、速度、位置(Z竖直)*/
			sins_kalman_estimate_vertical();
			#endif
		}	
	}
}

/* 2.获取GPS控制数据*/
#define RTOS_THREAD_STACK_SIZE_GPS_CTRL_DATA_GET	   	(2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_gps_ctrl_data_get_stack[RTOS_THREAD_STACK_SIZE_GPS_CTRL_DATA_GET];
static struct rt_thread gps_ctrl_data_get_thread;	

void rt_entry_gps_ctrl_data_get(void* parameter)
{
	while(1)
	{
		/*等待信号量被释放*/
		rt_sem_take(&gps_update_sem, RT_WAITING_FOREVER);	

		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->GpsDataGet));
		
		/*获取gps控制数据*/
		gps_fix_position_data_get(g_sGpsM8N.PvtData, &g_sAttitudeAll.GpsData);	
	}
}

/* 3.获取OPTIC FLOW控制数据*/
#define RTOS_THREAD_STACK_SIZE_OPTICFLOW_CTRL_DATA_GET   	(2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_opticflow_ctrl_data_get_stack[RTOS_THREAD_STACK_SIZE_OPTICFLOW_CTRL_DATA_GET];
static struct rt_thread opticflow_ctrl_data_get_thread;	

void rt_entry_opticflow_ctrl_data_get(void* parameter)
{
	while(1)
	{
		/*等待信号量被释放*/
		rt_sem_take(&opflow_update_sem, RT_WAITING_FOREVER);	

		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->OpFlowDataGet));		
		
		/*获取opticflow控制数据*/
		opflow_Offset_Relative_To_Home(g_sOpFlowUpixelsLC306.OpFlowData, \
									   g_psSinsReal->curPosition[EARTH_FRAME_Z], \
									   &g_sAttitudeAll);
	}
}

/* 4.GPS水平数据获取及数据融合 */
#define RTOS_THREAD_STACK_SIZE_GPS_DATA_HORIZONTAL_FUSION   	(2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_gps_data_horizontal_fusion_stack[RTOS_THREAD_STACK_SIZE_GPS_DATA_HORIZONTAL_FUSION];
static struct rt_thread gps_data_horizontal_fusion_thread;	

void rt_entry_gps_data_horizontal_fusion(void* parameter)
{
	while(1)
	{
		/*等待信号量被释放*/
		rt_sem_take(&gps_hor_fusion_sem, RT_WAITING_FOREVER);	

		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->GpsHorFusion));
		
		#ifdef HW_CUT__USE_GPS	/*GPS*/
		/*判断HOME是否已经设定(每次上电且GPS可用只设定一次)*/
		gps_home_location_set();
		#endif
		
		/*机体系->导航系 加速度*/
		sins_get_body_relative_earth_acc(g_psAccSINS);		
		
		/*HOME已经设定,即可进行数据融合*/
		if (g_sUav_Status.HOME_SET_STATUS == UAV_HOME_SET_YES)
		{
			#ifdef SINS_DATA_FUSION__HOR_THIRDORDER
			/*水平X、Y方向:捷联惯导(三阶互补)融合数据*/
			sins_thirdorder_complement_horizontal();
			#endif
			
			#ifdef SINS_DATA_FUSION__HOR_KALMAN
			/*卡尔曼滤波求水平方向上的加速度、速度、位置(X,Y水平)*/
			sins_kalman_estimate_horizontal();
			#endif
		}		
	}
}

/* 5.OPTIC FLOW水平数据获取及数据融合 */
#define RTOS_THREAD_STACK_SIZE_OPTICFLOW_DATA_HORIZONTAL_FUSION   	(2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_opticflow_data_horizontal_fusion_stack[RTOS_THREAD_STACK_SIZE_OPTICFLOW_DATA_HORIZONTAL_FUSION];
static struct rt_thread opticflow_data_horizontal_fusion_thread;

void rt_entry_opticflow_data_horizontal_fusion(void* parameter)
{
	while(1)
	{
		/*等待信号量被释放*/
		rt_sem_take(&opflow_hor_fusion_sem, RT_WAITING_FOREVER);

		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->OpflowHorFusion));
		
		/*对光流数据进行处理,计算出需要的速度,位移信息*/
		if (g_sUav_Status.UavSenmodStatus.Horizontal.Opticflow.DATA_STATUS == UAV_SENMOD_DATA_OK)
		{

		}
	}
}

/*3************ 飞控控制线程堆栈分配(01/01) *************/
#define RTOS_THREAD_STACK_SIZE_UAV_CTRL_SYSTEM			  (2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_uav_ctrl_system_stack[RTOS_THREAD_STACK_SIZE_UAV_CTRL_SYSTEM];
static struct rt_thread uav_ctrl_system_thread;

void rt_entry_uav_ctrl_system(void* parameter)
{
	while(1)
	{	
		/*等待信号量被释放*/
		rt_sem_take(&uav_ctrl_sem, RT_WAITING_FOREVER);		
		
		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->UavCtrl));
		
		/*更新遥控数据(必须在"检测遥控和飞行器通信是否正常"前面执行)*/
		remot_get_all_channel_data(g_psRemotData, g_psReceiverAnaly);
	
		/*检测遥控和飞行器通信是否正常(必须在 "更新遥控数据"后面执行)*/
		status_check_uav_wireless(&g_sUavRemotCMCDog, g_psUav_Status);
	
		/*检测遥控锁定状态*/
		remot_aircraft_lock_and_unlock();
		
		/*控制系统计算*/
		ctrl_auto_control_system_dp();
		
		/*控制系统输出及施加PWM到电调驱动电机*/
		ctrl_auto_control_system_output();
	}
}

/*4************ 系统校准线程堆栈分配(01/01) *************/
#define RTOS_THREAD_STACK_SIZE_UAV_CALIB_SYSTEM	 	  	  (2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_uav_calib_system_stack[RTOS_THREAD_STACK_SIZE_UAV_CALIB_SYSTEM];
static struct rt_thread uav_calib_system_thread;

void rt_entry_uav_calib_system(void* parameter)
{
	while(1)
	{	
		/*等待信号量被释放*/
		rt_sem_take(&uav_calib_sem, RT_WAITING_FOREVER);		
		
		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->UavCalib));		
		
		/*加速度计校准*/
		if ((g_sUav_Status.LOCK_STATUS == UAV_LOCK_YES) && \
			(g_sHciShowPage.SHOW_DATA_STATUS != UAV_HCI_SHOW_ENABLE))
		{
			/*获取遥控指定校准面序号*/
			g_psAccCalibSystem->CUR_SIDE_INDEX = calib_acc_sensor_check();
			
			/*有效面序才开始采样+校准*/
			if ((g_psAccCalibSystem->CUR_SIDE_INDEX != SIDE_INDEX_NULL) || \
				(g_psAccCalibSystem->ENTRY_STATUS == ENTRY_CALIB_INTO) || \
				(g_psAccCalibSystem->ENTRY_STATUS == ENTRY_CALIB_EXIT))
			{
				calib_acc_sensor_dp(g_psAccCalibSystem->CUR_SIDE_INDEX);
			}
		}
		
		/*磁力计校准*/
		if ((g_sUav_Status.LOCK_STATUS == UAV_LOCK_YES) && \
			(g_sHciShowPage.SHOW_DATA_STATUS != UAV_HCI_SHOW_ENABLE))
		{
			/*获取遥控指定校准面序号*/
			g_psMagCalibSystem->CUR_SIDE_INDEX = calib_mag_sensor_check();
			
			/*有效面序才开始采样+校准*/
			if ((g_psMagCalibSystem->CUR_SIDE_INDEX != SIDE_INDEX_NULL) || \
				(g_psMagCalibSystem->ENTRY_STATUS == ENTRY_CALIB_INTO) || \
				(g_psMagCalibSystem->ENTRY_STATUS == ENTRY_CALIB_EXIT))				
			{
				calib_mag_sensor_dp(g_psMagCalibSystem->CUR_SIDE_INDEX);
			}
		}
		
		/*电调行程校准,进入后需要拔掉电池后进入*/
		if (1){}
		
		/*水平角度校准*/
		if (1){}
			
		/*遥控行程校准*/
		if (1){}
	}
}

/*5************ 飞行日志线程堆栈分配(01/01) *************/
#define RTOS_THREAD_STACK_SIZE_TFSD_FLY_LOG	 	  (2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_tfsd_fly_log_stack[RTOS_THREAD_STACK_SIZE_TFSD_FLY_LOG];
static struct rt_thread tfsd_fly_log_thread;

void rt_entry_thread_tfsd_fly_log(void* parameter)
{	
	while(1)
	{
		/*等待信号量被释放*/
		rt_sem_take(&tfsd_fly_log_sem, RT_WAITING_FOREVER);
		
		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->FlyLog));		
		
		/*更新CPU使用率*/
		g_sUav_Status.UavProgrameStatus.CpuUse.major = cpu_usage_major; /*CPU 使用率 整数*/
		g_sUav_Status.UavProgrameStatus.CpuUse.minor = cpu_usage_minor; /*CPU 使用率 小数*/	
	}
}

/*6************ 人机交互线程堆栈分配(02/02) *************/
/* OLED地面站显示 */
#define RTOS_THREAD_STACK_SIZE_HCI_OLED_SHOW 	  (2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_hci_oled_show_stack[RTOS_THREAD_STACK_SIZE_HCI_OLED_SHOW];
static struct rt_thread hci_oled_show_thread;

void rt_entry_thread_hci_oled_show(void* parameter)
{
	while(1)
	{
		/*等待信号量被释放*/
		rt_sem_take(&hci_oled_show, RT_WAITING_FOREVER);		
		
		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->HciOledShow));
		
		/*获取遥控控制OLED显示*/
		hci_remot_switch_show_status(&g_sHciShowPage);
		
		/*OLED显示数据*/
		hci_show_on_run_progress();
	}
}

/* 下位机和上位机交互 */
#define RTOS_THREAD_STACK_SIZE_HCI_SLAVE_AND_HOST 	  (2048)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_hci_slave_and_host_stack[RTOS_THREAD_STACK_SIZE_HCI_SLAVE_AND_HOST];
static struct rt_thread hci_slave_and_host_thread;

void rt_entry_thread_hci_slave_and_host(void* parameter)
{
	while(1)
	{
		/*等待信号量被释放*/
		rt_sem_take(&hci_host_slave_sem, RT_WAITING_FOREVER);		
		
		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->HciHostSlave));
		
		/*上传数据至上位机*/
		if (g_psPidSystem->PidSettingSystem.PARA_DOWNLOAD_STATUS != PID_PARA_DOWNLOAD_START)
		{
			if ((g_sUserSendHostSystem.HOST_TARG == USER_HOST_CHOOSE_ANO) && \
				(g_sUserSendHostSystem.MSG_ID == USER_ANO_MSG_EXCHANGE))									 
			{
				ANO_DT_Data_Exchange();																		 /*匿名上位机: 数据交换*/
			}
			else if ((g_sUserSendHostSystem.HOST_TARG == USER_HOST_CHOOSE_ANO) && \
					 (g_sUserSendHostSystem.MSG_ID != USER_ANO_MSG_EXCHANGE))
			{
				user_ANO_Send_Host_Wave_Data(g_sUserSendHostSystem.MSG_ID, g_sUserSendHostSystem.period_5MS);   /*匿名上位机: 上传波形数据*/
			}
			else if ((g_sUserSendHostSystem.HOST_TARG == USER_HOST_CHOOSE_VCAN) && \
					 (g_sUserSendHostSystem.MSG_ID != USER_ANO_MSG_EXCHANGE))
			{
				user_VCAN_Send_Host_Wave_Data(g_sUserSendHostSystem.MSG_ID, g_sUserSendHostSystem.period_5MS);  /*山外上位机: 上传波形数据*/			
			}
		}	
	}
}

/*7************ 调度检测线程堆栈分配(01/01) *************/
/* 任务调度状态 */
#define RTOS_THREAD_STACK_SIZE_TASK_STATUS_CHECK 	  (1024)
ALIGN(RT_ALIGN_SIZE)

static u8 thread_task_status_check_stack[RTOS_THREAD_STACK_SIZE_TASK_STATUS_CHECK];
static struct rt_thread task_status_check_thread;

void rt_entry_thread_task_status_check(void* parameter)
{
	while(1)
	{
		/*等待信号量被释放*/
		rt_sem_take(&task_status_check_sem, RT_WAITING_FOREVER);		
		
		/*线程执行周期计算*/
		get_Period_Execute_Time_Info(&(g_psSystemPeriodExecuteTime->TaskStatusCheck));
		
		/*安全:判断任务调度是否正常,及故障处理*/
		safe_task_status_check(&gs_SafeOperation);
		
		/*OLED显示数据时禁止解锁*/
		#if (SAFE_FORBID_UNLOCK_WHEN_OLED_SHOW == SYS_ENABLE)
		safe_watch_oled_uav_auto_lock(&g_sUav_Status, &g_sHciShowPage);
		#endif
	}
}

/*======================= 线程任务创建及开始调度 =======================*/
/*创建线程*/
void rtos_thread_create(void)
{
	rt_err_t err;
	
/*0********** 基础模块控制线程堆栈分配(01/01) ***********/
/*0.基础模块*/
	err = rt_thread_init(&base_module_thread,
                         "rt_base_module",
						 rt_entry_thread_base_module,
                         RT_NULL,
                         thread_base_module_stack,
                         sizeof(thread_base_module_stack),
						 RTOS_PR_BASE_MODULE,
                         20);
	if (err == RT_EOK)
    {
        rt_thread_startup(&base_module_thread);
    }                                                                                           										                                                    												                  										     

/*1*************** 参数读写系统(01/01) ******************/
/* 0.参数读写 */
	err = rt_thread_init(&stor_para_rdwr_thread,
                         "rt_stor_para_rdwr",
						 rt_entry_thread_stor_para_rdwr,
                         RT_NULL,
                         thread_stor_para_rdwr_stack,
                         sizeof(thread_stor_para_rdwr_stack),
						 RTOS_PR_STOR_PARA_RDWR,
                         20);
	if (err == RT_EOK)
    {
        rt_thread_startup(&stor_para_rdwr_thread);
    }	
	
/*2****** 传感器数据获取及处理线程堆栈分配(06/06) *******/
/* 0.加速度计和陀螺仪(IMU)+磁力计(MAG)数据获取并计算欧拉角 */                                                
	err = rt_thread_init(&euler_angle_calculate_thread,
                         "rt_euler_angle",
						 rt_entry_thread_euler_angle_calculate,
                         RT_NULL,
                         thread_euler_angle_calculate_stack,
                         sizeof(thread_euler_angle_calculate_stack),
						 RTOS_PR_EULER_ANGLE_CALCULATE,
                         20);
	if (err == RT_EOK)
    {
        rt_thread_startup(&euler_angle_calculate_thread);
    }	
	
/* 1.竖直传感器数据获取及数据融合 */                                               
	err = rt_thread_init(&vertical_fusion_thread,
                         "rt_ver_fus",
						 rt_entry_thread_vertical_fusion,
                         RT_NULL,
                         thread_vertical_fusion_stack,
                         sizeof(thread_vertical_fusion_stack),
						 RTOS_PR_DATA_VERTICAL_FUSION,
                         20);
	if (err == RT_EOK)
    {
        rt_thread_startup(&vertical_fusion_thread);
    }
	
/* 2.获取GPS控制数据*/	
	err = rt_thread_init(&gps_ctrl_data_get_thread,
                         "rt_gps_data",
						 rt_entry_gps_ctrl_data_get,
                         RT_NULL,
                         thread_gps_ctrl_data_get_stack,
                         sizeof(thread_gps_ctrl_data_get_stack),
						 RTOS_PR_GPS_CTRL_DATA_GET,
                         20);
	if (err == RT_EOK)
    {
        rt_thread_startup(&gps_ctrl_data_get_thread);
    }	
	
/* 3.获取OPTIC FLOW控制数据*/
	err = rt_thread_init(&opticflow_ctrl_data_get_thread,
                         "rt_opflow_data",
						 rt_entry_opticflow_ctrl_data_get,
                         RT_NULL,
                         thread_opticflow_ctrl_data_get_stack,
                         sizeof(thread_opticflow_ctrl_data_get_stack),
						 RTOS_PR_OPFLOW_CTRL_DATA_GET,
                         20);
	if (err == RT_EOK)
    {
        rt_thread_startup(&opticflow_ctrl_data_get_thread);
    }		
	
/* 4.GPS水平数据获取及数据融合 */                                                
	err = rt_thread_init(&gps_data_horizontal_fusion_thread,
                         "rt_gps_hor_fus",
						 rt_entry_gps_data_horizontal_fusion,
                         RT_NULL,
                         thread_gps_data_horizontal_fusion_stack,
                         sizeof(thread_gps_data_horizontal_fusion_stack),
						 RTOS_PR_GPS_HORIZONTAL_FUSION,
                         20);
	if (err == RT_EOK)
    {
        rt_thread_startup(&gps_data_horizontal_fusion_thread);
    }
	
/* 5.OPTIC FLOW水平数据获取及数据融合 */                                                
	err = rt_thread_init(&opticflow_data_horizontal_fusion_thread,
                         "rt_opflow_hor_fus",
						 rt_entry_opticflow_data_horizontal_fusion,
                         RT_NULL,
                         thread_opticflow_data_horizontal_fusion_stack,
                         sizeof(thread_opticflow_data_horizontal_fusion_stack),
						 RTOS_PR_OPFLOW_HORIZONTAL_FUSION,
                         20);
	if (err == RT_EOK)
    {
        rt_thread_startup(&opticflow_data_horizontal_fusion_thread);
    }
	
/*3************ 飞控核心线程堆栈分配(01/01) *************/
/* 0.控制系统 */
	err = rt_thread_init(&uav_ctrl_system_thread,
						 "rt_uav_ctrl",
						 rt_entry_uav_ctrl_system,
						 RT_NULL,
						 thread_uav_ctrl_system_stack,
						 sizeof(thread_uav_ctrl_system_stack),
						 RTOS_PR_UAV_CTRL_SYSTEM,
						 20);
	if (err == RT_EOK)
    {
		rt_thread_startup(&uav_ctrl_system_thread);
	}

/*4************ 系统校准线程堆栈分配(01/01) *************/
/* 0.传感器校准 */
	err = rt_thread_init(&uav_calib_system_thread,
						 "rt_uav_calib",
						 rt_entry_uav_calib_system,
						 RT_NULL,
						 thread_uav_calib_system_stack,
						 sizeof(thread_uav_calib_system_stack),
						 RTOS_PR_UAV_CALIB_SYSTEM,
						 20);
	if (err == RT_EOK)
    {
		rt_thread_startup(&uav_calib_system_thread);
	}
	
/*5************ 飞行日志线程堆栈分配(01/01) *************/
/* 0.SD卡记录日志 */
	err = rt_thread_init(&tfsd_fly_log_thread,
						 "rt_tfsd_fly_log",
						 rt_entry_thread_tfsd_fly_log,
						 RT_NULL,
						 thread_tfsd_fly_log_stack,
						 sizeof(thread_tfsd_fly_log_stack),
						 RTOS_PR_TFSD_FLY_LOG,
						 20);
	if (err == RT_EOK)
    {
		rt_thread_startup(&tfsd_fly_log_thread);
	}
	
/*6************ 人机交互线程堆栈分配(02/02) *************/
/* 0.OLED地面站显示 */
	err = rt_thread_init(&hci_oled_show_thread,
						 "rt_hci_oled",
						 rt_entry_thread_hci_oled_show,
						 RT_NULL,
						 thread_hci_oled_show_stack,
						 sizeof(thread_hci_oled_show_stack),
						 RTOS_PR_HCI_OLED_SHOW,
						 20);
	if (err == RT_EOK)
    {
		rt_thread_startup(&hci_oled_show_thread);
	}

/* 1.下位机和上位机交互 */	
	err = rt_thread_init(&hci_slave_and_host_thread,
						 "rt_hci_hs",
						 rt_entry_thread_hci_slave_and_host,
						 RT_NULL,
						 thread_hci_slave_and_host_stack,
						 sizeof(thread_hci_slave_and_host_stack),
						 RTOS_PR_HCI_SLAVE_AND_HOST,
						 20);
	if (err == RT_EOK)
    {
		rt_thread_startup(&hci_slave_and_host_thread);
	}	
	
/*7************ 调度检测线程堆栈分配(01/01) *************/	
	err = rt_thread_init(&task_status_check_thread,
						 "rt_task_check",
						 rt_entry_thread_task_status_check,
						 RT_NULL,
						 thread_task_status_check_stack,
						 sizeof(thread_task_status_check_stack),
						 RTOS_PR_TASK_STATUS_CHECK,
						 20);
	if (err == RT_EOK)
    {
		rt_thread_startup(&task_status_check_thread);
	}	
}

/*NVIC初始化*/
void mcu_nvic_init(void)
{
	msp_NVIC_Init(NVIC_PRIO_GROUP_2);
}

/*MCU初始化*/
void mcu_driver_init(void)
{
//	nop_delay_init(); /*用于测试当前MCU的while(1--){__NOP()}用时*/
	
	sys_Mcu_Peripheral_Init();
}

/*硬件初始化*/
void hardware_init(void)
{
	bsp_BoardLib_Init(&g_sBoardStatus);
}

/*系统参数读取*/
void uav_system_init(void)
{
	/*滤波器初始化*/
	filter_origin_data_filter_init(&g_sFilterTarg);	
	
	/*传感器校准参数读取*/
	calib_all_sensor_parameter_read();
	
	/*控制算法参数初始化*/
	ctrl_autocontrol_para_init(&g_sCtrlSysStatus);
	
	/*初始四元数初始化*/
	ahrs_quaternion_init(&g_sAhrsQuater);
}

/*初始化后工作*/
void work_after_system_init(void)
{
	/*TIM定时器初始化及启动*/
	sys_mcu_tim_init();
	
	/*MCU_EXTI初始化*/
	sys_mcu_exti_init();
	
	/*使能Debug Uart Rx*/
	msp_uart_recv_data(&g_sDebugUart);
	
	/*标记系统初始化完毕*/
	g_sUav_Status.UavProgrameStatus.INIT_STATUS = UAV_PROGRAME_INIT_FINISH;
}

/*RTOS 组件初始化*/
/*********** 信号量(定时执行) ***********/	
/*=== 基础外设 ===*/
struct rt_semaphore base_module_sem;    
/*=== 传感器数据获取及姿态融合 ===*/
struct rt_semaphore euler_angle_sem;  
struct rt_semaphore ver_fusion_sem;
struct rt_semaphore gps_update_sem;
struct rt_semaphore opflow_update_sem;
struct rt_semaphore gps_hor_fusion_sem;  
struct rt_semaphore opflow_hor_fusion_sem;  
/*=== 控制系统 ===*/
struct rt_semaphore uav_ctrl_sem;

/*=== 校准系统 ===*/
struct rt_semaphore uav_calib_sem;
/*=== 飞行相关 ===*/
struct rt_semaphore tfsd_fly_log_sem;
/*=== 人机交互 ===*/
struct rt_semaphore hci_oled_show;
struct rt_semaphore hci_host_slave_sem;
/*=== 任务调度状态 ===*/
struct rt_semaphore task_status_check_sem;

/*********** 事件(被动执行) ***********/
/*=== 参数读写 ===*/
struct rt_event para_rdwr_event;

/********** 互斥锁(资源保护) ***********/
/*=== I2C通信 ===*/
//struct rt_mutex i2c_mutex;
//struct rt_mutex gps_mag_i2c_mutex;
/*=== SPI通信 ===*/
//struct rt_mutex spi_mutex;

void rtos_unit_init(void)
{	
	/*******************************信号量*****************************/	
	/*=== 基础外设 ===*/
	rt_sem_init(&base_module_sem, "base_module_sem", 0, RT_IPC_FLAG_FIFO);		

	/*=== 传感器数据获取及姿态融合 ===*/  
	rt_sem_init(&euler_angle_sem, "euler_angle_sem", 0, RT_IPC_FLAG_FIFO);	  
	rt_sem_init(&ver_fusion_sem, "ver_fusion_sem", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&gps_update_sem, "gps_update_sem", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&opflow_update_sem, "opflow_update_sem", 0, RT_IPC_FLAG_FIFO);	
	rt_sem_init(&gps_hor_fusion_sem, "gps_hor_fusion_sem", 0, RT_IPC_FLAG_FIFO);	 
	rt_sem_init(&opflow_hor_fusion_sem, "opflow_hor_fusion_sem", 0, RT_IPC_FLAG_FIFO);	

	/*=== 控制系统 ===*/
	rt_sem_init(&uav_ctrl_sem, "uav_ctrl_sem", 0, RT_IPC_FLAG_FIFO);	

	/*=== 校准系统 ===*/
	rt_sem_init(&uav_calib_sem, "uav_calib_sem", 0, RT_IPC_FLAG_FIFO);	

	/*=== 飞行相关 ===*/
	rt_sem_init(&tfsd_fly_log_sem, "tfsd_fly_log_sem", 0, RT_IPC_FLAG_FIFO);	

	/*=== 人机交互 ===*/	
	rt_sem_init(&hci_oled_show, "hci_oled_sem", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&hci_host_slave_sem, "hci_hs_sem", 0, RT_IPC_FLAG_FIFO);	
	
	/*=== 任务调度状态 ===*/
	rt_sem_init(&task_status_check_sem, "task_status_sem", 0, RT_IPC_FLAG_FIFO);	
	
	/********************************事件******************************/
	/*=== 参数读写 ===*/
	rt_event_init(&para_rdwr_event, "para_rdwr_event", RT_IPC_FLAG_FIFO);
	
	/********************************互斥锁*****************************/	
	/*=== I2C通信 ===*/	
//	rt_mutex_init(&i2c_mutex, "i2c_mutex", RT_IPC_FLAG_FIFO);
//	rt_mutex_init(&gps_mag_i2c_mutex, "gmag_i2c_mutex", RT_IPC_FLAG_FIFO);	
	
	/*=== SPI通信 ===*/
//	rt_mutex_init(&spi_mutex, "spi_mutex", RT_IPC_FLAG_FIFO);	
}

