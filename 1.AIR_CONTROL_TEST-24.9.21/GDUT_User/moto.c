/**
  ******************************************************************************
  * @file    moto.c
  * @author  梁立言
  * @version V1.1.0
  * @date    2023/3/29
  * @brief   
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "moto.h"
#include "math.h"
#include "calculation.h"
#include "tim.h"
#include "HareWare.h"

/* Private  variables ---------------------------------------------------------*/
MOTO_REAL_INFO MOTOR_REAL_INFO[8] = {0}; // 1-4分别对应顺时针方向的底盘电机
PID MOTOR_PID_RPM[8];	//速度pid信息 1-4底盘电机 5-6发射电机
PID MOTOR_PID_POS[8];	//位置pid信息

PID yaw_pid;			//偏航角控制PID																							//111111111111


extern ROBOT_CHASSIS Robot_Chassis;


MOTOR_RPM MOTOR_TARGET_RPM;
MOTOR_POS MOTOR_TARGET_POS;


/**
  * @brief  M3508初始化
	* @param  None
	* @retval None
  * @attention
  *先将所有电机初始化成3508的样子
  */
void M3508_Motor_Init(void)
{
	//数度环pid
	//PID *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone
	
	PID_parameter_init(&MOTOR_PID_RPM[0], 12.0f, 0.4f, 0.1f, 10000, 10000, -0.5);
	PID_parameter_init(&MOTOR_PID_RPM[1], 12.0f, 0.4f, 0.1f, 10000, 10000, -0.5);
	PID_parameter_init(&MOTOR_PID_RPM[2], 12.0f, 0.4f, 0.1f, 10000, 10000, -0.5);
	// 发射电机PID初始化
	PID_parameter_init(&MOTOR_PID_RPM[3], 12.0f, 1.0f, 0.1f, 10000, 10000, -0.5);
	PID_parameter_init(&MOTOR_PID_RPM[4], 12.0f, 1.0f, 0.1f, 10000, 10000, -0.5);

	
	// arm电机
	PID_parameter_init(&MOTOR_PID_RPM[5] , 12.0f, 1.0f, 0.1f, 20000, 20000, -0.5);
	//2006
	//夹爪电机
	PID_parameter_init(&MOTOR_PID_RPM[6] , 12.0f, 0.4f, 0.0f, 7000, 16384, 1);
	//发射电机
	PID_parameter_init(&MOTOR_PID_RPM[7] , 12.0f, 1.0f, 0.1f, 16384, 16384, 1);
	
	//位置环pid
	PID_parameter_init(&MOTOR_PID_POS[0] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[1] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[2] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[3] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[4] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[5] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[6] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[7] , 100, 0, 1, 7000, 7000, 0.05);
	
//	PID_parameter_init(&yaw_pid ,  0.1f,0.01f,0.02f, 1000.0f, 0.0f, 1.0f);	
	PID_parameter_init(&yaw_pid ,  0.1f,0.01f,0.02f, 3.0f, 0.0f, 1.0f);//11111111111
	
	//电机类型
	MOTOR_REAL_INFO[0].type = RM_3508;
	MOTOR_REAL_INFO[1].type = RM_3508;
	MOTOR_REAL_INFO[2].type = RM_3508;
	MOTOR_REAL_INFO[3].type = RM_3508;
	MOTOR_REAL_INFO[4].type = RM_3508;
	MOTOR_REAL_INFO[5].type = RM_3508;
	MOTOR_REAL_INFO[6].type = RM_3508;
	MOTOR_REAL_INFO[7].type = RM_3508;
	
	//电机模式
	MOTOR_REAL_INFO[0].unitMode = SPEED_CONTROL_MODE;
	MOTOR_REAL_INFO[1].unitMode = SPEED_CONTROL_MODE;
	MOTOR_REAL_INFO[2].unitMode = SPEED_CONTROL_MODE;
	MOTOR_REAL_INFO[3].unitMode = SPEED_CONTROL_MODE;
	MOTOR_REAL_INFO[4].unitMode = MOTO_OFF;
	MOTOR_REAL_INFO[5].unitMode = MOTO_OFF;
	MOTOR_REAL_INFO[6].unitMode = MOTO_OFF;
	MOTOR_REAL_INFO[7].unitMode = MOTO_OFF;

}


// 利用电机通过CAN反馈的数据更新m3508的状态信息
// 接受频率：1kHz
void m3508_update_m3508_info(CAN_RxHeaderTypeDef *msg,uint8_t	can1_RxData[8])
{
	switch(msg -> StdId)  // 检测标准ID
	{
    case M3508_CHASSIS_MOTOR_ID_1:
		{ 
			MOTOR_REAL_INFO[0].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			MOTOR_REAL_INFO[0].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			MOTOR_REAL_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_2:
		{ 
			MOTOR_REAL_INFO[1].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			MOTOR_REAL_INFO[1].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			MOTOR_REAL_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_3:
		{ 
			MOTOR_REAL_INFO[2].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			MOTOR_REAL_INFO[2].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			MOTOR_REAL_INFO[2].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;	
		

		default: 
			 break;
	}
}



//发送电流
void chassis_m3508_send_motor_currents(void)
{
	
	/***********************************用于ID为 1 2 3 4 的电机*********************************/
	CAN_TxHeaderTypeDef tx_message_1;
  uint8_t send_buf1[8] = {0};
	uint32_t msg_box1;
	// 配置控制段
	tx_message_1.IDE = CAN_ID_STD;//报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
	tx_message_1.RTR = CAN_RTR_DATA;//报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
	tx_message_1.DLC = 0x08;//数据段长度
	tx_message_1.TransmitGlobalTime = DISABLE;
	// 配置仲裁段和数据段	
	tx_message_1.StdId = 0x200;  // 用于ID为 1 2 3 4 的电机
	
	send_buf1[0] = (uint8_t)(MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
	send_buf1[1] = (uint8_t) MOTOR_REAL_INFO[0].TARGET_CURRENT;
	send_buf1[2] = (uint8_t)(MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
	send_buf1[3] = (uint8_t) MOTOR_REAL_INFO[1].TARGET_CURRENT;
	send_buf1[4] = (uint8_t)(MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
	send_buf1[5] = (uint8_t) MOTOR_REAL_INFO[2].TARGET_CURRENT;
	send_buf1[6] = (uint8_t)(MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
	send_buf1[7] = (uint8_t) MOTOR_REAL_INFO[3].TARGET_CURRENT;

	                                                                                                                                                                                          
	    if (HAL_CAN_AddTxMessage(&hcan1,&tx_message_1,send_buf1,&msg_box1)!= HAL_OK) {
        // Failed to add message to the transmit mailbox
    }		
}


void calculateWheelSpeeds(double world_vx, double world_vy, double omega, float *RPM1, float *RPM2, float *RPM3) 
	{
    *RPM1 = world_vx + L * omega*1.0;   															//顺时针为正
    *RPM2 = world_vx*0.5 - world_vy*(number/2) + L * omega*1.0;
    *RPM3 = world_vx*0.5 + world_vy*(number/2) + L * omega*1.0;
	}
	
	
/**
  * @brief  MotorCtrl电机数据计算
	* @param  None
	* @retval None
  */

void MotorCtrl(void)
{	
//   if(PPM_Connected_Flag == 1)
//  {	
//	YawAdjust(Target_Angle);
//	
//	calculateWheelSpeeds(Robot_Chassis.World_V[1], Robot_Chassis.World_V[0], Robot_Chassis.World_V[2], 
//	&MOTOR_REAL_INFO[0].TARGET_RPM, &MOTOR_REAL_INFO[2].TARGET_RPM, &MOTOR_REAL_INFO[1].TARGET_RPM) ;
//	
	for(int i = 0; i < 3; i++)
	{		
		switch(MOTOR_REAL_INFO[i].unitMode)
		{
			case POSITION_CONTROL_MODE://位置模式
				PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_PID_POS[i].output);//速度环
				PID_position_PID_calculation(&MOTOR_PID_POS[i], MOTOR_REAL_INFO[i].REAL_ANGLE, MOTOR_REAL_INFO[i].TARGET_POS);//位置环
				
				
			
				break;
			case SPEED_CONTROL_MODE://速度模式
                PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_REAL_INFO[i].TARGET_RPM);//速度环
			    
			     MOTOR_REAL_INFO[i].TARGET_CURRENT =MOTOR_PID_RPM[i].output;//电流赋值
				break;
			case HOMEINGMODE://校准模式
	      HomingMode(&MOTOR_REAL_INFO[i]);
			  PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_REAL_INFO[i].TARGET_RPM);//速度环
			  MOTOR_PID_RPM[i].output = MaxMinLimit(MOTOR_PID_RPM[i].output,MOTOR_REAL_INFO[i].homingMode.current);//限制home模式时电流值
				break;
			case VELOCITY_PLANNING_MODE://梯形模式
				VelocityPlanningMODE(&MOTOR_REAL_INFO[i]);
				PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_REAL_INFO[i].TARGET_RPM);//速度环
				break;
				
			case CURRENT_MODE://电流模式
				
				//什么都不执行，直接电流赋值
				break;
				
			case MOTO_OFF://电机关闭
				MOTOR_REAL_INFO[i].TARGET_CURRENT = 0.0f;//电流赋值
				break;
			default:break;
		}

	}
	
	chassis_m3508_send_motor_currents();
	
//  }
}

float YawAdjust_error=0;

/**
* @brief  YawAdjust偏航角控制
* @note		将偏航角控制在目标角度
* @param  Target_angle:要限制的值
*		  加入判断---> 如果使用机器人坐标系下驱动，应该更改对应赋值的对象
* @retval 成功返回1，不成功返回0
*/
int bb=0;
int YawAdjust(float Target_angle)
{
 
	 // 计算误差
   if(ROBOT_REAL_POS_DATA.POS_YAW*Target_angle >= 0)
   {
      YawAdjust_error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
   }
   else
   {
		 if(ABS(ROBOT_REAL_POS_DATA.POS_YAW)+ABS(Target_angle) <= 180) 
		 	YawAdjust_error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
		 else 
		 {		 	
		 	//YawAdjust_error = Target_angle - ROBOT_WORLD_POS_INFO.Angle;
			AngleLimit(&YawAdjust_error);
			 bb++;
		 }
   }
   
   // 直接利用PID输出角速度
   PID_position_PID_calculation_by_error(&yaw_pid, YawAdjust_error);
 
   
	   Robot_Chassis.World_V[2]= yaw_pid.output;	// 底盘角速度 单位：rad/s
   
	 if(ABS(YawAdjust_error)<0.5)
		return 1;
	 else 
	 {
		 return 0;
	 }
}	


/**
* @brief  AngleLimit角度限幅
* @note		将角度限制在-180°到180°
* @param  angle:要限制的值
* @retval 
*/

void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(*angle>180.0f)
		{
			*angle-=360.0f;
			AngleLimit(angle);
		}
		else if(*angle<-180.0f)
		{
			*angle+=360.0f;
			AngleLimit(angle);
		}
	}
	
	recursiveTimes--;
}



/**
  * @brief  速度控制
  * @param   target_vel目标速度
  * @retval 
  */
float CurrentCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_current)
{
	MOTOR_REAL_INFO->unitMode = CURRENT_MODE;
  MOTOR_REAL_INFO->TARGET_CURRENT = target_current;
	
	return 0;
}

/**
  * @brief  速度控制
  * @param   target_vel目标速度
  * @retval 
  */
float VelCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_vel)
{
	MOTOR_REAL_INFO->unitMode = SPEED_CONTROL_MODE;
  MOTOR_REAL_INFO->TARGET_RPM = target_vel;
	
	return 0;
}

/**
  * @brief  位置控制(新位置环程序)
  * @param  target_pos目标位置
  * @retval 
  */
float PosCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_pos)
{
	MOTOR_REAL_INFO->unitMode = POSITION_CONTROL_MODE;
	MOTOR_REAL_INFO->TARGET_POS = target_pos;
	if(ABS(MOTOR_REAL_INFO->TARGET_POS-target_pos)<1)return 1;	
	else return 0;

}

/**
  * @brief  Homing mode 回零模式
  * @param  None
  * @retval None
  */

void HomingMode(MOTO_REAL_INFO *MOTOR_REAL_INFO)
{
	int signvel=1.0f;
	float output;
	MOTOR_REAL_INFO->homingMode.flag=0;	
  if(MOTOR_REAL_INFO[0].homingMode.vel>=0)signvel=-1.0f;
		
	MOTOR_REAL_INFO->TARGET_RPM = MOTOR_REAL_INFO->homingMode.vel;

	
	if(fabsf(MOTOR_REAL_INFO->RPM) <=30){		//2
		MOTOR_REAL_INFO->homingMode.cnt++;
	}else{
		MOTOR_REAL_INFO->homingMode.cnt = 0;
	}
	
	if(MOTOR_REAL_INFO->homingMode.cnt >= 50)//500ms
	{														
		//MOTOR_REAL_INFO->TARGET_POS = MOTOR_REAL_INFO->REAL_ANGLE + signvel*18.0f;//往回一圈
		//清除输出
		MOTOR_REAL_INFO->homingMode.cnt = 0;
		MOTOR_REAL_INFO->REAL_ANGLE=0.0f;	
		MOTOR_REAL_INFO->homingMode.flag=1;
		MOTOR_REAL_INFO->unitMode = SPEED_CONTROL_MODE;
    MOTOR_REAL_INFO->TARGET_RPM = 0;
	}
}


/**
  * @brief  `
	* @param  target_torque目标转矩，用电流表示
	          target_pos目标位置
	* @retval none
  */
void Pos_TorqueCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,int16_t target_torque,float target_pos)
{
	
  MOTOR_REAL_INFO->unitMode = POSITION_TORQUE_MODE;
	MOTOR_REAL_INFO->pos_torquemode.Pos = target_pos;
	MOTOR_REAL_INFO->pos_torquemode.TARGET_TORQUE = target_torque;
}

/**
  * @brief  
	* @param  target_torque目标转矩，用电流表示
	          target_vel目标位置
	* @retval none
  */
void Vel_TorqueCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,int16_t target_torque,float target_Vel)
{	
  MOTOR_REAL_INFO->unitMode = SPEED_TARQUE_CONTROL_MODE;
	MOTOR_REAL_INFO->vel_torquemode.Vel = target_Vel;
	MOTOR_REAL_INFO->vel_torquemode.TARGET_TORQUE = target_torque;
}

/**
  * @brief  max min limit
	* @param  inDat:
	* @retval outDat
  */
float MaxMinLimit(float val,float limit)
{
	if(val > limit) val =  limit;
	if(val <-limit) val = -limit;
	
	return val;
}
/**
  * @brief  伪梯度规划位置控制，加入电流限位。进入限位后加入延迟
	* @param  //电机结构体  pp_po  pp2  3508_REAL_INFO
//速度微分限幅  v_xian
//目标位置 tar_position
//电流限位值  current_key
//延时  dalay_time
//进入下一个状态的标志位  next_flag
//第一次进入的标志位  first_flag,每次使用的时候先让标志位 置1
	* @retval none
叶俊雄写的，有问题找他
*/

float position_control_an_delay(PID *pp_po,PID *pp_rpm,MOTO_REAL_INFO *info,float v_xian,float tar_position,float current_key,int dalay_time,int next_flag,int first_flag)
{ 
 static int PCAD_tim,PCAD_tim2,PCAD_STOP;
 if(first_flag==1){PCAD_tim = 0;PCAD_STOP = 0;PCAD_STOP = 0;}//?*?这里内部pid还没有清零
 else
	 {
  
  if(PCAD_STOP==1){tar_position = info->REAL_ANGLE;}
  PID_position_PID_calculation(pp_po, info->REAL_ANGLE ,tar_position);
  if(pp_po->output >= v_xian)pp_po->output = v_xian;
  if(pp_po->output <= -v_xian)pp_po->output = -v_xian;
  PID_incremental_PID_calculation(pp_rpm, info->RPM ,pp_po->output);
   /* 进入电流限幅  进入循环 */
   if(info->CURRENT >= current_key   &&   info->CURRENT <= -current_key  )
   {
    if( PCAD_tim<=4){PCAD_tim++;}
    else {PCAD_tim =0;PCAD_STOP = 1;}
   }
   else {PCAD_tim =0;}
   /* 延时 */
   if(PCAD_STOP==1)
   {if(PCAD_tim2 <=dalay_time){PCAD_tim2++;}
    else {PCAD_tim2 =0;next_flag = 1;}
   }
   
  }
 return 0 ;
}

//M3508电机角度积分
void M3508AngleIntegral(MOTO_REAL_INFO *M3508_MOTOR)
{
	float delta_pos = 0;
	
	// 记录第一次进入时的数据
	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}
	
	// 计算变化的角度
	if(M3508_MOTOR->RPM >= 0)
	{
		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波
			{
				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 19;	//减速比
			}
		}
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//减速比
		}
		
		// 滤波
		if(delta_pos > 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分	
	}
	else
	{
		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波			
			{
				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos /19;	//减速比
			}
		}	
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//减速比
		}
		
		// 滤波
		if(delta_pos < 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分
	}

	// 存储角度值 
	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
}

/**
  * @brief  速度规划
	* @param  
	* @retval none
  */
//                                                   开始位置   结束位置    开始的速度(RPM 绝对值)  最大的速度	末尾的速度   加速路程的比例   减速路程的比例
void VelocityPlanningMODE(MOTO_REAL_INFO *M3508_MOTOR)	
{
	static int cnt;//记时用
	float Ssu;   //总路程
	float Sac;   //加速路程
	float Sde;   //减速路程
	float Sco;   //匀速路程
	float Aac;   //加速加速度
	float Ade;   //减速加速度
	float S;     //当前路程
	// 如果所配数据有误，则不执行速度规划		
	if((M3508_MOTOR->velocity_planning.Rac > 1) || (M3508_MOTOR->velocity_planning.Rac < 0) ||		//加速路程的比例
		 (M3508_MOTOR->velocity_planning.Rde > 1) || (M3508_MOTOR->velocity_planning.Rde < 0) ||	//减速路程的比例
		 (M3508_MOTOR->velocity_planning.Vmax < M3508_MOTOR->velocity_planning.Vstart) )			//最大的速度<开始的速度 
	{
		M3508_MOTOR->TARGET_RPM = 0;  // 令夹爪不运动
		return;
	}
	// 匀速模式
	if(M3508_MOTOR->velocity_planning.Pstart == M3508_MOTOR->velocity_planning.Pend)	//开始位置=结束位置
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vmax;	//开始的速度*最大的速度
		return;
	}
	
	// 计算一些变量
	Ssu = ABS(M3508_MOTOR->velocity_planning.Pend - M3508_MOTOR->velocity_planning.Pstart); 	//总路程   
	Sac = Ssu * M3508_MOTOR->velocity_planning.Rac;		//加速路程 =	总路程 * 加速路程的比例
	Sde = Ssu * M3508_MOTOR->velocity_planning.Rde;		//减速路程 =	总路程 * 减速路程的比例
	Sco = Ssu - Sac - Sde;		//匀速路程 = 总路程 - 加速路程 - 减速路程
	Aac = (M3508_MOTOR->velocity_planning.Vmax * M3508_MOTOR->velocity_planning.Vmax - M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vstart) / (2.0f * Sac);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
	Ade = (M3508_MOTOR->velocity_planning.Vend * M3508_MOTOR->velocity_planning.Vend -   M3508_MOTOR->velocity_planning.Vmax *   M3508_MOTOR->velocity_planning.Vmax) / (2.0f * Sde);	  
	
	// 过滤异常情况
	if(((M3508_MOTOR->velocity_planning.Pend > M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->velocity_planning.Pstart)) ||		//[(结束位置 > 开始位置) && (处理过的真实角度pos <开始位置)]	||
		 ((M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->velocity_planning.Pstart)))		//	[(结束位置 < 开始位置) && (处理过的真实角度pos >开始位置)]
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart;	//TARGET_RPM = 开始的速度
	}
	else if(((M3508_MOTOR->velocity_planning.Pend > M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->velocity_planning.Pend)) ||
		      ((M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->velocity_planning.Pend)))
	{
		M3508_MOTOR->TARGET_RPM =M3508_MOTOR->velocity_planning.Vstart;	//TARGET_RPM = 末尾的速度
	}
	else
	{
		S = ABS(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pstart);      //开始位置
		
		// 规划RPM
		if     (S < Sac)       M3508_MOTOR->TARGET_RPM = sqrt(2.0f * Aac * S + M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vstart);               // 加速阶段
		else if(S < (Sac+Sco)) M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vmax;                                                        // 匀速阶段
		else                   M3508_MOTOR->TARGET_RPM = sqrt(M3508_MOTOR->velocity_planning.Vend * M3508_MOTOR->velocity_planning.Vend - 2.0f * Ade * ABS(Ssu - S));  // 减速阶段
	}
	 
	// 分配合适的正负号
	if(M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) M3508_MOTOR->TARGET_RPM = -M3508_MOTOR->TARGET_RPM;
	//判断是否完成
	if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pend)) < 3)
		M3508_MOTOR->velocity_planning.flag = 1;//设置标志位			
						
   if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pend)) > 3)
			M3508_MOTOR->velocity_planning.flag = 0;
}

/**
  * @brief  设置速度规划的参数，开启速度规划控制
	* @param  
	* float Pstart;        //开始位置
	* float Pend;          //结束位置
	* float Vstart;        //开始的速度           // 单位：RPM 绝对值
	* float Vmax;          //最大的速度
	* float Vend;          //末尾的速度
	* float Rac;           //加速路程的比例
	* float Rde;           //减速路程的比例
	* @retval none
  */
void Velocity_Planning_setpos(MOTO_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,float Vstart,float Vmax,float Vend,float Rac,float Rde)
{
	M3508_MOTOR->unitMode = VELOCITY_PLANNING_MODE;//配置模式
	M3508_MOTOR->velocity_planning.Pstart = Pstart;
	M3508_MOTOR->velocity_planning.Pend = Pend;
	M3508_MOTOR->velocity_planning.Vstart = Vstart;
	M3508_MOTOR->velocity_planning.Vmax = Vmax;
	M3508_MOTOR->velocity_planning.Vend = Vend;
	M3508_MOTOR->velocity_planning.Rac = Rac;
	M3508_MOTOR->velocity_planning.Rde = Rde;
	
}
/**
  * @brief  低通滤波器
	* @param  input当前输入信号
  *         prev_output上一个输出信号
  *         prev_input上一个输入信号
  *         cutoff_freq截止频率
  *         sample_rate采样率
	* @retval none
  */

double filter(double input, double prev_output, double prev_input, double cutoff_freq, double sample_rate) {
    double RC = 1.0 / (2.0 *PI * cutoff_freq);
    double alpha = 1.0 / (1.0 + RC * sample_rate);
    double output = alpha * (input + prev_input) + (1 - alpha) * prev_output;
    return output;
}



