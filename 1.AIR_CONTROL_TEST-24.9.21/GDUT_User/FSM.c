#include "FSM.h"
#include "cmsis_os.h"
#include "comunication.h"
#include "robot.h"
#include "HareWare.h"
//来自lly6/28 1：43的标记，要不然这是哪个代码我都不知道
//cb修改记录：激光加上角度判断
MOVE_STATE_ITEMS MOVE_STATE;
SHOOTING_STATE_ITEMS SHOOTING_STATE; 
ROBOT_FLAG_Type ROBOT_FLAG; 

 float V_robot_manual[3];
 float Yaw_robot_manual = 0;
 float Yaw_robot_manual_error = 0;
 int shoot_calibration_cnt=0;//
extern SHOOT_ITEMS SHOOT_STATE;
extern int move_ok;
//ARM_STATE_ITEMS ARM1_STATE;
	float move_time_counter = 0;
	
	int move_ok1=0;
	int move_1=0;//用于判断电动推杆的移动状态
	int direction=0;
	int laser_point_flag=0;//用于判断激光逼近远点，还是近点
	float POSX_TrapezoidPlaning=0;
	float POSY_TrapezoidPlaning=0;
	int test1flag=0;
	
void FSM_Init(void)
{
  MOVE_STATE=MOVE_STOP;
	SHOOTING_STATE=control_nineblock_0;
	KEY_DATA.KEY_armtop=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11);
	KEY_DATA.KEY_armbottom=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);
	KEY_DATA.KEY_push=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13);
	KEY_DATA.KEY_left_clamp=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15);
	KEY_DATA.KEY_right_clamp=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_14);
	
	//SHOOT_DATA.push_state = hold;
	armstate = 0;
	
	SHOOT_DATA.yaw=1.2;//云台yaw初始化
}
/**************************************************移动状态机*********************************************/
void move_FSM(void)
{
	
//	POSX_TrapezoidPlaning=(float)location_b;
//			 POSY_TrapezoidPlaning=(float)location_k;
	if(SWA<1500) 
	{		
		//停止
		if(SWB<1200)
		{
			move_ok=0;
			MOVE_STATE=MOVE_STOP;
		}
		
		//手动
		if(SWB>1200&&SWB<1700)
		{
			MOVE_STATE=MOVE_FREE;
			
			move_ok=1;
		}
		
		//自动
		if(SWB>1700)
		{
			move_ok=0;
			if(SWC<1200)
			{
				//到左边装弹
				if(ROCK_R_X>1950)
				{
					//更新梯形规划初始值
					POSX_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_X;
					POSY_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_Y;
					MOVE_STATE=MOVE_1_LOAD_POINT;			
				}
				//到右边装弹
				else if (ROCK_R_X<1050)
				{
					//更新梯形规划初始值
					POSX_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_X;
					POSY_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_Y;					
					MOVE_STATE=MOVE_2_LOAD_POINT	;
				}
				//重启
				else if (ROCK_R_Y>1950)
				{
					//更新梯形规划初始值
					POSX_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_X;
					POSY_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_Y;
					MOVE_STATE=MOVE_1_RESTART;
				}
				else if(ROCK_R_Y<1050)//发射
				{
					//更新梯形规划初始值
					POSX_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_X;
					POSY_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_Y;
					MOVE_STATE=MOVE_SHOOT;
				}
			}
			//改用自动模式，暂时取消
//		if (SWC>1700)//C下来
//			 {
//				 /************************手动测试*******************************/
//				 MOVE_STATE=MOVE_LOAD;
//				 /*******************************************************/
//				 
//			
//			 if(direction==1)
//			 {
//			  POSX_TrapezoidPlaning=(float)location_b;
//				POSY_TrapezoidPlaning=(float)location_k;
//			 }
//			 else if(direction==2)
//			 {
//			  POSX_TrapezoidPlaning=(float)location_k;
//				POSY_TrapezoidPlaning=(float)location_b;
//			 }
//			 else
//			 {
//				POSX_TrapezoidPlaning=(float)location_y;
//				POSY_TrapezoidPlaning=(float)location_x;
//			 }
//	
//			 }
		
		}
	}

}

/*********************************移动状态的行为*********************************************/
void move(void)
{
	
	/************************手动测试所用*****************************/
	//当角度小于-20度，direction=1；
//	if(ROBOT_REAL_POS_DATA.POS_YAW<-20)
//	{
//		direction=1;
//	}
//	else if(ROBOT_REAL_POS_DATA.POS_YAW>20)
//	{
//		direction=2;
//	}
//	else
//		direction=3;
	
	
	/****************************************************************/
	
	if(SWA<1400)
	{
	//	GUN_DATA.SHOOT_DATA_L.push_state = hold;//推环位置不动			
   switch(MOVE_STATE)
    {
			
			case MOVE_STOP:
				move_ok=0;//停止，不能动
				Robot_Chassis.World_V[0]=0;//y轴
				Robot_Chassis.World_V[1]=0;//x轴
				Robot_Chassis.World_V[2]=0;//w轴
				ROBOT_FLAG.Chassis_send=0;
			  break;
			
			////取环点
			case MOVE_1_LOAD_POINT:
//			 move_time_counter += 0.01f;
//                                                                             POS_end  YAW
					direction=1;
			 if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,-4500,600,-45,100,100,3000,0.1,0.6))
//     if(PathPlan(move_time_counter,2.8,7, X0 , Y0, Yaw0))
			{
		
			//更新激光初始值
       POSX_TrapezoidPlaning=(float)location_b;
			 POSY_TrapezoidPlaning=(float)location_k;
				move_time_counter = 0;
				armstate=1;//放下夹爪
//				if(location_b<500&&location_k<500)
//				MOVE_STATE = MOVE_LASER_NEAR;
//				else
					MOVE_STATE = MOVE_LASER;
					//MOVE_STATE =MOVE_STOP;
				
        direction=1;//表示，向左取环										
			}
				break;
			case MOVE_2_LOAD_POINT:
//			 move_time_counter += 0.01f;
                  		direction=2;                                                       
			 if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,4900,550,45,100,100,3900,0.2,0.45))
//     if(PathPlan(move_time_counter,2.8,7, X0 , Y0, Yaw0))
			{
				direction=2; 
		
       POSX_TrapezoidPlaning=(float)location_k;
			 POSY_TrapezoidPlaning=(float)location_b;
				move_time_counter = 0;
				arm_midle_flag=1;//夹爪居中模式
				armstate=1;//放下夹爪
//			if(location_b<500&&location_k<500)
//				 
//				MOVE_STATE = MOVE_LASER_NEAR;
//				else
					MOVE_STATE = MOVE_LASER;
        laser_point_flag=0;				
				
					
			}
				break;	

			case MOVE_SHOOT://任意地方到射环
				direction=3;
				if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,250,770,0,100,100,2800,0.2,0.68)
					||location_y<700)//通过激光提前退出
				//if(chassis_TrapezoidPlaning_laser(0,0,8280,80,0,100,0,200,0.3,0.6,location_y,location_x))
				{	
					direction=3;
					POSX_TrapezoidPlaning=(float)location_x;
			    POSY_TrapezoidPlaning=(float)location_y;
					MOVE_STATE = MOVE_LASER;	
					direction=3;//中间
					
					
				}
				break;
				
			case MOVE_1_RESTART:						 
				if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,0,0,0,100,100,1000,0.2,0.68))
				{	
					MOVE_STATE = MOVE_STOP;
					ROBOT_REAL_POS_DATA.robot_location=robot_start;	
				}
				break;
				
			case MOVE_FREE:

		 
				break;
				
			case MOVE_LASER:
			
			if(1)//自动
				{	
					shoot_calibration_cnt=0;
				if(direction==1)//-45,laser_point_flag等于1的时候，进行逼近近点
					{		
             				
					if(Laser_calibration_1(645, 445,-45,250,direction))//表示，往左边取环
				
						//if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,567,487,-45,200,0,600,0.3,0.6,location_b,location_k))
						{
						test1flag=1;
							//等待测试
//						ACTION_GL_POS_DATA.REAL_X=-4900.0f;
//						ACTION_GL_POS_DATA.REAL_Y=100.0f;
						MOVE_STATE = MOVE_LASER_NEAR;
							//MOVE_STATE = MOVE_STOP;
							ROBOT_REAL_POS_DATA.robot_location=robot_load_left;
						}					
					}	
    
				else if(direction==2)//45,laser_point_flag等于1的时候，进行逼近近点
					{
						 //if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,560,500,45,50,0,200,0.3,0.6,location_k,location_b))
						  if(Laser_calibration_1(630, 465,45,250,direction))//表示，往右边取环
						{
						MOVE_STATE = MOVE_LASER_NEAR;
						ROBOT_REAL_POS_DATA.robot_location=robot_load_right;	
						}				
					}
					
					
				else if(direction==3)//0
					{
						//采用PD跟踪器
					 if(Laser_calibration(5792,690,0,200 ))
						//if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,5810,690,0,100,0,300,0.5,0.5,location_x,location_y))//去发射
						{						
						MOVE_STATE = MOVE_LASER_NEAR;
						ROBOT_REAL_POS_DATA.robot_location=robot_shoot;	
						}		
					}
				else MOVE_STATE = MOVE_STOP;


				}						


			break;
				
			case MOVE_LASER_NEAR:
				
			if(direction==1)//-45,laser_point_flag等于1的时候，进行逼近近点
					{
					//零时想到的，可能有点意思
						//确实有意思，就这样搞
						//在这打个标记，防止傻狗cb搞不见了
             				
					//if(Laser_calibration_1(215,180,-45,200,direction)||(KEY_DATA.KEY_left_clamp==0&&KEY_DATA.KEY_right_clamp==0))//表示，往左边取环
					if((location_k<240&&location_b<200)||(KEY_DATA.KEY_left_clamp==0&&KEY_DATA.KEY_right_clamp==0)||SWD<1200)
					//	if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,358,499,-45,100,0,500,0.3,0.6,location_b,location_k))
						{
						Robot_Chassis.World_V[0]=0;
						Robot_Chassis.World_V[1]=0;
						Robot_Chassis.World_V[2]=0;
						test1flag=1; 
						ACTION_GL_POS_DATA.REAL_X=-4900.0f;
						ACTION_GL_POS_DATA.REAL_Y=100.0f;
						MOVE_STATE = MOVE_STOP;
						ROBOT_REAL_POS_DATA.robot_location=robot_load_left;
					
							//armstate=3;//升起夹爪
						}	
					else 
					{
					YawAdjust(-45);	
//方法一，到位后直接倒车						
					// 1. 定义一个变换矩阵世界转机器人
					float cos_theta = cos(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
					float sin_theta = sin(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
					// 2. 转换为机器人坐标
					Robot_Chassis.World_V[1] =  -(-200) * sin_theta;
					Robot_Chassis.World_V[0] =	(-200) * cos_theta;					
/*方法二，用激光辅助校准						
					//输入为两个激光的差值	
					float delta_x_laser;
					delta_x_laser=loaction_b-location_k;
					 // 直接利用PID输出速度
					PID_position_PID_calculation_by_error(&catch_ring_pid, delta_x_laser);
					// 1. 定义一个变换矩阵世界转机器人
					float cos_theta = cos(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
					float sin_theta = sin(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
					// 2. 转换为机器人坐标
					Robot_Chassis.World_V[1] = (catch_ring_pid.output) * cos_theta - (-50) * sin_theta;
					Robot_Chassis.World_V[0] = (catch_ring_pid.output) * sin_theta + (-50) * cos_theta;
*/						
						
					}
					
					}	
    
				else if(direction==2)//45,laser_point_flag等于1的时候，进行逼近近点
					{
						if((location_k<240&&location_b<200)||(KEY_DATA.KEY_left_clamp==0&&KEY_DATA.KEY_right_clamp==0)||SWD<1200)
				//		if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,231,250,45,50,0,200,0.3,0.6,location_k,location_b))
					//	if(Laser_calibration_1(225,180,45,200,direction)||(KEY_DATA.KEY_left_clamp==0&&KEY_DATA.KEY_right_clamp==0))//表示，往右边取环
						{
						Robot_Chassis.World_V[0]=0;
						Robot_Chassis.World_V[1]=0;
						Robot_Chassis.World_V[2]=0;
						MOVE_STATE = MOVE_STOP;
						ACTION_GL_POS_DATA.REAL_X=5300.0f;
						ACTION_GL_POS_DATA.REAL_Y=120.0f;
						ROBOT_REAL_POS_DATA.robot_location=robot_load_right;
						//armstate=3;//升起夹爪
						}				
					else 
						{
						YawAdjust(45);	
//方法一，到位后直接倒车						
						// 1. 定义一个变换矩阵世界转机器人
						float cos_theta = cos(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
						float sin_theta = sin(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
						// 2. 转换为机器人坐标
						Robot_Chassis.World_V[1] =  -(-200) * sin_theta;
						Robot_Chassis.World_V[0] =	(-200) * cos_theta;					
/*方法二，用激光辅助校准						
						//输入为两个激光的差值	
						float delta_x_laser;
						delta_x_laser=loaction_b-location_k;
						// 直接利用PID输出速度
						PID_position_PID_calculation_by_error(&catch_ring_pid, delta_x_laser);
						// 1. 定义一个变换矩阵世界转机器人
						float cos_theta = cos(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
						float sin_theta = sin(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
						// 2. 转换为机器人坐标
						Robot_Chassis.World_V[1] = (catch_ring_pid.output) * cos_theta - (-50) * sin_theta;
						Robot_Chassis.World_V[0] = (catch_ring_pid.output) * sin_theta + (-50) * cos_theta;
*/												
					}
					}
					
					
					else if(direction==3)//0
					{
						if(shoot_calibration_cnt<=250)
						{
							
							Robot_Chassis.World_V[0]=80;
							Robot_Chassis.World_V[1]=0;
							Robot_Chassis.World_V[2]=0;
							shoot_calibration_cnt++;
						}
						if(shoot_calibration_cnt>250)
						{
							Robot_Chassis.World_V[0]=0;
							Robot_Chassis.World_V[1]=0;
							Robot_Chassis.World_V[2]=0;
							shoot_calibration_cnt++;							
						}
//						if(shoot_calibration_cnt>300)
//						{
//						//校准
//							ACTION_GL_POS_DATA.REAL_X=189.0f;
//							ACTION_GL_POS_DATA.REAL_Y=1000.0f;
//							ROBOT_FLAG.Chassis_send=1;
//							shoot_calibration_cnt++;		
//						}
						if(shoot_calibration_cnt>500)
						{
							shoot_calibration_cnt=0;
							MOVE_STATE = MOVE_STOP;
						}
					}
					
				else MOVE_STATE = MOVE_STOP;
				break;

				case MOVE_LOAD: 
					if(ROCK_L_X>1460&&ROCK_L_X<1540)ROCK_L_X=1500;				
					V_robot_manual[1]=-(ROCK_L_X-1500)*0.1f;//vx
					V_robot_manual[0]=-50;//vy一直向后
					if(ROCK_R_X>1460&&ROCK_R_X<1540)ROCK_R_X=1500; 			
					if(1490>ROCK_R_X)Yaw_robot_manual_error-=0.02f;//vw			
					if(1510<ROCK_R_X)Yaw_robot_manual_error+=0.02f;//vw
					//确定方向
					if(direction==1)
					YawAdjust(-45.0f+Yaw_robot_manual_error);
					if(direction==2)
					YawAdjust(45.0f+Yaw_robot_manual_error); 
					// 1. 定义一个变换矩阵世界转机器人
					float cos_theta = cos(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
					float sin_theta = sin(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
					// 2. 转换为机器人坐标
					Robot_Chassis.World_V[1] = V_robot_manual[1] * cos_theta - V_robot_manual[0] * sin_theta;
					Robot_Chassis.World_V[0] = V_robot_manual[1] * sin_theta + V_robot_manual[0] * cos_theta;	
					//回零
					if(SWC<1700&&SWC>1200) //C在中间
						{
						Robot_Chassis.World_V[1]=0;
						Robot_Chassis.World_V[0]=0;
						Robot_Chassis.World_V[2]=0;
						Yaw_robot_manual_error=0;	
						MOVE_STATE = MOVE_STOP;
						}
				
				break;
			
			
			default:break;
				
   }
 }
}

/*********************************************射击状态机******************************************/
//左边的发射机构
void shoot_FSM(void)
{	

if(SWA>1600)
	{	
//		if(move_1==0)//初始化为0，电动推杆能推进
//		{
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
//			osDelay(100);//电动推杆移动的距离取决于延迟的时间。
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
//			move_1=1;//置1后，电动推杆 将不在移动
//		}
		MOVE_STATE = MOVE_STOP;//停止移动
		Robot_Chassis.World_V[1]=0;
		Robot_Chassis.World_V[0]=0;
		Robot_Chassis.World_V[2]=0;
		//校准
		ACTION_GL_POS_DATA.REAL_X=189.0f;
		ACTION_GL_POS_DATA.REAL_Y=1000.0f;
		ROBOT_FLAG.Chassis_send=1;
		get_9block();
//		SHOOTING_STATE = control_nineblock;
		move_ok = 0;
		
//	if(SWD>1600){SHOOT_DATA.tiny_yaw=SHOOT_DATA.tiny_pitch=0;}
//		if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw += 0.01f;}
//		if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw -= 0.01f;}
//		if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch += 0.01f;}
//		if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch -= 0.01f;}	
	}
			
}




/***********************************************射击状态的每个动作******************************************/
void shoot(void)
{
	if(SWA<1400)
	{
				SHOOTING_STATE=control_nineblock_5;//停止射击
				
				SHOOT_DATA.tiny_yaw = 0;
				SHOOT_DATA.tiny_pitch =0;
				VelCrl(&MOTOR_REAL_INFO[4],0);
				VelCrl(&MOTOR_REAL_INFO[3],0);
	}
	if(SWA>1600)//当SWA剥下来，才能进行射击状态
	{	pwm_key_flag=0;
		if(SWD<1300)
		{
//		__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 2000);
//		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1000);
//    osDelay(500);
			SHOOT_DATA.push_state = back;//把推环缩回来
		
		}
		
		switch(SHOOTING_STATE)
		{
			case control_nineblock_5://停止射击//清空所有数据
				SHOOT_DATA.yaw = 0;
				SHOOT_DATA.pitch = 0;
				SHOOT_DATA.tiny_yaw = 0;
				SHOOT_DATA.tiny_pitch =0;
				VelCrl(&MOTOR_REAL_INFO[4],0);
				VelCrl(&MOTOR_REAL_INFO[3],0);
			  break;
		
			case control_nineblock_1://射击近左1号柱
				//没有这个状态
				break;
				
			case control_nineblock_2://射击近中2号柱				
				SHOOT_DATA.yaw = 2.375;
				SHOOT_DATA.pitch = -0.4369;		
				VelCrl(&MOTOR_REAL_INFO[4],1500);
				VelCrl(&MOTOR_REAL_INFO[3],-1200);
		
				
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[1] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[1] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[1] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[1] -= 0.001f;}	
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[1];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[1];								
				break;
			
			case control_nineblock_3://射击近右3号柱
				SHOOT_DATA.yaw = -4.367;
				SHOOT_DATA.pitch = 0.587;	
				VelCrl(&MOTOR_REAL_INFO[4],2300);
				VelCrl(&MOTOR_REAL_INFO[3],-3100);
				
			
	
			
			
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[2] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[2] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[2] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[2] -= 0.001f;}	
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[2];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[2];					
				break;
				
			case control_nineblock_4://射击中左4号柱
							SHOOT_DATA.yaw = 3.849;
				SHOOT_DATA.pitch = 0.371;	
				VelCrl(&MOTOR_REAL_INFO[4],2700);
				VelCrl(&MOTOR_REAL_INFO[3],-3300);
		
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[5] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[5] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[5] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[5] -= 0.001f;}
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[5];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[5];	
				break;

			case control_nineblock_6://射击中右6号柱
//				SHOOT_DATA.yaw = -0.032;
//				SHOOT_DATA.pitch = 0.799;	
//				VelCrl(&MOTOR_REAL_INFO[4],3000);
//				VelCrl(&MOTOR_REAL_INFO[3],-3500);
			
			SHOOT_DATA.yaw = -0.05;
				SHOOT_DATA.pitch = 0.453;	
				VelCrl(&MOTOR_REAL_INFO[4],2700);
				VelCrl(&MOTOR_REAL_INFO[3],-3300);
		
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[5] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[5] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[5] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[5] -= 0.001f;}
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[5];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[5];				
				break;

			case control_nineblock_7://远左柱子				
				SHOOT_DATA.yaw = 2.838;
				SHOOT_DATA.pitch = -0.635;		
				VelCrl(&MOTOR_REAL_INFO[4],3200);
				VelCrl(&MOTOR_REAL_INFO[3],-3500);
			
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[6] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[6] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[6] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[6] -= 0.001f;}
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[6];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[6];				
				break;

			case control_nineblock_8://射击8最高号柱				
//				SHOOT_DATA.yaw = 1.853;
//				SHOOT_DATA.pitch = -0.394;	
//				VelCrl(&MOTOR_REAL_INFO[4],3300);
//				VelCrl(&MOTOR_REAL_INFO[3],-3800);
//			
			
				SHOOT_DATA.yaw =1.805;
				SHOOT_DATA.pitch = 0.12;	
							VelCrl(&MOTOR_REAL_INFO[4],3500);
				VelCrl(&MOTOR_REAL_INFO[3],-4300);
			
			
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[7] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[7] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[7] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[7] -= 0.001f;}
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[7];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[7];				
				break;

			case control_nineblock_9://远右柱子
//				SHOOT_DATA.yaw = 0.712;//0.3
//				SHOOT_DATA.pitch = 0.806;//0.389		
//				VelCrl(&MOTOR_REAL_INFO[4],3500);
//				VelCrl(&MOTOR_REAL_INFO[3],-4000);
			/*空调冷气*/
//							SHOOT_DATA.yaw = 0.625;//0.3 0.611
//				SHOOT_DATA.pitch = 0.397;//0.389		 0775
//				VelCrl(&MOTOR_REAL_INFO[4],3300);
//				VelCrl(&MOTOR_REAL_INFO[3],-4100);
			
			SHOOT_DATA.yaw = 0.602;//0.3 0.611
				SHOOT_DATA.pitch = 0.655;//0.389		 0775
			VelCrl(&MOTOR_REAL_INFO[4],3500);
				VelCrl(&MOTOR_REAL_INFO[3],-4300);
			
//					SHOOT_DATA.yaw = 0.611;//0.3 0.611
//				SHOOT_DATA.pitch = -0.1899;//0.389		 0775
//				VelCrl(&MOTOR_REAL_INFO[4],3200);
//				VelCrl(&MOTOR_REAL_INFO[3],-3500);
			
//						SHOOT_DATA.yaw = 0.628;//0.3
//				SHOOT_DATA.pitch = 0.1;//0.389		
//				VelCrl(&MOTOR_REAL_INFO[4],4500);
//				VelCrl(&MOTOR_REAL_INFO[3],-4000);
//			
			
			
//					SHOOT_DATA.yaw = 0.601;//0.3
//				SHOOT_DATA.pitch = 0.362;//0.389		
//				VelCrl(&MOTOR_REAL_INFO[4],3200);
//				VelCrl(&MOTOR_REAL_INFO[3],-4000);
			
			
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[8] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[8] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[8] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[8] -= 0.001f;}
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[8];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[8];				
				break;
				
			case control_nineblock_0:
				//初始状态，什么都不执行
				break;
			default:break;
		}
 }

}
