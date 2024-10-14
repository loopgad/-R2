#ifndef __NAMESPACE_H
#define __NAMESPACE_H

#include <cstdint>
/*
命名空间参照以下格式
namespace ***_Namespace {
	inline int ***_Variable_name = 0; //inline关键字保证该变量只初始化一次
    
}

注意该文件只存放需要跨文件访问的变量，
在类内需要写一个Read_Global函数用于读取消息并分配给成员函数使用
*/


namespace ROS_Namespace{
	inline float Robot_Relative_x;
	inline float Robot_Relative_y;
	inline float Robot_Relative_Vx;
	inline float Robot_Relative_Vy;
}

namespace Motor_Namespace {
	typedef struct MOTO_REAL_INFO
	{
	// 电机模式
		//uint32_t unitMode;//电机模式
		// POSITION_CONTROL_MODE 位置模式
		// POSITION_TARQUE_CONTROL_MODE 位置_力矩模式
	    // SPEED_TARQUE_CONTROL_MODE 速度_力矩模式
		// SPEED_CONTROL_MODE 速度模式
		// MOTO_OFF 电机关机 --> 电流不发送
	    // VELOCITY_PLANNING_MODE 速度规划模式
	//
		//MotorType_TypeDef type; // 电机类型：m3508, m2006
		uint16_t ANGLE;   		// 采样角度						
		int16_t  RPM;			// 速度值			
		int16_t  CURRENT;       // 电流值
		int16_t  TARGET_CURRENT; // 目标电流值
		int16_t  TARGET_POS;     // 目标角度
		float    TARGET_RPM;     // 目标转速
		bool      Velflag;        // 速度为零时，置1 
	// 结构体


	// 角度积分时用到下面的变量
		float		 REAL_ANGLE;              // 处理过的真实角度（必须用float）
		uint8_t	 FIRST_ANGLE_INTEGRAL_FLAG; // 第一次角度积分标志
		uint16_t LAST_ANGLE;                 // 上一次角度
		int16_t filter_RPM;                  // 滤波后的速度值
	} MOTO_REAL_INFO;

	inline MOTO_REAL_INFO MOTOR_REAL_INFO[8];
}

namespace Remote_Namespace {
	inline uint16_t PPM_Databuf[10]={0};//只用到前8个通道
	inline uint16_t &SWA = PPM_Databuf[7];
	inline uint16_t &SWB = PPM_Databuf[6];
	inline uint16_t &SWC = PPM_Databuf[5];
	inline uint16_t &SWD = PPM_Databuf[4];
	inline uint16_t &ROCK_R_X = PPM_Databuf[3];
	inline uint16_t &ROCK_R_Y = PPM_Databuf[2];
	inline uint16_t &ROCK_L_Y = PPM_Databuf[1];
	inline uint16_t &ROCK_L_X = PPM_Databuf[0];
    

}

namespace ROBOT_Namespace {
	typedef struct ROBOT_REAL_POS
	{
  		float POS_X;
  		float POS_Y;     
  		float POS_YAW;
  		int robot_location;
	
	}ROBOT_REAL_POS;


	
	typedef struct ROBOT_CHASSIS
	{
		float World_V[3]; // Y , X , W
		float Robot_V[3]; // Y , X , W
	//float Position[2];
	//float Motor_RPM[4];
		float expect_angle ;
		float Angle;
		bool flag;//标志位
	
	} ROBOT_CHASSIS;

    inline ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0, 0, 0};
	inline ROBOT_CHASSIS Robot_Chassis;
	
}
namespace Action_Namespace {
	typedef struct ACTION_GL_POS//action����
	{
		float POS_X;
		float POS_Y;
		float YAW;
		float W_Z;

		float LAST_POS_X;
		float LAST_POS_Y;
		float LAST_YAW;

		float DELTA_POS_X;
		float DELTA_POS_Y;
		float DELTA_YAW;	
	
		float REAL_X;
		float REAL_Y;
		float REAL_YAW;
	
		float OFFSET_YAW;
	} ACTION_GL_POS;

	inline volatile float action_Data[6];
	inline volatile ACTION_GL_POS ACTION_GL_POS_DATA;


}

namespace ROS_Namespace {

    

}


#endif