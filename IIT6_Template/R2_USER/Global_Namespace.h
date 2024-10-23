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
	// enum {
	// 	 POSITION_CONTROL_MODE, //位置模式
	// 	 POSITION_TARQUE_CONTROL_MODE, //位置_力矩模式
	//      SPEED_TARQUE_CONTROL_MODE, //速度_力矩模式
	// 	 SPEED_CONTROL_MODE, //速度模式
	// 	 MOTO_OFF, //电机关机 --> 电流不发送
	//      VELOCITY_PLANNING_MODE //速度规划模式
	// } UnitMode;
	typedef struct MOTO_REAL_INFO
	{
	// 电机模式
		uint8_t unitMode;//电机模式
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

namespace Xbox_Namespace {

    inline uint8_t xbox_raw_data[64];  // 存储接收到的最终数据

	typedef struct XBOX_STATE
	{
		//摇杆数据
		    // 摇杆和扳机键的霍尔传感器值（16位整数）
    	uint_fast16_t joyHori_LX;      // 左摇杆水平方向
    	uint_fast16_t joyVert_LY;      // 左摇杆垂直方向
    	uint_fast16_t joyHori_RX;      // 右摇杆水平方向
    	uint_fast16_t joyVert_RY;      // 右摇杆垂直方向
    	uint_fast16_t trigLT;        // 左扳机键
    	uint_fast16_t trigRT;        // 右扳机键
		
		//按键状态
  		bool btnA_State;
		bool btnB_State;
		bool btnX_State;
		bool btnY_State;
		bool btnRB_State;
		bool btnLB_State;
		//速度档位
		uint_fast8_t Speed_Threshold;
		//切换底盘解算
		uint_fast8_t Base_Mode;
	
	}XBOX_STATE;

	inline XBOX_STATE Xbox_State_Info;

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
	typedef struct ACTION_GL_POS { // 动作全局位置结构体
    float POS_X;               // 当前X坐标
    float POS_Y;               // 当前Y坐标
    float YAW;                 // 当前偏航角度（绕Z轴旋转的角度）
    float W_Z;                 // W轴在Z方向上的分量

    float LAST_POS_X;          // 上一次的X坐标
    float LAST_POS_Y;          // 上一次的Y坐标
    float LAST_YAW;            // 上一次的偏航角度

    float DELTA_POS_X;         // X坐标的变化量（当前位置 - 上一次位置）
    float DELTA_POS_Y;         // Y坐标的变化量
    float DELTA_YAW;           // 偏航角度的变化量

    float REAL_X;              // 实际的X坐标，可能用于与虚拟坐标系的转换
    float REAL_Y;              // 实际的Y坐标
    float REAL_YAW;            // 实际的偏航角度

    float OFFSET_YAW;          // 偏航角度的偏移量
} ACTION_GL_POS;

	inline volatile float action_Data[6];
	inline volatile ACTION_GL_POS ACTION_GL_POS_DATA;


}



#endif