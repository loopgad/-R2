#include "Global_Namespace.h"
#pragma pack(push, 1) // 设置为1字节对齐\

//// 初始化 Xbox_Namespace 中的变量
//Xbox_Namespace::XBOX_STATE Xbox_State_Info = {}; // 使用零初始化


    Xbox_Namespace::XBOX_STATE Xbox_State_Info = {};

    Motor_Namespace::MOTO_REAL_INFO MOTOR_REAL_INFO[8] = {};

    // 初始化 ROBOT_Namespace 中的变量
    ROBOT_Namespace::ROBOT_CHASSIS Robot_Chassis = {}; // 使用零初始化
		// 初始化 Action_Namespace 中的变量
	Action_Namespace::ACTION_GL_POS ACTION_GL_POS_DATA = {};




#pragma pack(pop)


