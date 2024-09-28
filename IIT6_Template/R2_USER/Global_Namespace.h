#include <FreeRTOS.h>

/*
命名空间参照以下格式
namespace ***_Namespace {
	inline int ***_Variable_name = 0; //inline关键字保证该变量只初始化一次
    
}

注意该文件只存放需要跨文件访问的变量，
在类内需要写一个Read_Global函数用于读取消息并分配给成员函数使用
*/

typedef struct PID 
{
	float  Proportion;         // 比例常数
	float  Integral;           // 积分常数
	float  Derivative;         // 微分常数
	float  Penu_Error;          // 倒数第二次误差  
	float  Prev_Error;          // 上一次误差  
	float  Error;              // 当前误差  
	float  Delta_Error;             //误差变化率  
	float  Sum_Error;           // 误差积累和  
	float  Integral_max;        // 积分上限  
	float  output;             //  输出值 
	float  output_max;          //  输出上限
	float  error_max;           //  误差上限 
	bool first_flag;        //  首次运行标志位
	float  deadzone;           //  死区 
}PID;


namespace Motor_Namespace {

    

}

namespace Remote_Namespace {

    

}

namespace Action_Namespace {

    

}

namespace ROS_Namespace {

    

}
