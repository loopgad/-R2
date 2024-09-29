#include <cstdint>
/*
命名空间参照以下格式
namespace ***_Namespace {
	inline int ***_Variable_name = 0; //inline关键字保证该变量只初始化一次
    
}

注意该文件只存放需要跨文件访问的变量，
在类内需要写一个Read_Global函数用于读取消息并分配给成员函数使用
*/




namespace Motor_Namespace {

    

}

namespace Remote_Namespace {
	inline uint16_t SWA;
	inline uint16_t SWB;
	inline uint16_t SWC;
	inline uint16_t SWD;
	inline uint16_t ROCK_R_X;
	inline uint16_t ROCK_R_Y;
	inline uint16_t ROCK_L_Y;
	inline uint16_t ROCK_L_X;
    

}

namespace Action_Namespace {




}

namespace ROS_Namespace {

    

}
