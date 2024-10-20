
#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "Task_Manager.h"
#include "Callback_Function.h"
#include "Global_Namespace.h"

#ifdef __cplusplus
}
#endif

using namespace Xbox_Namespace;

// 定义最大和最小的射击转速
#define MAX_SHOOT_RPM_UP 4600.0f
#define MAX_SHOOT_RPM_DOWN 3600.0f

// 手柄数据结构体定义
typedef struct
{
    // 按键数据（布尔类型，true代表按下，false代表未按下）
    bool btnY;              // Y键
    bool btnY_last;         // 上一次的Y键状态
    bool btnB;              // B键
    bool btnB_last;         // 上一次的B键状态
    bool btnA;              // A键
    bool btnA_last;         // 上一次的A键状态
    bool btnX;              // X键
    bool btnX_last;         // 上一次的X键状态
    bool btnShare;          // Share键
    bool btnShare_last;     // 上一次的Share键状态
    bool btnStart;          // Start键
    bool btnStart_last;     // 上一次的Start键状态
    bool btnSelect;         // Select键
    bool btnSelect_last;    // 上一次的Select键状态
    bool btnXbox;           // Xbox键
    bool btnXbox_last;      // 上一次的Xbox键状态
    bool btnLB;             // 左肩键
    bool btnLB_last;        // 上一次的左肩键状态
    bool btnRB;             // 右肩键
    bool btnRB_last;        // 上一次的右肩键状态
    bool btnLS;             // 左摇杆按键
    bool btnLS_last;        // 上一次的左摇杆按键状态
    bool btnRS;             // 右摇杆按键
    bool btnRS_last;        // 上一次的右摇杆按键状态
    bool btnDirUp;          // 方向键上
    bool btnDirUp_last;     // 上一次的方向键上状态
    bool btnDirLeft;        // 方向键左
    bool btnDirLeft_last;   // 上一次的方向键左状态
    bool btnDirRight;       // 方向键右
    bool btnDirRight_last;  // 上一次的方向键右状态
    bool btnDirDown;        // 方向键下
    bool btnDirDown_last;   // 上一次的方向键下状态


    // 映射后的浮点数值，用于精确控制(暂时不需)
    /*
    float joyLHori_map;
    float joyLVert_map;
    float joyRHori_map;
    float joyRVert_map;
    float trigLT_map;
    float trigRT_map;
    */
} XboxControllerData_t;

// 手柄类定义
class xbox : public Task_Thread
{
public:
    XboxControllerData_t xbox_msgs; // 手柄状态数据结构

public:
    xbox(); // 构造函数

    // 更新手柄状态
    void update();

    // 检测按键的边沿变化，用于检测按键的状态切换
    inline bool detectButtonEdge(bool currentBtnState, bool *lastBtnState);

    //检测并更新A,B,X,Y按键状态
    void detectButtonEdge_A(bool currentBtnState, bool *lastBtnState);
    void detectButtonEdge_B(bool currentBtnState, bool *lastBtnState);
    void detectButtonEdge_X(bool currentBtnState, bool *lastBtnState);
    void detectButtonEdge_Y(bool currentBtnState, bool *lastBtnState);

    // 检测RB按键的边沿变化
    void detectButtonEdgeRb(bool currentBtnState, bool *lastBtnState);


    // 检测下降按键（降低速度等级）
    void detectButtonEdgeD(bool currentBtnState, bool *lastBtnState);

    // 检测上升按键（提升速度等级）
    void detectButtonEdgeI(bool currentBtnState, bool *lastBtnState);

    //检测action重启按键
    void detectButtonEdge_Action(bool currentBtnState, bool *lastBtnState);

    //任务运行函数
    void Task_Function();
};
#endif

// 以下是具体车辆的xbox遥控器类

// // 九期r1的遥控
// class xbox_r1n : public xbox, public ITaskProcessor
// {
// private:
//     /* data */
// public:
//     xbox_r1n(ACTION_GL_POS *ACTION_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_ = 1.50f, float MAX_ROBOT_SPEED_X_ = 1.50f, float MAX_ROBOT_SPEED_W_ = 3.60f);
//     void process_data();
// };

// 九期r2的遥控
// class xbox_r2n : public xbox, public ITaskProcessor
// {
// private:
//     RC9Protocol *robot_data_chain;

// public:
//     xbox_r2n(ACTION_GL_POS *ACTION_, RC9Protocol *robot_data_chain_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_ = 1.50f, float MAX_ROBOT_SPEED_X_ = 1.50f, float MAX_ROBOT_SPEED_W_ = 3.60f);
//     void process_data();
// };

#endif

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
inline void Action_Reset(void);//重啓action



