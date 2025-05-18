<<<<<<< HEAD
=======
/*
Copyright (c) 2024 loopgad 9th_R2_Member

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0

#pragma once

#include "Callback_Function.h"
#include "Global_Namespace.h"
#include "Xbox_Map_Table.h"


// xbox默认用串口1

// 定义最大和最小的射击转速
<<<<<<< HEAD
#define MAX_SHOOT_RPM_UP 4600.0f
#define MAX_SHOOT_RPM_DOWN 3600.0f
=======
#define MAX_SHOOT_RPM_H 4600.0f
#define MAX_SHOOT_RPM_L 3600.0f
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0

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
class xbox
{
public:
    XboxControllerData_t xbox_msgs; // 手柄状态数据结构

public:
    xbox(); // 构造函数

    // 更新手柄状态
    void update();

    // 检测按键的边沿变化，用于检测按键的状态切换
    inline bool detectButtonEdge(bool currentBtnState, bool *lastBtnState);

/*****************************物理状态按键***************************/
    //检测并更新A,B,X,Y按键状态
    void detectButtonEdge_A();
    void detectButtonEdge_B();
    void detectButtonEdge_X();
    void detectButtonEdge_Y();
    // 检测RB按键的边沿变化
    void detectButtonEdgeRb();
    // 检测LB按键的边沿变化
    void detectButtonEdgeLb();



/****************************功能按键********************************/
    // 检测下降按键（降低速度等级）
    void detectButtonEdgeD(bool currentBtnState, bool *lastBtnState);

    // 检测上升按键（提升速度等级）
    void detectButtonEdgeI(bool currentBtnState, bool *lastBtnState);

    // 检测LB键按下，切换底盘模式
    void detectButtonEdge_BaseMode(bool currentBtnState, bool *lastBtnState);

    //检测action重启按键
    void detectButtonEdge_Action(bool currentBtnState, bool *lastBtnState);
	
	//检测锁角度按键
	void detectButtonEdge_Lock_Yaw(bool currentBtnState, bool *lastBtnState);
	
    //任务运行函数
    void Task_Function();
};


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








