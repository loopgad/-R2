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


#include "Xbox_Control.h"

// xbox 构造函数，初始化成员变量
xbox::xbox()
{   
   

    // 初始化所有按钮状态为 false
    xbox_msgs.btnY = false;
    xbox_msgs.btnB = false;
    xbox_msgs.btnA = false;
    xbox_msgs.btnX = false;
    xbox_msgs.btnShare = false;
    xbox_msgs.btnStart = false;
    xbox_msgs.btnSelect = false;
    xbox_msgs.btnXbox = false;
    xbox_msgs.btnLB = false;
    xbox_msgs.btnRB = false;
    xbox_msgs.btnLS = false;
    xbox_msgs.btnRS = false;
    xbox_msgs.btnDirUp = false;
    xbox_msgs.btnDirLeft = false;
    xbox_msgs.btnDirRight = false;
    xbox_msgs.btnDirDown = false;

    // 初始化所有按钮的上一次状态为 false
    xbox_msgs.btnY_last = false;
    xbox_msgs.btnB_last = false;
    xbox_msgs.btnA_last = false;
    xbox_msgs.btnX_last = false;
    xbox_msgs.btnShare_last = false;
    xbox_msgs.btnStart_last = false;
    xbox_msgs.btnSelect_last = false;
    xbox_msgs.btnXbox_last = false;
    xbox_msgs.btnLB_last = false;
    xbox_msgs.btnRB_last = false;
    xbox_msgs.btnLS_last = false;
    xbox_msgs.btnRS_last = false;
    xbox_msgs.btnDirUp_last = false;
    xbox_msgs.btnDirLeft_last = false;
    xbox_msgs.btnDirRight_last = false;
    xbox_msgs.btnDirDown_last = false;

}

// update 函数，用于更新手柄的状态数据
void xbox::update()
{
    // 判断数据长度是否符合要求
    if (sizeof(Xbox_Namespace::xbox_raw_data) == 28)
    {
        // 将当前状态保存到“上一次状态”
        xbox_msgs.btnShare_last = xbox_msgs.btnShare;
        xbox_msgs.btnStart_last = xbox_msgs.btnStart;
        xbox_msgs.btnSelect_last = xbox_msgs.btnSelect;
        xbox_msgs.btnXbox_last = xbox_msgs.btnXbox;
        xbox_msgs.btnLB_last = xbox_msgs.btnLB;
        xbox_msgs.btnRB_last = xbox_msgs.btnRB;
        xbox_msgs.btnLS_last = xbox_msgs.btnLS;
        xbox_msgs.btnRS_last = xbox_msgs.btnRS;
        xbox_msgs.btnDirUp_last = xbox_msgs.btnDirUp;
        xbox_msgs.btnDirLeft_last = xbox_msgs.btnDirLeft;
        xbox_msgs.btnDirRight_last = xbox_msgs.btnDirRight;
        xbox_msgs.btnDirDown_last = xbox_msgs.btnDirDown;

        // 更新每个按键和摇杆的状态
        xbox_msgs.btnY = (bool)Xbox_Namespace::xbox_raw_data[0];
        xbox_msgs.btnB = (bool)Xbox_Namespace::xbox_raw_data[1];
        xbox_msgs.btnA = (bool)Xbox_Namespace::xbox_raw_data[2];
        xbox_msgs.btnX = (bool)Xbox_Namespace::xbox_raw_data[3];
        xbox_msgs.btnShare = Xbox_Namespace::xbox_raw_data[4];
        xbox_msgs.btnStart = Xbox_Namespace::xbox_raw_data[5];
        xbox_msgs.btnSelect = Xbox_Namespace::xbox_raw_data[6];
        xbox_msgs.btnXbox = Xbox_Namespace::xbox_raw_data[7];
        xbox_msgs.btnLB = Xbox_Namespace::xbox_raw_data[8];
        xbox_msgs.btnRB = Xbox_Namespace::xbox_raw_data[9];
        xbox_msgs.btnLS = Xbox_Namespace::xbox_raw_data[10];
        xbox_msgs.btnRS = Xbox_Namespace::xbox_raw_data[11];
        xbox_msgs.btnDirUp = Xbox_Namespace::xbox_raw_data[12];
        xbox_msgs.btnDirLeft = Xbox_Namespace::xbox_raw_data[13];
        xbox_msgs.btnDirRight = Xbox_Namespace::xbox_raw_data[14];
        xbox_msgs.btnDirDown = Xbox_Namespace::xbox_raw_data[15];

        /*
        // 调用通用的按键边沿检测函数
        detectButtonEdge_A();  // 检测A键
        detectButtonEdge_B();  // 检测B键
        detectButtonEdge_X();  // 检测X键
        detectButtonEdge_Y();  // 检测Y键
        detectButtonEdgeRb();  // 检测RB键
        detectButtonEdgeLb();  // 检测LB键
        */

		// 检测Y键,用于锁角度判断
		detectButtonEdge_Lock_Yaw(xbox_msgs.btnY, &xbox_msgs.btnY_last );
		
        // 处理速度控制按钮
        detectButtonEdgeD(xbox_msgs.btnX, &xbox_msgs.btnX_last);  // 速度档位降低
        detectButtonEdgeI(xbox_msgs.btnB, &xbox_msgs.btnB_last);  // 速度档位升高

        //处理模式切换按键
        detectButtonEdge_BaseMode(xbox_msgs.btnLB, &xbox_msgs.btnLB_last); //底盘模式切换

        //处理Action标志位
        detectButtonEdge_Action(xbox_msgs.btnShare, &xbox_msgs.btnShare_last); //重启Action标志位置位
		

        // 组合两个字节来形成16位的霍尔传感器值
        Xbox_Namespace::Xbox_State_Info.joyHori_LX = ((uint16_t)Xbox_Namespace::xbox_raw_data[16] << 8) | Xbox_Namespace::xbox_raw_data[17];
        Xbox_Namespace::Xbox_State_Info.joyVert_LY = ((uint16_t)Xbox_Namespace::xbox_raw_data[18] << 8) | Xbox_Namespace::xbox_raw_data[19];
        Xbox_Namespace::Xbox_State_Info.joyHori_RX = ((uint16_t)Xbox_Namespace::xbox_raw_data[20] << 8) | Xbox_Namespace::xbox_raw_data[21];
        Xbox_Namespace::Xbox_State_Info.joyVert_RY = ((uint16_t)Xbox_Namespace::xbox_raw_data[22] << 8) | Xbox_Namespace::xbox_raw_data[23];
        Xbox_Namespace::Xbox_State_Info.trigLT = ((uint16_t)Xbox_Namespace::xbox_raw_data[24] << 8) | Xbox_Namespace::xbox_raw_data[25];
        Xbox_Namespace::Xbox_State_Info.trigRT = ((uint16_t)Xbox_Namespace::xbox_raw_data[26] << 8) | Xbox_Namespace::xbox_raw_data[27];
    }
}

// 通用按钮边沿检测函数，用于检测按键的上升沿（按下时）
inline bool xbox::detectButtonEdge(bool currentBtnState, bool *lastBtnState)
{
    // 检测是否按键从未按下变为按下
    if (currentBtnState && !(*lastBtnState))
    {
        *lastBtnState = currentBtnState;
        // 按键按下事件处理逻辑可以放在这里
        return true;
    }
    // 更新上一次的按键状态
    *lastBtnState = currentBtnState;
    return false;
}

/*  ***********************************以按键区分的标志位**************************
第一次按下置位，第二次清除
// 检测A按键的上升沿（按下时）
void xbox::detectButtonEdge_A()
{
		Xbox_Namespace::Xbox_State_Info.btnA_State = detectButtonEdge(xbox_msgs.btnA, &xbox_msgs.btnA_last);
}

// 检测B按键的上升沿（按下时）
void xbox::detectButtonEdge_B()
{
		Xbox_Namespace::Xbox_State_Info.btnB_State = detectButtonEdge(xbox_msgs.btnB, &xbox_msgs.btnB_last);
}

// 检测X按键的上升沿（按下时）
void xbox::detectButtonEdge_X()
{
    Xbox_Namespace::Xbox_State_Info.btnX_State = detectButtonEdge(xbox_msgs.btnX, &xbox_msgs.btnX_last);
}

//检测Y按键的上升沿（按下时）
void xbox::detectButtonEdge_Y()
{
    Xbox_Namespace::Xbox_State_Info.btnY_State = detectButtonEdge(xbox_msgs.btnY, &xbox_msgs.btnY_last);
}


// 检测RB按键的上升沿（按下时）
void xbox::detectButtonEdgeRb()
{
    Xbox_Namespace::Xbox_State_Info.btnRB_State = detectButtonEdge(xbox_msgs.btnRB, &xbox_msgs.btnRB_last);
}

// 检测LB按键的上升沿（按下时）
void xbox::detectButtonEdgeLb()
{
    Xbox_Namespace::Xbox_State_Info.btnLB_State = detectButtonEdge(xbox_msgs.btnLB, &xbox_msgs.btnLB_last);
}


*/




/*****************************************************/

/********************************以功能区分的标志位**************************/
// 检测X键按下，降低速度等级
void xbox::detectButtonEdgeD(bool currentBtnState, bool *lastBtnState)
{
    if (currentBtnState && !(*lastBtnState))
    {
        // 降低速度的处理逻辑
        Xbox_Namespace::Xbox_State_Info.Speed_Threshold = (Xbox_Namespace::Xbox_State_Info.Speed_Threshold > 1 ) ? (--Xbox_Namespace::Xbox_State_Info.Speed_Threshold) : Xbox_Namespace::Xbox_State_Info.Speed_Threshold;
        //Xbox_Namespace::Xbox_State_Info.btnX_State = false; //清除状态
    }
    *lastBtnState = currentBtnState;
}

// 检测B键按下，提升速度等级
void xbox::detectButtonEdgeI(bool currentBtnState, bool *lastBtnState)
{
    if (currentBtnState && !(*lastBtnState))
    {
        // 提升速度的处理逻辑
        Xbox_Namespace::Xbox_State_Info.Speed_Threshold = (Xbox_Namespace::Xbox_State_Info.Speed_Threshold < 3 ) ? (++Xbox_Namespace::Xbox_State_Info.Speed_Threshold) : Xbox_Namespace::Xbox_State_Info.Speed_Threshold;
        //Xbox_Namespace::Xbox_State_Info.btnB_State = false; //清除状态
    }
    *lastBtnState = currentBtnState;
}


// 检测LB键按下，切换底盘模式
void xbox::detectButtonEdge_BaseMode(bool currentBtnState, bool *lastBtnState)
{
    if (currentBtnState && !(*lastBtnState))
    {
        // 切换底盘模式的处理逻辑
        Xbox_Namespace::Xbox_State_Info.Base_Mode = (Xbox_Namespace::Xbox_State_Info.Base_Mode < 3 ) ? (++Xbox_Namespace::Xbox_State_Info.Base_Mode) : 1;
        //Xbox_Namespace::Xbox_State_Info.btnLB_State = false; //清除状态
    }
    *lastBtnState = currentBtnState;
}

// 检测Share键按下，重启Action
void xbox::detectButtonEdge_Action(bool currentBtnState, bool *lastBtnState){
    if (currentBtnState && !(*lastBtnState))
    {
        // Set Action_Reset state
        Xbox_Namespace::Xbox_State_Info.Action_Reset = true; 
        //Xbox_Namespace::Xbox_State_Info.btnShare_State = false; //清除状态    
    }
    *lastBtnState = currentBtnState;
}


//检测按键的上升沿（按下时）：用于锁角度
void xbox::detectButtonEdge_Lock_Yaw(bool currentBtnState, bool *lastBtnState)
{	
	static bool flag = true; //用于在第二次按下时取消状态位
	if (currentBtnState && !(*lastBtnState))
    {
		if(flag){
			// Set Action_Reset state
        Xbox_Namespace::Xbox_State_Info.Lock_Yaw = true; 
		flag = false;
		}
        
		else{
			Xbox_Namespace::Xbox_State_Info.Lock_Yaw = false;
			flag = true;
		}
        //Xbox_Namespace::Xbox_State_Info.btnShare_State = false; //清除状态    
    }
    *lastBtnState = currentBtnState;
}

//任务进程如下函数内容
void xbox::Task_Function() {
    update();
}
