#include "Xbox_Control.h"

// xbox 构造函数，初始化成员变量
xbox::xbox(action *ACTION_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_, float MAX_ROBOT_SPEED_X_, float MAX_ROBOT_SPEED_W_) 
    : ACTION(ACTION_), control_chassis(control_chassis_), MAX_ROBOT_SPEED_Y(MAX_ROBOT_SPEED_Y_), MAX_ROBOT_SPEED_X(MAX_ROBOT_SPEED_X_), MAX_ROBOT_SPEED_W(MAX_ROBOT_SPEED_W_) 
{
}

// update 函数，用于更新手柄数据
void xbox::update(uint8_t data_id, uint8_t data_length, const uint8_t *data_char, const float *data_float) 
{
    if (data_length == 28) // 判断数据长度是否为28字节
    {
        // 解析按键数据 (bool 值)
        xbox_msgs.btnY = data_char[0];    // Y 按钮
        xbox_msgs.btnB = data_char[1];    // B 按钮
        xbox_msgs.btnA = data_char[2];    // A 按钮
        xbox_msgs.btnX = data_char[3];    // X 按钮
        xbox_msgs.btnShare = data_char[4]; // 分享按钮
        xbox_msgs.btnStart = data_char[5]; // 开始按钮
        xbox_msgs.btnSelect = data_char[6]; // 选择按钮
        xbox_msgs.btnXbox = data_char[7];   // Xbox 按钮
        xbox_msgs.btnLB = data_char[8];    // LB 按钮
        xbox_msgs.btnRB = data_char[9];    // RB 按钮
        xbox_msgs.btnLS = data_char[10];   // 左摇杆按钮
        xbox_msgs.btnRS = data_char[11];   // 右摇杆按钮
        xbox_msgs.btnDirUp = data_char[12];   // 方向键上
        xbox_msgs.btnDirLeft = data_char[13]; // 方向键左
        xbox_msgs.btnDirRight = data_char[14]; // 方向键右
        xbox_msgs.btnDirDown = data_char[15]; // 方向键下

        // 解析霍尔传感器值（16位数据，高8位和低8位拼接）
        xbox_msgs.joyLHori = ((uint16_t)data_char[16] << 8) | data_char[17];  // 左摇杆水平
        xbox_msgs.joyLVert = ((uint16_t)data_char[18] << 8) | data_char[19];  // 左摇杆垂直
        xbox_msgs.joyRHori = ((uint16_t)data_char[20] << 8) | data_char[21];  // 右摇杆水平
        xbox_msgs.joyRVert = ((uint16_t)data_char[22] << 8) | data_char[23];  // 右摇杆垂直
        xbox_msgs.trigLT = ((uint16_t)data_char[24] << 8) | data_char[25];    // 左扳机
        xbox_msgs.trigRT = ((uint16_t)data_char[26] << 8) | data_char[27];    // 右扳机
    }
}

// 按钮边沿检测函数，检测按键的状态变化（上升沿）
void xbox::detectButtonEdge(bool currentBtnState, bool *lastBtnState, uint8_t *toggleState, uint8_t maxState)
{
    if (currentBtnState && !(*lastBtnState)) // 检测到按键上升沿
    { 
        *toggleState = (*toggleState + 1) % (maxState + 1); // 更新状态

        // locking_heading = ROBOT_REAL_POS_DATA.POS_YAW_RAD;  // 示例代码，未使用
    }
    *lastBtnState = currentBtnState; // 更新上一次的按键状态
}

// 按钮边沿检测函数，检测RB按键的状态变化（上升沿）
void xbox::detectButtonEdgeRb(bool currentBtnState, bool *lastBtnState, uint8_t *toggleState, uint8_t maxState)
{
    if (currentBtnState && !(*lastBtnState)) // 检测到按键上升沿
    { 
        *toggleState = (*toggleState + 1) % (maxState + 1); // 更新状态
        locking_heading = ACTION->pose_data.yaw_rad; // 锁定当前方向角
    }
    *lastBtnState = currentBtnState; // 更新上一次的按键状态
}

// 检测按钮 X 是否按下，并更新速度等级减小
void xbox::detectButtonEdgeD(bool currentBtnState, bool *lastBtnState)
{
    if (currentBtnState && !(*lastBtnState)) // 检测到按键上升沿
    {
        if (speed_level > 0) // 如果当前速度等级大于 0，则减小速度等级
        {
            speed_level--;
        }
    }
    *lastBtnState = currentBtnState; // 更新上一次的按键状态
}

// 检测按钮 B 是否按下，并更新速度等级增大
void xbox::detectButtonEdgeI(bool currentBtnState, bool *lastBtnState)
{
    if (currentBtnState && !(*lastBtnState)) // 检测到按键上升沿
    {
        if (speed_level < 2) // 如果当前速度等级小于 2，则增大速度等级
        {
            speed_level++;
        }
    }
    *lastBtnState = currentBtnState; // 更新上一次的按键状态
}

// 底盘控制函数，根据按钮和摇杆状态控制底盘运动
void xbox::chassis_control()
{
    // 检测各个按钮的边沿并更新对应的标志位
    detectButtonEdgeRb(xbox_msgs.btnRB, &xbox_msgs.btnRB_last, &head_locking_flag, 1);
    detectButtonEdge(xbox_msgs.btnLS, &xbox_msgs.btnLS_last, &robot_stop_flag, 1);
    detectButtonEdge(xbox_msgs.btnRS, &xbox_msgs.btnRS_last, &world_robot_flag, 1);
    detectButtonEdge(xbox_msgs.btnLB, &xbox_msgs.btnLB_last, &catch_ball_flag, 1);
    detectButtonEdgeD(xbox_msgs.btnX, &xbox_msgs.btnX_last);  // 降低速度
    detectButtonEdgeI(xbox_msgs.btnB, &xbox_msgs.btnB_last);  // 提升速度
    detectButtonEdge(xbox_msgs.btnA, &xbox_msgs.btnA_last, &if_point_track_flag, 1);

    // 根据速度等级设置最大速度
    if (speed_level == 1)
    {
        MAX_ROBOT_SPEED_X = 1.20f;
        MAX_ROBOT_SPEED_Y = 1.20f;
        MAX_ROBOT_SPEED_W = 3.20f;
    }
    if (speed_level == 0)
    {
        MAX_ROBOT_SPEED_X = 0.40f;
        MAX_ROBOT_SPEED_Y = 0.40f;
        MAX_ROBOT_SPEED_W = 1.10f;
    }
    if (speed_level == 2)
    {
        MAX_ROBOT_SPEED_X = 1.96f;
        MAX_ROBOT_SPEED_Y = 1.96f;
        MAX_ROBOT_SPEED_W = 3.98f;
    }

    // 如果按下 Xbox 按钮，重启动作
    if (xbox_msgs.btnXbox == 1)
    {
        ACTION->restart();
    }

    if (xbox_msgs.joyLHori > 31000 && xbox_msgs.joyLHori < 350000)
    {
        xbox_msgs.joyLHori_map = 0.0f;
    }
    if (xbox_msgs.joyLHori <= 31000)
    {
        xbox_msgs.joyLHori_map = (31000.0f - (float)xbox_msgs.joyLHori) / 31000.0f;
    }
    if (xbox_msgs.joyLHori >= 35000)
    {
        xbox_msgs.joyLHori_map = (35000.0f - (float)xbox_msgs.joyLHori) / 30535.0f;
    }

    if (xbox_msgs.joyLVert > 31000 && xbox_msgs.joyLVert < 350000)
    {
        xbox_msgs.joyLVert_map = 0.0f;
    }
    if (xbox_msgs.joyLVert <= 31000)
    {
        xbox_msgs.joyLVert_map = (31000.0f - (float)xbox_msgs.joyLVert) / 31000.0f;
    }
    if (xbox_msgs.joyLVert >= 35000)
    {
        xbox_msgs.joyLVert_map = (35000.0f - (float)xbox_msgs.joyLVert) / 30535.0f;
    }

    if (xbox_msgs.joyRHori > 31000 && xbox_msgs.joyRHori < 35000)
    {
        xbox_msgs.joyRHori_map = 0.0f;
    }
    if (xbox_msgs.joyRHori <= 31000)
    {
        xbox_msgs.joyRHori_map = (31000.0f - (float)xbox_msgs.joyRHori) / 31000.0f;
    }
    if (xbox_msgs.joyRHori >= 35000)
    {
        xbox_msgs.joyRHori_map = (35000.0f - (float)xbox_msgs.joyRHori) / 30535.0f;
    }
    if (head_locking_flag == 1)
    {
        control_chassis->lock();
    }
    if (head_locking_flag == 0)
    {
        control_chassis->unlock();
    }
    if (world_robot_flag == 0 && robot_stop_flag == 0 && if_point_track_flag == 0)
    {
        control_chassis->switch_chassis_mode(remote_robotv);
        control_chassis->setrobotv(MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map, MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map, -MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map);
    }
    if (world_robot_flag == 1 && robot_stop_flag == 0 && if_point_track_flag == 0)
    {
        control_chassis->switch_chassis_mode(remote_worldv);
        control_chassis->setworldv(MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map, MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map, -MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map);
    }
    if (robot_stop_flag == 1)
    {
        control_chassis->switch_chassis_mode(chassis_standby);
    }
    if (if_point_track_flag == 1 && robot_stop_flag == 0)
    {
        control_chassis->switch_chassis_mode(point_tracking);
    }
}

xbox_r1n::xbox_r1n(action *ACTION_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_, float MAX_ROBOT_SPEED_X_, float MAX_ROBOT_SPEED_W_) : xbox(ACTION_, control_chassis_, MAX_ROBOT_SPEED_Y_, MAX_ROBOT_SPEED_X_, MAX_ROBOT_SPEED_W_)
{
}

void xbox_r1n::process_data()
{
    chassis_control();
}

xbox_r2n::xbox_r2n(action *ACTION_, RC9Protocol *robot_data_chain_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_, float MAX_ROBOT_SPEED_X_, float MAX_ROBOT_SPEED_W_) : xbox(ACTION_, control_chassis_, MAX_ROBOT_SPEED_Y_, MAX_ROBOT_SPEED_X_, MAX_ROBOT_SPEED_W_), robot_data_chain(robot_data_chain_)
{
}
void xbox_r2n::process_data()
{
    target_trackpoint_x = robot_data_chain->rx_frame_mat.data.msg_get[0];
    target_trackpoint_y = robot_data_chain->rx_frame_mat.data.msg_get[1];
    chassis_control();
}