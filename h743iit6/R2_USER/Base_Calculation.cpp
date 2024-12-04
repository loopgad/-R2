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


#include "Base_Calculation.h"




/*!
 * \fn     Kinematic_Analysis_Inverse
 * \brief  三全向轮运动逆解算
 *         直接设置目标速度值（单位是米每秒）
 *         程序内部计算：米每秒 --> 转每分钟（轮子） --> 转每分钟（转子）
 * \param  无
 * \retval 无
 */
void Calculation::Kinematic_Analysis_Inverse(void) {
    Motor_Namespace::MOTOR_REAL_INFO[0].TARGET_RPM = 19 * ((0 + Robot_Namespace::Robot_Chassis.Robot_V[x] + RADIUS * Robot_Namespace::Robot_Chassis.Robot_V[w]) * 60) / (Radius_Wheel * 2 * PI);
    Motor_Namespace::MOTOR_REAL_INFO[1].TARGET_RPM = 19 * ((-(0.866025403) * Robot_Namespace::Robot_Chassis.Robot_V[y] - (0.5) * Robot_Namespace::Robot_Chassis.Robot_V[x] + RADIUS * Robot_Namespace::Robot_Chassis.Robot_V[w]) * 60) / (Radius_Wheel * 2 * PI);
    Motor_Namespace::MOTOR_REAL_INFO[2].TARGET_RPM = 19 * (((0.866025403) * Robot_Namespace::Robot_Chassis.Robot_V[y] - (0.5) * Robot_Namespace::Robot_Chassis.Robot_V[x] + RADIUS * Robot_Namespace::Robot_Chassis.Robot_V[w]) * 60) / (Radius_Wheel * 2 * PI);
}

/*!
 * \fn     Axis_analyse_for_WORLDtoROBOT
 * \brief  完成世界坐标系下速度向机器人坐标系下的速度转换
 *         使用线速度进行转换
 * \param  无
 * \retval 无
 */
void Calculation::Axis_analyse_for_WORLDtoROBOT(void) {
    // x方向上的转换
    Robot_Namespace::Robot_Chassis.Robot_V[x] = cos(-Action_Namespace::ACTION_GL_POS_DATA.REAL_YAW * PI / 180) * Robot_Namespace::Robot_Chassis.World_V[x] 
                             + sin(-Action_Namespace::ACTION_GL_POS_DATA.REAL_YAW * PI / 180) * Robot_Namespace::Robot_Chassis.World_V[y];
    // y方向上的转换
    Robot_Namespace::Robot_Chassis.Robot_V[y] = -sin(-Action_Namespace::ACTION_GL_POS_DATA.REAL_YAW * PI / 180) * Robot_Namespace::Robot_Chassis.World_V[x] 
                             + cos(-Action_Namespace::ACTION_GL_POS_DATA.REAL_YAW * PI / 180) * Robot_Namespace::Robot_Chassis.World_V[y];
    // W方向上的转换
    Robot_Namespace::Robot_Chassis.Robot_V[w] = Robot_Namespace::Robot_Chassis.World_V[w];
}

// 处理死区
int_fast16_t Calculation::HandleDeadZone(int_fast16_t value, int lowerThreshold, int upperThreshold) {
    if (value > lowerThreshold && value < upperThreshold) {
        return lowerThreshold; // 死区
    }
    return value;
}

// 映射到百分比并乘速比因子
float Calculation::MapToPercentage(int_fast16_t value, int lowerThreshold, int upperThreshold) {
    if (value <= lowerThreshold) {
        return (float)(value - lowerThreshold) / lowerThreshold * Speed_Level_Map();
    } else {
        return (float)(value - upperThreshold) / upperThreshold * Speed_Level_Map();
    }
}

/*!
 * \fn     World_Control
 * \brief  始终以世界坐标下的Y方向为正Y
 *         处理手柄遥感数据并将其传入底盘解算
 * \param  无
 * \retval 无
 */
void Calculation::World_Control(void) {
    // 处理手柄遥感数据
    Xbox_Namespace::Xbox_State_Info.joyHori_LX = HandleDeadZone(Xbox_Namespace::Xbox_State_Info.joyHori_LX, 31000, 35000);
    Xbox_Namespace::Xbox_State_Info.joyVert_LY = HandleDeadZone(Xbox_Namespace::Xbox_State_Info.joyVert_LY, 31000, 35000);
    Xbox_Namespace::Xbox_State_Info.joyHori_RX = HandleDeadZone(Xbox_Namespace::Xbox_State_Info.joyHori_RX, 31000, 35000);

    Robot_Namespace::Robot_Chassis.World_V[x] = MapToPercentage(Xbox_Namespace::Xbox_State_Info.joyHori_LX, 31000, 35000);
    Robot_Namespace::Robot_Chassis.World_V[y] = MapToPercentage(Xbox_Namespace::Xbox_State_Info.joyVert_LY, 31000, 35000);

    if (Xbox_Namespace::Xbox_State_Info.joyHori_RX == 0) {
        if (Readed_Flag == 0) {
            Readed_Flag = 1;
            Lock_Yaw = Action_Namespace::ACTION_GL_POS_DATA.REAL_YAW;
        }
        YawAdjust(Lock_Yaw);
    } else {
        Robot_Namespace::Robot_Chassis.World_V[w] = MapToPercentage(Xbox_Namespace::Xbox_State_Info.joyHori_RX, 31000, 35000);
        Readed_Flag = 0;
    }

    Axis_analyse_for_WORLDtoROBOT();
    Kinematic_Analysis_Inverse();
}

/*!
 * \fn     Robot_Control
 * \brief  始终以机头方向为正Y
 *         处理手柄遥感数据并传入解算
 * \param  无
 * \retval 无
 */
void Calculation::Robot_Control(void) {
    // 处理手柄遥感数据
    Xbox_Namespace::Xbox_State_Info.joyHori_LX = HandleDeadZone(Xbox_Namespace::Xbox_State_Info.joyHori_LX, 31000, 35000);
    Xbox_Namespace::Xbox_State_Info.joyVert_LY = HandleDeadZone(Xbox_Namespace::Xbox_State_Info.joyVert_LY, 31000, 35000);
    Xbox_Namespace::Xbox_State_Info.joyHori_RX = HandleDeadZone(Xbox_Namespace::Xbox_State_Info.joyHori_RX, 31000, 35000);

    Robot_Namespace::Robot_Chassis.World_V[x] = MapToPercentage(Xbox_Namespace::Xbox_State_Info.joyHori_LX, 31000, 35000);
    Robot_Namespace::Robot_Chassis.World_V[y] = MapToPercentage(Xbox_Namespace::Xbox_State_Info.joyVert_LY, 31000, 35000);

    // 不同状态下机器人的自转速度逻辑
    if (!Xbox_Namespace::Xbox_State_Info.Lock_Yaw) {
        Robot_Namespace::Robot_Chassis.Robot_V[w] = MapToPercentage(Xbox_Namespace::Xbox_State_Info.joyHori_RX, 31000, 35000);
        Lock_Yaw = Action_Namespace::ACTION_GL_POS_DATA.REAL_YAW;
    } else if (Xbox_Namespace::Xbox_State_Info.joyHori_RX == 0) {
        Robot_Namespace::Robot_Chassis.Robot_V[w] = 0;
        Lock_Yaw = Action_Namespace::ACTION_GL_POS_DATA.REAL_YAW;
    } else {
        YawAdjust(Lock_Yaw);
    }

    Axis_analyse_for_WORLDtoROBOT();
    Kinematic_Analysis_Inverse();
}

/*!
 * \fn     Semi_auto_Control
 * \brief  默认世界坐标系
 *         半自动控制，处理手柄遥感数据
 * \param  无
 * \retval 无
 */
void Calculation::Semi_auto_Control(void) {
    // 处理手柄遥感数据
    Xbox_Namespace::Xbox_State_Info.joyHori_LX = HandleDeadZone(Xbox_Namespace::Xbox_State_Info.joyHori_LX, 31000, 35000);
    Xbox_Namespace::Xbox_State_Info.joyVert_LY = HandleDeadZone(Xbox_Namespace::Xbox_State_Info.joyVert_LY, 31000, 35000);
    Xbox_Namespace::Xbox_State_Info.joyHori_RX = HandleDeadZone(Xbox_Namespace::Xbox_State_Info.joyHori_RX, 31000, 35000);

    R = sqrt(ROS_Namespace::Robot_Relative_x * ROS_Namespace::Robot_Relative_x + ROS_Namespace::Robot_Relative_y * ROS_Namespace::Robot_Relative_y);
    Alpha = atan2(ROS_Namespace::Robot_Relative_y, ROS_Namespace::Robot_Relative_x);

    Vx_temp += ROS_Namespace::Robot_Relative_Vx;
    Vy_temp += ROS_Namespace::Robot_Relative_Vy;

    if (R != 1) {
        Vx_temp += -(R - 1) * cos(Alpha) * 0.003f;
        Vy_temp += (R - 1) * sin(Alpha) * 0.003f;
    }

    Vx_temp = -MapToPercentage(Xbox_Namespace::Xbox_State_Info.joyHori_LX, 31000, 35000) * sin(Alpha);
    Vy_temp = MapToPercentage(Xbox_Namespace::Xbox_State_Info.joyHori_LX, 31000, 35000) * cos(Alpha);

    Robot_Namespace::Robot_Chassis.World_V[x] = Vx_temp;
    Robot_Namespace::Robot_Chassis.World_V[y] = Vy_temp;

    if (Xbox_Namespace::Xbox_State_Info.joyHori_RX == 0) {
        if (Readed_Flag == 0) {
            Readed_Flag = 1;
            Lock_Yaw = Action_Namespace::ACTION_GL_POS_DATA.REAL_YAW;
        }
        YawAdjust(Lock_Yaw);
    } else {
        Robot_Namespace::Robot_Chassis.World_V[w] = MapToPercentage(Xbox_Namespace::Xbox_State_Info.joyHori_RX, 31000, 35000);
        Readed_Flag = 0;
    }

    Axis_analyse_for_WORLDtoROBOT();
    Kinematic_Analysis_Inverse();
}

/*!
 * \fn     YawAdjust
 * \brief  调整偏航角
 * \param  Target_Yaw 目标偏航角
 * \retval 无
 */
void Calculation::YawAdjust(float Target_Yaw) {
    NOW_Yaw = Action_Namespace::ACTION_GL_POS_DATA.REAL_YAW;
    Err_Yaw = NOW_Yaw - Target_Yaw;

    if (Err_Yaw > 180) Err_Yaw -= 360;
    else if (Err_Yaw < -180) Err_Yaw += 360;

    Robot_Namespace::Robot_Chassis.World_V[w] = (Err_Yaw / 180) * 0.3;
}

/*!
 * \fn     Speed_Level_Map
 * \brief  将不同的速度阈值映射到速度比例
 * \retval 速度比例值
 */
inline float Calculation::Speed_Level_Map(void) {
    switch (Xbox_Namespace::Xbox_State_Info.Speed_Threshold) {
        case 2:
            return 1.3f;
        case 3:
            return 1.8f;
        default:
            return 1.0f;
    }
}

/*!
 * \fn     Move_State
 * \brief  根据不同模式控制机器人的运动状态
 * \retval 无
 */
void Calculation::Move_State(void) {
    switch (Xbox_Namespace::Xbox_State_Info.Base_Mode) {
        case 1:
            Robot_Control();
            break;
        case 2:
            World_Control();
            break;
        case 3:
            Semi_auto_Control();
            break;
        default:
            Robot_Control();
            break;
    }
}

/*!
 * \fn     Task_Function
 * \brief  执行主任务函数
 * \retval 无
 */
void Calculation::Task_Function(void) {
    Move_State();
}
