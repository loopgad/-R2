<<<<<<< HEAD
#pragma once

#include <cstdint>
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

#pragma once

#include <cstdint >
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0

namespace ROS_Namespace {
    inline float Robot_Relative_x = 0.0f;
    inline float Robot_Relative_y = 0.0f;
    inline float Robot_Relative_Vx = 0.0f;
    inline float Robot_Relative_Vy = 0.0f;
}

namespace Motor_Namespace {
    enum unitMode {
        POSITION_CONTROL_MODE,
        POSITION_TARQUE_CONTROL_MODE,
        SPEED_TARQUE_CONTROL_MODE,
        SPEED_CONTROL_MODE,
        MOTO_OFF,
        VELOCITY_PLANNING_MODE
    };

    typedef struct MOTO_REAL_INFO {
        unitMode unitMode = SPEED_CONTROL_MODE; // 默认模式
        uint16_t ANGLE = 0;
        int16_t RPM = 0;
        int16_t CURRENT = 0;
        int16_t TARGET_CURRENT = 0;
        int16_t TARGET_POS = 0;
        float TARGET_RPM = 0.0f;
        bool Velflag = false;
    } MOTO_REAL_INFO;
<<<<<<< HEAD
    inline MOTO_REAL_INFO MOTOR_REAL_INFO[8];
} 

=======
    inline MOTO_REAL_INFO MOTOR_REAL_INFO[8] = {};
} 


>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
namespace Xbox_Namespace {
        typedef struct XBOX_STATE {
        int_fast16_t joyHori_LX = 33000;
        int_fast16_t joyVert_LY = 33000;
        int_fast16_t joyHori_RX = 33000;
        int_fast16_t joyVert_RY = 33000;
        int_fast16_t trigLT = 0;
        int_fast16_t trigRT = 0;
        bool btnA_State = false;
        bool btnB_State = false;
        bool btnX_State = false;
        bool btnY_State = false;
        bool btnRB_State = false;
        bool btnLB_State = false;
        bool btnShare_State = false;
        uint_fast8_t Speed_Threshold = 1;
        uint_fast8_t Base_Mode = 1;
        bool Action_Reset = false;
        bool Lock_Yaw = false;
        bool Air_Pump = false;
    } XBOX_STATE;

    inline uint8_t xbox_raw_data[28] = {0};  // 初始化为0
<<<<<<< HEAD
    inline Xbox_Namespace::XBOX_STATE Xbox_State_Info;
}


namespace ROBOT_Namespace {
=======
    inline Xbox_Namespace::XBOX_STATE Xbox_State_Info = {};
}


namespace Robot_Namespace {
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
    typedef struct ROBOT_CHASSIS {
        float World_V[3] = {0.0f, 0.0f, 0.0f};
        float Robot_V[3] = {0.0f, 0.0f, 0.0f};
        float expect_angle = 0.0f;
        float Angle = 0.0f;
        bool flag = false;
    } ROBOT_CHASSIS;
<<<<<<< HEAD
    inline ROBOT_CHASSIS Robot_Chassis;
=======
    inline ROBOT_CHASSIS Robot_Chassis = {};
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
}
   

namespace Action_Namespace {
    typedef struct ACTION_GL_POS {
        float POS_X = 0.0f;
        float POS_Y = 0.0f;
        float YAW = 0.0f;
        float W_Z = 0.0f;
        float LAST_POS_X = 0.0f;
        float LAST_POS_Y = 0.0f;
        float LAST_YAW = 0.0f;
        float DELTA_POS_X = 0.0f;
        float DELTA_POS_Y = 0.0f;
        float DELTA_YAW = 0.0f;
        float REAL_X = 0.0f;
        float REAL_Y = 0.0f;
        float REAL_YAW = 0.0f;
        float OFFSET_YAW = 0.0f;
    } ACTION_GL_POS;
<<<<<<< HEAD
    inline ACTION_GL_POS ACTION_GL_POS_DATA ; 
=======
    inline ACTION_GL_POS ACTION_GL_POS_DATA = {}; 
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
}

    

void initializeVariables();
