#pragma once

#include <cstdint>

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
    inline MOTO_REAL_INFO MOTOR_REAL_INFO[8];
} 

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
    inline Xbox_Namespace::XBOX_STATE Xbox_State_Info;
}


namespace ROBOT_Namespace {
    typedef struct ROBOT_CHASSIS {
        float World_V[3] = {0.0f, 0.0f, 0.0f};
        float Robot_V[3] = {0.0f, 0.0f, 0.0f};
        float expect_angle = 0.0f;
        float Angle = 0.0f;
        bool flag = false;
    } ROBOT_CHASSIS;
    inline ROBOT_CHASSIS Robot_Chassis;
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
    inline ACTION_GL_POS ACTION_GL_POS_DATA ; 
}

    

void initializeVariables();
