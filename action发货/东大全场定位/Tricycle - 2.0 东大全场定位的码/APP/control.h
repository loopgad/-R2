#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define amplify               10
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER          (0.866f) //    根号3/2   
#define L_PARAMETER          (1.0f) 
#define sqr3fen2             (1.1547f) 
#define PI                   (3.1416f)
#define Radius               (48.2f)  //底盘半径  L=302.85 cm
#define pulse_per_centimeter (787.5f)  ///******轮子转一圈脉冲数 n= 500*4*19=38000,轮子半径R=7.68cm,周长L=2*pi*R=50cm**  1du~0.84125cm***/
#define cm_per_angle         (0.84125f)  
#define dt                   (0.02f)
#define stop_error           (0.2f)

void Kinematic_Analysis(float Vx,float Vy,float Vz);
void World_Kinematic_Analysis(float Vx, float Vy, float Vz, float theta);
void XIANFU(int xy,int z);
void get_encoder(void);
void navigation(void);
void throw_ball(long int start_pos, long int acc_end_pos, long int break_pos, long int stop_pos, long int throw_speed);
void return_start_pos(long int start_pos);
#endif
