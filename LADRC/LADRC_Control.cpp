//
// Created by 32806 on 24-9-23.
//

#include "LADRC_Control.h"

LADRC_Control::LADRC_Control(){
    v1 = 0;
    v2 = 0;         //最速输出值
    r = 0.01;             //速度因子
    h = 0.1;             //积分步长
    z1 = 0;
    z2 = 0;
    z3 = 0;      //观测器输出
    w0 = 400;   //观测器带宽
    wc = 100;   //控制器带宽
    b0 = 0;     //系统参数
    u = 0;      //控制器输出 
}

LADRC_Control::LADRC_Control(float LADRC_r,float LADRC_h,float LADRC_w0,float LADRC_wc,float LADRC_b0){
    v1 = 0;
    v2 = 0;
    LADRC_Control::r = LADRC_r;
    LADRC_Control::h = LADRC_h;
    LADRC_Control::wc = LADRC_wc;
    LADRC_Control::w0 = LADRC_wc * 4;
    LADRC_Control::b0 = LADRC_b0;
    z1 = 0;
    z2 = 0;
    z3 = 0;
    u = 0;      //控制器输出 
}

LADRC_Control::~LADRC_Control(){
    u = 0;
}

void LADRC_Control::LADRC_DeInit(float LADRC_v1,float LADRC_v2,float LADRC_r,float LADRC_h,float LADRC_wc,float LADRC_b0)
{
    LADRC_Control::v1 = LADRC_v1;
    LADRC_Control::v2 = LADRC_v2;
    LADRC_Control::r = LADRC_r;
    LADRC_Control::h = LADRC_h;
    LADRC_Control::wc = LADRC_wc;
    LADRC_Control::w0 = LADRC_wc * 4;
    LADRC_Control::b0 = LADRC_b0;
    z1 = 0;
    z2 = 0;
    z3 = 0;
}


void LADRC_Control::LADRC_TD(float Expect)
{
    float fh= -r*r*(v1-Expect)-2*r*v2;
    v1+=v2*h;
    v2+=fh*h;
}


void LADRC_Control::LADRC_ESO(float FeedBack)
{
float Beita_01=3*w0;
    float Beita_02=3*w0*w0;
    float Beita_03=w0*w0*w0;

    float e= z1-FeedBack;
    z1+= (z2 - Beita_01*e)*h;
    z2+= (z3 - Beita_02*e + b0*u)*h;
}

void LADRC_Control::LADRC_LF()
{
    float Kp=wc*wc;
    float Kd=2*wc;
	/**
       *@Brief  按自抗扰入门书上kd = 2wc
       *@Before Kd=3*wc;
       *@Now    Kd=2*wc;
       */
    float e1=v1-z1;
    float e2=v2-z2;
    float u0=Kp*e1+Kd*e2;
    u=(u0-z3)/b0;
	if(u>2000)
		u=2000;
	else if(u<-2000)
		u=-2000;
}

void LADRC_Control::LADRC_Loop(float* Expect,float* RealTimeOut)
{
	float  Expect_Value = *Expect;
	float  Measure = *RealTimeOut;
    LADRC_TD(Expect_Value);
    LADRC_ESO(Measure); 
    LADRC_LF();
}

