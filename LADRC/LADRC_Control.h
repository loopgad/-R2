//
// Created by 32806 on 24-9-23.
//

#ifndef LADRC_CONTROL_H
#define LADRC_CONTROL_H



class LADRC_Control {

private:
    float v1,v2;         //最速输出值
    float r;             //速度因子
    float h;             //积分步长
    float z1,z2,z3;      //观测器输出
    float w0,wc,b0,u;    //观测器带宽 控制器带宽 系统参数 控制器输出

pubilc:
    LADRC_Control();
    ~LADRC_Control();

    void LADRC_DeInit(float v1,float v2,float r,float h,float z1,float z2,float z3,float w0,float wc,float b0,float u);

};



#endif //LADRC_CONTROL_H
