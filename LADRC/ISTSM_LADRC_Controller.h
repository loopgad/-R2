#ifndef ISTSM_LADRC_CONTROLLER_H
#define ISTSM_LADRC_CONTROLLER_H

#include <cmath>

class ISTSM_LADRC_Controller {
private:
    // 参数定义
    double b0;       // 系统参数估计值
    double k1, k2;   // STSM控制参数
    double beta1, beta2; // LESO参数
    double kp, ki;   // LSEF参数
    double tanh_c;   // tanh函数平滑参数

    // 状态变量
    double z21, z22; // LESO的状态
    double v_star;   // 滑模状态变量
    double omega_ref; // 目标速度

    // 超螺旋滑模控制中的tanh函数
    double smoothTanh(double x);

public:
    // 构造函数
    ISTSM_LADRC_Controller(double b0_, double k1_, double k2_,
                            double beta1_, double beta2_,
                            double kp_, double ki_, double tanh_c_);

    // 设置目标速度
    void setTargetSpeed(double targetSpeed);

    // 更新LESO
    void updateLESO(double omega, double u, double dt);

    // 计算控制信号
    double calculateControl(double dt);
};

#endif // ISTSM_LADRC_CONTROLLER_H
