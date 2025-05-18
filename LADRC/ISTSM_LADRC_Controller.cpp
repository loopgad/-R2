#include "ISTSM_LADRC_Controller.h"


// 构造函数
ISTSM_LADRC_Controller::ISTSM_LADRC_Controller(double b0_, double k1_, double k2_,
                                                 double beta1_, double beta2_,
                                                 double kp_, double ki_, double tanh_c_)
    : b0(b0_), k1(k1_), k2(k2_), beta1(beta1_), beta2(beta2_),
      kp(kp_), ki(ki_), tanh_c(tanh_c_),
      z21(0), z22(0), v_star(0), omega_ref(0) {}

// 设置目标速度
void ISTSM_LADRC_Controller::setTargetSpeed(double targetSpeed) {
    omega_ref = targetSpeed;
}

// 平滑的tanh函数
double ISTSM_LADRC_Controller::smoothTanh(double x) {
    return (exp(x / tanh_c) - exp(-x / tanh_c)) / (exp(x / tanh_c) + exp(-x / tanh_c));
}

// 更新LESO
void ISTSM_LADRC_Controller::updateLESO(double omega, double u, double dt) {
    double h1 = z21 - omega;
    double h1_tanh = smoothTanh(h1);

    z21 += dt * (z22 + beta1 * (-k1 * std::pow(std::fabs(h1), 0.5) * h1_tanh + v_star) + b0 * u);
    z22 += dt * (beta2 * (-k1 * std::pow(std::fabs(h1), 0.5) * h1_tanh + v_star));
    v_star += dt * (-k2 * h1_tanh);
}

// 计算控制信号
double ISTSM_LADRC_Controller::calculateControl(double dt) {
    double e1 = omega_ref - z21;
    double e1_tanh = smoothTanh(e1);

    double u0 = v_star - kp * std::pow(std::fabs(e1), 0.5) * e1_tanh;
    v_star += dt * (-ki * e1_tanh);

    return (u0 - z22) / b0;
}
