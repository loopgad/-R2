#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <limits>
using namespace std;

// 自定义输出函数
std::ostream& operator<<(std::ostream& os, const std::vector<Eigen::Matrix<double, 2, 1>>& vec) {
    os << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        if (i > 0) os << ", ";
        os << vec[i].transpose();  // 输出矩阵的转置（方便查看）
    }
    os << "]";
    return os;
}

// 模型预测控制（MPC）实现
pair<vector<Eigen::Vector2d>, vector<double>> position_control_mpc(
    const Eigen::Vector2d& state, // [x, y]
    const Eigen::Vector2d& target_pos, // 目标位置 [x, y]
    double time_step, // 时间步长
    int prediction_horizon = 10, // 预测时域长度
    double max_vx = 2.7, // 最大线速度
    double max_vy = 2.7  // 最大y方向速度
) {
    // 预测轨迹和控制输入的初始化
    vector<Eigen::Vector2d> predicted_trajectory;

    // 计算目标位置误差
    Eigen::Vector2d position_error = target_pos - state;
    double distance = position_error.norm() / 0.2;
    double distance_ = position_error.norm();
    // 计算误差距离，如果误差非常小，则将目标速度减小到零
    double threshold = 0.10; // 误差阈值，当目标位置接近时，速度应逐渐收敛到零
    if (distance_ < threshold) {
        return {{state}, {0.0, 0.0}}; // 返回零速度并直接退出
    }

    // 合成目标速度矢量 v_target
    Eigen::Vector2d v_target = position_error.normalized() * std::min(distance, max_vx);  // 限制最大速度

    // 分解合成速度矢量为 vx 和 vy
    double vx = v_target(0);
    double vy = v_target(1);

    // 预测轨迹
    vector<Eigen::Vector2d> trajectory;
    Eigen::Vector2d predicted_state = state;
    for (int step = 0; step < prediction_horizon; ++step) {
        predicted_state(0) += vx * time_step;
        predicted_state(1) += vy * time_step;
        trajectory.push_back(predicted_state);
    }

    // 计算代价函数：位置误差的平方和
    double cost = 0.0;
    for (int i = 0; i < prediction_horizon; ++i) {
        Eigen::Vector2d position_error = target_pos - trajectory[i];
        cost += position_error.squaredNorm();  // 只考虑位置误差的平方和
    }

    // 选择最小代价对应的控制输入
    double best_vx = vx;
    double best_vy = vy;
    if (cost < std::numeric_limits<double>::infinity()) {
        best_vx = vx;
        best_vy = vy;
    }

    // 返回控制输入
    vector<double> control = {best_vx, best_vy};

    // 返回轨迹和控制输入
    return {trajectory, control};
}

// 主函数
int main() {
    // 输入参数
    Eigen::Vector2d state(0.0, 0.0); // [x, y]
    Eigen::Vector2d target_pos(1.1, 1.5); // 目标位置
    double time_step = 0.02;              // 时间步长

    // 控制循环
    int max_steps = 100; // 最大步数
    for (int step = 0; step < max_steps; ++step) {
        cout << "Step " << step << ":\n";

        // 计算控制输入和新的状态
        auto [trajectory, control] = position_control_mpc(state, target_pos, time_step);

        // 输出控制速度（vx, vy）
        cout << "Control Velocities: ("
             << control[0] << ", "  // vx
             << control[1] << ")\n"; // vy

        // **状态更新**：根据控制输入更新状态
        state(0) += control[0] * time_step; // 更新x位置
        state(1) += control[1] * time_step; // 更新y位置

        // 如果位置足够接近目标位置，就结束
        if ((target_pos - state).norm() < 0.05) {
            cout << "Target reached.\n";
            break;
        }
    }

    // 输出最终位置
    cout << "Final Position: ("
         << state(0) << ", "  // x
         << state(1) << ")\n"; // y

    return 0;
}
