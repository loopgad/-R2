#pragma once

#include "Global_Namespace.h"
#include "Task_Process.h"
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>


//赋予模式离散数值用于constexpr判断
#define  Trapezoidal_Mode 1
#define  S_Shaped_Mode 2
#define  Polynomail_Mode 3
#define  PT_Mode 4

//用于选择速度规划方式
#define Speed_Plan_Mode Trapezoidal_Mode
//#define Speed_Plan_Mode S-Shaped_Mode
//#define Speed_Plan_Mode Polynomail_Mode
//#define Speed_Plan_Mode PT_Mode


// 节点类
class Node {
public:
    int x, y;   // 节点坐标
    float g, h; // g:从起点到当前节点的代价, h:启发式估价(到终点的预估距离)
    Node* parent;  // 父节点，用于回溯路径

    Node(int x_, int y_, float g_, float h_, Node* parent_ = nullptr);

    float f() const { return g + h; } // f = g + h

    bool operator<(const Node& other) const; // 重载小于号用于优先队列
};

// A*路径规划类
class AStarPlanner {
private:
    int gridSizeX;
    int gridSizeY;

    // 曼哈顿距离启发式函数
    float heuristic(int x1, int y1, int x2, int y2);

    // 检查节点是否在网格范围内
    bool isValid(int x, int y);

    // 回溯路径
    std::vector<Node> reconstructPath(Node* current);

public:
    AStarPlanner(int gridSizeX = 10, int gridSizeY = 10); // 构造函数中允许动态指定网格大小
    ~AStarPlanner();

    // 更新网格大小
    void updateGridSize(int newGridSizeX, int newGridSizeY);

    // 主路径规划方法
    std::vector<Node> planPath(int goalX, int goalY);

    //速度规划方法
    void speedPlan(std::vector<Node>);
    

};

//路径规划函数声明
// 梯形速度规划
inline void trapezoidal_velocity_planning(float x, float y, float max_v, float accel, float &vx, float &vy);
// S型速度规划
inline void s_curve_velocity_planning(float x, float y, float max_v, float accel, float jerk, float &vx, float &vy);
// 多项式速度规划
inline void polynomial_velocity_planning(float x, float y, float max_v, float t_total, float &vx, float &vy);
// PT速度规划
inline void pt_velocity_planning(float x, float y, float max_v, float k_p, float &vx, float &vy);
