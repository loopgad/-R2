#pragma once

#include "Global_Namespace.h"
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>

using namespace ROS_Namespace;

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
};

#endif // ASTAR_PLANNER_H
