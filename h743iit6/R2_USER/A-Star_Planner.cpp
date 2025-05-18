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


/*
Copyright (c) 2024 loopgad

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


#include "A-Star_Planner.h"

using namespace ROS_Namespace;
// Node类的构造函数
Node::Node(int x_, int y_, float g_, float h_, Node* parent_)
    : x(x_), y(y_), g(g_), h(h_), parent(parent_) {}

// 重载操作符，用于优先队列
bool Node::operator<(const Node& other) const {
    return f() > other.f();  // 优先队列需要f值最小的优先，所以用大于号
}

// AStarPlanner的构造函数，允许动态设置网格大小
AStarPlanner::AStarPlanner(int gridSizeX_, int gridSizeY_) 
    : gridSizeX(gridSizeX_), gridSizeY(gridSizeY_) {}

// AStarPlanner的析构函数
AStarPlanner::~AStarPlanner() {}

// 更新网格大小
void AStarPlanner::updateGridSize(int newGridSizeX, int newGridSizeY) {
    gridSizeX = newGridSizeX;
    gridSizeY = newGridSizeY;
}

// 曼哈顿距离作为启发式函数
float AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

// 检查节点是否在网格范围内
bool AStarPlanner::isValid(int x, int y) {
    return x >= 0 && y >= 0 && x < gridSizeX && y < gridSizeY;
}

// A*路径规划的核心方法
std::vector<Node> AStarPlanner::planPath(int goalX, int goalY) {
    // 将机器人相对坐标映射到当前网格坐标
    int startX = static_cast<int>(Robot_Relative_x * gridSizeX);
    int startY = static_cast<int>(Robot_Relative_y * gridSizeY);

    // 启动优先队列
    std::priority_queue<Node> openSet;
    std::vector<std::vector<bool>> closedSet(gridSizeX, std::vector<bool>(gridSizeY, false));

    openSet.emplace(startX, startY, 0, heuristic(startX, startY, goalX, goalY));

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        // 如果找到目标节点，则回溯路径
        if (current.x == goalX && current.y == goalY) {
            return reconstructPath(&current);
        }

        closedSet[current.x][current.y] = true;

        // 扩展邻居节点
        std::vector<std::pair<int, int>> neighbors = {
            {0, 1}, {1, 0}, {0, -1}, {-1, 0}
        };

        for (auto [dx, dy] : neighbors) {
            int nx = current.x + dx;
            int ny = current.y + dy;

            if (isValid(nx, ny) && !closedSet[nx][ny]) {
                float newG = current.g + 1;  // 假设移动到相邻节点的代价为1
                float newH = heuristic(nx, ny, goalX, goalY);
                openSet.emplace(nx, ny, newG, newH, new Node(current));
            }
        }
    }

    return {}; // 没有找到路径，返回空
}

// 回溯路径
std::vector<Node> AStarPlanner::reconstructPath(Node* current) {
    std::vector<Node> path;
    while (current) {
        path.push_back(*current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}


    // 梯形速度规划
inline void trapezoidal_velocity_planning(float x, float y, float max_v, float accel, float &vx, float &vy) {
    // 计算目标距离
    float dist = std::sqrt(x * x + y * y);
    
    // 根据距离计算加速度时间和减速度时间
    float t_accel = max_v / accel;
    float t_decel = max_v / accel;
    
    // 计算在加速区、匀速区和减速区时的速度
    float t_total = dist / max_v;
    if (t_total <= t_accel + t_decel) {
        max_v = std::sqrt(accel * dist); // 调整最大速度
    }

    // 计算速度方向
    float angle = std::atan2(y, x);
    vx = max_v * std::cos(angle);
    vy = max_v * std::sin(angle);

    //限速逻辑（待添加）

}


// S型速度规划
inline void s_curve_velocity_planning(float x, float y, float max_v, float accel, float jerk, float &vx, float &vy) {
    // 计算目标距离
    float dist = std::sqrt(x * x + y * y);
    
    // 计算加速度、匀速、减速度区间的持续时间
    float t_accel = std::sqrt(max_v / jerk);
    float t_decel = t_accel;
    
    // 根据距离调整最大速度
    if (dist < max_v * (t_accel + t_decel)) {
        max_v = std::sqrt(jerk * dist);
    }

    // 计算速度方向
    float angle = std::atan2(y, x);
    vx = max_v * std::cos(angle);
    vy = max_v * std::sin(angle);

        //限速逻辑（待添加）
}


// 多项式速度规划
inline void polynomial_velocity_planning(float x, float y, float max_v, float t_total, float &vx, float &vy) {
    // 使用五次多项式规划速度
    float a0 = 0, a1 = 0, a2 = 0, a3 = 10.0 / (t_total * t_total * t_total);
    float t = t_total / 2.0; // 假设当前时间为总时间的一半
    
    // 计算速度
    float velocity_factor = 3 * a3 * t * t;

    // 计算速度方向
    float angle = std::atan2(y, x);
    vx = velocity_factor * std::cos(angle);
    vy = velocity_factor * std::sin(angle);

        //限速逻辑（待添加）
}

// PT速度规划
inline void pt_velocity_planning(float x, float y, float max_v, float k_p, float &vx, float &vy) {
    // 计算目标距离
    float dist = std::sqrt(x * x + y * y);
    
    // 计算比例控制下的速度
    float velocity_factor = k_p * dist;
    if (velocity_factor > max_v) {
        velocity_factor = max_v; // 限制最大速度
    }

    // 计算速度方向
    float angle = std::atan2(y, x);
    vx = velocity_factor * std::cos(angle);
    vy = velocity_factor * std::sin(angle);

        //限速逻辑（待添加）
}




//速度规划实现
void AStarPlanner::speedPlan(std::vector<Node> path) {
    // 如果路径为空，直接返回
    if (path.empty()) {
        std::cout << "No path available for speed planning." << std::endl;
        return;
    }

    // 遍历路径中的每个点，进行速度规划
    for (size_t i = 1; i < path.size(); ++i) 
    {
        int x = path[i].x - path[i-1].x; // 计算相对坐标差
        int y = path[i].y - path[i-1].y;

        float &vx = ROBOT_Namespace::Robot_Chassis.Robot_V[1];
        float &vy = ROBOT_Namespace::Robot_Chassis.Robot_V[0];

        float max_v = 1.5, accel = 1.0, jerk = 0.5, t_total = 10.0, k_p = 0.6;//accel:加速,  jerk:加速度变化率,  t_total:总时间 ,  k_p:比例系数

        if constexpr(Speed_Plan_Mode == Trapezoidal_Mode) // 梯形速度规划
        {
            trapezoidal_velocity_planning(x, y, max_v, accel, ROBOT_Namespace::Robot_Chassis.World_V[0], ROBOT_Namespace::Robot_Chassis.World_V[1]);
        }
        else if constexpr(Speed_Plan_Mode == S_Shaped_Mode)// S型速度规划
        {
            s_curve_velocity_planning(x, y, max_v, accel, jerk, ROBOT_Namespace::Robot_Chassis.World_V[0], ROBOT_Namespace::Robot_Chassis.World_V[1]);
        }   
        else if constexpr(Speed_Plan_Mode == Polynomail_Mode)// 多项式速度规划
        {
            polynomial_velocity_planning(x, y, max_v, t_total, ROBOT_Namespace::Robot_Chassis.World_V[0], ROBOT_Namespace::Robot_Chassis.World_V[1]);
        }
        else if constexpr(Speed_Plan_Mode == PT_Mode)// PT速度规划
        {
            pt_velocity_planning(x, y, max_v, k_p, ROBOT_Namespace::Robot_Chassis.World_V[0], ROBOT_Namespace::Robot_Chassis.World_V[1]);
        }
        else {
            std::cout << "error:haven't planned" << std::endl;
        }
    }    
}


