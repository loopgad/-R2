#include "AStarPlanner.h"

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
