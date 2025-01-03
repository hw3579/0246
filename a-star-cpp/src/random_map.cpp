#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
#include "Point.h"  // 假设有Point类的定义
#include "random_map.h"
#include <cstdlib>
#include <ctime>

RandomMap::RandomMap(int size) : size(size) {
    map = std::vector<std::vector<std::array<uint8_t, 3>>>(size, std::vector<std::array<uint8_t, 3>>(size, { 255, 255, 255 }));
    GenerateObstacle();
}

// 返回地图尺寸
int RandomMap::GetSize() const {
    return size;
}

// 返回障碍物点的集合（常量引用，防止外部修改）
const std::vector<Point>& RandomMap::GetObstaclePoints() const {
    return obstacle_points;
}

// 返回地图数据（非常量引用，允许外部修改地图）
std::vector<std::vector<std::array<uint8_t, 3>>>& RandomMap::GetMap() {
    return map;
}

void RandomMap::GenerateObstacle() {
    obstacle_points.clear();

    // 添加水池障碍（中心圆形）
    Point fountain_center(240, 240);
    int fountain_radius = 40;
    AddCircleObstacle(fountain_center, fountain_radius);

    // 添加食品摊位障碍（正方形40x40）
    std::vector<Point> food_stalls = { {240, 120}, {160, 160}, {240, 340} };
    int food_width = 40;
    for (const auto& stall : food_stalls) {
        AddSquareObstacle(stall, food_width);
    }

    // 添加电梯障碍（正方形50x50）
    std::vector<Point> elevators = { {160, 400}, {320, 400} };
    int elevator_width = 50;
    for (const auto& elevator : elevators) {
        AddSquareObstacle(elevator, elevator_width);
    }

    // 添加购物商店障碍
    std::vector<Point> shopping_stores = { {0, 0}, {360, 0} };
    for (const auto& store : shopping_stores) {
        AddRectangleObstacle(store, 120, 480);
    }

    // 添加禁止区域障碍（顶部和底部水平区域）
    std::vector<Point> no_go_zone = { {120, 0}, {120, 440} };
    for (const auto& ngz : no_go_zone) {
        AddRectangleObstacle(ngz, 240, 40);
    }
}

void RandomMap::AddCircleObstacle(const Point& center, int radius) {
    for (int x = 0; x < size; ++x) {
        for (int y = 0; y < size; ++y) {
            if (std::pow(x - center.x, 2) + std::pow(y - center.y, 2) <= std::pow(radius, 2)) {
                map[x][y] = { 128, 128, 128 };  // 灰色障碍物
                obstacle_points.emplace_back(x, y);
            }
        }
    }
}

void RandomMap::AddSquareObstacle(const Point& center, int width) {
    int half_width = width / 2;
    for (int x = center.x - half_width; x < center.x + half_width; ++x) {
        for (int y = center.y - half_width; y < center.y + half_width; ++y) {
            if (IsValidCoordinate(x, y)) {
                map[x][y] = { 128, 128, 128 };  // 灰色障碍物
                obstacle_points.emplace_back(x, y);
            }
        }
    }
}

void RandomMap::AddRectangleObstacle(const Point& top_left, int width, int height) {
    for (int x = top_left.x; x < top_left.x + width; ++x) {
        for (int y = top_left.y; y < top_left.y + height; ++y) {
            if (IsValidCoordinate(x, y)) {
                map[x][y] = { 128, 128, 128 };  // 灰色障碍物
                obstacle_points.emplace_back(x, y);
            }
        }
    }
}

bool RandomMap::IsValidCoordinate(int x, int y) const {
    return x >= 0 && x < size && y >= 0 && y < size;
}