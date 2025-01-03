#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
#include "Point.h"  // ������Point��Ķ���
#include "random_map.h"
#include <cstdlib>
#include <ctime>

RandomMap::RandomMap(int size) : size(size) {
    map = std::vector<std::vector<std::array<uint8_t, 3>>>(size, std::vector<std::array<uint8_t, 3>>(size, { 255, 255, 255 }));
    GenerateObstacle();
}

// ���ص�ͼ�ߴ�
int RandomMap::GetSize() const {
    return size;
}

// �����ϰ����ļ��ϣ��������ã���ֹ�ⲿ�޸ģ�
const std::vector<Point>& RandomMap::GetObstaclePoints() const {
    return obstacle_points;
}

// ���ص�ͼ���ݣ��ǳ������ã������ⲿ�޸ĵ�ͼ��
std::vector<std::vector<std::array<uint8_t, 3>>>& RandomMap::GetMap() {
    return map;
}

void RandomMap::GenerateObstacle() {
    obstacle_points.clear();

    // ���ˮ���ϰ�������Բ�Σ�
    Point fountain_center(240, 240);
    int fountain_radius = 40;
    AddCircleObstacle(fountain_center, fountain_radius);

    // ���ʳƷ̯λ�ϰ���������40x40��
    std::vector<Point> food_stalls = { {240, 120}, {160, 160}, {240, 340} };
    int food_width = 40;
    for (const auto& stall : food_stalls) {
        AddSquareObstacle(stall, food_width);
    }

    // ��ӵ����ϰ���������50x50��
    std::vector<Point> elevators = { {160, 400}, {320, 400} };
    int elevator_width = 50;
    for (const auto& elevator : elevators) {
        AddSquareObstacle(elevator, elevator_width);
    }

    // ��ӹ����̵��ϰ�
    std::vector<Point> shopping_stores = { {0, 0}, {360, 0} };
    for (const auto& store : shopping_stores) {
        AddRectangleObstacle(store, 120, 480);
    }

    // ��ӽ�ֹ�����ϰ��������͵ײ�ˮƽ����
    std::vector<Point> no_go_zone = { {120, 0}, {120, 440} };
    for (const auto& ngz : no_go_zone) {
        AddRectangleObstacle(ngz, 240, 40);
    }
}

void RandomMap::AddCircleObstacle(const Point& center, int radius) {
    for (int x = 0; x < size; ++x) {
        for (int y = 0; y < size; ++y) {
            if (std::pow(x - center.x, 2) + std::pow(y - center.y, 2) <= std::pow(radius, 2)) {
                map[x][y] = { 128, 128, 128 };  // ��ɫ�ϰ���
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
                map[x][y] = { 128, 128, 128 };  // ��ɫ�ϰ���
                obstacle_points.emplace_back(x, y);
            }
        }
    }
}

void RandomMap::AddRectangleObstacle(const Point& top_left, int width, int height) {
    for (int x = top_left.x; x < top_left.x + width; ++x) {
        for (int y = top_left.y; y < top_left.y + height; ++y) {
            if (IsValidCoordinate(x, y)) {
                map[x][y] = { 128, 128, 128 };  // ��ɫ�ϰ���
                obstacle_points.emplace_back(x, y);
            }
        }
    }
}

bool RandomMap::IsValidCoordinate(int x, int y) const {
    return x >= 0 && x < size && y >= 0 && y < size;
}