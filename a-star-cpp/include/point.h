#ifndef POINT_H
#define POINT_H

#include <limits>
#include <cstddef>
#include <functional>

class Point {
public:
    // 构造函数，初始化坐标和默认值
    Point(int x = 0, int y = 0)
        : x(x), y(y), cost(std::numeric_limits<int>::max()), parent(nullptr) {}

    // 获取X和Y坐标
    int getX() const { return x; }
    int getY() const { return y; }

    // 获取和设置代价
    int getCost() const { return cost; }
    void setCost(int new_cost) { cost = new_cost; }

    // 获取和设置父节点
    Point* getParent() const { return parent; }
    void setParent(Point* new_parent) { parent = new_parent; }

    // 重载加法运算符
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }

    // 比较运算符，用于排序和哈希
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const Point& other) const {
        return cost < other.cost;
    }

    // 用于哈希表的自定义哈希函数
    struct HashFunction {
        size_t operator()(const Point& point) const {
            return std::hash<int>()(point.x) ^ (std::hash<int>()(point.y) << 1);
        }
    };


    // 成员变量
    int x, y;
    int cost;
    Point* parent;

private:
};

#endif // POINT_H
