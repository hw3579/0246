#ifndef POINT_H
#define POINT_H

#include <functional>

class Point {
public:
    // 构造函数
    Point(int x = 0, int y = 0)
        : x(x), y(y) {}

    bool operator<(const Point& other) const {
        return (x < other.x) || (x == other.x && y < other.y);
    }

	bool operator>(const Point& other) const {
		return (x > other.x) || (x == other.x && y > other.y);
	}

	Point operator-(const Point& other) const {
		return Point(x - other.x, y - other.y);
	}

    // 重载加法运算符
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }

    // 重载比较运算符
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    // 自定义哈希函数，用于哈希表
    struct HashFunction {
        size_t operator()(const Point& point) const {
            return std::hash<int>()(point.x) ^ (std::hash<int>()(point.y) << 1);
        }
    };

    // 坐标成员变量
    int x, y;
};

#endif // POINT_H
