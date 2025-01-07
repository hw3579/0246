#ifndef POINT_H
#define POINT_H

#include <functional>

class Point {
public:
    // ���캯��
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

    // ���ؼӷ������
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }

    // ���رȽ������
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    // �Զ����ϣ���������ڹ�ϣ��
    struct HashFunction {
        size_t operator()(const Point& point) const {
            return std::hash<int>()(point.x) ^ (std::hash<int>()(point.y) << 1);
        }
    };

    // �����Ա����
    int x, y;
};

#endif // POINT_H
