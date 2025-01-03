#ifndef POINT_H
#define POINT_H

#include <limits>
#include <cstddef>
#include <functional>

class Point {
public:
    // ���캯������ʼ�������Ĭ��ֵ
    Point(int x = 0, int y = 0)
        : x(x), y(y), cost(std::numeric_limits<int>::max()), parent(nullptr) {}

    // ��ȡX��Y����
    int getX() const { return x; }
    int getY() const { return y; }

    // ��ȡ�����ô���
    int getCost() const { return cost; }
    void setCost(int new_cost) { cost = new_cost; }

    // ��ȡ�����ø��ڵ�
    Point* getParent() const { return parent; }
    void setParent(Point* new_parent) { parent = new_parent; }

    // ���ؼӷ������
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }

    // �Ƚ����������������͹�ϣ
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const Point& other) const {
        return cost < other.cost;
    }

    // ���ڹ�ϣ����Զ����ϣ����
    struct HashFunction {
        size_t operator()(const Point& point) const {
            return std::hash<int>()(point.x) ^ (std::hash<int>()(point.y) << 1);
        }
    };


    // ��Ա����
    int x, y;
    int cost;
    Point* parent;

private:
};

#endif // POINT_H
