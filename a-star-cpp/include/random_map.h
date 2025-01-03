#ifndef RANDOM_MAP_H
#define RANDOM_MAP_H

#include <vector>
#include <array>
#include <cstdint>
#include "Point.h"

class RandomMap {
public:
    explicit RandomMap(int size = 480);

    void GenerateObstacle();
    int GetSize() const;
    const std::vector<Point>& GetObstaclePoints() const;
    std::vector<std::vector<std::array<uint8_t, 3>>>& GetMap();

private:
    int size;
    std::vector<Point> obstacle_points;
    std::vector<std::vector<std::array<uint8_t, 3>>> map;

    void AddCircleObstacle(const Point& center, int radius);
    void AddSquareObstacle(const Point& center, int width);
    void AddRectangleObstacle(const Point& top_left, int width, int height);
    bool IsValidCoordinate(int x, int y) const;
};

#endif // RANDOM_MAP_H