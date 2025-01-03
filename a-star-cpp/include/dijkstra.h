#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <array>
#include <opencv2/opencv.hpp>
#include "Point.h"

class Dijkstra {
public:
    Dijkstra(std::vector<std::vector<std::array<uint8_t, 3>>>& map, const Point& start_point, const Point& end_point, int robot_size = 10, bool visualizing_search = true);

    std::vector<Point> FindShortestPath();
    void AddCollisionVolume(const Point& robot_position);
    void Visualize(const std::vector<Point>& path);

private:
    std::vector<std::vector<std::array<uint8_t, 3>>>& map;  // ÒýÓÃµØÍ¼
    Point start_point;
    Point end_point;
    int robot_size;  // Robot size in half-units
    bool visualizing_search;
    int rows;
    int cols;

    bool IsValidPoint(const Point& point);
    std::vector<Point> GetNeighbors(const Point& point);
    std::unordered_map<Point, int, Point::HashFunction> distances;
    std::vector<Point> ReconstructPath(const std::unordered_map<Point, Point, Point::HashFunction>& predecessors);
    //std::vector<Point> ReconstructPath(const std::unordered_map<Point, Point>& predecessors);
};

#endif // DIJKSTRA_H
