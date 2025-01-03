#ifndef DIJKSTRA_EIGEN_H
#define DIJKSTRA_EIGEN_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "Point.h"
#include <ctime>
#include <string>
#include <unsupported/Eigen/CXX11/Tensor>
#define cross_size 5

class DijkstraEigen {
public:
    DijkstraEigen::DijkstraEigen(Eigen::Tensor<uint8_t, 3>& map,
        const Point& start_point,
        const Point& end_point,
        int robot_size = 10,
        bool visualizing_search = true);


    DijkstraEigen::DijkstraEigen(Eigen::Tensor<uint8_t, 3>& map,
        const Point& start_point,
        const std::vector<Point>& end_points,
        int robot_size,
        bool visualizing_search);

    std::vector<Point> FindShortestPath( Point end_point, std::vector<Point>& end_points);
    void AddCollisionVolume(const Point& robot_position);
    void Visualize(const std::vector<Point>& path, cv::Mat& image);

private:
    Eigen::Tensor<uint8_t, 3>& map;
    Point start_point;
    Point end_point;
	std::vector<Point> end_points;
    int robot_size;  // Robot size in half-units
    bool visualizing_search;
    int rows;
    int cols;

    bool IsValidPoint(const Point& point);
    std::vector<Point> GetNeighbors(const Point& point);
    std::unordered_map<Point, int, Point::HashFunction> distances;
    std::vector<Point> ReconstructPath(const std::unordered_map<Point, Point, Point::HashFunction>& predecessors);
};

std::string GetTimestamp();

#endif // DIJKSTRA_EIGEN_H