#ifndef DIJKSTRA_EIGEN_H
#define DIJKSTRA_EIGEN_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "point.h"
#include <ctime>
#include <string>
#include <unsupported/Eigen/CXX11/Tensor>
#define cross_size 5
#define showing_delay_times 1000

struct RobotSize {
    int main_size;  // robotsize
	int tank_size;  // tanksize

    RobotSize() : main_size(10), tank_size(4) {}

    RobotSize(int main_size, int tank_size)
        : main_size(main_size), tank_size(tank_size) {}
};


class DijkstraEigen {
public:
    DijkstraEigen(Eigen::Tensor<uint8_t, 3>& map,
        const Point& start_point,
        const Point& end_point,
        int robot_size = 10,
        bool visualizing_search = true);

    DijkstraEigen(Eigen::Tensor<uint8_t, 3>& map,
        const Point& start_point,
        const Point& end_point,
		RobotSize& structure_robot_size,
        bool visualizing_search = true);

    std::vector<Point> FindShortestPath( Point end_point, std::vector<Point>& end_points);
    std::vector<Point> FindShortestPath(Point end_point, std::vector<Point>& end_points, RobotSize& robot_size);
    void Visualize(const std::vector<Point>& path, cv::Mat& image);
    void Visualize(const std::vector<Point>& path, cv::Mat& image, RobotSize& robot_size);
    bool IsValidPoint(const Point& point);
    bool IsValidPoint(const Point& point, RobotSize& robot_size);
	void ExploreAllPoints();

private:
    Eigen::Tensor<uint8_t, 3>& map;
    Point start_point;
    Point end_point;
	std::vector<Point> end_points;
    int robot_size;  
    RobotSize structure_robot_size;
    bool visualizing_search;
    int rows;
    int cols;
    std::vector<Point> GetNeighbors(const Point& point);
    std::vector<Point> GetNeighbors(const Point& point, RobotSize& robot_size);
    std::unordered_map<Point, int, Point::HashFunction> distances;
    std::vector<Point> ReconstructPath(const std::unordered_map<Point, Point, Point::HashFunction>& predecessors);
};

std::string GetTimestamp();

#endif // DIJKSTRA_EIGEN_H