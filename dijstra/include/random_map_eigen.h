#ifndef RANDOM_MAP_EIGEN_H
#define RANDOM_MAP_EIGEN_H

#include <vector>
#include <Eigen/Dense>
#include "Point.h"
#include <opencv2/opencv.hpp>
#include <unsupported/Eigen/CXX11/Tensor>

class RandomMapEigen {
public:
    explicit RandomMapEigen(int size = 480);

    void GenerateObstacle();
    int GetSize() const;
    const std::vector<Point>& GetObstaclePoints() const;
    Eigen::Tensor<uint8_t, 3>& GetMap();
    void Draw_the_endpoints(Eigen::Tensor<uint8_t, 3>& map, const Point start_point, const std::vector<Point>& end_points);
    void RenderRobotOnMap(Eigen::Tensor<uint8_t, 3>& map, const Point& robot_position, int robot_size);
private:
    int size;
    std::vector<Point> obstacle_points;
	Eigen::Tensor<uint8_t, 3> map; // 3 channel image

    void AddCircleObstacle(const Point& center, int radius);
    void AddSquareObstacle(const Point& center, int width);
    void AddRectangleObstacle(const Point& top_left, int width, int height);
    bool IsValidCoordinate(int x, int y) const;
};

#endif // RANDOM_MAP_EIGEN_H
