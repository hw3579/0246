#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "random_Map.h"
#include "dijkstra.h"

int main() {
    // 创建随机地图
    RandomMap map;
    map.GenerateObstacle();

    // 起点和终点
    Point start_point(250, 50);
    Point end_point1(310, 170);
    Point end_point2(150, 70);
    Point end_point3(150, 300);
    Point end_point4(310, 330);
    Point finish_points(230, 430);

    Point end_point = end_point1;

    // 创建一个空白图像
    int image_size = map.GetSize();  // 地图尺寸
    cv::Mat image(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));  // 白色背景

    // 绘制障碍物
    for (const auto& p : map.GetObstaclePoints()) {
        int x = p.x, y = p.y;
        cv::rectangle(image, cv::Point(x, y), cv::Point(x + 1, y + 1), cv::Scalar(128, 128, 128), -1);  // 灰色障碍物
    }

    // 绘制起点和终点
    cv::rectangle(image, cv::Point(start_point.x, start_point.y), cv::Point(start_point.x + 1, start_point.y + 1), cv::Scalar(255, 0, 0), -1);  // 蓝色起点
    cv::rectangle(image, cv::Point(end_point.x, end_point.y), cv::Point(end_point.x + 1, end_point.y + 1), cv::Scalar(0, 0, 255), -1);  // 红色终点

    // 显示地图
    cv::Mat flipped_image;
    cv::flip(image, flipped_image, 0);  // 垂直翻转
    cv::imshow("Map", flipped_image);
    cv::waitKey(0);

    // 运行Dijkstra算法
    Dijkstra dijkstra(map.GetMap(), start_point, end_point);
    auto path = dijkstra.FindShortestPath();

    if (!path.empty()) {
        std::cout << "Shortest Path Found!" << std::endl;
        dijkstra.Visualize(path);
    }
    else {
        std::cout << "No path found." << std::endl;
    }

    return 0;
}
