#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "random_Map.h"
#include "dijkstra.h"

int main() {
    // ���������ͼ
    RandomMap map;
    map.GenerateObstacle();

    // �����յ�
    Point start_point(250, 50);
    Point end_point1(310, 170);
    Point end_point2(150, 70);
    Point end_point3(150, 300);
    Point end_point4(310, 330);
    Point finish_points(230, 430);

    Point end_point = end_point1;

    // ����һ���հ�ͼ��
    int image_size = map.GetSize();  // ��ͼ�ߴ�
    cv::Mat image(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));  // ��ɫ����

    // �����ϰ���
    for (const auto& p : map.GetObstaclePoints()) {
        int x = p.x, y = p.y;
        cv::rectangle(image, cv::Point(x, y), cv::Point(x + 1, y + 1), cv::Scalar(128, 128, 128), -1);  // ��ɫ�ϰ���
    }

    // ���������յ�
    cv::rectangle(image, cv::Point(start_point.x, start_point.y), cv::Point(start_point.x + 1, start_point.y + 1), cv::Scalar(255, 0, 0), -1);  // ��ɫ���
    cv::rectangle(image, cv::Point(end_point.x, end_point.y), cv::Point(end_point.x + 1, end_point.y + 1), cv::Scalar(0, 0, 255), -1);  // ��ɫ�յ�

    // ��ʾ��ͼ
    cv::Mat flipped_image;
    cv::flip(image, flipped_image, 0);  // ��ֱ��ת
    cv::imshow("Map", flipped_image);
    cv::waitKey(0);

    // ����Dijkstra�㷨
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
