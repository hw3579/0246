#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "random_map_eigen.h"
#include "dijkstra_eigen.h"
#include "main.h"




int main() {
    // 创建随机地图
    RandomMapEigen map(480); // 地图大小
    map.GenerateObstacle();

    // 定义起点和多个终点
    Point start_point(250, 50);
    Point end_point1(310, 170);
    Point end_point2(150, 70);
    Point end_point3(150, 300);
    Point end_point4(310, 330);
    Point finish_points(230, 430);
    std::vector<Point> end_points = {
        end_point1,
        end_point2,
        end_point3,
        end_point4,
        finish_points
    };

	//map.Draw_the_endpoints(map.GetMap(), start_point, end_points);

    // 遍历多个终点
    Point current_start_point = start_point;


    for (const auto& end_point : end_points) {
        // 创建一个空白图像
        int image_size = map.GetSize();
        cv::Mat image(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));  // 三通道白色背景

        // 绘制障碍物
        const auto& obstacle_map = map.GetMap();
        for (int x = 0; x < image_size; ++x) {
            for (int y = 0; y < image_size; ++y) {
                if (obstacle_map(x, y, 0) == 128) { // 灰色障碍物
                    image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128);
                }
            }
        }


        // 绘制起点和终点的十字
		Drawline(image, current_start_point, end_points);


        //没问题这里
        // 
        // 
        //显示初始地图
        //cv::Mat fliped_image;
        //cv::flip(image, fliped_image, 0);
        //cv::imshow("Map", fliped_image);
        //cv::waitKey(0);
        //cv::destroyWindow("Map"); 

        // 运行 Dijkstra 算法
		DijkstraEigen dijkstra(map.GetMap(), current_start_point, end_point); // end_points 为多个终点 end_point 为当前终点
        auto path = dijkstra.FindShortestPath(end_point, end_points);

        if (!path.empty()) {
            std::cout << "Shortest Path Found to (" << end_point.x << ", " << end_point.y << ")!" << std::endl;
            //输出距离
			std::cout << "The distance is " << path.size() << std::endl;
            dijkstra.Visualize(path, image); // 可视化路径
        }
        else {
            std::cout << "No path found to (" << end_point.x << ", " << end_point.y << ")." << std::endl;
        }

        // 更新起点为当前终点
        current_start_point = end_point;
    }

    cv::destroyAllWindows();
    return 0;
}


void Drawline(cv::Mat& image, const Point& current_start_point, const std::vector<Point>& end_points, bool y_inverse) {
    // 绘制起点的十字
        cv::line(image, cv::Point(current_start_point.x - cross_size, current_start_point.y), cv::Point(current_start_point.x + cross_size, current_start_point.y), cv::Scalar(0, 0, 255), 1); // 红色横线
        cv::line(image, cv::Point(current_start_point.x, current_start_point.y - cross_size), cv::Point(current_start_point.x, current_start_point.y + cross_size), cv::Scalar(0, 0, 255), 1); // 红色竖线

        // 遍历并绘制每个终点的十字
        for (const auto& end_point : end_points) {
            cv::line(image, cv::Point(end_point.x - cross_size, end_point.y), cv::Point(end_point.x + cross_size, end_point.y), cv::Scalar(255, 0, 0), 1); // 蓝色横线
            cv::line(image, cv::Point(end_point.x, end_point.y - cross_size), cv::Point(end_point.x, end_point.y + cross_size), cv::Scalar(255, 0, 0), 1); // 蓝色竖线
        }
}