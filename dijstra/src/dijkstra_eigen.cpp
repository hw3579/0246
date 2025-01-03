#include "dijkstra_eigen.h"
#include <cmath>
#include <ctime>
#include <queue>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "main.h"

DijkstraEigen::DijkstraEigen(Eigen::Tensor<uint8_t, 3>& map,
    const Point& start_point,
    const Point& end_point,
    int robot_size,
    bool visualizing_search)
    : map(map), start_point(start_point), end_point(end_point), robot_size(robot_size), visualizing_search(visualizing_search) {
    rows = map.dimension(0);
    cols = map.dimension(1);
}

DijkstraEigen::DijkstraEigen(Eigen::Tensor<uint8_t, 3>& map,
    const Point& start_point,
    const std::vector<Point>& end_points,
    int robot_size,
    bool visualizing_search)
    : map(map), start_point(start_point), end_points(end_points), robot_size(robot_size), visualizing_search(visualizing_search) {
    rows = map.dimension(0);
    cols = map.dimension(1);
}

bool DijkstraEigen::IsValidPoint(const Point& point) {
    int x = point.x, y = point.y;
    int half_size = robot_size;

    for (int dx = -half_size; dx < half_size; ++dx) {
        for (int dy = -half_size; dy < half_size; ++dy) {
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= rows || ny < 0 || ny >= cols || map(nx, ny, 0) == 128) {
                return false;
            }
        }
    }
    return true;
}

std::vector<Point> DijkstraEigen::GetNeighbors(const Point& point) {
    std::vector<Point> neighbors;
    std::array<Point, 4> directions = { Point(-1, 0), Point(1, 0), Point(0, -1), Point(0, 1) };

    for (const auto& dir : directions) {
        Point neighbor = point + dir;
        if (IsValidPoint(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }
    return neighbors;
}

std::vector<Point> DijkstraEigen::FindShortestPath(Point end_point, std::vector<Point>& end_points) {
    if (!IsValidPoint(start_point)) return {};

    std::priority_queue<std::pair<int, Point>, std::vector<std::pair<int, Point>>, std::greater<>> priority_queue;
    std::unordered_map<Point, int, Point::HashFunction> distances;
    std::unordered_map<Point, Point, Point::HashFunction> predecessors;

    priority_queue.emplace(0, start_point);
    distances[start_point] = 0;

    // 创建一个三通道图像用于可视化
    cv::Mat image(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            if (map(x, y, 0) == 128) {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128); // 灰色障碍物
            }
        }
    }

	// 绘制起点和终点的十字

	Drawline(image, start_point, end_points);

    //Drawline(image, start_point, end_points);
    int counter = 0;
    while (!priority_queue.empty()) {
        auto [current_distance, current_point] = priority_queue.top();
        priority_queue.pop();

        if (current_point == end_point) {
            auto path = ReconstructPath(predecessors);
            //Visualize(path, image);
            return path;
        }

        for (const auto& neighbor : GetNeighbors(current_point)) {
            int new_distance = current_distance + 1;
            if (distances.find(neighbor) == distances.end() || new_distance < distances[neighbor]) {
                distances[neighbor] = new_distance;
                predecessors[neighbor] = current_point;
                priority_queue.emplace(new_distance, neighbor);
            }
        }

        if (visualizing_search) {
            image.at<cv::Vec3b>(current_point.y, current_point.x) = cv::Vec3b(0, 255, 0); // 灰绿色标记
            //cv::flip(image, image, 0);
            counter++;
            if (counter % 1000 == 0) { // 每弹出10次才显示
                cv::Mat rotated_image;
				cv::flip(image, rotated_image, 0);
                cv::imshow("Dijkstra Visualization - Find path", rotated_image);
                cv::waitKey(1);
            }
        }
    }
    std::string filename = "./img/find_path_" + GetTimestamp() + ".png";
    cv::imwrite("Dijkstra Visualization - Find path.png", image);
	cv::destroyAllWindows();
    return {};
}

std::vector<Point> DijkstraEigen::ReconstructPath(const std::unordered_map<Point, Point, Point::HashFunction>& predecessors) {
    std::vector<Point> path;
    Point current = end_point;

    while (predecessors.find(current) != predecessors.end()) {
        path.push_back(current);
        current = predecessors.at(current);
    }
    path.push_back(start_point);
    std::reverse(path.begin(), path.end());
    return path;
}

void DijkstraEigen::Visualize(const std::vector<Point>& path, cv::Mat& image) {
    cv::Mat rotated_image;
	cv::flip(image, image, 0);
	cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);

	int counter = 0;

    for (const auto& point : path) {
        int half_size = robot_size;
        for (int dx = -half_size; dx < half_size; ++dx) {
            for (int dy = -half_size; dy < half_size; ++dy) {
                int nx = point.x + dx, ny = point.y + dy;
                if (nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
                    image.at<cv::Vec3b>(nx, ny) = cv::Vec3b(0, 0, 255);  // 标记路径为红色
                }
            }
        }
		counter++;
		if (counter % 10 == 0) {
			cv::rotate(image, rotated_image, cv::ROTATE_90_COUNTERCLOCKWISE);
			cv::imshow("Dijkstra Visualization", rotated_image);
			cv::waitKey(1);
		}
    }
    //保存图像
    // 生成带有时间戳的文件名
    std::string filename = "./img/Visualization" + GetTimestamp() + ".png";
    cv::imwrite(filename, rotated_image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void DijkstraEigen::AddCollisionVolume(const Point& robot_position) {
    int x_center = robot_position.x, y_center = robot_position.y;
    int half_size = robot_size;

    for (int x = x_center - half_size; x < x_center + half_size; ++x) {
        for (int y = y_center - half_size; y < y_center + half_size; ++y) {
            if (x >= 0 && x < rows && y >= 0 && y < cols) {
                map(x, y, 0) = 128;  // 标记为障碍物
            }
        }
    }
}
// 获取当前时间戳
std::string GetTimestamp() {
    std::time_t now = std::time(nullptr);
    char buf[20];
    std::strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", std::localtime(&now));
    return std::string(buf);
}


