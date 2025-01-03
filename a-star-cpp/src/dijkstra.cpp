#include "Dijkstra.h"
#include <cmath>
#include <queue>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "vector2mat.h"
#include <unordered_map>
#include <omp.h>

Dijkstra::Dijkstra( std::vector<std::vector<std::array<uint8_t, 3>>>& map, const Point& start_point, const Point& end_point, int robot_size, bool visualizing_search)
    : map(map), start_point(start_point), end_point(end_point), robot_size(robot_size), visualizing_search(visualizing_search) {
    rows = map.size();
    cols = map[0].size();
}

bool Dijkstra::IsValidPoint(const Point& point) {
    int x = point.x, y = point.y;
    int half_size = robot_size;

    for (int dx = -half_size; dx < half_size; ++dx) {
        for (int dy = -half_size; dy < half_size; ++dy) {
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= rows || ny < 0 || ny >= cols) return false;
            if (map[nx][ny] == std::array<uint8_t, 3>{128, 128, 128}) return false;
        }
    }
    return true;
}

std::vector<Point> Dijkstra::GetNeighbors(const Point& point) {
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

std::vector<Point> Dijkstra::FindShortestPath() {
    if (!IsValidPoint(start_point) || !IsValidPoint(end_point)) return {};

    std::priority_queue<std::pair<int, Point>, std::vector<std::pair<int, Point>>, std::greater<>> priority_queue;
    std::unordered_map<Point, int, Point::HashFunction> distances;
    std::unordered_map<Point, Point, Point::HashFunction> predecessors;
    std::set<Point> visited;

    priority_queue.emplace(0, start_point);
    distances[start_point] = 0;

    while (!priority_queue.empty()) {
        auto [current_distance, current_point] = priority_queue.top();
        priority_queue.pop();

        if (visited.count(current_point)) {
            visited.insert(current_point);
        };

        if (visualizing_search) {
            map[current_point.x][current_point.y] = { 0, 255, 0 };
            cv::Mat image = ConvertMapToMat(map);
			cv::Mat fliped_image;
			cv::rotate(image, fliped_image, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::imshow("Dijkstra Visualization", fliped_image); 
            cv::waitKey(1);
        }

        if (current_point == end_point) {
            auto path = ReconstructPath(predecessors);
            Visualize(path);
            return path;
        }

        for (const auto& neighbor : GetNeighbors(current_point)) {
            int distance = current_distance + 1;
            if (distances.find(neighbor) == distances.end() || distance < distances[neighbor]) {
                distances[neighbor] = distance;
                predecessors[neighbor] = current_point;
                priority_queue.emplace(distance, neighbor);
            }
        }
    }
    return {};
}

std::vector<Point> Dijkstra::ReconstructPath(const std::unordered_map<Point, Point, Point::HashFunction>& predecessors) {
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


void Dijkstra::Visualize(const std::vector<Point>& path) {
    for (const auto& point : path) {
        int half_size = robot_size;
        for (int dx = -half_size; dx < half_size; ++dx) {
            for (int dy = -half_size; dy < half_size; ++dy) {
                int nx = point.x + dx, ny = point.y + dy;
                if (nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
                    map[nx][ny] = { 0, 0, 255 };  // Mark path as red
                }
            }
        }
        cv::Mat image = ConvertMapToMat(map);
        cv::Mat fliped_image;
        cv::rotate(image, fliped_image, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::imshow("Dijkstra Visualization", fliped_image);
    }
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void Dijkstra::AddCollisionVolume(const Point& robot_position) {
    int x_center = robot_position.x, y_center = robot_position.y;
    int half_size = robot_size;

    for (int x = x_center - half_size; x < x_center + half_size; ++x) {
        for (int y = y_center - half_size; y < y_center + half_size; ++y) {
            if (x >= 0 && x < rows && y >= 0 && y < cols) {
                map[x][y] = { 128, 128, 128 };  // Mark as obstacle
            }
        }
    }
}
