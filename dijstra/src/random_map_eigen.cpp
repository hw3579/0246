#include "random_map_eigen.h"
#include <cmath>
#include <Eigen/Dense>
#include <algorithm>


#define cross_size 5

RandomMapEigen::RandomMapEigen(int size) : size(size), map(size, size, 3) {
	map.setConstant(255); // initialize the map with white color
    GenerateObstacle();
}

int RandomMapEigen::GetSize() const {
    return size;
}

const std::vector<Point>& RandomMapEigen::GetObstaclePoints() const {
    return obstacle_points;
}

Eigen::Tensor<uint8_t, 3>& RandomMapEigen::GetMap() {
    return map;
}

void RandomMapEigen::Draw_the_endpoints(Eigen::Tensor<uint8_t, 3>& map, const Point start_point, const std::vector<Point>& end_points) {
    try {
		// transform the Eigen::Tensor to cv::Mat
        cv::Mat cv_map(map.dimension(0), map.dimension(1), CV_8UC3, map.data());

		// check if the map is empty
        if (cv_map.empty()) {
			throw std::runtime_error("empty map");
        }

		// draw the start point
		cv::line(cv_map, cv::Point(start_point.x - cross_size, start_point.y), cv::Point(start_point.x + cross_size, start_point.y), cv::Scalar(0, 0, 255), 1); // red horizontal line
		cv::line(cv_map, cv::Point(start_point.x, start_point.y - cross_size), cv::Point(start_point.x, start_point.y + cross_size), cv::Scalar(0, 0, 255), 1); // red vertical line

		// draw the endpoints
        for (const auto& end_point : end_points) {
			cv::line(cv_map, cv::Point(end_point.x - cross_size, end_point.y), cv::Point(end_point.x + cross_size, end_point.y), cv::Scalar(255, 0, 0), 1); // blue horizontal line
			cv::line(cv_map, cv::Point(end_point.x, end_point.y - cross_size), cv::Point(end_point.x, end_point.y + cross_size), cv::Scalar(255, 0, 0), 1); // blue vertical line
        }

		// show the map
        cv::imshow("Map", cv_map);
        cv::waitKey(0);
    }
    catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void RandomMapEigen::GenerateObstacle() {
    obstacle_points.clear();

    Point fountain_center(240, 240);
    int fountain_radius = 40;
    AddCircleObstacle(fountain_center, fountain_radius);

    std::vector<Point> food_stalls = { {240, 120}, {180, 180}, {240, 340} };
    int food_width = 40;
    for (const auto& stall : food_stalls) {
        AddSquareObstacle(stall, food_width);
    }

    std::vector<Point> elevators = { {160, 400}, {320, 400} };
    int elevator_width = 50;
    for (const auto& elevator : elevators) {
        AddSquareObstacle(elevator, elevator_width);
    }

    std::vector<Point> shopping_stores = { {0, 0}, {360, 0} };
    for (const auto& store : shopping_stores) {
        AddRectangleObstacle(store, 120, 480);
    }

    std::vector<Point> no_go_zone = { {120, 0}, {120, 440} };
    for (const auto& ngz : no_go_zone) {
        AddRectangleObstacle(ngz, 240, 40);
    }
}

void RandomMapEigen::AddCircleObstacle(const Point& center, int radius) {
    for (int x = 0; x < size; ++x) {
        for (int y = 0; y < size; ++y) {
            if (std::pow(x - center.x, 2) + std::pow(y - center.y, 2) <= std::pow(radius, 2)) {
                for (int c = 0; c < 3; ++c) {
					map(x, y, c) = 128; // gray
                }
                obstacle_points.emplace_back(x, y);
            }
        }
    }
}

void RandomMapEigen::AddSquareObstacle(const Point& center, int width) {
    int half_width = width / 2;
    for (int x = center.x - half_width; x < center.x + half_width; ++x) {
        for (int y = center.y - half_width; y < center.y + half_width; ++y) {
            if (IsValidCoordinate(x, y)) {
                for (int c = 0; c < 3; ++c) {
					map(x, y, c) = 128; // gray
                }
                obstacle_points.emplace_back(x, y);
            }
        }
    }
}

void RandomMapEigen::AddRectangleObstacle(const Point& top_left, int width, int height) {
    for (int x = top_left.x; x < top_left.x + width; ++x) {
        for (int y = top_left.y; y < top_left.y + height; ++y) {
            if (IsValidCoordinate(x, y)) {
                for (int c = 0; c < 3; ++c) {
					map(x, y, c) = 128; // gray
                }
                obstacle_points.emplace_back(x, y);
            }
        }
    }
}

bool RandomMapEigen::IsValidCoordinate(int x, int y) const {
    return x >= 0 && x < size && y >= 0 && y < size;
}


void RandomMapEigen::RenderRobotOnMap(Eigen::Tensor<uint8_t, 3>& map, const Point& robot_position, int robot_size) {
    int half_size = robot_size;

    for (int dx = -half_size; dx <= half_size; ++dx) {
        for (int dy = -half_size; dy <= half_size; ++dy) {
            int nx = robot_position.x + dx;
            int ny = robot_position.y + dy;

            if (nx >= 0 && nx < map.dimension(1) && ny >= 0 && ny < map.dimension(0)) {
                for (int c = 0; c < 3; ++c) {
                    map(nx, ny, c) = 255; // white color
                }
            }
        }
    }
}