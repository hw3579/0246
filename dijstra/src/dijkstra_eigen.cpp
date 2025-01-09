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
    const Point& end_point,
    RobotSize& structure_robot_size,
    bool visualizing_search)
	: map(map), start_point(start_point), end_point(end_point), structure_robot_size(structure_robot_size), visualizing_search(visualizing_search) {
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


bool DijkstraEigen::IsValidPoint(const Point& point, RobotSize& robot_size) {
    int x = point.x, y = point.y;
    int main_half_size = robot_size.main_size ;
    int tank_half_size = robot_size.tank_size ;

	// check the main square
    for (int dx = -main_half_size; dx < main_half_size; ++dx) {
        for (int dy = -main_half_size; dy < main_half_size; ++dy) {
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= rows || ny < 0 || ny >= cols || map(nx, ny, 0) == 128) {
                return false;
            }
        }
    }

	// check the tank square (assuming the tank is behind the main square)
    for (int dx = -tank_half_size; dx < tank_half_size; ++dx) {
        for (int dy = -tank_half_size; dy < tank_half_size; ++dy) {
			int nx = x + dx, ny = y + dy - main_half_size - tank_half_size;  // water tank is behind the main square
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

std::vector<Point> DijkstraEigen::GetNeighbors(const Point& point, RobotSize& robot_size) {
    std::vector<Point> neighbors;
    std::array<Point, 4> directions = { Point(-1, 0), Point(1, 0), Point(0, -1), Point(0, 1) };

    for (const auto& dir : directions) {
        Point neighbor = point + dir;
        if (IsValidPoint(neighbor, robot_size)) {
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

	// create a blank image for visualization
    cv::Mat image(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            if (map(x, y, 0) == 128) {
				image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128); // gray color with obstacles
            }
        }
    }

	// draw the start point and end points

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
			image.at<cv::Vec3b>(current_point.y, current_point.x) = cv::Vec3b(0, 255, 0); // green color in searched area
            //cv::flip(image, image, 0);
            counter++;
			if (counter % showing_delay_times == 0) { // every 1000 iterations
                cv::Mat rotated_image;
				cv::flip(image, rotated_image, 0);
                cv::imshow("Dijkstra Visualization - Find path", rotated_image);
                cv::waitKey(1);
            }
        }
    }

    std::string filename = "./img/find_path_" + GetTimestamp() + ".png";
    cv::imwrite(filename, image);
	cv::imshow("Dijkstra Visualization - Find path", image);
	cv::destroyAllWindows();
    return {};
}


std::vector<Point> DijkstraEigen::FindShortestPath(Point end_point, std::vector<Point>& end_points, RobotSize& robot_size) {
    if (!IsValidPoint(start_point, robot_size)) return {};

    std::priority_queue<std::pair<int, Point>, std::vector<std::pair<int, Point>>, std::greater<>> priority_queue;
    std::unordered_map<Point, int, Point::HashFunction> distances;
    std::unordered_map<Point, Point, Point::HashFunction> predecessors;

    priority_queue.emplace(0, start_point);
    distances[start_point] = 0;

    // create a blank image for visualization
    cv::Mat image(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            if (map(x, y, 0) == 128) {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128); // gray color with obstacles
            }
        }
    }

    // draw the start point and end points

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

        for (const auto& neighbor : GetNeighbors(current_point, robot_size)) {
            int new_distance = current_distance + 1;
            if (distances.find(neighbor) == distances.end() || new_distance < distances[neighbor]) {
                distances[neighbor] = new_distance;
                predecessors[neighbor] = current_point;
                priority_queue.emplace(new_distance, neighbor);
            }
        }

        if (visualizing_search) {
            image.at<cv::Vec3b>(current_point.y, current_point.x) = cv::Vec3b(0, 255, 0); // green color in searched area
            //cv::flip(image, image, 0);
            counter++;
            if (counter % showing_delay_times == 0) { // every 1000 iterations
                cv::Mat rotated_image;
                cv::flip(image, rotated_image, 0);
                cv::imshow("Dijkstra Visualization - Find path", rotated_image);
                cv::waitKey(1);
            }
        }
    }
    std::string filename = "./img/find_path_" + GetTimestamp() + ".png";
    cv::imwrite(filename , image);
    cv::imshow("Dijkstra Visualization - Find path", image);
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

	int counter = 0;
    cv::flip(image, image, 0);

    for (const auto& point : path) {
        int half_size = robot_size;
        for (int dx = -half_size; dx < half_size; ++dx) {
            for (int dy = -half_size; dy < half_size; ++dy) {
                int nx = point.x + dx, ny = point.y + dy;
                if (nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
					image.at<cv::Vec3b>(image.rows - ny - 1, nx) = cv::Vec3b(0, 0, 255);  // path in red color
                }
            }
        }
		counter++;
		if (counter % 10 == 0) {
			cv::imshow("Dijkstra Visualization", image);
			cv::waitKey(1);
		}
    }

    for (const auto& point : path) {
        //draw the center point in black
        image.at<cv::Vec3b>(image.rows - point.y - 1, point.x) = cv::Vec3b(0, 0, 0);  // center point in black
    }
	// save the image with timestamp
    std::string filename = "./img/Visualization" + GetTimestamp() + ".png";
	cv::imshow("Dijkstra Visualization", image);
    cv::imwrite(filename, image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void DijkstraEigen::Visualize(const std::vector<Point>& path, cv::Mat& image, RobotSize& robot_size) {

    int counter = 0;
    cv::flip(image, image, 0);

    for (const auto& point : path) {
        int half_size = robot_size.main_size;
        for (int dx = -half_size; dx < half_size; ++dx) {
            for (int dy = -half_size; dy < half_size; ++dy) {
                int nx = point.x + dx, ny = point.y + dy;
                if (nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
                    image.at<cv::Vec3b>(image.rows - ny - 1, nx) = cv::Vec3b(0, 0, 255);  // path in red color
                }
            }
        }

		half_size = robot_size.tank_size;
		for (int dx = -half_size; dx < half_size; ++dx) {
			for (int dy = -half_size; dy < half_size; ++dy) {
				int nx = point.x + dx, ny = point.y + dy - robot_size.main_size - robot_size.tank_size;  // water tank is behind the main square
				if (nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
					image.at<cv::Vec3b>(image.rows - ny - 1, nx) = cv::Vec3b(0, 0, 255);  // path in red color
				}
			}
		}
        counter++;
        if (counter % 10 == 0) {
            cv::imshow("Dijkstra Visualization", image);
            cv::waitKey(1);
        }

    }

    //for (const auto& point : path) {
    //    //draw the center point in black
    //    image.at<cv::Vec3b>(image.rows - point.y - 1, point.x) = cv::Vec3b(0, 0, 0);  // center point in black
    //}
    // save the image with timestamp
    std::string filename = "./img/Visualization" + GetTimestamp() + ".png";
	cv::imshow("Dijkstra Visualization", image);
    cv::imwrite(filename, image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}


// get the current timestamp
std::string GetTimestamp() {
    std::time_t now = std::time(nullptr);
    char buf[20];
    struct tm timeinfo;
#ifdef _WIN32
    localtime_s(&timeinfo, &now);
#else
	localtime_r(&now, &timeinfo);
#endif
    std::strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", &timeinfo);
    return std::string(buf);
}

// Explore all points in the map
#include <unordered_set>
void DijkstraEigen::ExploreAllPoints() {
    std::queue<Point> points_to_explore;
    std::unordered_set<Point, Point::HashFunction> visited_points;

    points_to_explore.push(start_point);
    visited_points.insert(start_point);

	// create a blank image for visualization
    cv::Mat image(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            if (map(x, y, 0) == 128) {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128); // 灰色表示障碍物
            }
        }
    }

	// use function isValidPoint to check if the point is valid
    int image_size = map.dimension(0);
    std::vector<bool> all_points_validation;
    for (int x = 0; x < image_size; ++x) {
        for (int y = 0; y < image_size; ++y) {
            all_points_validation.push_back(IsValidPoint(Point(x, y)));
        }
    }
    
    for (int x = 0; x < image_size; ++x) {
        for (int y = 0; y < image_size; ++y) {
            if (all_points_validation[x * image_size + y]) {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
            }
        }
    }

	// make the robot move from left to right and then from right to left
    bool direction = true; 
	int robot_length = robot_size * 2; // robot length
    for (int y = 0; y < image_size; y += robot_length) {
        if (direction) {
            for (int x = 0; x < image_size; ++x) {
                Point current_point(x, y);
                if (IsValidPoint(current_point)) {
                    int half_size = robot_size;
                    for (int dx = -half_size; dx < half_size; ++dx) {
                        for (int dy = -half_size; dy < half_size; ++dy) {
                            int nx = x + dx, ny = y + dy;
                            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
								image.at<cv::Vec3b>(ny, nx) = cv::Vec3b(255, 0, 0); // blue color for the area the robot has passed
                            }
                        }
                    }
                    cv::imshow("Dijkstra Visualization - Explore all points", image);
                    cv::waitKey(1);
                }
            }
        }
        else {
            for (int x = image_size - 1; x >= 0; --x) {
                Point current_point(x, y);
                if (IsValidPoint(current_point)) {
                    int half_size = robot_size;
                    for (int dx = -half_size; dx < half_size; ++dx) {
                        for (int dy = -half_size; dy < half_size; ++dy) {
                            int nx = x + dx, ny = y + dy;
                            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
								image.at<cv::Vec3b>(ny, nx) = cv::Vec3b(255, 0, 0); // blue color for the area the robot has passed
                            }
                        }
                    }
                    cv::imshow("Dijkstra Visualization - Explore all points", image);
                    cv::waitKey(1);
                }
            }
        }
		direction = !direction; // change the direction
    }




	cv::imshow("Dijkstra Visualization - Explore all points", image);
	cv::waitKey(0);
}