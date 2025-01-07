#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "random_map_eigen.h"
#include "dijkstra_eigen.h"
#include "c_space.h"


#define is_include_tank false

int main() {
    
    // craete a random map
    RandomMapEigen map(480);
    RobotSize robot_size(10, 4);

    map.GenerateObstacle();
    // define the start point
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
    

    if (is_include_tank) {
        Point water_point = start_point - Point(0, robot_size.main_size + robot_size.tank_size); // the robot's water point center
        map.RenderRobotOnMap(map.GetMap(), water_point, robot_size.tank_size); // make the robot's water point is not an obstacle
    }


	// create a blank image
    int image_size = map.GetSize();
    cv::Mat image(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));  // 3-channel white image

	// draw obstacles
    const auto& obstacle_map = map.GetMap();
    for (int x = 0; x < image_size; ++x) {
        for (int y = 0; y < image_size; ++y) {
			if (obstacle_map(x, y, 0) == 128) { // gray obstacles
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128);
            }
        }
    }

    // get the all index of the map
    // check the all points validation

    std::vector<bool> all_points_validation;
    if (is_include_tank) {
        DijkstraEigen dijkstra(map.GetMap(), start_point, start_point, robot_size);

        for (int x = 0; x < image_size; ++x) {
            for (int y = 0; y < image_size; ++y) {
                //all_points_validation.push_back(dijkstra.IsValidPoint(Point(x, y)));
                all_points_validation.push_back(dijkstra.IsValidPoint(Point(x, y), robot_size));
            }
        }
    }
    if (!is_include_tank) {
        DijkstraEigen dijkstra(map.GetMap(), start_point, start_point);

        for (int x = 0; x < image_size; ++x) {
            for (int y = 0; y < image_size; ++y) {
                all_points_validation.push_back(dijkstra.IsValidPoint(Point(x, y)));
            }
        }
    }

	// draw the configuration space
	for (int x = 0; x < image_size; ++x) {
		for (int y = 0; y < image_size; ++y) {
			if (all_points_validation[x * image_size + y]) { 
				image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
			}
		}
	}

	// draw the start point and end points
    Drawline(image, start_point, end_points);

    // show
    cv::Mat fliped_image;
    cv::flip(image, fliped_image, 0);
    cv::imshow("Configuration Space", fliped_image);
	cv::imwrite("Configuration_Space.png", fliped_image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}

void Drawline(cv::Mat& image, const Point& current_start_point, const std::vector<Point>& end_points, bool y_inverse) {
	// draw the start point
	cv::line(image, cv::Point(current_start_point.x - cross_size, current_start_point.y), cv::Point(current_start_point.x + cross_size, current_start_point.y), cv::Scalar(0, 0, 255), 1); // red horizontal line
	cv::line(image, cv::Point(current_start_point.x, current_start_point.y - cross_size), cv::Point(current_start_point.x, current_start_point.y + cross_size), cv::Scalar(0, 0, 255), 1); // red vertical line

	// draw the end points
    for (const auto& end_point : end_points) {
		cv::line(image, cv::Point(end_point.x - cross_size, end_point.y), cv::Point(end_point.x + cross_size, end_point.y), cv::Scalar(255, 0, 0), 1); // blue horizontal line
		cv::line(image, cv::Point(end_point.x, end_point.y - cross_size), cv::Point(end_point.x, end_point.y + cross_size), cv::Scalar(255, 0, 0), 1); // blue vertical line
    }
}