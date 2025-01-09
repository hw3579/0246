#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "random_map_eigen.h"
#include "dijkstra_eigen.h"
#include "main.h"

#define is_include_tank true



int main() {
    // craete a random map
    RandomMapEigen map(480);
    map.GenerateObstacle();
    RobotSize robot_size(10, 4);

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



    Point current_start_point = start_point;

    if (is_include_tank) {
        Point water_point = current_start_point - Point(0, robot_size.main_size + robot_size.tank_size); // the robot's water point center
        map.RenderRobotOnMap(map.GetMap(), water_point, robot_size.tank_size); // make the robot's water point is not an obstacle
    }


    for (const auto& end_point : end_points) {
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

        // draw the start point and end points
        Drawline(image, current_start_point, end_points);


        //show the initial map
        //cv::Mat fliped_image;
        //cv::flip(image, fliped_image, 0);
        //cv::imshow("Map", fliped_image);
        //cv::waitKey(0);
        //cv::destroyWindow("Map"); 


        if (is_include_tank) {
            // create DijkstraEigen object
            DijkstraEigen dijkstra(map.GetMap(), current_start_point, end_point, robot_size);
            auto path = dijkstra.FindShortestPath(end_point, end_points, robot_size);

            if (!path.empty()) {
                std::cout << "Shortest Path Found to (" << end_point.x << ", " << end_point.y << ")!" << std::endl;
                // output the distance
                std::cout << "The distance is " << path.size() << std::endl;
                dijkstra.Visualize(path, image, robot_size); // visualize the path
            }
            else {
                std::cout << "No path found to (" << end_point.x << ", " << end_point.y << ")." << std::endl;
            }
        }

		if (!is_include_tank) {
			// create DijkstraEigen object
			DijkstraEigen dijkstra(map.GetMap(), current_start_point, end_point);
            //dijkstra.ExploreAllPoints();
			auto path = dijkstra.FindShortestPath(end_point, end_points);

			if (!path.empty()) {
				std::cout << "Shortest Path Found to (" << end_point.x << ", " << end_point.y << ")!" << std::endl;
				// output the distance
				std::cout << "The distance is " << path.size() << std::endl;
				dijkstra.Visualize(path, image); // visualize the path
			}
			else {
				std::cout << "No path found to (" << end_point.x << ", " << end_point.y << ")." << std::endl;
			}
		}

		// renew the start point
        current_start_point = end_point;
    }

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