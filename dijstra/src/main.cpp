#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "random_map_eigen.h"
#include "dijkstra_eigen.h"
#include "main.h"




int main() {
    // ���������ͼ
    RandomMapEigen map(480); // ��ͼ��С
    map.GenerateObstacle();

    // �������Ͷ���յ�
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

    // ��������յ�
    Point current_start_point = start_point;


    for (const auto& end_point : end_points) {
        // ����һ���հ�ͼ��
        int image_size = map.GetSize();
        cv::Mat image(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));  // ��ͨ����ɫ����

        // �����ϰ���
        const auto& obstacle_map = map.GetMap();
        for (int x = 0; x < image_size; ++x) {
            for (int y = 0; y < image_size; ++y) {
                if (obstacle_map(x, y, 0) == 128) { // ��ɫ�ϰ���
                    image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128);
                }
            }
        }


        // ���������յ��ʮ��
		Drawline(image, current_start_point, end_points);


        //û��������
        // 
        // 
        //��ʾ��ʼ��ͼ
        //cv::Mat fliped_image;
        //cv::flip(image, fliped_image, 0);
        //cv::imshow("Map", fliped_image);
        //cv::waitKey(0);
        //cv::destroyWindow("Map"); 

        // ���� Dijkstra �㷨
		DijkstraEigen dijkstra(map.GetMap(), current_start_point, end_point); // end_points Ϊ����յ� end_point Ϊ��ǰ�յ�
        auto path = dijkstra.FindShortestPath(end_point, end_points);

        if (!path.empty()) {
            std::cout << "Shortest Path Found to (" << end_point.x << ", " << end_point.y << ")!" << std::endl;
            //�������
			std::cout << "The distance is " << path.size() << std::endl;
            dijkstra.Visualize(path, image); // ���ӻ�·��
        }
        else {
            std::cout << "No path found to (" << end_point.x << ", " << end_point.y << ")." << std::endl;
        }

        // �������Ϊ��ǰ�յ�
        current_start_point = end_point;
    }

    cv::destroyAllWindows();
    return 0;
}


void Drawline(cv::Mat& image, const Point& current_start_point, const std::vector<Point>& end_points, bool y_inverse) {
    // ��������ʮ��
        cv::line(image, cv::Point(current_start_point.x - cross_size, current_start_point.y), cv::Point(current_start_point.x + cross_size, current_start_point.y), cv::Scalar(0, 0, 255), 1); // ��ɫ����
        cv::line(image, cv::Point(current_start_point.x, current_start_point.y - cross_size), cv::Point(current_start_point.x, current_start_point.y + cross_size), cv::Scalar(0, 0, 255), 1); // ��ɫ����

        // ����������ÿ���յ��ʮ��
        for (const auto& end_point : end_points) {
            cv::line(image, cv::Point(end_point.x - cross_size, end_point.y), cv::Point(end_point.x + cross_size, end_point.y), cv::Scalar(255, 0, 0), 1); // ��ɫ����
            cv::line(image, cv::Point(end_point.x, end_point.y - cross_size), cv::Point(end_point.x, end_point.y + cross_size), cv::Scalar(255, 0, 0), 1); // ��ɫ����
        }
}