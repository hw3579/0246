#ifndef MAIN_H
#define MAIN_H


#include<opencv2/opencv.hpp>

void Drawline(cv::Mat& image, const Point& current_start_point, const std::vector<Point>& end_points, bool y_inverse = false);


#endif // !MAIN_H