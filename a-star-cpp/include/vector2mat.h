#ifndef VECTOR2MAT_H
#define VECTOR2MAT_H


#include <opencv2/opencv.hpp>
#include <vector>
#include <array>

// ×ª»»º¯ÊýÉùÃ÷
cv::Mat ConvertMapToMat(const std::vector<std::vector<std::array<uint8_t, 3>>>& map);

#endif // VECTOR2MAT_H


