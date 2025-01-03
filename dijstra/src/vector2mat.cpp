#include "vector2mat.h"

// 转换函数声明
// 将地图转换为 OpenCV Mat
// 转换函数
cv::Mat ConvertMapToMat(const std::vector<std::vector<std::array<uint8_t, 3>>>& map) {
    int rows = map.size();
    int cols = map[0].size();

    // 创建一个连续的 1D 数组
    std::vector<uint8_t> continuous_data(rows * cols * 3);

    // 填充连续数组
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            const auto& pixel = map[i][j];
            int index = (i * cols + j) * 3;
            continuous_data[index + 0] = pixel[0];
            continuous_data[index + 1] = pixel[1];
            continuous_data[index + 2] = pixel[2];
        }
    }

    // 创建 Mat，使用连续内存
    return cv::Mat(rows, cols, CV_8UC3, continuous_data.data()).clone(); // 使用 clone 确保数据安全
}
