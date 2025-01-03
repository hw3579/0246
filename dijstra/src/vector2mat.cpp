#include "vector2mat.h"

// ת����������
// ����ͼת��Ϊ OpenCV Mat
// ת������
cv::Mat ConvertMapToMat(const std::vector<std::vector<std::array<uint8_t, 3>>>& map) {
    int rows = map.size();
    int cols = map[0].size();

    // ����һ�������� 1D ����
    std::vector<uint8_t> continuous_data(rows * cols * 3);

    // �����������
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            const auto& pixel = map[i][j];
            int index = (i * cols + j) * 3;
            continuous_data[index + 0] = pixel[0];
            continuous_data[index + 1] = pixel[1];
            continuous_data[index + 2] = pixel[2];
        }
    }

    // ���� Mat��ʹ�������ڴ�
    return cv::Mat(rows, cols, CV_8UC3, continuous_data.data()).clone(); // ʹ�� clone ȷ�����ݰ�ȫ
}
