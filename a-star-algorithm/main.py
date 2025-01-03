import numpy as np
import cv2

import random_map
import dijkstra
from tqdm import tqdm

map = random_map.RandomMap()
map.GenerateObstacle()


def IsObstacle(self, i, j):
    for p in self.obstacle_point:
        if i == p.x and j == p.y:
            return True
    return False

start_point = (250, 50)
end_point1 = (310, 170)
end_point2 = (150, 70)
end_point3 = (150, 300)
end_point4 = (310, 330)
finish_points = (230, 430)

end_point = end_point1

# 创建一个空白图像
image_size = map.size  # 每个网格点大小为1x1像素
image = np.ones((image_size, image_size, 3), dtype=np.uint8) * 255  # 白色背景

# 先渲染一个地图示例
# 绘制障碍物
for p in map.obstacle_point:
    x, y = p.x, p.y  # 转换 y 坐标
    cv2.rectangle(image, (x, y), (x+1, y+1), (128, 128, 128), -1)  # 灰色障碍物

# 绘制起点和终点
cv2.rectangle(image, (start_point[0], start_point[1]), (start_point[0] + 1, start_point[1] + 1), (255, 0, 0), -1)  # 蓝色起点
cv2.rectangle(image, (end_point[0], end_point[1]), (end_point[0] + 1, end_point[1] + 1), (0, 0, 255), -1)  # 红色终点

# 显示图像之前进行垂直翻转
flipped_image = cv2.flip(image, 0)
# 显示图像
cv2.imshow('Map', flipped_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 运行Dijkstra算法
dijkstra = dijkstra.Dijkstra(map.map, start_point, end_point)
path = dijkstra.find_shortest_path()

if path:
    # print("Shortest Path:", path)
    dijkstra.visualize(path)
else:
    print("No path found.")
