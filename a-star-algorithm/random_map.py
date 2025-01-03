# random_map.py

import numpy as np

import point

class RandomMap:
    def __init__(self, size=480):
        self.size = size  # 地图的尺寸
        self.obstacle_point = []
        self.map = np.ones((size, size, 3), dtype=np.uint8) * 255  # 初始化为白色
        self.GenerateObstacle()

    def GenerateObstacle(self):
        self.obstacle_point = []

        # 添加水池障碍（中心圆形）
        fountain_center = (240, 240)  # 中心坐标
        fountain_radius = 40
        for x in range(self.size):
            for y in range(self.size):
                if (x - fountain_center[0])**2 + (y - fountain_center[1])**2 <= fountain_radius**2:
                    self.map[x, y] = (128, 128, 128)  # 灰色障碍物
                    self.obstacle_point.append(point.Point(x, y))

        # 添加食品摊位障碍（正方形40x40）
        food_stalls = [(240, 120), (160, 160), (240, 340)]
        food_width = 40
        for stall in food_stalls:
            x_start, y_start = stall
            for x in range(x_start - 20, x_start + 20):
                for y in range(y_start - 20, y_start + 20):
                    self.map[x, y] = (128, 128, 128)  # 灰色障碍物
                    self.obstacle_point.append(point.Point(x, y))

        # 添加电梯障碍（正方形50x50）
        elevators = [(160, 400), (320, 400)]
        for elevator in elevators:
            x_start, y_start = elevator
            for x in range(x_start - 25, x_start + 25):
                for y in range(y_start - 25, y_start + 25):
                    self.map[x, y] = (128, 128, 128)  # 灰色障碍物
                    self.obstacle_point.append(point.Point(x, y))

        # 添加购物商店障碍 1
        shopping_stores = [(0, 0), (360, 0)]
        for store in shopping_stores:
            x_start, y_start = store
            for x in range(x_start, x_start + 120):
                for y in range(y_start, y_start + 480):
                    self.map[x, y] = (128, 128, 128)  # 灰色障碍物
                    self.obstacle_point.append(point.Point(x, y))

        # 添加禁止区域障碍（顶部和底部水平区域）
        no_go_zone = [(120, 0), (120, 440)]
        for ngz in no_go_zone:
            x_start, y_start = ngz
            for x in range(x_start, x_start + 240):  # 底部区域
                for y in range(y_start, y_start + 40):
                    self.map[x, y] = (128, 128, 128)  # 灰色障碍物
                    self.obstacle_point.append(point.Point(x, y))
