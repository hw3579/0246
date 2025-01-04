# dijkstra.py

import sys
import time

import numpy as np

import cv2

import point
import random_map
import heapq
from tqdm import tqdm

class Dijkstra:
    def __init__(self, map, start_point, end_point, robot_size=10, visualizing_search=False):
        self.map = map
        self.start_point = start_point
        self.end_point = end_point
        self.robot_size = robot_size  # Robot size in half units (20x20 = 10)
        self.visualizing_search = visualizing_search
        self.rows, self.cols = map.shape[:2]

    def is_valid_point(self, point):
        x, y = point
        half_size = self.robot_size
        for dx in range(-half_size, half_size):
            for dy in range(-half_size, half_size):
                nx, ny = x + dx, y + dy
                if not (0 <= nx < self.rows and 0 <= ny < self.cols):
                    return False
                if tuple(self.map[nx, ny]) == (128, 128, 128):
                    return False
        return True

    def get_neighbors(self, point):
        x, y = point
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
        neighbors = []
        for dx, dy in directions:
            neighbor = (x + dx, y + dy)
            if self.is_valid_point(neighbor):
                neighbors.append(neighbor)
        return neighbors

    def find_shortest_path(self):
        if not self.is_valid_point(self.start_point) or not self.is_valid_point(self.end_point):
            return None

        priority_queue = []
        heapq.heappush(priority_queue, (0, self.start_point))  # (distance, point)
        distances = {self.start_point: 0}
        predecessors = {self.start_point: None}
        visited = set()

        while priority_queue:
            current_distance, current_point = heapq.heappop(priority_queue)
            
            if current_point in visited:
                continue
            visited.add(current_point)

            if self.visualizing_search:
                self.map[current_point[0], current_point[1]] = (0, 255, 0)  # Green for visited
                # for i in range(-10, 11):
                #     for j in range(-10, 11):
                #         x, y = current_point[0] + i, current_point[1] + j
                #         if 0 <= x < self.map.shape[0] and 0 <= y < self.map.shape[1]:
                #             self.map[x, y] = (0, 255, 0)
                rotated_image = cv2.rotate(self.map, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow("Dijkstra Visualization", rotated_image)
                cv2.waitKey(50)  # Adjust delay for smoother visualization

            if current_point == self.end_point:
                path = self.reconstruct_path(predecessors)
                self.visualize(path)  # Visualize the final path
                return path

            for neighbor in self.get_neighbors(current_point):
                distance = current_distance + 1  # Assuming uniform cost
                if neighbor not in distances or distance < distances[neighbor]:
                    distances[neighbor] = distance
                    predecessors[neighbor] = current_point
                    heapq.heappush(priority_queue, (distance, neighbor))
                    print("Current Point:", current_point, "Neighbor:", neighbor, "Distance:", distance)
        return None  # No path found

    def reconstruct_path(self, predecessors):
        path = []
        current = self.end_point
        while current is not None:
            path.append(current)
            current = predecessors[current]
        path.reverse()
        return path

    def visualize(self, path):
        for point in path:
            x, y = point
            half_size = self.robot_size
            for dx in range(-half_size, half_size):
                for dy in range(-half_size, half_size):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.rows and 0 <= ny < self.cols:
                        self.map[nx, ny] = (0, 0, 255)  # Red for path
            rotated_image = cv2.rotate(self.map, cv2.ROTATE_90_COUNTERCLOCKWISE)  # Rotate visualization
            cv2.imshow("Dijkstra Visualization", rotated_image)
            # cv2.waitKey(5)  # 50 ms delay for visualization

        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def add_collision_volume(self, robot_position):
        """Mark a collision volume centered at the robot's position on the map."""
        x_center, y_center = robot_position
        half_size = self.robot_size
        for x in range(x_center - half_size, x_center + half_size):
            for y in range(y_center - half_size, y_center + half_size):
                if 0 <= x < self.rows and 0 <= y < self.cols:
                    self.map[x, y] = (128, 128, 128)  # Mark as obstacle
