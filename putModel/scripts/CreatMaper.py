#!/usr/bin/env python3
import cv2
import numpy as np
import math

class ColorDetector:
    def __init__(self, config=None):

        #红蓝点坐标
        self.red_coords = []
        self.blue_coords = []
        
        #半圆坐标
        self.red_yellow_points = []

        self.blue_yellow_points = []
        
        # 使用默认配置，或通过传入自定义配置进行覆盖
        default_config = {
            'grid_size': 50,
            'min_contour_area': 50,
            'circularity_threshold': 0,
            'line_thickness': 30,
            'marker_size': 20
        }
        self.CONFIG = config if config else default_config

    def read_image(self, image_path):
        image = cv2.imread(image_path)
        if image is None:
            raise ValueError("无法读取图像文件")
        return image

    # def create_grid(self, height, width):
    #     blank_image = np.ones((height, width, 3), np.uint8) * 255
    #     for x in range(0, width, self.CONFIG['grid_size']):
    #         cv2.line(blank_image, (x, 0), (x, height), (0, 0, 0), self.CONFIG['line_thickness'])
    #     for y in range(0, height, self.CONFIG['grid_size']):
    #         cv2.line(blank_image, (0, y), (width, y), (0, 0, 0), self.CONFIG['line_thickness'])
    #     return blank_image

    def process_contours(self, contours, color, blank_image, center_x, center_y):
        coordinates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if area > self.CONFIG['min_contour_area'] and perimeter > 0:
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                if circularity > self.CONFIG['circularity_threshold']:
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    relative_x, relative_y = int(x - center_x), int(y - center_y)
                    coordinates.append((relative_x, relative_y))
                    cv2.circle(blank_image, center, self.CONFIG['marker_size'], color, -1)
        return coordinates

    def detect_colors(self, image):
        height, width, _ = image.shape
        center_x, center_y = width // 2, height // 2
        

        blank_image = np.ones((height, width, 3), np.uint8) * 255

        # 转换为 HSV 格式进行颜色检测
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 检测蓝色区域
        blue_mask = cv2.inRange(hsv_image, np.array([100, 50, 50]), np.array([130, 255, 255]))
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, 
                                     cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        _,blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.blue_coords = self.process_contours(blue_contours, (255, 0, 0), blank_image, center_x, center_y)

        # 检测红色区域
        red_mask1 = cv2.inRange(hsv_image, np.array([0, 50, 50]), np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv_image, np.array([160, 50, 50]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, 
                                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        _,red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.red_coords = self.process_contours(red_contours, (0, 0, 255), blank_image, center_x, center_y)

        return blank_image

    def show_results(self, image):
        cv2.imshow('Grid with Marked Centers', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def FindPointRightAndDrawSemiCircles(self, blank_image):
        # 计算图片中心坐标，用作相对坐标系的原点
        height, width, _ = blank_image.shape
        center_x, center_y = width // 2, height // 2

        # 检查蓝色点并绘制右半圆
        if len(self.blue_coords) >= 2:
            # 按相对 x 坐标降序排列蓝色点，取最右边的两个
            sorted_blue = sorted(self.blue_coords, key=lambda point: point[0], reverse=True)
            right_blue_points = sorted_blue[:2]
            print("最右边蓝坐标（相对中心）:", right_blue_points)

            # 计算圆心和半径（基于相对坐标）
            relative_center_x = (right_blue_points[0][0] + right_blue_points[1][0]) // 2
            relative_center_y = (right_blue_points[0][1] + right_blue_points[1][1]) // 2
            center = (center_x + relative_center_x, center_y + relative_center_y)  # 转换为绝对坐标
            radius = int(math.sqrt((right_blue_points[0][0] - relative_center_x) ** 2 + 
                                (right_blue_points[0][1] - relative_center_y) ** 2))

            # 用 X 标记圆心
            cv2.drawMarker(blank_image, center, (255, 0, 0), cv2.MARKER_TILTED_CROSS, self.CONFIG['marker_size'], self.CONFIG['line_thickness'])

            # 绘制右半圆
            cv2.ellipse(blank_image, center, (radius, radius), 0, -90, 90, (255, 0, 0), self.CONFIG['line_thickness'])

            for angle in range(-90, 91, 15):  # 每隔15度放一个点
                radian = math.radians(angle)
                point_x = int(center[0] + radius * math.cos(radian))
                point_y = int(center[1] + radius * math.sin(radian))
                self.blue_yellow_points.append((point_x, point_y))
                cv2.circle(blank_image, (point_x, point_y), 3, (0, 255, 255), -1)  # 黄色点

            print("黄色点坐标（蓝色半圆上）:", self.blue_yellow_points)

        else:
            print("蓝色点不足两个，无法绘制半圆")

        # 检查红色点并绘制右半圆
        if len(self.red_coords) >= 2:
            # 按相对 x 坐标降序排列红色点，取最右边的两个
            sorted_red = sorted(self.red_coords, key=lambda point: point[0], reverse=True)
            right_red_points = sorted_red[:2]
            print("最右边红坐标（相对中心）:", right_red_points)

            # 计算圆心和半径（基于相对坐标）
            relative_center_x = (right_red_points[0][0] + right_red_points[1][0]) // 2
            relative_center_y = (right_red_points[0][1] + right_red_points[1][1]) // 2
            center = (center_x + relative_center_x, center_y + relative_center_y)  # 转换为绝对坐标
            radius = int(math.sqrt((right_red_points[0][0] - relative_center_x) ** 2 + 
                                (right_red_points[0][1] - relative_center_y) ** 2))

            # 用 X 标记圆心
            cv2.drawMarker(blank_image, center, (0, 0, 255), cv2.MARKER_TILTED_CROSS, self.CONFIG['marker_size'], self.CONFIG['line_thickness'])

            # 绘制右半圆
            cv2.ellipse(blank_image, center, (radius, radius), 0, -90, 90, (0, 0, 255), self.CONFIG['line_thickness'])

            # 在右半圆上绘制黄色点
            for angle in range(-90, 91, 15):  # 每隔15度放一个点
                radian = math.radians(angle)
                point_x = int(center[0] + radius * math.cos(radian))
                point_y = int(center[1] + radius * math.sin(radian))
                self.red_yellow_points.append((point_x, point_y))
                cv2.circle(blank_image, (point_x, point_y), 3, (0, 255, 255), -1)  # 黄色点

            print("黄色点坐标（红色半圆上）:", self.red_yellow_points)

        else:
            print("红色点不足两个，无法绘制半圆")
            
        return blank_image




    

    def run(self, image_path):
        try:
            image = self.read_image(image_path)
            blank_image = self.detect_colors(image)
            print(f"蓝色坐标: {self.blue_coords}")
            print(f"红色坐标: {self.red_coords}")
            
            # 找到最右边的蓝色和红色点并绘制半圆
            blank_image = self.FindPointRightAndDrawSemiCircles(blank_image)
            
            # 显示结果
            self.show_results(blank_image)
            
        except Exception as e:
            print(f"发生错误: {str(e)}")


if __name__ == "__main__":
    detector = ColorDetector()
    detector.run('src/putModel/map_picture/map.png')
