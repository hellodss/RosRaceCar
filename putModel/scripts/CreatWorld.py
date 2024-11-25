import cv2
import numpy as np
import rospy
import math
from pathlib import Path
from typing import List, Tuple, Dict, Optional
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

class ColorDetector:
    def __init__(self, config: Optional[Dict] = None):
        # 初始化颜色坐标列表
        self.red_coords: List[Tuple[int, int]] = []
        self.blue_coords: List[Tuple[int, int]] = []
        self.yellow_coords: List[Tuple[int, int]] = []
        self.image: Optional[np.ndarray] = None
        self.scale_factor: float = 0.0246  # 缩放因子，用于将像素坐标转换为世界坐标

        # 默认配置，可通过传入config参数覆盖
        self.CONFIG = config or {
            'grid_size': 50,
            'min_contour_area': 50,
            'circularity_threshold': 0,
            'line_thickness': 1,
            'marker_size': 5
        }

    def read_image(self, image_path: str) -> None:
        # 读取图像文件
        self.image = cv2.imread(image_path)
        if self.image is None:
            raise ValueError(f"无法读取图像文件: {image_path}")

    def create_grid(self, height: int, width: int) -> np.ndarray:
        # 创建一个带网格的空白图像
        blank_image = np.ones((height, width, 3), np.uint8) * 255
        for x in range(0, width, self.CONFIG['grid_size']):
            cv2.line(blank_image, (x, 0), (x, height), (0, 0, 0), self.CONFIG['line_thickness'])
        for y in range(0, height, self.CONFIG['grid_size']):
            cv2.line(blank_image, (0, y), (width, y), (0, 0, 0), self.CONFIG['line_thickness'])
        return blank_image

    def process_contours(self, contours: List, color: Tuple[int, int, int], blank_image: np.ndarray, center_x: int, center_y: int) -> List[Tuple[int, int]]:
        # 处理轮廓，提取符合条件的坐标
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

    def detect_colors(self) -> np.ndarray:
        # 检测图像中的蓝色和红色
        height, width, _ = self.image.shape
        center_x, center_y = width // 2, height // 2
        blank_image = self.create_grid(height, width)
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # 检测蓝色
        blue_mask = cv2.inRange(hsv_image, np.array([100, 50, 50]), np.array([130, 255, 255]))
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        _,blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.blue_coords = self.process_contours(blue_contours, (255, 0, 0), blank_image, center_x, center_y)

        # 检测红色
        red_mask1 = cv2.inRange(hsv_image, np.array([0, 50, 50]), np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv_image, np.array([160, 50, 50]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        _,red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.red_coords = self.process_contours(red_contours, (0, 0, 255), blank_image, center_x, center_y)

        return blank_image

    def find_point_right_and_draw_semicircles(self, blank_image: np.ndarray) -> np.ndarray:
        # 找到最右边的点并绘制半圆
        height, width, _ = self.image.shape
        center_x, center_y = width // 2, height // 2

        def process_color_points(coords: List[Tuple[int, int]], color: Tuple[int, int, int]) -> None:
            if len(coords) >= 2:
                right_points = sorted(coords, key=lambda point: point[0], reverse=True)[:2]
                print(f"最右边{color}坐标（相对中心）:", right_points)

                relative_center_x = (right_points[0][0] + right_points[1][0]) // 2
                relative_center_y = (right_points[0][1] + right_points[1][1]) // 2
                print(f"半圆中心点（相对中心）: ({relative_center_x}, {relative_center_y})")
                radius = int(math.sqrt((right_points[0][0] - relative_center_x) ** 2 + 
                                    (right_points[0][1] - relative_center_y) ** 2))
                print(f"半圆半径: {radius}")

                # 在图像上绘制标记和半圆
                cv2.drawMarker(blank_image, (center_x + relative_center_x, center_y + relative_center_y), color, cv2.MARKER_TILTED_CROSS, self.CONFIG['marker_size'], self.CONFIG['line_thickness'])
                cv2.ellipse(blank_image, (center_x + relative_center_x, center_y + relative_center_y), (radius, radius), 0, -90, 90, color, self.CONFIG['line_thickness'])

                # 在半圆上生成黄色点
                for angle in range(-90, 91, 3):
                    radian = math.radians(angle)
                    point_x = int(relative_center_x + radius * math.cos(radian))
                    point_y = int(relative_center_y + radius * math.sin(radian))
                    self.yellow_coords.append((point_x, point_y))
                    cv2.circle(blank_image, (center_x + point_x, center_y + point_y), 3, (0, 255, 255), -1)

                print(f"黄色点坐标（相对中心）:", self.yellow_coords)
            else:
                print(f"{color}点不足两个，无法绘制半圆")

        process_color_points(self.blue_coords, (255, 0, 0))
        process_color_points(self.red_coords, (0, 0, 255))

        return blank_image

    def spawn_model_in_gazebo(self, model_name: str, model_file: str, position: Tuple[float, float, float]) -> None:
        # 在Gazebo中生成模型
        try:
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            
            model_path = Path(model_file)
            if not model_path.exists():
                raise FileNotFoundError(f"Model file not found: {model_file}")
            
            with model_path.open("r") as f:
                model_xml = f.read()

            model_pose = Pose()
            model_pose.position.x, model_pose.position.y, model_pose.position.z = position
            spawn_model_service(model_name, model_xml, "", model_pose, "world")
            rospy.loginfo(f"Model {model_name} spawned at position {position}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn model {model_name}: {str(e)}")
        except FileNotFoundError as e:
            rospy.logerr(str(e))

    def spawn_models(self) -> None:
        # 在Gazebo中生成所有模型
        rospy.init_node('spawn_models')
        
        print(f"Scale factor: {self.scale_factor}")

        def spawn_color_models(coords: List[Tuple[int, int]], color: str, model_file: str) -> None:
            for idx, (x, y) in enumerate(coords):
                world_x = x * self.scale_factor
                world_y = -y * self.scale_factor  # 注意Y轴方向需要反转
                print(f"Spawning {color} model at ({world_x:.2f}, {world_y:.2f})")
                self.spawn_model_in_gazebo(f"{color}_model_{idx}", model_file, (world_x, world_y, 0))
            print(f"Spawned {len(coords)} {color} models")

        spawn_color_models(self.red_coords, "red", "src/putModel/model/construction_cone_red/model.sdf")
        spawn_color_models(self.blue_coords, "blue", "src/putModel/model/construction_cone_blue/model.sdf")
        spawn_color_models(self.yellow_coords, "yellow", "src/putModel/model/circlePoint/model.sdf")

    def show_results(self, image: np.ndarray) -> None:
        # 显示结果图像
        cv2.imshow('Grid with Marked Centers', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def run(self, image_path: str, show_image: bool = False) -> None:
        # 运行整个流程
        try:
            self.read_image(image_path)
            blank_image = self.detect_colors()
            self.find_point_right_and_draw_semicircles(blank_image)
            self.spawn_models()
            if show_image:
                self.show_results(blank_image)
            rospy.signal_shutdown("Task completed")  # 添加这行来关闭ROS节点
        except Exception as e:
            print(f"发生错误: {str(e)}")
            rospy.signal_shutdown("Error occurred")  # 错误发生时也关闭节点

if __name__ == "__main__":
    detector = ColorDetector()
    detector.run('src/putModel/map_picture/map.png', show_image=True)  # 设置 show_image=True 如果你想显示图像