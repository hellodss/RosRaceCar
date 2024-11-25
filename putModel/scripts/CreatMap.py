import cv2
import numpy as np

red_coords = []
blue_coords = []

CONFIG = {
    'grid_size': 50,
    'min_contour_area': 50,
    'circularity_threshold': 0,
    'line_thickness': 1,
    'marker_size': 5
}

def read_image(image_path):
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError("无法读取图像文件")
    return image

def create_grid(height, width):
    blank_image = np.ones((height, width, 3), np.uint8) * 255
    for x in range(0, width, CONFIG['grid_size']):
        cv2.line(blank_image, (x, 0), (x, height), (0, 0, 0), CONFIG['line_thickness'])
    for y in range(0, height, CONFIG['grid_size']):
        cv2.line(blank_image, (0, y), (width, y), (0, 0, 0), CONFIG['line_thickness'])
    return blank_image

def process_contours(contours, color, blank_image, center_x, center_y):
    coordinates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if area > CONFIG['min_contour_area'] and perimeter > 0:
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            if circularity > CONFIG['circularity_threshold']:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                relative_x, relative_y = int(x - center_x), int(y - center_y)
                coordinates.append((relative_x, relative_y))
                cv2.circle(blank_image, center, CONFIG['marker_size'], color, -1)
    return coordinates

def main():
    try:
        # 读取图像
        image = read_image('src/PutModel/map_picture/map.png')
        height, width, _ = image.shape
        center_x, center_y = width // 2, height // 2

        # 创建网格
        blank_image = create_grid(height, width)

        # 颜色检测
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 蓝色检测
        blue_mask = cv2.inRange(hsv_image, np.array([100, 50, 50]), np.array([130, 255, 255]))
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, 
                                   cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        _, blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_coords = process_contours(blue_contours, (255, 0, 0), blank_image, center_x, center_y)

        # 红色检测
        red_mask1 = cv2.inRange(hsv_image, np.array([0, 50, 50]), np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv_image, np.array([160, 50, 50]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, 
                                  cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        _,red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_coords = process_contours(red_contours, (0, 0, 255), blank_image, center_x, center_y)

        print(f"蓝色坐标: {blue_coords}")
        print(f"红色坐标: {red_coords}")

        # 显示结果
        cv2.imshow('Grid with Marked Centers', blank_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"发生错误: {str(e)}")

if __name__ == "__main__":
    main()