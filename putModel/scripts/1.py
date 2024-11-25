import cv2
import numpy as np

# 读取图像
image = cv2.imread("map.png")

# 将图像转换为HSV颜色空间
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 定义蓝色和红色的HSV范围
blue_lower = np.array([100, 150, 100])  # 蓝色下限
blue_upper = np.array([140, 255, 255])  # 蓝色上限
red_lower1 = np.array([0, 150, 100])    # 红色下限（低区）
red_upper1 = np.array([10, 255, 255])   # 红色上限（低区）
red_lower2 = np.array([160, 150, 100])  # 红色下限（高区）
red_upper2 = np.array([180, 255, 255])  # 红色上限（高区）

# 创建蓝色和红色的掩膜
mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)
mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
mask_red = cv2.bitwise_or(mask_red1, mask_red2)  # 合并红色区域掩膜

# 对蓝色和红色掩膜进行形态学操作来去除噪声
kernel = np.ones((3, 3), np.uint8)
mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

# 查找蓝色和红色区域的轮廓
contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 在原图上绘制轮廓
output = image.copy()
cv2.drawContours(output, contours_blue, -1, (255, 0, 0), 2)  # 用蓝色标记蓝色虚线
cv2.drawContours(output, contours_red, -1, (0, 0, 255), 2)   # 用红色标记红色虚线

# 显示结果
cv2.imshow("Detected Lines", output)
cv2.waitKey(0)
cv2.destroyAllWindows()
