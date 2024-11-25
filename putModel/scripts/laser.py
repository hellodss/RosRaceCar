#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
class ObstacleDetector:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('obstacle_detector', anonymous=True)
        
        # 订阅激光扫描话题
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # 小车坐标系原点
        self.car_x = 0.0
        self.car_y = 0.0
    def scan_callback(self, scan_msg):
        # 遍历激光扫描数据
        for i, distance in enumerate(scan_msg.ranges):
            # 过滤无效数据和最大距离
            if distance < scan_msg.range_max and distance > scan_msg.range_min:
                # 计算角度（弧度）
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                
                # 计算障碍物坐标
                obstacle_x = distance * math.cos(angle)
                obstacle_y = distance * math.sin(angle)
                
                # 打印障碍物坐标
                print(f"障碍物坐标: x = {obstacle_x:.2f}, y = {obstacle_y:.2f}")
    def run(self):
        # 保持节点运行
        rospy.spin()
def main():
    try:
        detector = ObstacleDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()