#!/usr/bin/env python2
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
class YellowLaneDetector:
    def __init__(self):

        rospy.init_node('yellow_lane_detector', anonymous=True)
        

        self.bridge = CvBridge()
        

        self.image_sub = rospy.Subscriber('/real_sense/rgb/image_raw', Image, self.image_callback)
        
    def image_callback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
 
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
       
        _,contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        
        if contours:

            max_contour = max(contours, key=cv2.contourArea)
            
       
            rows, cols = mask.shape
            [vx, vy, x, y] = cv2.fitLine(max_contour, cv2.DIST_L2, 0, 0.01, 0.01)
            
  
            left_y = int((-x * vy / vx) + y)
            right_y = int(((cols - x) * vy / vx) + y)
            

            cv2.line(cv_image, (cols - 1, right_y), (0, left_y), (0, 255, 0), 2)
        

        cv2.imshow("Yellow Lane Detection", cv_image)
        cv2.waitKey(1)

    def run(self):

        rospy.spin()
def main():
    try:
        lane_detector = YellowLaneDetector()
        lane_detector.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
if __name__ == '__main__':
    main()