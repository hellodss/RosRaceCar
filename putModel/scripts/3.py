#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Header

def publish_image():

    rospy.init_node('image_publisher', anonymous=True)


    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

 
    bridge = CvBridge()


    cv_image = cv2.imread("src/putModel/scripts/red.jpg")


    if cv_image is None:
        rospy.logerr("Failed to load image")
        return


    header = Header()
    header.stamp = rospy.Time.now()


    try:
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    except cv_bridge.CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


    ros_image.header = header


    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        image_pub.publish(ros_image)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass
