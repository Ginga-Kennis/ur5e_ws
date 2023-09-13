#!/usr/bin/env python

import os
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
import cv2
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    path = '/home/ginga/catkin_ws/src/ur5e_control/img'
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    # 正常終了時の処理
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite(os.path.join(path,"test_img.jpg"), cv2_img)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/color/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()       
