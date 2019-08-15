#!/usr/bin/env python

import cv2
import urllib 
import numpy as np
from sensor_msgs.msg import Image 
import roslib
import sys
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 
import argparse



URL = "rtsp://admin:1234@192.168.1.30:50/ipcam_mjpeg.sdp"
URL1 = "rtsp://admin:1234@192.168.1.7:554/ipcam_mjpeg.sdp"


ipcam = cv2.VideoCapture(URL)
ipcam1 = cv2.VideoCapture(URL1)

rospy.init_node('IPCamera', anonymous=True)
bridge=CvBridge()

image_pub = rospy.Publisher("cam0/image_raw", Image,queue_size=10)
image_pub1 = rospy.Publisher("cam1/image_raw", Image,queue_size=10)

r=rospy.Rate(15)
if __name__ == '__main__':

    while not rospy.is_shutdown():
            stat, I = ipcam.read()
            stat1, I1 = ipcam1.read()

            image_pub.publish(bridge.cv2_to_imgmsg(I, "bgr8"))
            image_pub1.publish(bridge.cv2_to_imgmsg(I1, "bgr8"))
            # print("ffs")
            # cv2.imshow('Image1', I)

            # cv2.imshow('Image', I1)
            #image_message = cv2.cv.fromarray(I)
            # image_message = np.asarray(image_message[:,:])
            r.sleep()


#  rosbag record /velodyne_points /cam0/image_raw /cam1/image_raw
