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



URL = "rtsp://admin:1234@192.168.1.3/ipcam_mjpeg.sdp"


ipcam = cv2.VideoCapture(URL)
rospy.init_node('IPCamera', anonymous=True)
bridge=CvBridge()
image_pub = rospy.Publisher("cam0/image_raw", Image,queue_size=10)
r=rospy.Rate(15)
if __name__ == '__main__':

    while not rospy.is_shutdown():
   	    stat, I = ipcam.read()

    	    #cv2.imshow('Image', I)
            #image_message = cv2.cv.fromarray(I)
           # image_message = np.asarray(image_message[:,:])
            image_pub.publish(bridge.cv2_to_imgmsg(I, "bgr8"))
	    r.sleep()


