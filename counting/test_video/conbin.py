#!/usr/bin/env python

import os
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl.pcl_visualization
visual = pcl.pcl_visualization.CloudViewing()
# https://www.twblogs.net/a/5c378893bd9eee35b3a59e63
pc_clound =[]
def imcallback(data):
    bridge = CvBridge()
    try:
         global frame
         frame=bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

def on_new_point_cloud(data):
    pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for p in pc:
        pc_list.append( [p[0],p[1],p[2]] )

    if len(pc_clound)<58:
        pc_clound.append(pc_list)
    else:
        pc_clound.pop(0)
        pc_clound.append(pc_list)
    p = pcl.PointCloud()
    p.from_list(pc_clound[0])
    visual.ShowMonochromeCloud(p, b'cloud')
    
    # print(pc_clound[0][0][0])

rospy.init_node('detector_node')
image_sub = rospy.Subscriber("cam0/image_raw", Image,imcallback)
point_sub =rospy.Subscriber("/velodyne_points", PointCloud2, on_new_point_cloud)
frame=cv2.imread("detection.bmp")

while not rospy.is_shutdown():

      cv2.imshow('Object detector', frame)
      cv2.waitKey(1)
