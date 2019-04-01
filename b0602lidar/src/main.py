#!/usr/bin/env python
from B0602Lidar import B0602Lidar
from math import pi
###ROS library
import rospy
#import math
from sensor_msgs.msg import LaserScan


lidar = B0602Lidar()
count=0

pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
rospy.init_node('car_strage', anonymous=True)
rate = rospy.Rate(30)
msg_data=LaserScan()
for i in range(0,361):
    msg_data.ranges.append(0)
while not rospy.is_shutdown():
    output_data=lidar.run()
    msg_data.header.frame_id='laser'
    # msg_data.ranges[:]=[]
    msg_data.angle_min= -180 * (pi/180)
    msg_data.angle_max= 180 * (pi/180)
    msg_data.angle_increment =1 * (pi/180)

    # msg_data.time_increment  = (1 / 50) / (360)
    # msg_data.scan_time=0.5
    msg_data.range_min =0 
    msg_data.range_max = 5000
    # output_data[]
    count+=1
    # print(output_data)
    # if count%10==0:
    if len(output_data['data'])>0:
        for i in range(len(output_data['data'])):
            if int(output_data['data'][i][0]) >360:
                angle_=int(output_data['data'][i][0])-360
            else:
                angle_=int(output_data['data'][i][0])
            if angle_>360:
                break
            # print(angle_)
            msg_data.ranges[angle_]=output_data['data'][i][1]/1000
            pub.publish(msg_data)
    else:
        print("b0602___bad")
    rate.sleep()
    
