#!/usr/bin/env python

from __future__ import print_function


import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision.msg import image_cv
from goood.msg import num
import numpy as np



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


'''
def callback(data):
    pass
    rospy.loginfo("I heard %s", data.data)
def callpicture(pic):
    rospy.loginfo("%d",pic.weight)
    mat = np.array(pic.data,dtype=np.uint8)
 
    color_image=np.ones((pic.hight,pic.weight, 3), dtype=np.uint8) *100
    print(len(pic.data))
    mat=np.reshape(mat,(pic.hight,pic.weight, 3))
    # color_image.astype(np.uint8)
    # print(color_image)
    # for i in range (pic.hight):
	#     for j in range (pic.weight):
	# 		for k in range (3):
	# 	         color_image[i,j,k] =mat[(i*pic.weight*3)+(j*3)+k]

    cv2.imshow("ss",mat)
    cv2.waitKey(0)
def callnum(data):
    pass
    # rospy.loginfo("I hearssssd %s", data.x)

def listener():
    rospy.init_node('car_test', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("pic_source", image_cv, callpicture)
    rospy.Subscriber("num", num, callnum)
    
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        # rospy.spinOnce()
if __name__ == '__main__':
    listener()
'''
