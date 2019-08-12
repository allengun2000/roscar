#!/usr/bin/env python
# Import packages
import os
import cv2
import numpy as np
import tensorflow as tf
import sys

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

from keras.preprocessing import image
from keras.models import Model #set up model
from keras.layers.normalization import BatchNormalization
from keras.callbacks import ModelCheckpoint
from keras.callbacks import EarlyStopping
from keras.layers import Input, Dropout, Activation
from keras.layers.convolutional import Conv2D,Conv2DTranspose,UpSampling2D
from keras.layers.pooling import MaxPooling2D
from scipy import misc
from keras.models import Sequential 
import matplotlib.pyplot as plt
from keras.models import load_model
from imageio import imread
from keras.preprocessing import image
from keras.models import model_from_json
import threading
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# Name of the directory containing the object detection module we're using
MODEL_NAME = 'inference_graph'
MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
IMAGE_NAME = 'night1.avi'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'/home/allen/linux/catkin_ws/src/counting/test_video/inference_graph/frozen_inference_graph.pb')
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'/home/allen/linux/catkin_ws/src/counting/test_video/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb')
density_model = model_from_json(open('/home/allen/linux/catkin_ws/src/counting/test_video/model.json').read())
density_model.load_weights('/home/allen/linux/catkin_ws/src/counting/test_video/CSRNET.h5')
# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,'data','labelmap.pbtxt')

# Path to image
PATH_TO_IMAGE = os.path.join(CWD_PATH,IMAGE_NAME)

# Number of classes the object detector can identify
NUM_CLASSES = 1

label_map = label_map_util.load_labelmap('/home/allen/linux/catkin_ws/src/counting/test_video/data/labelmap.pbtxt')
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)

# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

# class ipcamCapture:
#     def __init__(self, URL):
#         self.Frame = []
#         self.status = False
#         self.isstop = False
		
	
#         self.capture = cv2.VideoCapture(URL)

#     def start(self):
	
#         print('ipcam started!')
#         #threading.Thread(target=self.queryframe, daemon=True, args=()).start()
#         clearner=threading.Thread(target=self.queryframe, args=())
#         clearner.daemon = True
#         clearner.start()

#     def stop(self):
	
#         self.isstop = True
#         print('ipcam stopped!')
   
#     def getframe(self):
	
#         return self.Frame
        
#     def queryframe(self):
#         while (not self.isstop):
#             self.status, self.Frame = self.capture.read()
        
#         self.capture.release()



def imcallback(data):
    bridge = CvBridge()
    try:
         global frame
         frame=bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)



time.sleep(1)

rospy.init_node('detector_node')
image_sub = rospy.Subscriber("cam0/image_raw", Image,imcallback)

frame=cv2.imread("/home/allen/1.png")

while not rospy.is_shutdown():
    frame_=frame.copy()
    image_expanded = np.expand_dims(frame_, axis=0)
    frame_ = cv2.resize(frame_, (1920, 1080), interpolation=cv2.INTER_CUBIC)
    # Perform the actual detection by running the model with the image as input
    (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_expanded})
    # ymin xmim ymax xmax
    # Draw the results of the detection (aka 'visulaize the results')
    density_cut1 = frame_[50:540,0:1920] 
    
    density_cut2 = frame_[0:540,0:1920] 
    density_cut1 = cv2.cvtColor(density_cut1, cv2.COLOR_BGR2GRAY)
    
    density = image.img_to_array(density_cut1)
    density = (density - 127.5) / 128
    # cv2.imshow('Object detector', density)
    
    cv2.waitKey(1)
    density = np.reshape(density, (1, 490, 1920, 1), order="F")
    density_map=density_model.predict(density)
    density_map = density_map.reshape(122, 480)
    density_count = np.sum(density_map)
    density_count = int(density_count)  

    density_map = cv2.resize(density_map, (1920, 488), interpolation=cv2.INTER_CUBIC)
    zeros = np.zeros((52,1920))
    zeros = zeros.astype(np.uint8)
    density_fusion = np.vstack((zeros,density_map))
                  
    density_fusion = cv2.resize(density_map, (1920, 540), interpolation=cv2.INTER_CUBIC)
    misc.imsave('density_fusion.bmp',density_fusion)	

    density_color = cv2.imread("density_fusion.bmp", cv2.IMREAD_GRAYSCALE)
    density_color = cv2.applyColorMap(density_color, cv2.COLORMAP_JET)
    # cv2.imwrite('density_color.bmp',density_color)

    density_cut2 = density_cut2.astype(np.uint8)
    overlapping = cv2.addWeighted(density_color, 0.3, density_cut2, 0.7, 0)
    # cv2.imwrite('overlapping.bmp',overlapping)

    detection_count = 0
    score_thresh = 0.3
    num = scores[0]
    for i in range(len(num)):
        if num[i] > score_thresh:
            
            
            cc = boxes[0]
            for i in range(detection_count):
                ymin = cc[i,0]*1080   #ymin
                xmin = cc[i,1]*1920   #xmin
                ymax = cc[i,2]*1080   #ymax
                xmax = cc[i,3]*1920   #xmax 
                center = float((ymin+ymax)/2)
                #if center > 540:
		detection_count = detection_count+1
                cv2.rectangle(frame_, (int(xmin), int(ymin)), (int(xmax),  int(ymax)), (255, 0, 0), 2)  


            text = "Count:"+str(detection_count)
    #cv2.putText(src, text, (5, 30), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)   
    #src = cv2.cvtColor(src, cv2.COLOR_BGR2RGB)
    # cv2.imwrite('detection.bmp',frame)
                
    
    rows, cols, channels = overlapping.shape
    roi = frame_[0:rows, 0:cols]
    logogray = cv2.cvtColor(overlapping, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(logogray, 20, 255, cv2.THRESH_BINARY)
    mask_inv = cv2.bitwise_not(mask)			
    img1_bg = cv2.bitwise_and(roi, roi, mask = mask_inv)
    img2_fg = cv2.bitwise_and(overlapping, overlapping, mask = mask)
    dst = cv2.add(img1_bg, img2_fg)
    frame_[0:rows, 0:cols] = dst
    dst = cv2.add(img1_bg, img2_fg)
    frame_[0:rows, 0:cols] = dst
    total_count = density_count + detection_count

    text = "Count:"+str(total_count)
    cv2.putText(frame_, text, (5, 30), cv2.FONT_HERSHEY_PLAIN, 5.0, (0, 0, 255), 5) 
    # cv2.imwrite('Front.bmp',frame)
    frame_=cv2.resize(frame_,(480,270))
    cv2.imshow('Object detector', frame_)    
    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break
    rate = rospy.Rate(10)
    rate.sleep()
