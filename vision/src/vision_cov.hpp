#include "ros/ros.h"
#include "vision/image_cv.h"
#include "std_msgs/Int32MultiArray.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
using namespace std;
using namespace cv;

cv::Mat  Main_frame;
cv::Mat  gray_image;