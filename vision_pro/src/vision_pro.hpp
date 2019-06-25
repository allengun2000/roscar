#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vision/image_cv.h"



#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/core/internal.hpp>
#include <iostream>



#include "vision_pro/line_inform.h"

using namespace std;

cv::Mat Main_frame;
int img_h;
int img_w;