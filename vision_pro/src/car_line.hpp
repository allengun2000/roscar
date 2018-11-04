#define _USE_MATH_DEFINES
#include <QCoreApplication>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vision/image_cv.h"
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/framework/tensor.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/core/internal.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cstdlib>
#include <limits>
#include <stack>
#include <sstream>
#include <math.h>
#include <cmath> 



using namespace tensorflow;
using namespace cv;
using namespace std;
using namespace cv::gpu;
cv::Mat Main_frame;
cv::Mat img_1;
cv::Mat img_;
cv::Mat frame;
cv::Mat frame_resize;
cv::Mat learning_img;
Session* session;
int height=84;
int width=256;
//------gradient------------
//cv::Mat img_segment_hsv;
cv::Mat edge_img;
cv::Mat img_angle;
int haarlike_angle[2] = { 0 };
//------ROI_I_feature-------
cv::Mat haar_img;
typedef struct  vanishpoint {
    float x;
    float y;
}VP;
VP vertex;
std::vector<cv::Point2f> line_r;
std::vector<cv::Point2f> line_l;
double parameter_r[2], p_r[2] = {};
double parameter_l[2], p_l[2] = {};
float direction_angle[2] = { 0 };
int count_ransac = 0;
//------ROI_II---------------
std::vector<cv::Point2f> line_right;
std::vector<cv::Point2f> line_left;
//------ROI_II------------
cv::Mat img_r, img_l, binary_img, hy_img;
std::vector<cv::Point2f> rightpoint_roiII;
std::vector<cv::Point2f> leftpoint_roiII;
//------houghtransfer------------------
std::vector<cv::Point2f> multiplevp;
std::vector<cv::Point2f> bounary_l;
std::vector<cv::Point2f> bounary_r;
float Vvp = 0;
int model_vi[10] = { 120, 124, 130, 138, 148, 160, 178, 200, 228, 276 };
//------estimateroadparameter--------------
double x_parameter[5] = { 0 };
//------particle--------------
bool predict_hy = false;
int heighttest[25] = { 124, 126, 128, 130, 134, 138, 144, 150, 156, 164, 172, 180, 188, 196, 204, 212, 220, 228, 236, 244, 252, 260, 268, 276, 284 };
int range[25] = { 2, 2, 2, 2, 2, 6, 6, 6, 6, 6, 10, 10, 10, 10, 10, 14, 14, 14, 14, 14, 18, 18, 18, 18, 18 };
//int range[25] = { 2, 2, 2, 2, 2, 5, 5, 5, 5, 5, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 14, 14, 14, 14, 14 };
//int range[25] = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 };
const int stateNum = 5;
const int measureNum = 5;
const int sampleNum = 2000;
CvConDensation* condens = cvCreateConDensation(stateNum, measureNum, sampleNum);

int count1=0;
int count2=0;
cv::Mat Lineim;
//std::fstream fd;

void gradient();
void ROI_I_feature();
void ROI_II_feature();
void drawline(std::vector<cv::Point2f> point, cv::Mat* img, cv::Scalar color);
bool fittingCurve(std::vector<cv::Point2f> &vec, int times, double *p);
std::vector<cv::Point2f> getSample(int n, vector<cv::Point2f> ps);
VP find_intersection(std::vector<cv::Point2f> l1, std::vector<cv::Point2f> l2);
cv::Mat searchfeaturepoint(cv::Mat ROI, cv::Mat ROI_II, float ang);
std::vector<cv::Point2f> curvefit(std::vector<cv::Point2f> point);
float getdistance(cv::Point2f a, cv::Point2f b);
float getangle(cv::Point pointO, cv::Point pointA);
void houghtransfer();
void estimateroadparameter();
double leastsquares(vector<Point2f> hya);
std::vector<cv::Point2f> transformpoint(std::vector<cv::Vec4i> vec, int vi);
void Condensationtracking();
void gradient_compute();
void learning_model();
Status LoadGraph(string graph_file_name, Session** session);
void anglecompute();

int img_h;
int img_w;