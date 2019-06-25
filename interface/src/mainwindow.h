#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QTimer>
#include<QTime>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <QPixmap>
#include <QPainter>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "qcustomplot.h"
#include<qwt/qwt_dial.h>
#include<qwt/qwt_dial_needle.h>
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "vision_pro/line_inform.h"
using namespace std;
using namespace cv;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv,QWidget *parent = 0);
    ~MainWindow();
public:
ros::NodeHandle *n;

    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;
    ros::Publisher pub_cmd;
    ros::Publisher pub_parm;
    geometry_msgs::Pose2D cmd_msg;
    std_msgs::Bool parm_change;
    cv::Mat  Main_frame;
    cv::Mat  result_frame;
    image_transport::ImageTransport *it;
    image_transport::Subscriber sub_image ;
    image_transport::Subscriber sub_image1 ;
    QVector<double> X_polt;
    QVector<double> Y_polt;
    QVector<double> X_polt_L;
    QVector<double> Y_polt_L;
    QVector<double> X_polt_R;
    QVector<double> Y_polt_R;
    std::vector<double> HSV;
    double wheel_showangle;

    QwtDialSimpleNeedle *dial_needle_;
//    QwtDialSimpleNeedle *dial_needle_1;
//    QwtDialSimpleNeedle *dial_needle_2;
//    QwtDialSimpleNeedle *dial_needle_3;
    bool joy_touch=0;
    void keyPressEvent(QKeyEvent * event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent * e);
    void mouseMoveEvent(QMouseEvent * e);
private slots:
    void on_start_clicked();
    void ShowData();
    void callback(const std_msgs::String::ConstPtr& msg);
    void callback1(const geometry_msgs::Pose2D::ConstPtr& msg);
    void callback2(const std_msgs::Float32::ConstPtr& msg);
    void callback3(const std_msgs::Float64::ConstPtr& msg);
    void callback4(const vision_pro::line_inform::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
    QPixmap cvMatToQPixmap( const cv::Mat &inMat);
    QImage cvMatToQImage( const cv::Mat &inMat );


    void on_stop_clicked();

    void on_learn_stateChanged(int arg1);

    void on_hmax_valueChanged(double value);

    void on_h_min_valueChanged(double value);

    void on_s_max_valueChanged(double value);

    void on_s_min_valueChanged(double value);

    void on_v_max_valueChanged(double value);

    void on_v_min_valueChanged(double value);

private:
    Ui::MainWindow *ui;
    QTimer *mTimer;

};

#endif // MAINWINDOW_H
