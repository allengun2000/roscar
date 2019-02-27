#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QUrl>
#include <QThread>
#include <QTimer>
#include <QTime>
#include"modbus.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#define AddressTest 40001
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv,QWidget *parent = 0);
    ~MainWindow();



private:


public:
ros::NodeHandle *n;

    ros::Publisher pub;
    ros::Subscriber sub;
    
    void callback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("I heard: [%s]", msg->data.c_str());;
    }



private slots:
    void on_btn_Connect_clicked();

    void on_btn_Send_clicked();

    void on_btn_Read_clicked();

    void on_pushButton_clicked();
    
    void ShowData();

private:
    modbus_t *mb;
    Ui::MainWindow *ui;
    QTimer *mTimer;
};

#endif // MAINWINDOW_H
