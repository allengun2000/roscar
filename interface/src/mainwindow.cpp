#include "mainwindow.h"
#include "ui_mainwindow.h"
MainWindow::MainWindow(int argc, char** argv ,QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ros::init(argc, argv, "interface_golf_allenGood") ;
    n  = new ros::NodeHandle();


    ros::Rate loop_rate(10);
    // ros::spinOnce();

    sub = n->subscribe("/test",10 ,&MainWindow::callback ,this);
    sub1 = n->subscribe("/cmd",10 ,&MainWindow::callback1 ,this);
    sub2= n->subscribe("/wheelFB",10,&MainWindow::callback2,this);
    sub3= n->subscribe("/duino_velocity",10,&MainWindow::callback3,this);
    sub4= n->subscribe("/line_info",10,&MainWindow::callback4,this);
    it = new image_transport::ImageTransport(*n);
    sub_image = it->subscribe("car_line/image_raw", 1, &MainWindow::imageCallback,this);
    sub_image1 = it->subscribe("/debug_image", 1, &MainWindow::imageCallback1,this);
    pub_cmd  = n->advertise<geometry_msgs::Pose2D>("/cmd",0);
    pub_parm  = n->advertise<std_msgs::Bool>("/ParmIsChange",0);
    parm_change.data=1;
    HSV.push_back(360);HSV.push_back(0);HSV.push_back(255);HSV.push_back(0);HSV.push_back(255);HSV.push_back(0);
    n->setParam("/hsv",HSV);

    ui->oil->setMode(QwtDial::RotateNeedle);
    ui->oil->setValue(0);
    dial_needle_ = new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow, true, Qt::gray, Qt::darkGray);
    ui->oil->setNeedle(dial_needle_);
    ui->oil->show();
    ui->oil_num->setText(QString("%1").arg(0));

    ui->wheel->setMode(QwtDial::RotateNeedle);
    ui->wheel->setValue(0);
//    dial_needle_1 = new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow, true, Qt::gray, Qt::darkGray);
    ui->wheel->setNeedle(dial_needle_);
    ui->wheel->show();
    ui->wheel_num->setText(QString("%1").arg(0));

    ui->wheel_2->setMode(QwtDial::RotateNeedle);
    ui->wheel_2->setValue(0);
//    dial_needle_2 = new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow, true, Qt::gray, Qt::darkGray);
    ui->wheel_2->setNeedle(dial_needle_);
    ui->wheel_2->show();
    ui->wheel_num_2->setText(QString("%1").arg(0));

    ui->speed->setMode(QwtDial::RotateNeedle);
    ui->speed->setValue(0);
//    dial_needle_ = new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow, true, Qt::gray, Qt::darkGray);
    ui->speed->setNeedle(dial_needle_);
    ui->speed->show();
    ui->speed_num->setText(QString("%1").arg(0));


    ui->way_info->addGraph();
    ui->way_info->graph(0)->setPen(QPen(Qt::blue));
//    ui->way_info->graph(0)->setBrush(QBrush(QColor(0,0,255,20)));
//    ui->way_info->graph(0)->setLineStyle(QCPGraph::lsNone);
//    ui->way_info->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 4));
    ui->way_info->addGraph();
    ui->way_info->graph(1)->setPen(QPen(Qt::red));
//    ui->way_info->graph(1)->setBrush(QBrush(QColor(0,0,255,20)));
    ui->way_info->addGraph();
    ui->way_info->graph(2)->setPen(QPen(Qt::green));
//    ui->way_info->graph(2)->setBrush(QBrush(QColor(0,0,255,20)));
    ui->way_info->addGraph();
    ui->way_info->graph(3)->setPen(QPen(Qt::black));
        ui->way_info->addGraph();
    ui->way_info->graph(4)->setPen(QPen(Qt::cyan));
    
    ui->way_info->xAxis2->setVisible(true);
    ui->way_info->xAxis2->setTickLabels(false);
    ui->way_info->yAxis2->setVisible(true);
    ui->way_info->yAxis2->setTickLabels(false);
    ui->way_info->xAxis->setRange(-400,400);
    ui->way_info->yAxis->setRange(0, 800);

    ui->way_info->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    Mat img(250, 250, CV_8UC3, Scalar(255,255,255));
    circle(img, Point(125,125), 125, Scalar(0,0,255), 3);
    circle(img, Point(125,125), 10, Scalar(0,0,0), -1);
    line(img,Point(0,125),Point(250,125),Scalar(0,0,0));
    line(img,Point(125,0),Point(125,500),Scalar(0,0,0));
    QPixmap des_pic =cvMatToQPixmap(img);
    ui->joystick->setPixmap(des_pic);


    mTimer=new QTimer(this);
    mTimer->setTimerType(Qt::PreciseTimer);
    mTimer->setInterval(10);
    connect(mTimer,SIGNAL(timeout()),this,SLOT(ShowData()));
    mTimer->start();
}

MainWindow::~MainWindow()
{   mTimer->stop();
    ros::shutdown();
    delete ui;
}

void MainWindow::on_start_clicked()
{
        ros::NodeHandle nh;
    nh.setParam("/statego",1);
     ui->statusBar->showMessage(tr("Connect"));

}
void MainWindow::ShowData()
{

    ros::spinOnce();

}
void MainWindow::callback(const std_msgs::String::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
    QString s =msg->data.c_str();
//    ui->speed->setText(s);


}
void MainWindow::callback1(const geometry_msgs::Pose2D::ConstPtr& msg){
    ui->oil_num->setText(QString("%1").arg(msg->x));
    ui->oil->setValue(msg->x);
    ui->oil->show();
    ui->wheel_num->setText(QString("%1").arg(msg->theta));
    ui->wheel->setValue(msg->theta);
    ui->wheel->show();
    double ang=((msg->theta +720)* (70) / (1440) -35)* M_PI / 180;
    QVector<double> X;
    QVector<double> Y;
    X.push_back(0);Y.push_back(0);
    X.push_back(200*sin(ang));
    Y.push_back(200*cos(ang));
    ui->way_info->graph(3)->setData(X,Y);
    ui->way_info->replot();

}
void MainWindow::callback2(const std_msgs::Float32::ConstPtr& msg){
    wheel_showangle = msg->data;
    ui->wheel_num_2->setText(QString("%1").arg(msg->data));
    ui->wheel_2->setValue(msg->data);
    ui->wheel_2->show();
    QVector<double> X;
    QVector<double> Y;
    X.push_back(0);Y.push_back(0);
    X.push_back(200*sin((msg->data)* M_PI / 180));
    Y.push_back(200*cos((msg->data)* M_PI / 180));
    ui->way_info->graph(4)->setData(X,Y);
    ui->way_info->replot();
}
void MainWindow::callback3(const std_msgs::Float64::ConstPtr& msg){
    ui->speed_num->setText(QString("%1").arg(msg->data));
    ui->speed->setValue(msg->data);
    ui->speed->show();
}
void MainWindow::callback4(const vision_pro::line_inform::ConstPtr& msg){
    Y_polt.clear();
    X_polt.clear();
    Y_polt.resize(0);
    X_polt.resize(0);
    Y_polt_L.clear();
    X_polt_L.clear();
    Y_polt_L.resize(0);
    X_polt_L.resize(0);
    Y_polt_R.clear();
    X_polt_R.clear();
    Y_polt_R.resize(0);
    X_polt_R.resize(0);

    for(int i=0;i<msg->dot_num;i++){
        Y_polt.push_back(-msg->mid_y[i]);
        X_polt.push_back(msg->mid_x[i]);
        Y_polt_L.push_back(-msg->line1_y[i]);
        X_polt_L.push_back(msg->line1_x[i]);
        Y_polt_R.push_back(-msg->line2_y[i]);
        X_polt_R.push_back(msg->line2_x[i]);
    }
    ui->way_info->graph(0)->setData(Y_polt,X_polt );
    ui->way_info->graph(1)->setData(Y_polt_L,X_polt_L );
    ui->way_info->graph(2)->setData(Y_polt_R,X_polt_R );
    ui->way_info->replot();
}
void MainWindow::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {

    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        Main_frame=cv_ptr->image;
        cv::resize(Main_frame,result_frame,Size(ui->picture->width(),ui->picture->height()),0,0,INTER_LINEAR);
//        line(result_frame, Point(result_frame.cols/2,result_frame.rows), \
             Point(result_frame.cols/2,result_frame.rows-200), Scalar(255,255,54), 5);
        QPixmap des_pic =cvMatToQPixmap(result_frame);
        ui->picture->setPixmap(des_pic);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}
void MainWindow::imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {

    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        Main_frame=cv_ptr->image;
        cv::resize(Main_frame,result_frame,Size(ui->picture->width(),ui->picture->height()),0,0,INTER_LINEAR);
//        line(result_frame, Point(result_frame.cols/2,result_frame.rows), \
             Point(result_frame.cols/2,result_frame.rows-200), Scalar(255,255,54), 5);
        QPixmap des_pic =cvMatToQPixmap(result_frame);
        ui->picture_2->setPixmap(des_pic);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}


QImage MainWindow::cvMatToQImage( const cv::Mat &inMat ){
  switch ( inMat.type() )
  {
    // 8-bit, 4 channel
    case CV_8UC4:{
      QImage image( inMat.data,
                    inMat.cols, inMat.rows,
                    static_cast<int>(inMat.step),
                    QImage::Format_ARGB32 );
      return image;
    }

    // 8-bit, 3 channel
    case CV_8UC3:{
      QImage image( inMat.data,
                    inMat.cols, inMat.rows,
                    static_cast<int>(inMat.step),
                    QImage::Format_RGB888 );
      return image.rgbSwapped();
    }

    // 8-bit, 1 channel
    case CV_8UC1:{
      static QVector<QRgb>  sColorTable( 256 );
        // only create our color table the first time
        if ( sColorTable.isEmpty() ){
          for ( int i = 0; i < 256; ++i ){
            sColorTable[i] = qRgb( i, i, i );
          }
        }

      QImage image( inMat.data,
                    inMat.cols, inMat.rows,
                    static_cast<int>(inMat.step),
                    QImage::Format_Indexed8 );

      image.setColorTable( sColorTable );

      return image;
    }

    default:
//      qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
      ROS_INFO("ASM::cvMatToQImage() - cv::Mat image type not handled in switch:");
      break;
  }
  return QImage();
}

QPixmap MainWindow::cvMatToQPixmap( const cv::Mat &inMat){
    return QPixmap::fromImage(cvMatToQImage( inMat ));
}



void MainWindow::mousePressEvent(QMouseEvent * event)
{
    QPoint windowPoint =event->globalPos() - this->pos();

    double x_m=windowPoint.x()-ui->joystick->geometry().center().x();
    double y_m=ui->joystick->geometry().top()+ui->joystick->geometry().height()/2-windowPoint.y()+40;
    double z=sqrt(x_m*x_m + y_m*y_m)/25;
    double h=(x_m==0)?9999999999999999:y_m/x_m;
    double w=atan(h)*180/CV_PI;
    w=(w>=0)?90-w:-(w+90);
    w=w*8;
    w=(x_m<=0)?-abs(w):abs(w);
    z=(y_m<0)?-abs(z):abs(z);
    if(abs(x_m)<125 && abs(y_m)<125 && z<5){
//    qDebug() << "press" <<x_m  << ":" <<y_m<<"::"<<z<<"::"<<w;
    joy_touch=1;
    cmd_msg.x=z;
    cmd_msg.theta=w;
    pub_cmd.publish(cmd_msg);
    }
}

void MainWindow::mouseMoveEvent(QMouseEvent * e)
{
     QPoint windowPoint =e->globalPos() - this->pos();
     double x_m=windowPoint.x()-ui->joystick->geometry().center().x();
     double y_m=ui->joystick->geometry().top()+ui->joystick->geometry().height()/2-windowPoint.y()+40;
     double z=sqrt(x_m*x_m + y_m*y_m)/25;
     double h=(x_m==0)?9999999999999999:y_m/x_m;
     double w=atan(h)*180/CV_PI;
     w=(w>=0)?90-w:-(w+90);
     w=w*8;
     w=(x_m<=0)?-abs(w):abs(w);
     z=(y_m<0)?-abs(z):abs(z);
    if(abs(x_m)<125 && abs(y_m)<125 && joy_touch==1 && z<5){
//        qDebug() << "move" <<x_m  << ":" <<y_m<<"::"<<z<<"::"<<w;
        cmd_msg.x=z;
        cmd_msg.theta=w;
        pub_cmd.publish(cmd_msg);
    }
}
//滑鼠 鬆開
void MainWindow::mouseReleaseEvent(QMouseEvent * e)
{
    QPoint windowPoint =e->globalPos() - this->pos();
    if(joy_touch==1){
//    qDebug() << "Release" << windowPoint.x() << ":" << windowPoint.y();
        cmd_msg.x=0;
        cmd_msg.theta=0;
    pub_cmd.publish(cmd_msg);
    }
    joy_touch=0;
}

void MainWindow::keyPressEvent(QKeyEvent * event)
{
 if(ui->key_board->isChecked()){
switch (event->key())
{
case Qt::Key_W:
//qDebug() <<"GO";
    cmd_msg.x=-1;
    cmd_msg.theta=0;
pub_cmd.publish(cmd_msg);
break;
case Qt::Key_S:
    cmd_msg.x=5;
    cmd_msg.theta=90;
    pub_cmd.publish(cmd_msg);
//qDebug() <<"S";
break;
case Qt::Key_D:
    cmd_msg.x=0;
    cmd_msg.theta=0;
    pub_cmd.publish(cmd_msg);
//qDebug() <<"D";
break;
case Qt::Key_A:
    cmd_msg.x=5;
    cmd_msg.theta=90;
    pub_cmd.publish(cmd_msg);
//qDebug() <<"A";
break;
default:
qDebug() << event->key();
break;
}
}
}

void MainWindow::on_stop_clicked()
{
    ros::NodeHandle nh;
    nh.setParam("/statego",0);
    ui->statusBar->showMessage(tr("stop"));
    ui->learn->setChecked(false);

}

void MainWindow::on_learn_stateChanged(int arg1)
{
    ros::NodeHandle nh;

     if(ui->learn->isChecked()){
      nh.setParam("/statego",2);
     }else{
     nh.setParam("/statego",0);
     }
}


void MainWindow::on_hmax_valueChanged(double value)
{
    ros::NodeHandle nh;
        HSV[0]=value;
        nh.setParam("/golf/hsv",HSV);
        pub_parm.publish(parm_change);
}

void MainWindow::on_h_min_valueChanged(double value)
{
    ros::NodeHandle nh;
        HSV[1]=value;
        nh.setParam("/golf/hsv",HSV);
        pub_parm.publish(parm_change);
}

void MainWindow::on_s_max_valueChanged(double value)
{
    ros::NodeHandle nh;
        HSV[2]=value;
        nh.setParam("/golf/hsv",HSV);
        pub_parm.publish(parm_change);
}

void MainWindow::on_s_min_valueChanged(double value)
{
    ros::NodeHandle nh;
        HSV[3]=value;
        nh.setParam("/golf/hsv",HSV);
        pub_parm.publish(parm_change);
}

void MainWindow::on_v_max_valueChanged(double value)
{
    ros::NodeHandle nh;
        HSV[4]=value;
        nh.setParam("/golf/hsv",HSV);
        pub_parm.publish(parm_change);
}

void MainWindow::on_v_min_valueChanged(double value)
{
    ros::NodeHandle nh;
        HSV[5]=value;
        nh.setParam("/golf/hsv",HSV);
        pub_parm.publish(parm_change);
}
