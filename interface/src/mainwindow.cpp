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
    it = new image_transport::ImageTransport(*n);
    sub_image = it->subscribe("/usb_cam/image_raw", 1, &MainWindow::imageCallback,this);
    pub_cmd  = n->advertise<geometry_msgs::Quaternion>("/cmd",0);


    ui->oil->setMode(QwtDial::RotateNeedle);
    ui->oil->setValue(0);
    dial_needle_ = new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow, true, Qt::gray, Qt::darkGray);
    ui->oil->setNeedle(dial_needle_);
    ui->oil->show();
    ui->oil_num->setText(QString("%1").arg(0));

    ui->wheel->setMode(QwtDial::RotateNeedle);
    ui->wheel->setValue(0);
    dial_needle_1 = new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow, true, Qt::gray, Qt::darkGray);
    ui->wheel->setNeedle(dial_needle_1);
    ui->wheel->show();
    ui->wheel_num->setText(QString("%1").arg(0));


    ui->way_info->addGraph();
    ui->way_info->graph(0)->setPen(QPen(Qt::blue));
    ui->way_info->graph(0)->setBrush(QBrush(QColor(0,0,255,20)));
    ui->way_info->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->way_info->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 4));
    ui->way_info->xAxis2->setVisible(true);
    ui->way_info->xAxis2->setTickLabels(false);
    ui->way_info->yAxis2->setVisible(true);
    ui->way_info->yAxis2->setTickLabels(false);
    ui->way_info->xAxis->setRange(0,100);
    ui->way_info->yAxis->setRange(0, 100);
    Y_polt.append(10);
    Y_polt.append(30);
    X_polt.append(20);
    X_polt.append(60);
    ui->way_info->graph(0)->setData(X_polt, Y_polt);
    ui->way_info->replot();
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

     ui->statusBar->showMessage(tr("Connect"));
//     ui->speed->setText(QString::number(999));

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
void MainWindow::callback1(const geometry_msgs::Quaternion::ConstPtr& msg){
    ui->oil_num->setText(QString("%1").arg(msg->z));
    ui->oil->setValue(msg->z);
    ui->oil->show();
    ui->wheel_num->setText(QString("%1").arg(msg->w));
    ui->wheel->setValue(msg->w);
    ui->wheel->show();
}
void MainWindow::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {

    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        Main_frame=cv_ptr->image;
        cv::resize(Main_frame,result_frame,Size(ui->picture->width(),ui->picture->height()),0,0,INTER_LINEAR);
        QPixmap des_pic =cvMatToQPixmap(result_frame);
        ui->picture->setPixmap(des_pic);
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
    w=(x_m<0)?-abs(w):abs(w);
    w=(w>0)?90-w:-(w+90);
    z=(y_m<0)?-abs(z):abs(z);
    if(abs(x_m)<125 && abs(y_m)<125 && z<5){
//    qDebug() << "press" <<x_m  << ":" <<y_m<<"::"<<z<<"::"<<w;
    joy_touch=1;
    cmd_msg.x=x_m;
    cmd_msg.y=y_m;
    cmd_msg.z=z;
    cmd_msg.w=w;
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
     w=(x_m<0)?-abs(w):abs(w);
     w=(w>0)?90-w:-(w+90);
     z=(y_m<0)?-abs(z):abs(z);
    if(abs(x_m)<125 && abs(y_m)<125 && joy_touch==1 && z<5){
//        qDebug() << "move" <<x_m  << ":" <<y_m<<"::"<<z<<"::"<<w;
        cmd_msg.x=x_m;
        cmd_msg.y=y_m;
        cmd_msg.z=z;
        cmd_msg.w=w;
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
    cmd_msg.y=0;
    cmd_msg.z=0;
    cmd_msg.w=0;
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
cmd_msg.x=0;
cmd_msg.y=62;
cmd_msg.z=2;
cmd_msg.w=0;
pub_cmd.publish(cmd_msg);
break;
case Qt::Key_S:
    cmd_msg.x=0;
    cmd_msg.y=0;
    cmd_msg.z=0;
    cmd_msg.w=0;
    pub_cmd.publish(cmd_msg);
//qDebug() <<"S";
break;
case Qt::Key_D:
    cmd_msg.x=62;
    cmd_msg.y=62;
    cmd_msg.z=2;
    cmd_msg.w=45;
    pub_cmd.publish(cmd_msg);
//qDebug() <<"D";
break;
case Qt::Key_A:
    cmd_msg.x=-62;
    cmd_msg.y=62;
    cmd_msg.z=2;
    cmd_msg.w=-45;
    pub_cmd.publish(cmd_msg);
//qDebug() <<"A";
break;
default:
qDebug() << event->key();
break;
}
}
}
