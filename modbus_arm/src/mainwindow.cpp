#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char** argv,QWidget *parent) : QMainWindow(parent)
,ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->lineEdit->setText(QLatin1Literal("127.0.0.1:502"));
    ros::init(argc, argv, "moddddd") ;
	ros::NodeHandle n;
	
    ros::Rate loop_rate(10);
    // ros::spinOnce();

    sub = n.subscribe("/vision/object",10 ,&MainWindow::callback ,this);


    mTimer=new QTimer(this);
    mTimer->setTimerType(Qt::PreciseTimer);
    mTimer->setInterval(100);
    connect(mTimer,SIGNAL(timeout()),this,SLOT(ShowData()));

}

MainWindow::~MainWindow()
{
	modbus_free(mb);
    delete ui;
}

void MainWindow::on_btn_Connect_clicked()
{

     const QUrl url = QUrl::fromUserInput(ui->lineEdit->text());
     mb=modbus_new_tcp(url.host().toStdString().c_str(),url.port());
    // mb = modbus_new_tcp("192.168.1.88",502);
     int res = modbus_connect(mb);
     modbus_set_slave(mb, 2);
     if(res<0)
         ui->statusBar->showMessage(tr("Connect failed"));
     else
         ui->statusBar->showMessage(tr("Connected"));

    
}

void MainWindow::on_btn_Send_clicked()
{


     int res=modbus_write_register(mb, 0x01, ui->lineEdit_2->text().toInt());
    //uint16_t regs[4]={5,6,7,8};
    
    //int res=modbus_write_registers(mb,0,4,regs);

   if(res ==-1 )
	   ui->statusBar->showMessage(tr("Write failed"));
}

void MainWindow::on_btn_Read_clicked()
{
	uint16_t tab_reg[1] = { 0 };
    int a=modbus_read_registers(mb, 0, 1, tab_reg);
	if (modbus_read_registers(mb, 0, 1, tab_reg) == -1) 
		ui->statusBar->showMessage(tr("Read failed"));
	
	ui->lineEdit_3->setText(QString::number(tab_reg[0]));
}

void MainWindow::on_pushButton_clicked()
{
// ros::spinOnce();
mTimer->start();
ROS_INFO("time start");
}

void MainWindow::ShowData()
{
    //  ROS_INFO("I hearssssss");
    ros::spinOnce();
}