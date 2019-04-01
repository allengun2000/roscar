#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>      /*標準輸入輸出定義*/
#include <stdlib.h>     /*標準函數庫定義*/
#include <unistd.h>     /*Unix 標準函數定義*/
#include <sys/types.h> 
#include <sys/stat.h>  
#include <fcntl.h>      /*檔控制定義*/
#include <termios.h>    /*PPSIX 終端控制定義*/
#include <errno.h>      /*錯誤號定義*/
#include <iostream>
using namespace std;



 void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	// ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc, char *argv[])
{

	ros::init(argc, argv, "rosserial");


	ros::NodeHandle n;


	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::Rate loop_rate(10);
	int count = 0;


int fd;
fd = open( "/dev/ttyACM0", O_RDWR);
if (-1 == fd){
perror(" 提示錯誤！");
}
struct  termios Opt;
tcgetattr(fd, &Opt);
cfsetispeed(&Opt,B115200);     /*設置為19200Bps*/
cfsetospeed(&Opt,B115200);
tcsetattr(fd,TCSANOW,&Opt);

while (ros::ok())
{
// tcsetattr(fd, TCSAFLUSH, &Opt);
char buf [50];
memset (&buf, '\0', sizeof buf);
unsigned char cmd[]={'i'};
int n_written = write( fd, cmd, sizeof(cmd) -1 );

int n = read( fd, &buf , sizeof buf );
if (n < 0)
{
     cout << "Error reading: " << strerror(errno) << endl;
}
cout << "Read: " << buf<< endl;



		// chatter_pub.publish(msg);
		// ros::spinOnce();
		// loop_rate.sleep();
		++count;
	}
close(fd);
	return 0;
}