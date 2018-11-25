#include "ros/ros.h"
#include "std_msgs/String.h"
//-------cssl include-------//
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "motion/checknum.h"
#include <stdio.h>
#include "cssl/cssl.h"
#include "cssl/port.h"
#include "cssl/cssl.c"
#include "cssl/uty.c"
#include "math.h"
#include <iostream>
using namespace std;
//====motor define=====

std_msgs::Float64 rpm_info;
std_msgs::Float64 PUU_info;
std_msgs::Float64 A_info;
std_msgs::Float64 rpm_info1;
std_msgs::Float64 PUU_info1;
std_msgs::Float64 A_info1;
std_msgs::Float64 rpm_info2;
std_msgs::Float64 PUU_info2;
std_msgs::Float64 A_info2;
int puu_control=0;
cssl_t *serial;
cssl_t *serial1;
cssl_t *serial2;
int State;
int State1;
int State2;
//====================//
//   cssl callback    //
//====================//

static void mcssl_callback(int id, uint8_t *buf, int length)
{
if(State==1){
rpm_info.data = buf[3] * 256 + buf[4];
if (rpm_info.data > 32767){
	rpm_info.data = -(65535 - rpm_info.data + 1);}
}else if(State==3){
	PUU_info.data = buf[6]*256*256+buf[3] * 256 + buf[4];
if (PUU_info.data > 8388608){
	PUU_info.data = -(16777216 - PUU_info.data + 1);}
PUU_info.data/=10000;
}else if(State==4){
	A_info.data = buf[5]*256*256*256+buf[6] * 256 *256+ buf[3]*256 +buf[4];
if (A_info.data > 2147483648){
	A_info.data = -(4294967296 - A_info.data + 1);}
A_info.data*=0.01;
}
fflush(stdout);
}
static void mcssl_callback1(int id, uint8_t *buf, int length)
{
if(State1==1){
rpm_info1.data = buf[3] * 256 + buf[4];
if (rpm_info1.data > 32767){
	rpm_info1.data = -(65535 - rpm_info1.data + 1);}
}else if(State1==3){
	PUU_info1.data = buf[6]*256*256+buf[3] * 256 + buf[4];
if (PUU_info1.data > 8388608){
	PUU_info1.data = -(16777216 - PUU_info1.data + 1);}
PUU_info1.data/=10000;
}else if(State1==4){
	A_info1.data = buf[5]*256*256*256+buf[6] * 256 *256+ buf[3]*256 +buf[4];
if (A_info1.data > 2147483648){
	A_info1.data = -(4294967296 - A_info1.data + 1);}
A_info1.data*=0.01;
}
fflush(stdout);
}
static void mcssl_callback2(int id, uint8_t *buf, int length)
{
if(State2==1){
rpm_info2.data = buf[3] * 256 + buf[4];
if (rpm_info2.data > 32767){
	rpm_info2.data = -(65535 - rpm_info2.data + 1);}
}else if(State2==3){
	PUU_info2.data = buf[6]*256*256+buf[3] * 256 + buf[4];
if (PUU_info2.data > 8388608){
	PUU_info2.data = -(16777216 - PUU_info2.data + 1);}
PUU_info2.data/=10000;
}else if(State2==4){
	A_info2.data = buf[5]*256*256*256+buf[6] * 256 *256+ buf[3]*256 +buf[4];
if (A_info2.data > 2147483648){
	A_info2.data = -(4294967296 - A_info2.data + 1);}
A_info2.data*=0.01;
}
fflush(stdout);
}
//====================//
//   cssl init        //
//====================//
int mcssl_init(/*const char *devs*/)
{
    char *devs;
    std::string port_name;

     cssl_start();

    // serial=cssl_open("/dev/ttyUSB0",mcssl_callback,0,57600,8,0,1);
	serial=cssl_open("/dev/ttyUSB0",mcssl_callback,0,57600,8,0,1);
    if (!serial) {
	printf("%s\n",cssl_geterrormsg());
    printf("---> Motion RS485 OPEN FAIL <---\n");
    fflush(stdout);
	return -1;
    }
	serial1=cssl_open("/dev/ttyUSB1",mcssl_callback1,0,57600,8,0,1);
    if (!serial1) {
	printf("%s\n",cssl_geterrormsg());
    printf("---> Motion RS485 OPEN FAIL <---\n");
    fflush(stdout);
	return -1;
    }
	serial2=cssl_open("/dev/ttyUSB2",mcssl_callback2,0,57600,8,0,1);
    if (!serial2) {
	printf("%s\n",cssl_geterrormsg());
    printf("---> Motion RS485 OPEN FAIL <---\n");
    fflush(stdout);
	return -1;
    }
	
    ///Turn on communication mode
    cssl_putchar(serial,0x01);
    cssl_putchar(serial,0x06);
    cssl_putchar(serial,0x02);
    cssl_putchar(serial,0x3c);
    cssl_putchar(serial,0x00);
    cssl_putchar(serial,0x05);
    cssl_putchar(serial,0x88);
    cssl_putchar(serial,0x7d);

	cssl_putchar(serial1,0x01);
    cssl_putchar(serial1,0x06);
    cssl_putchar(serial1,0x02);
    cssl_putchar(serial1,0x3c);
    cssl_putchar(serial1,0x00);
    cssl_putchar(serial1,0x05);
    cssl_putchar(serial1,0x88);
    cssl_putchar(serial1,0x7d);

	cssl_putchar(serial2,0x01);
    cssl_putchar(serial2,0x06);
    cssl_putchar(serial2,0x02);
    cssl_putchar(serial2,0x3c);
    cssl_putchar(serial2,0x00);
    cssl_putchar(serial2,0x05);
    cssl_putchar(serial2,0x88);
    cssl_putchar(serial2,0x7d);
    mdelay(20);
    State=0;
	State1=0;
	State2=0;
    return 1;
}

//====================//
//   cssl finish      //
//====================//
void mcssl_finish(){
    cssl_close(serial);
	cssl_close(serial1);
	cssl_close(serial2);
    cssl_stop();
}
unsigned int crc_chk(unsigned char* data, unsigned char length)
	   {
		   int j;
		   unsigned int reg_crc = 0xFFFF;
		   while (length--)
		   {
			   reg_crc ^= *data++;
			   for (j = 0; j<8; j++)
			   {
				   if (reg_crc & 0x01)
				   {
					   reg_crc = (reg_crc >> 1) ^ 0xA001;
				   }
				   else
				   {
					   reg_crc = (reg_crc >> 1);
				   }
			   }
		   }
		   return reg_crc;
	   }

 bool checkserver(motion::checknum::Request &req,motion::checknum::Response &res)
	   {
		   int j;
		   unsigned int reg_crc = 0xFFFF;
		   unsigned char data =req.data[0];
		   int len_temp=req.len;
		   while (req.len--)
		   {
			   data=req.data[len_temp-req.len-1];
			   reg_crc ^= data;
			   for (j = 0; j<8; j++)
			   {
				   if (reg_crc & 0x01)
				   {
					   reg_crc = (reg_crc >> 1) ^ 0xA001;
				   }
				   else
				   {
					   reg_crc = (reg_crc >> 1);
				   }
			   }
		   }
		   res.output=reg_crc;
		   return true;
	   }
void mcssl_send2motor(int motor_num,int rpm_value)
{
unsigned char send_data[13] = { 0,0,0,0,0,0,0,0,0,0,0,0,0 };
		int return_value, l_crc, h_crc;
unsigned char rpm_send_data[4] = { 0,0,0,0 };

	rpm_send_data[1] = rpm_value % 256;
	rpm_value >>= 8;
	rpm_send_data[0] = rpm_value % 256;
	rpm_value >>= 8;
	rpm_send_data[3] = rpm_value % 256;
	rpm_value >>= 8;
	rpm_send_data[2] = rpm_value % 256;
	rpm_value >>= 8;


	send_data[0] = 0x01;
	send_data[1] = 0x10;
	send_data[2] = 0x01;
	send_data[3] = 0x12;

	send_data[4] = 0x00;
	send_data[5] = 0x02;
	send_data[6] = 0x04;

	memcpy(&send_data[7], rpm_send_data, 4);

	return_value = crc_chk(send_data, 11);
	l_crc = return_value % 256;
	h_crc = return_value / 256;

	send_data[11] = l_crc;
	send_data[12] = h_crc;
    switch(motor_num){
	case 0:
    cssl_putchar(serial,send_data[0]);
	cssl_putchar(serial,send_data[1]);
	cssl_putchar(serial,send_data[2]);
	cssl_putchar(serial,send_data[3]);
	cssl_putchar(serial,send_data[4]);
	cssl_putchar(serial,send_data[5]);
	cssl_putchar(serial,send_data[6]);
	cssl_putchar(serial,send_data[7]);
	cssl_putchar(serial,send_data[8]);
	cssl_putchar(serial,send_data[9]);
	cssl_putchar(serial,send_data[10]);
	cssl_putchar(serial,send_data[11]);
	cssl_putchar(serial,send_data[12]);
    State=2;
	break;
	case 1:
    cssl_putchar(serial1,send_data[0]);
	cssl_putchar(serial1,send_data[1]);
	cssl_putchar(serial1,send_data[2]);
	cssl_putchar(serial1,send_data[3]);
	cssl_putchar(serial1,send_data[4]);
	cssl_putchar(serial1,send_data[5]);
	cssl_putchar(serial1,send_data[6]);
	cssl_putchar(serial1,send_data[7]);
	cssl_putchar(serial1,send_data[8]);
	cssl_putchar(serial1,send_data[9]);
	cssl_putchar(serial1,send_data[10]);
	cssl_putchar(serial1,send_data[11]);
	cssl_putchar(serial1,send_data[12]);
    State1=2;
	break;
	case 2:
    cssl_putchar(serial2,send_data[0]);
	cssl_putchar(serial2,send_data[1]);
	cssl_putchar(serial2,send_data[2]);
	cssl_putchar(serial2,send_data[3]);
	cssl_putchar(serial2,send_data[4]);
	cssl_putchar(serial2,send_data[5]);
	cssl_putchar(serial2,send_data[6]);
	cssl_putchar(serial2,send_data[7]);
	cssl_putchar(serial2,send_data[8]);
	cssl_putchar(serial2,send_data[9]);
	cssl_putchar(serial2,send_data[10]);
	cssl_putchar(serial2,send_data[11]);
	cssl_putchar(serial2,send_data[12]);
    State2=2;
	break;
	}


}

void motionfeedback(){
    cssl_putchar(serial,0x01);
	cssl_putchar(serial,0x03);
	cssl_putchar(serial,0x00);
	cssl_putchar(serial,0x12);
	cssl_putchar(serial,0x00);
	cssl_putchar(serial,0x02);
	cssl_putchar(serial,0x64);
	cssl_putchar(serial,0x0e);
    State=1;
	cssl_putchar(serial1,0x01);
	cssl_putchar(serial1,0x03);
	cssl_putchar(serial1,0x00);
	cssl_putchar(serial1,0x12);
	cssl_putchar(serial1,0x00);
	cssl_putchar(serial1,0x02);
	cssl_putchar(serial1,0x64);
	cssl_putchar(serial1,0x0e);
    State1=1;
	cssl_putchar(serial2,0x01);
	cssl_putchar(serial2,0x03);
	cssl_putchar(serial2,0x00);
	cssl_putchar(serial2,0x12);
	cssl_putchar(serial2,0x00);
	cssl_putchar(serial2,0x02);
	cssl_putchar(serial2,0x64);
	cssl_putchar(serial2,0x0e);
    State2=1;
	mdelay(20);
}
void Puufeedback(){
    cssl_putchar(serial,0x01);
	cssl_putchar(serial,0x03);
	cssl_putchar(serial,0x00);
	cssl_putchar(serial,0x14);
	cssl_putchar(serial,0x00);
	cssl_putchar(serial,0x02);
	cssl_putchar(serial,0x84);
	cssl_putchar(serial,0x0f);
    State=3;
    cssl_putchar(serial1,0x01);
	cssl_putchar(serial1,0x03);
	cssl_putchar(serial1,0x00);
	cssl_putchar(serial1,0x14);
	cssl_putchar(serial1,0x00);
	cssl_putchar(serial1,0x02);
	cssl_putchar(serial1,0x84);
	cssl_putchar(serial1,0x0f);
    State1=3;
    cssl_putchar(serial2,0x01);
	cssl_putchar(serial2,0x03);
	cssl_putchar(serial2,0x00);
	cssl_putchar(serial2,0x14);
	cssl_putchar(serial2,0x00);
	cssl_putchar(serial2,0x02);
	cssl_putchar(serial2,0x84);
	cssl_putchar(serial2,0x0f);
    State2=3;
	mdelay(20);
}

void Afeedback(){
    cssl_putchar(serial,0x01);
	cssl_putchar(serial,0x03);
	cssl_putchar(serial,0x00);
	cssl_putchar(serial,0x16);
	cssl_putchar(serial,0x00);
	cssl_putchar(serial,0x02);
	cssl_putchar(serial,0x25);
	cssl_putchar(serial,0xcf);
    State=4;
	cssl_putchar(serial1,0x01);
	cssl_putchar(serial1,0x03);
	cssl_putchar(serial1,0x00);
	cssl_putchar(serial1,0x16);
	cssl_putchar(serial1,0x00);
	cssl_putchar(serial1,0x02);
	cssl_putchar(serial1,0x25);
	cssl_putchar(serial1,0xcf);
    State1=4;
	cssl_putchar(serial2,0x01);
	cssl_putchar(serial2,0x03);
	cssl_putchar(serial2,0x00);
	cssl_putchar(serial2,0x16);
	cssl_putchar(serial2,0x00);
	cssl_putchar(serial2,0x02);
	cssl_putchar(serial2,0x25);
	cssl_putchar(serial2,0xcf);
    State2=4;
	mdelay(20);
}
void MotorSpeedcall(const geometry_msgs::Twist::ConstPtr& msg)
{
	
	mcssl_send2motor(0,int(msg->linear.x));
	mcssl_send2motor(1,int(msg->linear.y));
	mcssl_send2motor(2,int(msg->linear.z));
	mcssl_send2motor(3,int(msg->angular.x));
	mcssl_send2motor(4,int(msg->angular.y));
	mcssl_send2motor(5,int(msg->angular.z));
	mdelay(20);

}
int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "asdaA3");

	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("check_num", checkserver);
	ros::Subscriber motion_sub3 = n.subscribe("/Motorcmd", 10, MotorSpeedcall);
	ros::Publisher A_pub = n.advertise<geometry_msgs::Twist>("/MotorA", 1000);
	ros::Publisher puu_pub= n.advertise<geometry_msgs::Twist>("/MotorFB", 1000);
	ros::Publisher speed_pub = n.advertise<geometry_msgs::Twist>("/speedFB", 1000);
	geometry_msgs::Twist motora_msg;
	geometry_msgs::Twist motorpuu_msg;
	geometry_msgs::Twist motorspeed_msg;

	int count = 0;
	 do{if(mcssl_init() > 0){
        break;
        }else{
          usleep(1000000);//1s = 1,000,000 us
        }}while(ros::ok());
    ROS_INFO("Motion is running\n");
    ros::Rate loop_rate(30);
	while (ros::ok())
	{

		Afeedback();
		Puufeedback();
		motora_msg.linear.x=A_info.data;
		motora_msg.linear.y=A_info1.data;
		motora_msg.linear.z=A_info2.data;
		motorpuu_msg.linear.x=PUU_info.data;
		motorpuu_msg.linear.y=PUU_info1.data;
		motorpuu_msg.linear.z=PUU_info2.data;
		A_pub.publish(motora_msg);
		puu_pub.publish(motorpuu_msg);


		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
