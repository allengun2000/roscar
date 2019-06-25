//-------cssl include-------//
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <stdio.h>
#include "cssl/cssl.h"
#include "cssl/port.h"
#include "cssl/cssl.c"
#include "cssl/uty.c"
#include "math.h"
//using std::string;
//====motor define=====

std_msgs::Int32 rpm_info;
std_msgs::Float32 PUU_info;
std_msgs::Float32 A_info;
int puu_control=0;
cssl_t *serial;
int State;
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
	PUU_info.data = buf[5]*256*256*256+buf[6] * 256 *256+ buf[3]*256 +buf[4];
if (PUU_info.data > 2147483648){
	PUU_info.data = -(4294967296 - PUU_info.data + 1);}
PUU_info.data/=1513.91667;
PUU_info.data=(PUU_info.data +720)* (70) / (1440) -35;
}else if(State==4){
	A_info.data = buf[5]*256*256*256+buf[6] * 256 *256+ buf[3]*256 +buf[4];
if (A_info.data > 2147483648){
	A_info.data = -(4294967296 - A_info.data + 1);}
A_info.data*=0.01;
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

    serial=cssl_open("/dev/ttyUSB0",mcssl_callback,0,57600,8,0,1);
	// serial=cssl_open("/dev/ttyACM0",mcssl_callback,0,57600,8,0,1);
    if (!serial) {
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
    mdelay(20);
    State=0;
    return 1;
}

//====================//
//   cssl finish      //
//====================//
void mcssl_finish(){
    cssl_close(serial);
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
void mcssl_send2motor(int rpm_value){
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
	send_data[2] = 0x06;
	send_data[3] = 0x06;

	send_data[4] = 0x00;
	send_data[5] = 0x02;
	send_data[6] = 0x04;

	memcpy(&send_data[7], rpm_send_data, 4);

	return_value = crc_chk(send_data, 11);
	l_crc = return_value % 256;
	h_crc = return_value / 256;

	send_data[11] = l_crc;
	send_data[12] = h_crc;

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
	mdelay(20);
}
void puu_send2motor(int puu_value,int puu_state)
{
puu_state=(puu_state>=0 && puu_state<=6)?puu_state+1:1;
unsigned char send_data[13] = { 0,0,0,0,0,0,0,0,0,0,0,0,0 };
		int return_value, l_crc, h_crc;
unsigned char puu_send_data[4] = { 0,0,0,0 };

	puu_send_data[1] = puu_value % 256;
	puu_value >>= 8;
	puu_send_data[0] = puu_value % 256;
	puu_value >>= 8;
	puu_send_data[3] = puu_value % 256;
	puu_value >>= 8;
	puu_send_data[2] = puu_value % 256;
	puu_value >>= 8;


	send_data[0] = 0x01;
	send_data[1] = 0x10;
	switch(puu_state){
	case 1:
		send_data[2] = 0x06;
		send_data[3] = 0x06;
		break;
	case 2:
		send_data[2] = 0x06;
		send_data[3] = 0x0a;
		break;
	case 3:
		send_data[2] = 0x06;
		send_data[3] = 0x0e;
		break;
	case 4:
		send_data[2] = 0x06;
		send_data[3] = 0x12;
		break;
	case 5:
		send_data[2] = 0x06;
		send_data[3] = 0x16;
		break;
	case 6:
		send_data[2] = 0x06;
		send_data[3] = 0x1a;
		break;
	default:
		send_data[2] = 0x06;
		send_data[3] = 0x06;
		break;
	}


	send_data[4] = 0x00;
	send_data[5] = 0x02;
	send_data[6] = 0x04;

	memcpy(&send_data[7], puu_send_data, 4);

	return_value = crc_chk(send_data, 11);
	l_crc = return_value % 256;
	h_crc = return_value / 256;

	send_data[11] = l_crc;
	send_data[12] = h_crc;

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
	mdelay(20);
//puu
	send_data[13];
	puu_send_data[1] = puu_state % 256;
	puu_state >>= 8;
	puu_send_data[0] = puu_state % 256;
	puu_state >>= 8;
	////低位元
	puu_send_data[3] = puu_state % 256;
	puu_state >>= 8;
	puu_send_data[2] = puu_state % 256;
	puu_state >>= 8;
	//高位元

	send_data[0] = 0x01;//motor 機台號碼
	send_data[1] = 0x10;//命令碼10 寫入多組字組
	send_data[2] = 0x05;//傳送地址a 01
	send_data[3] = 0x0e;//傳送地址b 12

						//兩個字元
	send_data[4] = 0x00;//00
	send_data[5] = 0x02;//02
	send_data[6] = 0x04;//4 byte 04

	memcpy(&send_data[7], puu_send_data, 4);//rpm 放入命令中

											//crc 碼生成 函式起動
    return_value = crc_chk(send_data, 11);
	l_crc = return_value % 256;
    h_crc = return_value / 256;
	//crc 放入 send_data
	send_data[11] = l_crc;
	send_data[12] = h_crc;
	
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
	cssl_putchar(serial,send_data[11]); //0x8c
	cssl_putchar(serial,send_data[12]); //0xb3

	mdelay(20);
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
	mdelay(20);
}
