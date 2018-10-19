/* Example application of Columbo Simple Serial Library
 * Copyright 2003 Marcin Siennicki <m.siennicki@cloos.pl>
 * see COPYING file for details */

#include <stdio.h>
#include <unistd.h>

#include "cssl/cssl.h"
#include "cssl/port.h"
#include "cssl/cssl.c"
#include "cssl/uty.c"
/* if it is time to finish */
static int finished=0;


/* example callback, it gets its id, buffer, and buffer length */
static void callback(int id,
		     uint8_t *buf,
		     int length)
{
    int i;
int rpm_info = buf[3] * 256 + buf[4];
if (rpm_info > 32767)
	{
	rpm_info = -(65535 - rpm_info + 1);}
	printf("%d\n",rpm_info);
    fflush(stdout);
}
int rpm_value=1000;
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



int main()
{
    cssl_t *serial;

    cssl_start();
    
    serial=cssl_open("/dev/ttyUSB0",callback,0,
		     57600,8,0,1);

    if (!serial) {
	printf("%s\n",cssl_geterrormsg());
	return -1;
    }

    cssl_putchar(serial,0x01);
    cssl_putchar(serial,0x06);
    cssl_putchar(serial,0x02);
    cssl_putchar(serial,0x3c);
    cssl_putchar(serial,0x00);
    cssl_putchar(serial,0x05);
    cssl_putchar(serial,0x88);
    cssl_putchar(serial,0x7d);
    mdelay(20);
    unsigned char send_data[13] = { 0,0,0,0,0,0,0,0,0,0,0,0,0 };
	rpm_value = 0;
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
	mdelay(20);
	cssl_putchar(serial,0x01);
	cssl_putchar(serial,0x03);
	cssl_putchar(serial,0x00);
	cssl_putchar(serial,0x12);
	cssl_putchar(serial,0x00);
	cssl_putchar(serial,0x02);
	cssl_putchar(serial,0x64);
	cssl_putchar(serial,0x0e);
	mdelay(20);
    while (!finished)
	pause();


    cssl_close(serial);
    cssl_stop();

    return 0;
}