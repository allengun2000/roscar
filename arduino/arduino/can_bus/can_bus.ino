#include <CANBus.h>

#include <ros.h>
#include <std_msgs/Float64.h>
ros::NodeHandle nh;
std_msgs::Float64 steer_msg;
ros::Publisher p("/steer", &steer_msg);

unsigned char len = 0;
unsigned char buf[5];
unsigned char stmp[2] = {0x05, 0x0C}; // 0x05=0101 => Clear the old zero position
unsigned char stmp2[2] = {0x03, 0x0C}; // 0x03=0011 => Set up the zero position,0x0c
unsigned char transfer_data[7];
double angle = 0;
int count = 0;

void setup() {
  Serial.begin(57600);
  CAN.begin(CAN_500KBPS);

  CAN.sendMsgBuf(0x7C0, 0, 2, stmp);  //0x7C0 = CAN-ID Kind of Message;stmp = clear the zero position
  delay(1000);
  CAN.sendMsgBuf(0x7C0, 0, 2, stmp2);  //0x7C0 = CAN-ID Kind of Message;stmp2 = set up the zero position
  delay(1000);
}

void loop() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);    // 讀取資料

    transfer_data[0] = 0x30;
    transfer_data[1] = 0x41;
    transfer_data[6] = 0xFF;

    transfer_data[2] = buf[0];
    transfer_data[3] = buf[1];
    transfer_data[4] = 0;
    transfer_data[5] = 0;

    angle = buf[0] + buf[1] * 256;
    if (angle <= 32767){
      angle = angle * 0.1;
    }
    else{
      angle = (angle - 65536) * 0.1;
    }

//    for (int i = 0; i < 7; i++) {
//      Serial.print(transfer_data[i]);
//      Serial.print("\t");
//    }
    steer_msg.data=angle;
    p.publish(&steer_msg);
  }

//  if (count < 65536) {
//    count ++;
//  }
//  else {
//    count = 0;
//  }

}
