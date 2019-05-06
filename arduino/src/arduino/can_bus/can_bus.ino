#include <CANBus.h>
#include <Encoder.h>
 
volatile unsigned long num1 = 0L;
volatile unsigned long num2 = 0L;


unsigned char len = 0;
unsigned char buf[50];
unsigned char stmp[2] = {0x05, 0x0C}; // 0x05=0101 => Clear the old zero position
unsigned char stmp2[2] = {0x03, 0x0C}; // 0x03=0011 => Set up the zero position,0x0c
unsigned char transfer_data[7];
unsigned char cmd[10];
int angle = 0;
int black =42; 
int white = 43; 
int orange = 44; 

void setup() {
  Serial1.begin(57600);

  CAN.begin(CAN_500KBPS);
  CAN.sendMsgBuf(0x7C0, 0, 2, stmp);  //0x7C0 = CAN-ID Kind of Message;stmp = clear the zero position
  delay(1000);
  CAN.sendMsgBuf(0x7C0, 0, 2, stmp2);  //0x7C0 = CAN-ID Kind of Message;stmp2 = set up the zero position
  delay(1000);
   Enc0.begin(MODE_AB_PHASE);
   pinMode(4, OUTPUT);
   pinMode(5, OUTPUT);
   digitalWrite(4, LOW); 
    analogWrite(5, 0);
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
      angle = angle;
    }
    else{
      angle = (angle - 65536);
    }

//    for (int i = 0; i < 7; i++) {
//      Serial.print(transfer_data[i]);
//      Serial.print("\t");
//    }
  }



     Serial1.print(0x30);
     Serial1.print('\r');
     Serial1.print(angle);
     Serial1.print('\r');
Serial1.println(Enc0.read()); //4294967296

if (Serial1.available() > 0) {
  
if (Serial1.read() == 's') {
Serial1.readBytes(cmd, 1);
if(cmd[0]=='1'){
  digitalWrite(4, HIGH); 
}
if(cmd[0]=='0'){
  digitalWrite(4, LOW); 
}
}
if (Serial1.read() == 'o'){
int size_= Serial1.readBytesUntil('\n', cmd, 10);
int speed_=0;
if(size_==0){
   Serial1.flush();
}
  for (int i = 0 ; i < size_ ; i++) {
    speed_=speed_+(cmd[i]-48)*pow(10,size_-i-1);
 }
 if(speed_<=255 && speed_>=0){
        analogWrite(5, speed_);}
}
 Serial1.flush();
}


}
