
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <Servo.h>
ros::NodeHandle  nh;
Servo myservo;
int a=0;
float car_wheelyam=100;
int car_speed=189;
void messageCb( const std_msgs::Float32MultiArray& msg){
  a=msg.data[1];
}
void messageCb1( const std_msgs::Int32& msg){
  car_speed=msg.data;
}
void messageCb2( const std_msgs::Float32& msg){
  car_wheelyam=msg.data;
}

std_msgs::Float32MultiArray car_msg;
ros::Publisher carinfo("/car_info", &car_msg);
ros::Subscriber<std_msgs::Float32MultiArray> s("/car_info", &messageCb);
ros::Subscriber<std_msgs::Int32> c_speed("/car_speed", &messageCb1);
ros::Subscriber<std_msgs::Float32> c_wheel("/car_wheel", &messageCb2);

void setup()
{
  myservo.attach(10);
  myservo.write(car_wheelyam);   
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(carinfo);
  nh.subscribe(s);
  nh.subscribe(c_speed);
  nh.subscribe(c_wheel);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); 
  car_msg.data_length=10;
  car_msg.data = (float *)malloc(sizeof(float)*10);
}

void loop()
{
  for(int i=0;i<10;i++){
  car_msg.data[i]=i;}
  carinfo.publish(&car_msg);
  if(a==1){
    digitalWrite(LED_BUILTIN, HIGH); 
  }
  nh.spinOnce();
  
  delay(1);
}
