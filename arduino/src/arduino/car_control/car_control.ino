
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <Servo.h>
ros::NodeHandle  nh;
Servo myservo;

float car_wheelyam=100; //50-150
int car_speed=189; //189-195

void messageCb1( const std_msgs::Int32& msg){
  car_speed=msg.data;
}
void messageCb2( const std_msgs::Float32& msg){
  car_wheelyam=msg.data;
}

std_msgs::Float32MultiArray car_msg;
ros::Publisher carinfo("/car_info", &car_msg);
ros::Subscriber<std_msgs::Int32> c_speed("/car_speed", &messageCb1);
ros::Subscriber<std_msgs::Float32> c_wheel("/car_wheel", &messageCb2);

void setup()
{
  myservo.attach(10);
  myservo.write(car_wheelyam);   
  Serial.begin(57600);
  pinMode(3, OUTPUT);
  nh.initNode();
  nh.advertise(carinfo);
  nh.subscribe(c_speed);
  nh.subscribe(c_wheel);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); 
  car_msg.data_length=2;
  car_msg.data = (float *)malloc(sizeof(float)*2);
}

void loop()
{
  
  car_msg.data[0]=car_speed;
  car_msg.data[1]=car_wheelyam;
  carinfo.publish(&car_msg);
 nh.spinOnce();
 analogWrite(3,car_speed);
 myservo.write(car_wheelyam);

}
