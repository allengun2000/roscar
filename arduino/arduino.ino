
#include <ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle  nh;
int x=0;
int y=0;
int yam=0;
void messageCb(const geometry_msgs::Twist& msg){
  x=msg.linear.x; 
  y=msg.linear.y; 
  yam=msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", messageCb );


std_msgs::Float32MultiArray feedback;
ros::Publisher p("/arduino_feedback", &feedback);


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
//    pinMode(11, OUTPUT);
//  pinMode(10, OUTPUT);
//    pinMode(9, OUTPUT);
//  pinMode(8, OUTPUT);
    pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
    pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(sub);
  feedback.data_length=2;
}

void loop()
{
  feedback.data[0]=x;
  feedback.data[1]=yam;
  p.publish(&feedback);
  if(x==2){digitalWrite(LED_BUILTIN,HIGH);}
  nh.spinOnce();
  delay(1);
if(x==2){
    analogWrite(3, 100);
    analogWrite(2, 0); 
//
    analogWrite(5, 100);
    analogWrite(4, 0); 
//
    
    analogWrite(8, 100);
    analogWrite(9, 0);
//
    analogWrite(10, 100); 
    analogWrite(11, 0);}
else if(x==-2){
    analogWrite(3, 0);
    analogWrite(2, 100); 
//
    analogWrite(5, 0);
    analogWrite(4, 100); 
//
    
    analogWrite(8, 0);
    analogWrite(9, 100);
//
    analogWrite(10, 0); 
    analogWrite(11, 100);}
else if(yam==2){
    analogWrite(3, 100);
    analogWrite(2, 0); 
//
    analogWrite(5, 0);
    analogWrite(4, 100); 
//
    
    analogWrite(8, 100);
    analogWrite(9, 0);
//
    analogWrite(10, 0); 
    analogWrite(11, 100);}
else if(yam==-2){
    analogWrite(3, 0);
    analogWrite(2, 100); 
//
    analogWrite(5, 100);
    analogWrite(4, 0); 
//
    
    analogWrite(8, 0);
    analogWrite(9, 100);
//
    analogWrite(10, 100); 
    analogWrite(11, 0);
    }else{
    analogWrite(3, 0);
    analogWrite(2, 0); 
//
    analogWrite(5, 0);
    analogWrite(4, 0); 
//
    
    analogWrite(8, 0);
    analogWrite(9, 0);
//
    analogWrite(10, 0); 
    analogWrite(11, 0);}
    
}
