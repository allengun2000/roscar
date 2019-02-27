
#include <ros.h>
#include <geometry_msgs/Twist.h>

int ledPin = 13;
#define STEPS 3200  //馬達是200步，但是板子又把他切成16份
#define pi 3.1415926
#define ROBOT_RADIUS 1

byte stepPin1 = 7;              //綠色
byte directionPin1 = 9;         //橘色    
byte stepPin2 = 12;              //黃色
byte directionPin2 = 10;        //橘色
byte stepPin3 = 13;             //黃色 
byte directionPin3 = 11;        //橘色
byte ENA=3;
//TX2-->RX1  紫紅紅棕

int liner_x=10;
int liner_y=10;
int angle_z=0;

int motorA=0; //-1000-1000  //100-0.1
int motorB=0;
int motorC=0;
const double mAngle1Cos(cos(5*M_PI/6));
const double mAngle2Cos(cos(M_PI/6));
const double mAngle3Cos(cos(3*M_PI/2));

const double mAngle1Sin(sin(M_PI/6));
const double mAngle2Sin(sin(5*M_PI/6));
const double mAngle3Sin(sin(3*M_PI/2));
const double rad2rpm =  60.0  / (2.0*M_PI);

void messageCb1( const geometry_msgs::Twist& msg){
    motorA = mAngle1Cos*msg.linear.y + mAngle1Sin*msg.linear.x + ROBOT_RADIUS*msg.angular.z*(-1);
    motorB = mAngle2Cos*msg.linear.y + mAngle2Sin*msg.linear.x + ROBOT_RADIUS*msg.angular.z*(-1);
    motorC = mAngle3Cos*msg.linear.y + mAngle3Sin*msg.linear.x + ROBOT_RADIUS*msg.angular.z*(-1);
}
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> three_motor("/turtle1/cmd_vel", &messageCb1);
void setup() {
  Serial.begin(57600);
  pinMode(stepPin1, OUTPUT); 
  pinMode(directionPin1, OUTPUT); 
  pinMode(stepPin2, OUTPUT); 
  pinMode(directionPin2, OUTPUT); 
  pinMode(stepPin3, OUTPUT); 
  pinMode(directionPin3, OUTPUT); 
  pinMode(ENA, OUTPUT); 
  pinMode(13, OUTPUT); 
  digitalWrite(ENA, HIGH);
  nh.initNode();
  nh.subscribe(three_motor);
}
unsigned long start = 0;
void stepmotor(){
  start++;
  int time_pluse_A,time_pluse_B,time_pluse_C;
  time_pluse_A=100/abs(motorA);
  time_pluse_B=100/abs(motorB);
  time_pluse_C=100/abs(motorC);

  if(motorA!=0 && start%time_pluse_A==0){
  if(motorA>0){
  analogWrite(directionPin1, 255);
  }else{
  analogWrite(directionPin1, 0);
  }
//  Serial.println(time_pluse_A);
  digitalWrite(stepPin1, HIGH);
   delayMicroseconds(100);
  digitalWrite(stepPin1, LOW);
  delayMicroseconds(100);
    digitalWrite(stepPin1, HIGH);
     delayMicroseconds(100);
  digitalWrite(stepPin1, LOW);
  delayMicroseconds(100);
    digitalWrite(stepPin1, HIGH);
     delayMicroseconds(100);
  digitalWrite(stepPin1, LOW);
  delayMicroseconds(100);
    digitalWrite(stepPin1, HIGH);
     delayMicroseconds(100);
  digitalWrite(stepPin1, LOW);
   delayMicroseconds(100);

  }
  if(motorB!=0 && start%time_pluse_B==0){
  if(motorB>0){
  analogWrite(directionPin2, 255);
  }else{
  analogWrite(directionPin2, 0);
  }
//  Serial.println(time_pluse_B);
  digitalWrite(stepPin2, HIGH);
   delayMicroseconds(100);
  digitalWrite(stepPin2, LOW);
  delayMicroseconds(100);
    digitalWrite(stepPin2, HIGH);
     delayMicroseconds(100);
  digitalWrite(stepPin2, LOW);
  delayMicroseconds(100);
    digitalWrite(stepPin2, HIGH);
     delayMicroseconds(100);
  digitalWrite(stepPin2, LOW);
  delayMicroseconds(100);
    digitalWrite(stepPin2, HIGH);
     delayMicroseconds(100);
  digitalWrite(stepPin2, LOW);
  delayMicroseconds(100);
  }
  if(motorC!=0 && start%time_pluse_C==0){
  if(motorC>0){
  analogWrite(directionPin3, 255);
  }else{
  analogWrite(directionPin3, 0);
  }
//  Serial.println(time_pluse_C);
  digitalWrite(stepPin3, HIGH);
   delayMicroseconds(100);
  digitalWrite(stepPin3, LOW);
  delayMicroseconds(100);
    digitalWrite(stepPin3, HIGH);
  delayMicroseconds(100);
  digitalWrite(stepPin3, LOW);
  delayMicroseconds(100);
    digitalWrite(stepPin3, HIGH);
     delayMicroseconds(100);
  digitalWrite(stepPin3, LOW);
  delayMicroseconds(100);
    digitalWrite(stepPin3, HIGH);
  delayMicroseconds(100);
  digitalWrite(stepPin3, LOW);
  delayMicroseconds(100);
  
  }
}


void loop() {
  stepmotor();
 nh.spinOnce();

//  while(Serial.available()){
//     digitalWrite(ENA, LOW);
//  }
}
