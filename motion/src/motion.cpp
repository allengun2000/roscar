/*
 * csll is for serial port (RS232)
 */
 
#include "ros/ros.h"
#include "MotorControl.h"



using std::string;

/*==============================================================================*/
//Topic call back
/*==============================================================================*/
void motionCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
if(msg->linear.x==2)
    mcssl_send2motor(1000);
if(msg->angular.z==2)
    mcssl_send2motor(-1000);
if(msg->angular.z==0 && msg->linear.x==0)
    mcssl_send2motor(0);
}
void PuuCallback(const std_msgs::Int32::ConstPtr& msg)
{
puu_control=msg->data;

mcssl_send2motor(puu_control-PUU_info.data);
}
void MotorSpeedcall(const std_msgs::Int32::ConstPtr& msg)
{
mcssl_send2motor(msg->data);
}
int main(int argc, char **argv)
{
    //Initial
    ros::init(argc, argv, "motion");
    ros::NodeHandle n("~");
    PUU_info.data=0;
    //motion subscriber
    ros::Subscriber motion_sub = n.subscribe("/turtle1/cmd_vel", 1, motionCallback);
    ros::Subscriber motion_sub1 = n.subscribe("/Puu_control", 1, PuuCallback);
    ros::Subscriber motion_sub3 = n.subscribe("/MotorSpeed", 1, MotorSpeedcall);
    ros::Publisher feedback_pub  = n.advertise<std_msgs::Int32>("/speedFB",0);
    ros::Publisher feedback_pub1  = n.advertise<std_msgs::Float32>("/MotorFB",0);
    rpm_info.data=0;
        do{if(mcssl_init() > 0){
        break;
        }else{
          usleep(1000000);//1s = 1,000,000 us
        }}while(ros::ok());
    ROS_INFO("Motion is running\n");
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        motionfeedback();
        feedback_pub.publish(rpm_info);
        Puufeedback();
        feedback_pub1.publish(PUU_info);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //RS232 finish
    printf("Motion is stop\n");
    mcssl_finish();
    return 0;
}
