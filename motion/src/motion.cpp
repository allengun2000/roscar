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
    mcssl_send2motor(2000);
if(msg->angular.z==2)
    mcssl_send2motor(-1000);
if(msg->angular.z==-2)
    mcssl_send2motor(0);
}


int main(int argc, char **argv)
{
    //Initial
    ros::init(argc, argv, "motion");
    ros::NodeHandle n("~");
    //motion subscriber
    ros::Subscriber motion_sub = n.subscribe("/turtle1/cmd_vel", 1, motionCallback);
    ros::Publisher feedback_pub  = n.advertise<std_msgs::Int32>("/motorFB",0);
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
        ros::spinOnce();
        loop_rate.sleep();
    }
    //RS232 finish
    printf("Motion is stop\n");
    mcssl_finish();
    return 0;
}
