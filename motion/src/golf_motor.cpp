#include "ros/ros.h"
#include "MotorControl.h"
#include "geometry_msgs/Pose2D.h"


using std::string;

/*==============================================================================*/
//Topic call back
/*==============================================================================*/

void cmdCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
// puu_control=msg->theta;
// ROS_INFO("I heard: [%d]", int(msg->theta*1513.91667));
puu_send2motor(int(msg->theta*1513.91667));
}

int main(int argc, char **argv)
{
    //Initial
    ros::init(argc, argv, "golf_motion");
    ros::NodeHandle n("~");
    PUU_info.data=0;
    //motion subscriber

    ros::Subscriber motion_sub = n.subscribe("/cmd", 1, cmdCallback);

    
    ros::Publisher feedback_pub1  = n.advertise<std_msgs::Float32>("/wheelFB",0);

    
        do{if(mcssl_init() > 0){
        break;
        }else{
          usleep(1000000);//1s = 1,000,000 us
        }}while(ros::ok());
    ROS_INFO("Motion is running\n");
    ros::Rate loop_rate(30);
    while(ros::ok())
    {

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
