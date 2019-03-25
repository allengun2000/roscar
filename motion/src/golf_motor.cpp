#include "ros/ros.h"
#include "MotorControl.h"
#include "geometry_msgs/Quaternion.h"


using std::string;

/*==============================================================================*/
//Topic call back
/*==============================================================================*/

void cmdCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
puu_control=msg->w;

puu_send2motor(msg->w*1000);
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
