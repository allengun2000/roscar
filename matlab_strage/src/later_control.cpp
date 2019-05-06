#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "later_car_types.h"
#include "rt_nonfinite.h"
#include "later_car.h"
#include "later_car_terminate.h"
#include "later_car_initialize.h"

/* Function Declarations */
static void argInit_1x3_real_T(double result[3]);
static boolean_T argInit_boolean_T();
static double argInit_real_T();
static void main_later_car();

/* Function Definitions */
static void argInit_1x3_real_T(double result[3])
{
  double result_tmp;

  /* Loop over the array to initialize each element. */
  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result_tmp = argInit_real_T();
  result[0] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[1] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[2] = argInit_real_T();
}

static boolean_T argInit_boolean_T()
{
  return false;
}

static double argInit_real_T()
{
  return 0.0;
}

static void main_later_car()
{
  double refPose_tmp[3];
  double b_refPose_tmp[3];
  double c_refPose_tmp[3];
  double steerCmd;

  /* Initialize function 'later_car' input arguments. */
  /* Initialize function input argument 'refPose'. */
  argInit_1x3_real_T(refPose_tmp);

  /* Initialize function input argument 'currPose'. */
  /* Call the entry-point 'later_car'. */
  b_refPose_tmp[0] = refPose_tmp[0];
  c_refPose_tmp[0] = refPose_tmp[0];
  b_refPose_tmp[1] = refPose_tmp[1];
  c_refPose_tmp[1] = refPose_tmp[1];
  b_refPose_tmp[2] = refPose_tmp[2];
  c_refPose_tmp[2] = refPose_tmp[2];
  steerCmd = later_car(b_refPose_tmp, c_refPose_tmp, argInit_real_T(),
                       argInit_boolean_T(), argInit_real_T(), argInit_real_T(),
                       argInit_real_T());
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
 void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc, char *argv[])
{

	ros::init(argc, argv, "later_control");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::Rate loop_rate(10);


	int count = 0;
	while (ros::ok())
	{

  later_car_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_later_car();

  /* Terminate the application.
     You do not need to do this more than one time. */
  later_car_terminate();

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}