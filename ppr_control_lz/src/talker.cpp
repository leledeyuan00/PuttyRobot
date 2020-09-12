
#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>

#include <robot_state_publisher/robot_state_publisher.h>

#include <sstream>

#include <cmath>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ppr_controller_lz");

  ros::NodeHandle n;

  ros::Publisher chatter1_pub = n.advertise<std_msgs::Float64>("/parabot/joint1_position_controller/command", 1);
  ros::Publisher chatter2_pub = n.advertise<std_msgs::Float64>("/parabot/joint2_position_controller/command", 1);
  ros::Publisher chatter3_pub = n.advertise<std_msgs::Float64>("/parabot/joint3_position_controller/command", 1);

  std_msgs::Float64 msg1;
  std_msgs::Float64 msg2;
  std_msgs::Float64 msg3;

  msg1.data = 0;
  msg2.data = 0;
  msg3.data = 0;
  ros::Rate loop_rate(10);

  int i = 0;

  while (ros::ok())
  {

    i++;
    msg1.data = 0.0135 * sin(0.03*i) +  0.0135;
    msg2.data = 0.0135 * sin(0.03*i + 0.785)  + 0.0135;
    msg3.data = 0.0135 * sin(0.03*i + 1.57)  + 0.0135;


    chatter1_pub.publish(msg1);
    chatter2_pub.publish(msg2);
    chatter3_pub.publish(msg3);

    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;

}
