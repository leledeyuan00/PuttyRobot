#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_init"); 
  ros::NodeHandle n;
  ros::Publisher chatter1_pub, chatter2_pub, chatter3_pub;

  chatter1_pub = n.advertise<std_msgs::Float64>("/parabot/joint1_position_controller/command", 1);
  chatter2_pub = n.advertise<std_msgs::Float64>("/parabot/joint2_position_controller/command", 1);
  chatter3_pub = n.advertise<std_msgs::Float64>("/parabot/joint3_position_controller/command", 1);
  ros::Rate loop_rate(10); 
  int count = 0;
  while (ros::ok())
  {
  std_msgs::Float64 msg1;
  std_msgs::Float64 msg2;
  std_msgs::Float64 msg3;

    msg1.data = 0;
    msg2.data = 0;
    msg3.data = 0;

    chatter1_pub.publish(msg1);
    chatter2_pub.publish(msg2);
    chatter3_pub.publish(msg3);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}