#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void chatter1_Callback( const sensor_msgs::LaserScan::ConstPtr &msg)
{
  ROS_INFO("laser1_position: %.20f",  msg->ranges[0]);  // ！！！float32[] 表示这是一个数组元素，故ranges后面需要加上[0] ！！！
}

void chatter2_Callback( const sensor_msgs::LaserScan::ConstPtr &msg)
{
  ROS_INFO("laser2_position: %.20f",  msg->ranges[0]);  // ！！！float32[] 表示这是一个数组元素，故ranges后面需要加上[0] ！！！
}

void chatter3_Callback( const sensor_msgs::LaserScan::ConstPtr &msg)
{
  ROS_INFO("laser3_position: %.20f \n",  msg->ranges[0]);  // ！！！float32[] 表示这是一个数组元素，故ranges后面需要加上[0] ！！！
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_info");

  ros::NodeHandle n1, n2, n3;

  ros::Subscriber sub1 = n1.subscribe("/parabot/laser1", 1, chatter1_Callback);
  ros::Subscriber sub2 = n2.subscribe("/parabot/laser2", 1, chatter2_Callback);
  ros::Subscriber sub3 = n3.subscribe("/parabot/laser3", 1, chatter3_Callback);

  ros::spin();

  return 0;
}

