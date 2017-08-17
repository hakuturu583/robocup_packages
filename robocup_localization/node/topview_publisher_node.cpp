//headers in robocup_localization package
#include <topview_publisher.h>

//headers for ROS
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "topview_publisher_node");
  topview_publisher topview_pub;
  ros::spin();
  return 0;
}
