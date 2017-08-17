#include <robocup_map_server.h>
//headers for ROS
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robocup_map_server");
  ros::NodeHandle nh;
  robocup_map_server map_server = robocup_map_server(nh);
  ros::spin();
  return 0;
}
