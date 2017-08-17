#include <local_map_server.h>

//headers for ROS
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "local_map_server");
  ros::NodeHandle nh;
  local_map_server map_server = local_map_server(nh);
  ros::spin();
  return 0;
}
