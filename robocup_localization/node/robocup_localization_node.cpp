//headers in robocup_localization package
#include <robocup_localization.h>

//headers for ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//headers for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robocup_localization_node");
  localizer robocup_localizer = localizer();
  ros::spin();
  return 0;
}
