#ifndef INTERSECT_PLANE_H_INCLUDED
#define INTERSECT_PLANE_H_INCLUDED

//headers for standartd libraries
#include <string.h>

//headers for ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

//headers for Eigen
#include <Eigen/Core>

//headers for dynamic reconfigure
#include <robocup_localization/Params_pointcloud_publisherConfig.h>
#include <dynamic_reconfigure/server.h>

class intersector
{
private:
  // <<members values>>
  std::string plane_frame;
  std::string ray_frame;
  tf::TransformListener listener;
  geometry_msgs::PoseStamped table_pose;
  // <<members functions>>
  void get_rpy(geometry_msgs::Quaternion q,double &roll,double &pitch,double &yaw);
public:
  // <<members values>>
  // <<members functions>>
  intersector(std::string plane_frame, std::string ray_frame);
  bool cast_ray(geometry_msgs::PoseStamped ray_pose, geometry_msgs::PointStamped& crossing_point);
};
#endif
