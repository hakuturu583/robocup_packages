#ifndef LOCALIZER_H_INCLUDED
#define LOCALIZER_H_INCLUDED

#include <particle_filter.h>
#include <robocup_msgs/WhiteLines.h>
#include <robocup_msgs/RobotLocation.h>

//headers for ROS
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

//headers fir boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

//headers for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

class localizer
{
private:
  ros::NodeHandle nh;
  ros::Subscriber field_model_sub,white_lines_sub,imu_sub;
  ros::Publisher location_pub,azimuth_pub;
  //member values for tf
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;
  tf2_ros::TransformBroadcaster* tf_broadcaster;
  std::string field_frame;
  robocup_msgs::WhiteLines field_model;
  //parameters for particle filters
  Eigen::Vector2d control_input;
  PFilter* pf;
  robocup_msgs::RobotLocation location;
  robocup_msgs::WhiteLines detected_lines;
  Eigen::VectorXd weights;
  int num_particles;
  double random_transition_size;
  double azimuth_now;
  double azimuth_to_goal;
  //flags
  volatile bool is_field_model_recieved;
  void callback_field_model(const robocup_msgs::WhiteLines& msg);
  void callback_white_lines(const robocup_msgs::WhiteLines& msg);
  void callback_imu(const sensor_msgs::Imu& msg);
  void update_transform();
  //member functions for particle filter
  Eigen::Vector2d get_nearest_point_2d(Eigen::Vector2d origin_point,Eigen::Vector2d start_point,Eigen::Vector2d end_point);
  void create_particle_filter(int num, double random_transition_size);
  void calculate_weights();
  double calculate_likelihood(robocup_msgs::RobotLocation hypothesis);
public:
  localizer();
};

#endif
