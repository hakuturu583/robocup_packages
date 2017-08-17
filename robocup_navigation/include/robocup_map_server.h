#ifndef ROBOCUP_MAPPER_H_INCLUDED
#define ROBOCUP_MAPPER_H_INCLUDED
//headers for standard libraries
#include <vector>

//headers for ROS
#include <ros/ros.h>
#include <robocup_msgs/RobocupObject.h>
#include <robocup_msgs/RobotLocation.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//headers for dynamic parmeters
#include <dynamic_reconfigure/server.h>
#include <robocup_navigation/robocup_map_server_dynamic_paramsConfig.h>

class robocup_map_server
{
private:
  //<<member functions>>
  void object_callback_top(const robocup_msgs::RobocupObject::ConstPtr& msg);
  void object_callback_bottom(const robocup_msgs::RobocupObject::ConstPtr& msg);
  void location_callback(const robocup_msgs::RobotLocation& msg);
  void config_callback(robocup_navigation::robocup_map_server_dynamic_paramsConfig &config, uint32_t level);
  void update_map();
  int get_cell_value(int m,int i);
  int transform_object_points();
  //<<member values>>
  //ros
  ros::NodeHandle nh;
  ros::Subscriber camera_top_object_sub;
  ros::Subscriber camera_bottom_object_sub,location_sub;
  ros::Publisher map_pub,object_pub;
  //tf
  tf2_ros::Buffer* tfBuffer;
  tf2_ros::TransformListener* tfListener;
  //parameters for dynamic reconfigure
  dynamic_reconfigure::Server<robocup_navigation::robocup_map_server_dynamic_paramsConfig>* param_server;
  dynamic_reconfigure::Server<robocup_navigation::robocup_map_server_dynamic_paramsConfig>::CallbackType f_type;
  //parametes for mapping
  double mapping_x_length,mapping_y_length;//(m)
  double mapping_resolution;//(m/cell)
  double mapping_margin;//(m)
  robocup_msgs::RobotLocation location;
  robocup_msgs::RobocupObject object_top,object_bottom;
  nav_msgs::OccupancyGrid map;
  std::string object_detector_node_top,object_detector_node_bottom;
public:
  robocup_map_server(ros::NodeHandle nh);
};
#endif
