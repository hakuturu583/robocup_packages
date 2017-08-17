#include <robocup_map_server.h>

//headers for standard libraries
#include <vector>
#include <iostream>
#include <math.h>

//headers for ROS
#include <ros/ros.h>
#include <robocup_msgs/RobocupObject.h>
#include <robocup_msgs/RobotLocation.h>
#include <robocup_msgs/Robot.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

//headers for dynamic parmeters
#include <dynamic_reconfigure/server.h>
#include <robocup_navigation/robocup_map_server_dynamic_paramsConfig.h>

robocup_map_server::robocup_map_server(ros::NodeHandle nh)
{
  this->nh = nh;
  mapping_x_length = 10;
  mapping_y_length = 7;
  mapping_resolution = 0.05;
  param_server = new dynamic_reconfigure::Server<robocup_navigation::robocup_map_server_dynamic_paramsConfig>();
  f_type = boost::bind(&robocup_map_server::config_callback, this , _1, _2);
  param_server->setCallback(f_type);
  ros::param::param<std::string>("object_detector_node_top", object_detector_node_top,"/object_detector_top");
  ros::param::param<std::string>("object_detector_node_bottom", object_detector_node_bottom,"/object_detector_bottom");
  tfBuffer = new tf2_ros::Buffer();
  tfListener = new tf2_ros::TransformListener(*tfBuffer);

  //writing map info
  map.info.resolution = mapping_resolution;
  map.info.width = (int)mapping_x_length/mapping_resolution;
  map.info.height = (int)mapping_y_length/mapping_resolution;
  map.info.origin.position.x = -mapping_x_length/2;
  map.info.origin.position.y = -mapping_y_length/2;
  map.info.origin.position.z = 0;
  tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,0);
  geometry_msgs::Quaternion quat_Msg;
  quaternionTFToMsg(quat,quat_Msg);
  map.info.origin.orientation = quat_Msg;
  map.header.frame_id = "map";
  //define subscriber and publisher
  map_pub = nh.advertise<nav_msgs::OccupancyGrid>(ros::this_node::getNamespace()+"/navigation_map", 1000);
  camera_top_object_sub = nh.subscribe(ros::this_node::getName()+"/CameraTop/object", 10, &robocup_map_server::object_callback_top, this);
  camera_bottom_object_sub = nh.subscribe(ros::this_node::getName()+"/CameraBottom/object", 10, &robocup_map_server::object_callback_bottom, this);
}

void robocup_map_server::config_callback(robocup_navigation::robocup_map_server_dynamic_paramsConfig &config, uint32_t level)
{
  this->mapping_margin = config.mapping_margin;
}

int robocup_map_server::transform_object_points()
{
  int num_robots_top = object_top.robots.size();
  for(int i=0;i<num_robots_top;i++)
  {
    if("map" != object_top.robots[i].point.header.frame_id)
    {
      geometry_msgs::PointStamped point;
      geometry_msgs::TransformStamped trans;
      try
      {
        trans = tfBuffer->lookupTransform("map", object_top.robots[i].point.header.frame_id,ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        return -1;
      }
      tf2::doTransform(object_top.robots[i].point,object_top.robots[i].point,trans);
    }
  }
  int num_robots_bottom = object_bottom.robots.size();
  for(int i=0;i<num_robots_bottom;i++)
  {
    if("map" != object_bottom.robots[i].point.header.frame_id)
    {
      geometry_msgs::PointStamped point;
      geometry_msgs::TransformStamped trans;
      try
      {
        trans = tfBuffer->lookupTransform("map", object_bottom.robots[i].point.header.frame_id,ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        return -1;
      }
      tf2::doTransform(object_bottom.robots[i].point,object_bottom.robots[i].point,trans);
    }
  }
  return 0;
}

int robocup_map_server::get_cell_value(int m,int i)
{
  double x,y;
  int num_robots_top = object_top.robots.size();
  x = ((map.info.resolution*(double)m)-(mapping_x_length/2)+(map.info.resolution*((double)m+1))-(mapping_x_length/2))/2;
  y = ((map.info.resolution*(double)i)-(mapping_y_length/2)+(map.info.resolution*((double)i+1))-(mapping_y_length/2))/2;
  if(fabs(x) >= (mapping_x_length/2-0.3))
  {
    return 100;
  }
  if(fabs(y) >= (mapping_y_length/2-0.3))
  {
    return 100;
  }
  for(int i=0;i<num_robots_top;i++)
  {
    double r = 0;
    r = sqrt(pow(object_top.robots[i].point.point.x-x,2.0) + pow(object_top.robots[i].point.point.y-y,2.0));
    if(r <= (mapping_margin + object_top.robots[i].radius))
    {
      return 100;
    }
  }
  int num_robots_bottom = object_bottom.robots.size();
  for(int i=0;i<num_robots_bottom;i++)
  {
    double r = 0;
    r = sqrt(pow(object_bottom.robots[i].point.point.x-x,2.0) + pow(object_bottom.robots[i].point.point.y-y,2.0));
    if(r <= (mapping_margin + object_bottom.robots[i].radius))
    {
      return 100;
    }
  }
  return 0;
}

void robocup_map_server::update_map()
{
  {
    //update header
    ros::Time now = ros::Time::now();
    map.info.map_load_time = now;
    map.header.stamp = now;
    //transform camera frame to base_footprint frame
    if(transform_object_points() == 0)
    {
      //clear map data
      map.data.clear();
      for( int i=0; i<map.info.height; i++ )
      {
        for( int m=0; m<map.info.width; m++ )
        {
          //insert each cell value
          map.data.push_back(get_cell_value(m,i));
        }
      }
      map_pub.publish(map);
    }
  }
}

void robocup_map_server::location_callback(const robocup_msgs::RobotLocation& msg)
{
  location = msg;
}

void robocup_map_server::object_callback_top(const robocup_msgs::RobocupObject::ConstPtr& msg)
{
  object_top = *msg;
  update_map();
}

void robocup_map_server::object_callback_bottom(const robocup_msgs::RobocupObject::ConstPtr& msg)
{
  object_bottom = *msg;
  update_map();
}
