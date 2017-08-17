#ifndef POINTCLOUD_PUBLISHER_H_INCLUDED
#define POINTCLOUD_PUBLISHER_H_INCLUDED

//headers in robocup_localization package
#include <intersect_plane.h>

//headers for standartd libraries
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <vector>

//headers for boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/optional.hpp>

//headers for ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>

//headers for dynamic reconfigure
#include <robocup_localization/Params_pointcloud_publisherConfig.h>
#include <dynamic_reconfigure/server.h>

//headers for opencv
#include <opencv2/core/core.hpp>

//headers for PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class pointcloud_publisher
{
private:
  // <<members values>>
  //parameters for ROS
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher pointcloud_pub,field_pub;
  int seq;
  //parameters for image processing
  cv::Matx33d camera_matrix;
  std::vector<double> distortion_coefficients;
  //parameters for frames
  std::string plane_frame;
  std::string camera_frame;
  //green value threshold (HSV)
  double green_h_th_low;
  double green_h_th_up;
  double green_s_th;
  double green_v_th;
  //minimum and Field Area
  double min_field_area;
  //parameters for view angles
  double view_angle_width;
  double view_angle_height;
  //parameters for pointcloud filters
  double leaf_size;
  double side_length;
  //path
  std::string config_filename;
  std::string config_filepath;
  std::string camera_yaml_path;
  //flags
  bool is_camera_params_loaded;
  bool is_pointcloud_filter_working;
  //intersector
  intersector* Intersector;
  dynamic_reconfigure::Server<robocup_localization::Params_pointcloud_publisherConfig>* param_server;
  dynamic_reconfigure::Server<robocup_localization::Params_pointcloud_publisherConfig>::CallbackType f_type;

  // <<members functions>>
  //callback functions for ROS
  void image_callback(const sensor_msgs::ImageConstPtr& msg);
  void config_callback(robocup_localization::Params_pointcloud_publisherConfig &config, uint32_t leve);
  //other functions
  void load_camera_params();
  //image processing functions
  cv::Mat filter(cv::Mat src);
  //<image processing function> (General-purpos)
  cv::Mat extract_color(cv::Mat src, double h_th_low, double h_th_up, double s_th, double v_th);
  cv::Mat remove_small_area(cv::Mat src, double th_area);
  cv::Mat remove_large_area(cv::Mat src, double th_area);
  //<image processing function> (Particular-purpos)
  cv::Mat extract_green_area(cv::Mat src);
  cv::Mat thin_out_field_proposal_area(cv::Mat src);
  cv::Mat extract_field(cv::Mat src,cv::Mat mask);
  //functions for publishing pointcloud
  void publish_pointcloud(cv::Mat field_image);
  bool add_point(cv::Mat& field_image, geometry_msgs::Point32& point, float& color_b, float& color_g, float& color_r, int x, int y, ros::Time time_stamp);
  void color_filter_single(float& color_b, float& color_g, float& color_r, float th);
  void color_filter(std::vector<float>& color_b, std::vector<float>& color_g, std::vector<float>& color_r, float th);

public:
  // <<members functions>>
  //constructor
  pointcloud_publisher(std::string config_filename);
};

#endif
