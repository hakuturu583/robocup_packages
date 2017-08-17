//headers in robocup_localization package
#include <topview_publisher.h>

//headers for ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

//headers for opencv and image_transport
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//headers for tf
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

//headers for std libs
#include <math.h>

topview_publisher::topview_publisher() : it(nh)
{
  camera_info_recieved = false;
  tf_buffer = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);
  tf_broadcaster = new tf2_ros::TransformBroadcaster();
  nh.getParam(ros::this_node::getName()+"/camera_topic_name", camera_topic_name);
  nh.getParam(ros::this_node::getName()+"/camera_info_topic_name", camera_info_topic_name);
  nh.getParam(ros::this_node::getName()+"/camera_frame_name", camera_frame_name);
  nh.getParam(ros::this_node::getName()+"/simulated_camera_height", simulated_camera_height);
  nh.getParam(ros::this_node::getName()+"/simulated_camera_frame", simulated_camera_frame);
  image_pub =  it.advertise(ros::this_node::getName()+"/top_view", 1);
  camera_info_sub = nh.subscribe(camera_info_topic_name, 1, &topview_publisher::camera_info_callback, this);
  image_sub = it.subscribe(camera_topic_name, 1, &topview_publisher::image_callback, this);
}

topview_publisher::~topview_publisher()
{
}

void topview_publisher::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  K_matrix = cv::Matx33d(msg->K[0],msg->K[1],msg->K[2],msg->K[3],msg->K[4],msg->K[5],msg->K[6],msg->K[7],msg->K[8]);
  D_vec = cv::Vec4d(msg->D[0],msg->D[1],msg->D[2],msg->D[3]);
}

void topview_publisher::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    ros::Time now = ros::Time::now();
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped = tf_buffer->lookupTransform("base_footprint", camera_frame_name, ros::Time(0));
    geometry_msgs::PoseStamped camera_pose;
    camera_pose.header.stamp = now;
    camera_pose.header.frame_id = camera_frame_name;
    camera_pose.pose.position.x = 0;
    camera_pose.pose.position.y = 0;
    camera_pose.pose.position.z = 0;
    camera_pose.pose.orientation.x = 0;
    camera_pose.pose.orientation.y = 0;
    camera_pose.pose.orientation.z = 0;
    camera_pose.pose.orientation.w = 1;
    tf2::doTransform(camera_pose,camera_pose,transform_stamped);
    tf2::Quaternion q(camera_pose.pose.orientation.x, camera_pose.pose.orientation.y, camera_pose.pose.orientation.z, camera_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    cv_ptr->image = create_top_view(cv_ptr->image,pitch,camera_pose.pose.position.z);
    image_pub.publish(cv_ptr->toImageMsg());
    //ROS_WARN_STREAM("R="<<roll<<",P="<<pitch<<",Y="<<yaw);
    geometry_msgs::TransformStamped transform_stamped_from_base_footprint_to_simulated_camera;
    transform_stamped_from_base_footprint_to_simulated_camera.header.stamp = now;
    transform_stamped_from_base_footprint_to_simulated_camera.header.frame_id = "base_footprint";
    transform_stamped_from_base_footprint_to_simulated_camera.child_frame_id = simulated_camera_frame;
    transform_stamped_from_base_footprint_to_simulated_camera.transform.translation.x = camera_pose.pose.position.x;
    transform_stamped_from_base_footprint_to_simulated_camera.transform.translation.y = camera_pose.pose.position.y;
    transform_stamped_from_base_footprint_to_simulated_camera.transform.translation.z = simulated_camera_height;
    tf2::Quaternion quat;
    quat.setRPY(0, M_PI/2, 0);
    transform_stamped_from_base_footprint_to_simulated_camera.transform.rotation.x = quat.x();
    transform_stamped_from_base_footprint_to_simulated_camera.transform.rotation.y = quat.y();
    transform_stamped_from_base_footprint_to_simulated_camera.transform.rotation.z = quat.z();
    transform_stamped_from_base_footprint_to_simulated_camera.transform.rotation.w = quat.w();
    tf_broadcaster->sendTransform(transform_stamped_from_base_footprint_to_simulated_camera);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

cv::Mat topview_publisher::create_top_view(cv::Mat src,double theta,double height)
{
  cv::Mat undistorted_image = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
  cv::undistort(src, undistorted_image, K_matrix, D_vec);
  cv::Mat top_image = cv::Mat::zeros(undistorted_image.rows, undistorted_image.cols, CV_8UC3);
  double Hvc = simulated_camera_height;
  double Hc = height;
  double Dvc = 0.0;
  double f = K_matrix(0,0);
  double fp = f;
  double s = sin(theta);
  double c = cos(theta);
  int cx = undistorted_image.cols/2;
  int cy = undistorted_image.rows;
  int cxp = undistorted_image.cols/2;
  int cyp = undistorted_image.rows;
  for (int y = 0; y < top_image.rows; y++)
  {
    for (int x = 0; x < top_image.cols; x++)
    {
      int xOrg = x - cx;
      int yOrg = - y + cy;
      double oldX = 0.5 + (Hvc / Hc) * (f / fp) * c * ( s/c - (yOrg*Hvc*s - fp*Hc*c + fp*Dvc*s) / (fp*Hc*s + Hvc*yOrg*c + fp*Dvc*c) ) * xOrg;
      double oldY = 0.5 + f * ((yOrg*Hvc*s - fp*Hc*c + fp*Dvc*s)/(fp*Hc*s + Hvc*yOrg*c + fp*Dvc*c));
      oldX = oldX + cxp;
      oldY = -oldY + cyp;
      if (oldX < 0 || top_image.cols - 1 < oldX || oldY < 0 || top_image.rows - 1 < oldY )
      {
        continue;
      }
      if((int)oldX + 1 >= top_image.cols || (int)oldY + 1 >= top_image.rows)
      {
        top_image.data[(y * top_image.cols + x) * top_image.channels()] = undistorted_image.data[((int)oldY * top_image.cols + (int)oldX) * top_image.channels()];
        top_image.data[(y * top_image.cols + x) * top_image.channels() + 1] = undistorted_image.data[((int)oldY * top_image.cols + (int)oldX) * top_image.channels() + 1];
        top_image.data[(y * top_image.cols + x) * top_image.channels() + 2] = undistorted_image.data[((int)oldY * top_image.cols + (int)oldX) * top_image.channels() + 2];
        continue;
      }
      for (int i = 0; i < top_image.channels(); i++)
      {
        uchar f11 = undistorted_image.data[((int)oldY * top_image.cols + (int)oldX) * top_image.channels() + i];
        uchar f12 = undistorted_image.data[(((int)oldY + 1) * top_image.cols + (int)oldX) * top_image.channels() + i];
        uchar f21 = undistorted_image.data[((int)oldY * top_image.cols + (int)oldX + 1) * top_image.channels() + i];
        uchar f22 = undistorted_image.data[(((int)oldY + 1) * top_image.cols + (int)oldX + 1) * top_image.channels() + i];
        double dx2 = (int)oldX + 1 - oldX;
        double dx1 = oldX - (int)oldX;
        double dy2 = (int)oldY + 1 - oldY;
        double dy1 = oldY - (int)oldY;
        top_image.data[(y * top_image.cols + x) * top_image.channels() + i] = dy2 * (f11 * dx2 + f21 * dx1) + dy1 * (f12 * dx2 + f22 * dx1);
      }
    }
  }
  return top_image;
}
