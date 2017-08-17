#include <robocup_localization.h>
#include <field_info_loader.h>
#include <robocup_msgs/WhiteLines.h>
#include <robocup_msgs/RobotLocation.h>

#include <math.h>
#include <vector>

//headers for ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//headers fir boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/math/constants/constants.hpp>


//headers for Eigen
#include <Eigen/Core>
#include <Eigen/LU>

localizer::localizer()
{
  control_input = Eigen::Vector2d::Zero();
  is_field_model_recieved = false;
  location.x = 0;
  location.y = 0;
  location.theta = 0;
  location.stamp = ros::Time::now();
  tf_buffer = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);
  tf_broadcaster = new tf2_ros::TransformBroadcaster();
  location_pub = nh.advertise<robocup_msgs::RobotLocation>("robot_location", 1);
  azimuth_pub = nh.advertise<std_msgs::Float32>("azimuth", 1);
  nh.getParam(ros::this_node::getName()+"/num_particles", num_particles);
  nh.getParam(ros::this_node::getName()+"/azimuth_to_goal", azimuth_to_goal);
  nh.getParam(ros::this_node::getName()+"/random_transition_size", random_transition_size);
  create_particle_filter(num_particles,random_transition_size);
  boost::thread publish_transform_thread(boost::bind(&localizer::update_transform, this));
  imu_sub = nh.subscribe("/imu", 1, &localizer::callback_imu, this);
  field_model_sub = nh.subscribe(ros::this_node::getName()+"/field_model", 1, &localizer::callback_field_model, this);
  white_lines_sub = nh.subscribe(ros::this_node::getName()+"/white_lines", 1, &localizer::callback_white_lines, this);
}

void localizer::callback_imu(const sensor_msgs::Imu& msg)
{
  double roll,pitch,yaw;
  double pi = boost::math::constants::pi<double>();
  tf2::Quaternion quat(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  azimuth_now = yaw - azimuth_to_goal;
  if(azimuth_now < 0)
  {
    double i = ceil(azimuth_now/pi);
    if(i != 0)
    {
      azimuth_now = azimuth_now + pi * 2;
    }
  }
  else
  {
    double i = floor(azimuth_now/pi);
    if(i != 0)
    {
      azimuth_now = azimuth_now - pi * 2;
    }
  }
}

void localizer::callback_white_lines(const robocup_msgs::WhiteLines& msg)
{
  detected_lines = msg;
  if(detected_lines.header.frame_id == "base_footprint")
  {
    geometry_msgs::TransformStamped transform_map_base_footprint;
    //transform map frame to base_footprint frame
    try
    {
      transform_map_base_footprint = tf_buffer->lookupTransform(detected_lines.header.frame_id, "base_footprint", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    for(int i = 0; i < detected_lines.circles.size() ;i++)
    {
      geometry_msgs::PointStamped center_point;
      center_point.point.x = detected_lines.circles[i].center.x;
      center_point.point.y = detected_lines.circles[i].center.y;
      center_point.point.z = detected_lines.circles[i].center.z;
      center_point.header = detected_lines.header;
      //trasnform circle
      tf2::doTransform(center_point,center_point,transform_map_base_footprint);
      detected_lines.circles[i].center.x  = center_point.point.x;
      detected_lines.circles[i].center.y = center_point.point.y;
      detected_lines.circles[i].center.z = center_point.point.z;
    }
    for(int i = 0; i < detected_lines.lines.size() ;i++)
    {
      geometry_msgs::PointStamped start_point;
      start_point.point.x = detected_lines.lines[i].start_point.x;
      start_point.point.y = detected_lines.lines[i].start_point.y;
      start_point.point.z = detected_lines.lines[i].start_point.z;
      start_point.header = detected_lines.header;
      //transform straight line(start point)
      tf2::doTransform(start_point,start_point,transform_map_base_footprint);
      detected_lines.lines[i].start_point.x  = start_point.point.x;
      detected_lines.lines[i].start_point.y = start_point.point.y;
      detected_lines.lines[i].start_point.z = start_point.point.z;

      geometry_msgs::PointStamped end_point;
      end_point.point.x = detected_lines.lines[i].end_point.x;
      end_point.point.y = detected_lines.lines[i].end_point.y;
      end_point.point.z = detected_lines.lines[i].end_point.z;
      end_point.header = detected_lines.header;
      //transform straight line(end point)
      tf2::doTransform(end_point,end_point,transform_map_base_footprint);
      detected_lines.lines[i].end_point.x = end_point.point.x;
      detected_lines.lines[i].end_point.y = end_point.point.y;
      detected_lines.lines[i].end_point.z = end_point.point.z;
    }
  }
  //update particle filter state
  pf->control_input = control_input;
  pf->predictionNextState();
  calculate_weights();
  pf->updateWeight(weights);
  pf->resampleParticles();
  pf->updateEstimatedState();
  location = pf->location;
  for(int i = 0; i<4 ;i++)
  {
    location.covariance_matrix[i] = pf->covariance_matrix(i);
  }
  location.maximum_weight = pf->maximum_weight;
  location.theta = azimuth_now;
  location_pub.publish(location);
}

void localizer::callback_field_model(const robocup_msgs::WhiteLines& msg)
{
  is_field_model_recieved = true;
  field_frame = msg.header.frame_id;
  field_model = msg;
}

void localizer::update_transform()
{
  //transform publish rate
  ros::Rate rate = ros::Rate(30);
  //tranasorm from base_footprint frame to map frame
  geometry_msgs::TransformStamped transform_stamped;
  while (ros::ok())
  {
    //checking field model
    if(is_field_model_recieved == true)
    {
      std_msgs::Float32 azimuth_msg;
      azimuth_msg.data = location.theta;
      azimuth_pub.publish(azimuth_msg);
      transform_stamped.header.stamp = ros::Time::now();
      transform_stamped.header.frame_id = "base_footprint";
      transform_stamped.child_frame_id = field_frame;
      transform_stamped.transform.translation.x = -location.x;
      transform_stamped.transform.translation.y = -location.y;
      transform_stamped.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, -location.theta);
      transform_stamped.transform.rotation.x = q.x();
      transform_stamped.transform.rotation.y = q.y();
      transform_stamped.transform.rotation.z = q.z();
      transform_stamped.transform.rotation.w = q.w();
      tf_broadcaster->sendTransform(transform_stamped);
    }
    rate.sleep();
  }
}

void localizer::create_particle_filter(int num,double random_transition_size)
{
  weights = Eigen::VectorXd::Zero(num_particles);
  //setting upper and lower
  Eigen::Vector2d upper = Eigen::Vector2d::Zero();
  Eigen::Vector2d lower = Eigen::Vector2d::Zero();
  //-5.0 <= x <= 5.0
  //-3.5 <= x <= 3.5
  upper << 5.0, 3.5;
  lower << -5.0, -3.5;
  pf = new PFilter(num, upper, lower, random_transition_size);
}

double localizer::calculate_likelihood(robocup_msgs::RobotLocation hypothesis)
{
  std::vector<robocup_msgs::Circle> circles = field_model.circles;
  std::vector<robocup_msgs::Circle> detected_circles = detected_lines.circles;
  double circle_result = 1;
  double line_result = 1;
  if(circles.size() != 0 && detected_circles.size() != 0)
  {
    //matrix of matching rate of circle radius.(detection circle and modeled circle)
    Eigen::MatrixXd circle_radius_matching_matrix = Eigen::MatrixXd::Zero(circles.size(),detected_circles.size());
    //matrix of matching rate of circle position.(detection circle and modeled circle)
    Eigen::MatrixXd circle_position_matching_matrix = Eigen::MatrixXd::Zero(circles.size(),detected_circles.size());
    //Todo Use Eigen
    for(int i = 0; i < circles.size() ;i++)
    {
      for(int m = 0; m < detected_circles.size() ;m++)
      {
        circle_radius_matching_matrix(i,m) = circles[i].radius/fabs(detected_circles[m].radius-circles[i].radius);
        double detected_circles_x_map = (detected_circles[m].center.x+hypothesis.x)*cos(hypothesis.theta)-(detected_circles[m].center.y+hypothesis.y)*sin(hypothesis.theta);
        double detected_circles_y_map = (detected_circles[m].center.x+hypothesis.x)*sin(hypothesis.theta)+(detected_circles[m].center.y+hypothesis.y)*cos(hypothesis.theta);
        double position_error = std::sqrt(std::pow(detected_circles_x_map-circles[i].center.x,2)+std::pow(detected_circles_y_map-circles[i].center.y,2));
        circle_position_matching_matrix(i,m) = 1/position_error;
      }
    }
    Eigen::MatrixXd result_matrix = circle_radius_matching_matrix * circle_position_matching_matrix.transpose();
    circle_result = result_matrix.sum();
  }
  std::vector<robocup_msgs::StraightLine> straight_lines = field_model.lines;
  std::vector<robocup_msgs::StraightLine> detected_straight_lines = detected_lines.lines;
  if(straight_lines.size() != 0 && detected_straight_lines.size() != 0)
  {
    //matrix of matching rate of line direction.(detection lines and modeled lines)
    Eigen::MatrixXd line_direction_matching_matrix = Eigen::MatrixXd::Zero(straight_lines.size(),detected_straight_lines.size());
    //matrix of matching rate of nearest point.(detection lines and modeled lines)
    Eigen::MatrixXd nearest_point_matching_matrix = Eigen::MatrixXd::Zero(straight_lines.size(),detected_straight_lines.size());
    //Todo Use Eigen
    for(int i = 0; i < straight_lines.size() ;i++)
    {
      for(int m = 0; m < detected_straight_lines.size() ;m++)
      {
        //calculate nearest point of robot footprint to detected lines.(compare nearest point between modeled lines and detected lines)
        Eigen::Vector2d base_footprint_map;
        base_footprint_map << hypothesis.x,hypothesis.y;

        Eigen::Vector2d start_point;
        double detected_straight_lines_start_point_x_map =
          (detected_straight_lines[m].start_point.x+hypothesis.x)*cos(hypothesis.theta)-(detected_straight_lines[m].start_point.y+hypothesis.y)*sin(hypothesis.theta);
        double detected_straight_lines_start_point_y_map =
          (detected_straight_lines[m].start_point.x+hypothesis.x)*cos(hypothesis.theta)+(detected_straight_lines[m].start_point.y+hypothesis.y)*sin(hypothesis.theta);
        start_point << detected_straight_lines_start_point_x_map,detected_straight_lines_start_point_y_map;

        Eigen::Vector2d end_point;
        double detected_straight_lines_end_point_x_map =
          (detected_straight_lines[m].end_point.x+hypothesis.x)*cos(hypothesis.theta)-(detected_straight_lines[m].end_point.y+hypothesis.y)*sin(hypothesis.theta);
        double detected_straight_lines_end_point_y_map =
          (detected_straight_lines[m].end_point.x+hypothesis.x)*cos(hypothesis.theta)+(detected_straight_lines[m].end_point.y+hypothesis.y)*sin(hypothesis.theta);
        end_point << detected_straight_lines_end_point_x_map,detected_straight_lines_start_point_y_map;

        double theta_detected = atan2(end_point[1]-start_point[1],end_point[0]-start_point[0]);

        Eigen::Vector2d detected_crossing_point = get_nearest_point_2d(base_footprint_map,start_point,end_point);

        Eigen::Vector2d modeled_start_point;
        modeled_start_point << straight_lines[i].start_point.x,straight_lines[i].start_point.y;
        Eigen::Vector2d modeled_end_point;
        modeled_end_point << straight_lines[i].end_point.x,straight_lines[i].end_point.y;
        Eigen::Vector2d origin_point = Eigen::Vector2d::Zero();

        Eigen::Vector2d modeled_crossing_point = get_nearest_point_2d(origin_point,modeled_start_point,modeled_end_point);

        double theta_modeled = atan2(modeled_end_point[1]-modeled_start_point[1],modeled_end_point[0]-modeled_start_point[0]);

        Eigen::Vector2d position_error = modeled_end_point - detected_crossing_point;
        line_direction_matching_matrix(i,m) = 1/fabs(theta_modeled-theta_detected);
        nearest_point_matching_matrix(i,m) = 1/position_error.norm();
      }
    }
    //matrix of matching rate of lines.(detection lines and modeled lines)
    Eigen::MatrixXd result_matrix = line_direction_matching_matrix * nearest_point_matching_matrix.transpose();
    line_result = result_matrix.sum();
  }
  return circle_result*line_result;
}

Eigen::Vector2d localizer::get_nearest_point_2d(Eigen::Vector2d origin_point,Eigen::Vector2d start_point,Eigen::Vector2d end_point)
{
  Eigen::Vector2d crossing_point,detected_line_vec;
  detected_line_vec = end_point - start_point;
  Eigen::Vector2d from_start_point_to_crossing_point_vec = origin_point-start_point;
  from_start_point_to_crossing_point_vec = from_start_point_to_crossing_point_vec.normalized();
  double distance_from_start_point_to_crossing_point = from_start_point_to_crossing_point_vec.dot(detected_line_vec);
  crossing_point = distance_from_start_point_to_crossing_point*from_start_point_to_crossing_point_vec-from_start_point_to_crossing_point_vec;
  return crossing_point;
}

void localizer::calculate_weights()
{
  std::vector<Particle*> particles;
  particles = pf->getParticles();
  for (int i=0; i<num_particles; i++)
  {
    //create particle
    robocup_msgs::RobotLocation hypothesis;
    hypothesis.x = particles[i]->state(0);
    hypothesis.y = particles[i]->state(1);
    hypothesis.theta = azimuth_now;
    weights(i) = calculate_likelihood(hypothesis);
  }
}
