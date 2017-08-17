//headers for ROS
#include <ros/ros.h>
#include <robocup_msgs/RobotLocation.h>

//headers for tf
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//headers fir boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class world_frame_publisher
{
private:
  tf2_ros::TransformBroadcaster* tf_broadcaster;
  ros::Subscriber location_sub;
  robocup_msgs::RobotLocation robot_location;
  ros::NodeHandle nh;
  void callback_location(const robocup_msgs::RobotLocation& msg);
public:
  void run();
  world_frame_publisher();
  ~world_frame_publisher();
};

world_frame_publisher::world_frame_publisher()
{
  tf_broadcaster = new tf2_ros::TransformBroadcaster();
  run();
  //boost::thread publish_transform_thread(boost::bind(&world_frame_publisher::run, this));
  //location_sub = nh.subscribe(ros::this_node::getName()+"/robot_location", 1, &world_frame_publisher::callback_location, this);
}

world_frame_publisher::~world_frame_publisher()
{

}

void world_frame_publisher::callback_location(const robocup_msgs::RobotLocation& msg)
{

}

void world_frame_publisher::run()
{
  ros::Rate rate = ros::Rate(30);
  geometry_msgs::TransformStamped transform_stamped;
  while (ros::ok())
  {
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "base_footprint";
    transform_stamped.child_frame_id = "world";
    transform_stamped.transform.translation.x = 1.5;
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(transform_stamped);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robocup_localization_node");
  world_frame_publisher publisher = world_frame_publisher();
  ros::spin();
  return 0;
}
