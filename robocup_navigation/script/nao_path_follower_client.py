#! /usr/bin/env python
import rospy
import actionlib
import naoqi_bridge_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path

class client:
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient('walk_path', naoqi_bridge_msgs.msg.FollowPathAction)
        self.path_sub = rospy.Subscriber("target_path", Path, self.callback)
        self.path_pub = rospy.Publisher("execute_path", Path, queue_size=1)
        self.goal_count = -1
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def callback(self,data):
        #try:
            #self.transform = self.tfBuffer.lookup_transform('map', 'odom', rospy.Time())
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #return
        #for pose_stamped in data.poses:
            #pose_stamped = tf2_geometry_msgs.do_transform_pose(pose_stamped, self.transform)
        #data.header.frame_id = 'odom'
        self.path_pub.publish(data)
        self.goal_count = self.goal_count + 1
        self.action_client.wait_for_server()
        goal = naoqi_bridge_msgs.msg.FollowPathGoal(path=data)
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('follow_path_client_py')
    fp_client = client()
    rospy.spin()
