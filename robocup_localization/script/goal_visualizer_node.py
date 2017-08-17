#!/usr/bin/env python
import rospy
import tf
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion,Vector3

class goal_marker_publisher:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.define_messages()
        self.friend_goal_marker_pub = rospy.Publisher("friend/goal",Marker,queue_size=1)
        self.enemy_goal_marker_pub = rospy.Publisher("enemy/goal",Marker,queue_size=1)
        self.run()

    def define_messages(self):
        self.friend_goal_marker = Marker()
        self.enemy_goal_marker = Marker()
        self.friend_goal_marker.mesh_resource = "package://robocup_localization/data/friend_goal.dae"
        self.friend_goal_marker.header.frame_id = "map"
        self.friend_goal_marker.frame_locked = True
        self.friend_goal_marker.id = 10
        self.friend_goal_marker.type = self.friend_goal_marker.MESH_RESOURCE
        self.friend_goal_marker.action = self.friend_goal_marker.ADD
        self.friend_goal_marker.mesh_use_embedded_materials = True
        self.friend_goal_marker.color.r = 0
        self.friend_goal_marker.color.g = 0
        self.friend_goal_marker.color.b = 1
        self.friend_goal_marker.color.a = 1
        self.friend_goal_marker.scale.x = 1
        self.friend_goal_marker.scale.y = 1
        self.friend_goal_marker.scale.z = 1
        self.friend_goal_marker.pose.position.x = 4.55
        self.friend_goal_marker.pose.position.y = 0.0
        self.friend_goal_marker.pose.position.z = 0.0
        self.friend_goal_marker.pose.orientation = self.euler_to_quaternion(Vector3(0.0, 0.0, -math.pi/2))

        self.enemy_goal_marker.mesh_resource = "package://robocup_localization/data/enemy_goal.dae"
        self.enemy_goal_marker.header.frame_id = "map"
        self.enemy_goal_marker.frame_locked = True
        self.enemy_goal_marker.id = 10
        self.enemy_goal_marker.type = self.enemy_goal_marker.MESH_RESOURCE
        self.enemy_goal_marker.action = self.enemy_goal_marker.ADD
        self.enemy_goal_marker.mesh_use_embedded_materials = True
        self.enemy_goal_marker.color.r = 1
        self.enemy_goal_marker.color.g = 0
        self.enemy_goal_marker.color.b = 0
        self.enemy_goal_marker.color.a = 1
        self.enemy_goal_marker.scale.x = 1
        self.enemy_goal_marker.scale.y = 1
        self.enemy_goal_marker.scale.z = 1
        self.enemy_goal_marker.pose.position.x = -4.55
        self.enemy_goal_marker.pose.position.y = 0.0
        self.enemy_goal_marker.pose.position.z = 0.0
        self.enemy_goal_marker.pose.orientation = self.euler_to_quaternion(Vector3(0.0, 0.0, math.pi/2))

    def publish_messages(self):
        self.friend_goal_marker.header.stamp = rospy.Time.now()
        self.enemy_goal_marker.header.stamp = rospy.Time.now()
        self.friend_goal_marker_pub.publish(self.friend_goal_marker)
        self.enemy_goal_marker_pub.publish(self.enemy_goal_marker)

    def euler_to_quaternion(self,euler):
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def run(self):
        while not rospy.is_shutdown():
            self.publish_messages()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('goal_visualization', anonymous=False)
    marker_pub = goal_marker_publisher()
    rospy.spin()
