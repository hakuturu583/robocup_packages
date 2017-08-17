#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from robocup_msgs.msg import Event
import numpy as np
import tf

class watcher:
    def __init__(self):
        self.event_msg = Event()
        self.event_msg.node_info = rospy.get_name()
        self.event_msg.type = self.event_msg.IMU
        self.event_msg.level = self.event_msg.WARNING
        self.event_msg.event_name = "ROBOT_FALLS"
        self.evemt_pub = rospy.Publisher("/event",Event,queue_size=1)
        self.imu_yaw_pub = rospy.Publisher("imu_yaw",Float32,queue_size=1)
        self.imu_sub = rospy.Subscriber('imu',Imu,self.callback,queue_size=1)

    def callback(self,data):
        euler = tf.transformations.euler_from_quaternion((data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))
        if np.abs(euler[0]) > np.pi / 6:
            self.event_msg.id = self.event_msg.ROBOT_FALLS
            self.evemt_pub.publish(self.event_msg)
            return
        if np.abs(euler[1]) > np.pi / 6:
            self.event_msg.id = self.event_msg.ROBOT_FALLS
            self.evemt_pub.publish(self.event_msg)
            return
        msg = Float32()
        msg.data = euler[2]
        self.imu_yaw_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('imu_watcher', anonymous=False)
    imu_watcher = watcher()
    rospy.spin()
