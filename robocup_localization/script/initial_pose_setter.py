#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

class setter:
    def __init__(self):
        self.msg = PoseWithCovarianceStamped()
        self.init_pose_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size=100)
        self.run()

    def run(self):
        rate = rospy.Rate(1)
        self.msg.header.frame_id = "map"
        while not rospy.is_shutdown():
            self.msg.header.stamp = rospy.Time(0)
            self.init_pose_pub.publish(self.msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('initial_pose_setter', anonymous=True)
    initial_pose_setter = setter()
