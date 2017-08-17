#!/usr/bin/env python
import rospy
from robocup_msgs.msg import RobotLocation
from std_msgs.msg import Empty,Float32

class resetter:
    def __init__(self):
        self.sensor_reset_pub = rospy.Publisher(rospy.get_namespace()+"/sensor_reset",Empty,queue_size=1)
        self.expansion_reset_pub = rospy.Publisher(rospy.get_namespace()+"/expansion_reset",Float32,queue_size=1)
        self.variance_threshold = rospy.get_param(rospy.get_name()+'/variance_threshold', 0.5)
        self.expansion_range = rospy.get_param(rospy.get_name()+'/expansion_range', 1.0)
        self.minimum_reset_duration = rospy.Duration(rospy.get_param(rospy.get_name()+'/minimum_reset_duration', 10))
        self.last_elapsed = rospy.Time.now()
        self.location_sub = rospy.Subscriber(rospy.get_namespace()+"/robot_location",RobotLocation,self.location_callback)
    def location_callback(self,data):
        now = rospy.Time.now()
        if now - self.last_elapsed > self.minimum_reset_duration:
            if data.covariance_matrix[0] < self.variance_threshold:
                msg = Float32()
                msg.data = self.expansion_range
                self.expansion_reset_pub.publish(msg)
                self.last_elapsed = now
                return
            if data.covariance_matrix[3] < self.variance_threshold:
                msg = Float32()
                msg.data = self.expansion_range
                self.expansion_reset_pub.publish(msg)
                self.last_elapsed = now
                return
            if data.maximum_weight < 0.0105:
                msg = Empty()
                self.sensor_reset_pub.publish(msg)
                self.last_elapsed = now
                return

if __name__ == '__main__':
    rospy.init_node('pf_resetter', anonymous=False)
    pf_resetter = resetter()
    rospy.spin()
