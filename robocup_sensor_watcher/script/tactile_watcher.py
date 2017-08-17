#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from naoqi_bridge_msgs.msg import Bumper
from robocup_msgs.msg import Event

class watcher():
    def __init__(self):
        self.event_msg = Event()
        self.event_msg.node_info = rospy.get_name()
        self.event_msg.type = self.event_msg.TACTILE
        self.event_msg.level = self.event_msg.NORMAL
        self.event_pub = rospy.Publisher("/event",Event,queue_size=10)
        self.bumper_sub = rospy.Subscriber("bumper",Bumper,self.bumper_callback)
        self.foot_contact_sub = rospy.Subscriber("foot_contact",Bool,self.foot_contact_callback)

    def foot_contact_callback(self,data):
        if data.data == False:
            self.event_msg.stamp = rospy.Time.now()
            self.event_msg.id = self.event_msg.FOOT_LEAVE_GROUND
            self.event_msg.event_name = "FOOT_LEAVE_GROUND"
            self.event_pub.publish(self.event_msg)

    def bumper_callback(self,data):
        self.event_msg.stamp = rospy.Time.now()
        if data.bumper == data.right and data.state == data.stateReleased:
            self.event_msg.id = self.event_msg.RIGHT_BUMPER_RELEASED
            self.event_msg.event_name = "RIGHT_BUMPER_RELEASED"
            self.event_pub.publish(self.event_msg)
        if data.bumper == data.right and data.state == data.statePressed:
            self.event_msg.id = self.event_msg.RIGHT_BUMPER_PRESSED
            self.event_msg.event_name = "RIGHT_BUMPER_PRESSED"
            self.event_pub.publish(self.event_msg)
        if data.bumper == data.left and data.state == data.stateReleased:
            self.event_msg.id = self.event_msg.LEFT_BUMPER_RELEASED
            self.event_msg.event_name = "LEFT_BUMPER_RELEASED"
            self.event_pub.publish(self.event_msg)
        if data.bumper == data.left and data.state == data.statePressed:
            self.event_msg.id = self.event_msg.LEFT_BUMPER_PRESSED
            self.event_msg.event_name = "LEFT_BUMPER_PRESSED"
            self.event_pub.publish(self.event_msg)

if __name__ == '__main__':
    rospy.init_node('tactile_watcher', anonymous=False)
    imu_watcher = watcher()
    rospy.spin()
