#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from robocup_msgs.msg import Event

class watcher:
    sonar_count_left = 0
    sonar_count_right = 0
    def __init__(self):
        self.event_msg = Event()
        self.event_msg.node_info = rospy.get_name()
        self.event_msg.type = self.event_msg.SONAR
        self.event_pub = rospy.Publisher("/event",Event,queue_size=10)
        self.sonar_left_sub = rospy.Subscriber("/sonar/left/sonar",Range,self.sonar_left_callback)
        self.sonar_right_sub = rospy.Subscriber("/sonar/right/sonar",Range,self.sonar_right_callback)
    def sonar_right_callback(self,data):
        if data.range < 0.4:
            self.sonar_count_right = self.sonar_count_right + 1
        else:
            self.sonar_count_right = 0
        if self.sonar_count_right >= 30:
            self.event_msg.stamp = rospy.Time.now()
            self.event_msg.id = self.event_msg.SOMETHING_ON_ROBOT_RIGHT
            self.event_msg.level = self.event_msg.WARNING
            self.event_msg.event_name = "SOMETHING_ON_ROBOT_RIGHT"
            self.event_pub.publish(self.event_msg)
        else:
            self.event_msg.stamp = rospy.Time.now()
            self.event_msg.id = self.event_msg.NOTHING_ON_ROBOT_RIGHT
            self.event_msg.level = self.event_msg.NORMAL
            self.event_msg.event_name = "NOTHING_ON_ROBOT_RIGHT"
            self.event_pub.publish(self.event_msg)

    def sonar_left_callback(self,data):
        if data.range < 0.4:
            self.sonar_count_left = self.sonar_count_left + 1
        else:
            self.sonar_count_left = 0
        if self.sonar_count_left >= 30:
            self.event_msg.stamp = rospy.Time.now()
            self.event_msg.id = self.event_msg.SOMETHING_ON_ROBOT_LEFT
            self.event_msg.level = self.event_msg.WARNING
            self.event_msg.event_name = "SOMETHING_ON_ROBOT_LEFT"
            self.event_pub.publish(self.event_msg)
        else:
            self.event_msg.stamp = rospy.Time.now()
            self.event_msg.id = self.event_msg.NOTHING_ON_ROBOT_LEFT
            self.event_msg.level = self.event_msg.NORMAL
            self.event_msg.event_name = "NOTHING_ON_ROBOT_LEFT"
            self.event_pub.publish(self.event_msg)

if __name__ == '__main__':
    rospy.init_node('sonar_watcher', anonymous=True)
    sonar_watcher = watcher()
    rospy.spin()
