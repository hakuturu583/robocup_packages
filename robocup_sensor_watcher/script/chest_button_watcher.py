#!/usr/bin/env python
import rospy
from robocup_msgs.msg import Event,ChestButton

class watcher():
    def __init__(self):
        self.chest_button_sub = rospy.Subscriber("/chest_button",ChestButton,self.chest_button_callback)
        self.event_msg = Event()
        self.event_msg.node_info = rospy.get_name()
        self.event_msg.type = self.event_msg.CHEST_BUTTON
        self.event_msg.level = self.event_msg.COMMAND
        self.event_pub = rospy.Publisher("/event",Event,queue_size=10)
    def chest_button_callback(self,data):
        if data.status == data.PRESSED:
            self.event_msg.id = self.event_msg.CHEST_BUTTON_PRESSED
            self.event_msg.event_name = "CHEST_BUTTON_PRESSED"
            self.event_msg.stamp = rospy.Time.now()
            self.event_pub.publish(self.event_msg)
        if data.status == data.RELEASED:
            self.event_msg.id = self.event_msg.CHEST_BUTTON_RELEASED
            self.event_msg.event_name = "CHEST_BUTTON_RELEASED"
            self.event_msg.stamp = rospy.Time.now()
            self.event_pub.publish(self.event_msg)

if __name__ == '__main__':
    rospy.init_node('chest_button_watcher', anonymous=False)
    hest_button_watcher = watcher()
    rospy.spin()
