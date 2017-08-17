#!/usr/bin/env python
import rospy
from robocup_msgs.msg import Event,Kick

class watcher:
    def __init__(self):
        self.event_msg = Event()
        self.event_msg.node_info = rospy.get_name()
        self.event_msg.type = self.event_msg.MOTION
        self.event_msg.level = self.event_msg.NORMAL
        self.kick_sub = rospy.Subscriber("/motion/kick",Kick,self.kick_callback)
        self.evemt_pub = rospy.Publisher("/event",Event,queue_size=10)

    def kick_callback(self,data):
        if data.result == Kick.FAIL:
            self.event_msg.id = self.event_msg.KICK_FAILED
            self.event_msg.event_name = "KICK_FAILED"
            self.event_msg.stamp = rospy.Time.now()
            self.event_pub.publish(self.event_msg)
        if data.result == Kick.SUCCESS:
            self.event_msg.id = self.event_msg.KICK_SUCCEED
            self.event_msg.event_name = "KICK_SUCCEED"
            self.event_msg.stamp = rospy.Time.now()
            self.event_pub.publish(self.event_msg)

if __name__ == '__main__':
    rospy.init_node('motion_watcher', anonymous=False)
    motion_watcher = watcher()
    rospy.spin()
