#!/usr/bin/env python
import rospy
from robocup_msgs.msg import RobocupObject,Event

class watcher:
    def __init__(self):
        self.ball_lost_duration = rospy.Duration(rospy.get_param('ball_lost_duration',3))
        self.ball_was_last_seen = rospy.Time.now()
        self.event_msg = Event()
        self.event_msg.node_info = rospy.get_name()
        self.event_msg.type = self.event_msg.VISION
        self.event_msg.level = self.event_msg.NORMAL
        self.event_pub = rospy.Publisher("/event",Event,queue_size=10)
        self.object_top_sub = rospy.Subscriber("/object_detector_top/object", RobocupObject, self.obj_callback)
        self.object_top_sub = rospy.Subscriber("/object_detector_bottom/object", RobocupObject, self.obj_callback)

    def obj_callback(self,data):
        if len(data.balls) != 0:
            self.event_msg = Event()
            self.event_msg.stamp = rospy.Time.now()
            self.event_msg.id = self.event_msg.BALL_DETECTED
            self.event_msg.event_name = "BALL_DETECTED"
            self.ball_was_last_seen = data.stamp
            self.event_pub.publish(self.event_msg)
        if len(data.robots) != 0:
            self.event_msg = Event()
            self.event_msg.stamp = rospy.Time.now()
            self.event_msg.id = self.event_msg.ROBOT_DETECTED
            self.event_msg.event_name = "ROBOT_DETECTED"
            self.event_pub.publish(self.event_msg)
        if (data.stamp - self.ball_was_last_seen) >= self.ball_lost_duration:
            self.event_msg = Event()
            self.event_msg.stamp = rospy.Time.now()
            self.event_msg.id = self.event_msg.BALL_LOST
            self.event_msg.event_name = "BALL_LOST"
            self.event_pub.publish(self.event_msg)

if __name__ == '__main__':
    rospy.init_node('vision_system_watcher', anonymous=True)
    vision_system_watcher = watcher()
    rospy.spin()
