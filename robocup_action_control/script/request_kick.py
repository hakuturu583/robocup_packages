#!/usr/bin/env python
from robocup_action_control.srv import *
from robocup_msgs.msg import Kick,Event
import rospy
import rospkg
import sys
import threading

class request_kick_server:
    def __init__(self):
        rospy.init_node('request_kick_server')
        s = rospy.Service('request_kick', RequestKick, self.kick)
        rospack = rospkg.RosPack()
        path = rospack.get_path('robocup_action_control')+'/script/motion'
        sys.path.append(path)
        self.nao_ip = rospy.get_param('/nao_ip')
        self.roscore_ip = rospy.get_param('/roscore_ip')
        self.kick_msg = Kick()
        self.is_kicking = False
        self.init_flag()
        self.kick_pub = rospy.Publisher('/motion/kick',Kick,queue_size=10)
        self.event_sub = rospy.Subscriber("event",Event,self.callback)

    def init_flag(self):
        self.is_r_bumper_pressed = False
        self.is_r_bumper_relessed = False
        self.is_l_bumper_pressed = False
        self.is_l_bumper_relessed = False

    def callback(self,data):
        if data.id == Event.RIGHT_BUMPER_PRESSED:
            self.is_r_bumper_pressed = True
        if data.id == Event.RIGHT_BUMPER_RELEASED:
            self.is_r_bumper_relessed = True
        if data.id == Event.LEFT_BUMPER_PRESSED:
            self.is_l_bumper_pressed = True
        if data.id == Event.LEFT_BUMPER_RELEASED:
            self.is_l_bumper_relessed = True

    def check_result(self,leg):
        if leg == "r":
            if self.is_r_bumper_pressed == True and self.is_r_bumper_relessed == True:
                return Kick.SUCCESS
            else:
                return Kick.FAIL
        if leg == "l":
            if self.is_l_bumper_pressed == True and self.is_l_bumper_relessed == True:
                return Kick.SUCCESS
            else:
                return Kick.FAIL

    def kick(self,req):
        import kick_left
        import kick_right
        self.is_kicking = True
        self.init_flag()
        if req.leg == req.LEFT:
            try:
                kick_left.play(self.roscore_ip,self.nao_ip)
                self.kick_msg.leg = Kick.LEFT
                self.kick_msg.result = self.check_result("l")
                self.kick_pub.publish(self.kick_msg)
            except Exception,e:
                rospy.logerr(e)
        if req.leg == req.RIGHT:
            try:
                kick_right.play(self.roscore_ip,self.nao_ip)
                self.kick_msg.leg = Kick.RIGHT
                self.kick_msg.result = self.check_result("r")
                self.kick_pub.publish(self.kick_msg)
            except Exception,e:
                rospy.logerr(e)
        self.is_kicking = False
        self.init_flag()
        return RequestKickResponse(self.kick_msg.result)

if __name__ == '__main__':
    server = request_kick_server()
    rospy.spin()
