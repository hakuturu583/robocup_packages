#! /usr/bin/env python
import rospy
import actionlib
import robocup_action_control.msg

class KickBall_Action():
    def __init__(self):
        self.server = actionlib.SimpleActionServer('kick_ball', KickBall_Action, self.execute, False)
        self.server.start()
    def execute(self, goal):
        self.server.set_succeeded()
