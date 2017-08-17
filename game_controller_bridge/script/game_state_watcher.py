#!/usr/bin/env python
from naoqi import *
from robocup_msgs.msg import GameState,PenaltyInfo,Event
from std_msgs.msg import String
import rospy

class watcher:
    def __init__(self):
        self.subscribed_state = GameState()
        self.penalty_state_before = PenaltyInfo.PENALTY_NONE
        self.event_msg = Event()
        self.event_msg.node_info = rospy.get_name()
        self.event_msg.type = self.event_msg.COMMAND
        self.event_msg.level = self.event_msg.NORMAL
        self.speech_pub = rospy.Publisher('/nao_robot/speech',String,queue_size=10)
        self.event_pub = rospy.Publisher('/event',Event,queue_size=10)
        self.game_state_sub = rospy.Subscriber('game_state',GameState,self.callback,queue_size=10)
    def callback(self,data):
        self.subscribed_state = data
        if self.is_state_changed() == True and self.subscribed_state.penalty_info.penalty != PenaltyInfo.PENALTY_NONE:
            speech_msg = String()
            speech_msg.data = "penalized"
            self.event_msg.event_name = "ROBOT_PENALIZED"
            self.event_msg.id = self.event_msg.ROBOT_PENALIZED
            self.speech_pub.publish(speech_msg)
            self.event_pub.publish(self.event_msg)
        if self.is_state_changed() == True and self.subscribed_state.penalty_info.penalty == PenaltyInfo.PENALTY_NONE:
            speech_msg = String()
            speech_msg.data = "not penalized"
            self.event_msg.event_name = "ROBOT_NOT_PENALIZED"
            self.event_msg.id = self.event_msg.ROBOT_NOT_PENALIZED
            self.speech_pub.publish(speech_msg)
            self.event_pub.publish(self.event_msg)
        self.penalty_state_before = self.subscribed_state.penalty_info.penalty

    def is_state_changed(self):
        if self.penalty_state_before != self.subscribed_state.penalty_info.penalty:
            return True
        else:
            return False

if __name__ == '__main__':
    rospy.init_node('game_state_watcher', anonymous=False)
    game_state_watcher = watcher()
    rospy.spin()
