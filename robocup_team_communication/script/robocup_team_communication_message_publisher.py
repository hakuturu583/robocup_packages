#!/usr/bin/env python
from naoqi import *
import rospy
from robocup_msgs.msg import TeamCommunication

# create python module
class robotCommunicationMessageReciever(ALModule):
    self.team_communication_msg = TeamCommunication()
    self.team_communication_pub = rospy.Publisher("/robocup_team_communication/recieved_message",TeamCommunication,queue_size=10)
    def robotCommunicationMessageRecived(self, strVarName, value):
        self.team_communication_msg.team_number = value[0]
        self.team_communication_msg.player_number = value[1]
        self.team_communication_msg.robot_location.x = value[2]
        self.team_communication_msg.robot_location.y = value[3]
        self.team_communication_msg.robot_location.theta = value[4]
        self.team_communication_msg.playing_state.id = value[5]
        self.team_communication_msg.penalty_info.penalty = value[6]
        self.team_communication_msg.penalty_info.secsTillUnpenalised = value[7]
        self.team_communication_pub.publish(self.team_communication_msg)

class naoqi_communicator:
    def __init__(self):
        # call method
        self.nao_ip = rospy.get_param('/nao_ip')
        self.roscore_ip = rospy.get_param('/roscore_ip')
        self.robotCommunicationMessageRecieverModule = robotCommunicationMessageReciever("robotCommunicationMessageRecieverModule")
        self.memoryProx = ALProxy("ALMemory", self.nao_ip, 9559)
        self.memoryProx.subscribeToEvent("TeamCommunicationMessageRecieved","robotCommunicationMessageRecieverModule", "robotCommunicationMessageRecived") #  event is case sensitive !

if __name__ == '__main__':
    rospy.init_node('robocup_message_sender', anonymous=False)
    communicator = naoqi_communicator()
    rospy.spin()
