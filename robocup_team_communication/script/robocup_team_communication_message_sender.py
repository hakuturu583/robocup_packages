#!/usr/bin/env python
from naoqi import *
import rospy
from robocup_msgs.msg import TeamCommunication

# create python module
class robotCommunicationEstablishedSignalReciever(ALModule):
    def robotCommunicationEstablished(self, strVarName, value):
        team_number = rospy.get_param(rospy.get_name()+'/team_number')
        if value[0] == team_number:
            self.data = ()
            self.communication_sub = rospy.Subscriber('/robocup_team_communication/send_message',TeamCommunication,self.callback_team_communication,queue_size=10)
            self.broker = ALBroker("broker",self.roscore_ip,9559,value[2],9559)
            self.memoryProx = ALProxy("ALMemory")
            rate = rospy.Rate(10)
            while not rospy.is_shutdown:
                self.memoryProx.raiseEvent("TeamCommunicationMessageRecieved", self.data)

    def callback_team_communication(self,data):
        self.data = (data.team_number,data.player_number,data.robot_location.x,data.robot_location.y,data.robot_location.theta,data.playing_state.id,data.penalty_info.penalty,data.penalty_info.secsTillUnpenalised)

class naoqi_communicator:
    def __init__(self):
        # call method
        self.nao_ip = rospy.get_param('/nao_ip')
        self.roscore_ip = rospy.get_param('/roscore_ip')
        self.robotCommunicationEstablishedSignalRecieverModule = robotCommunicationEstablishedSignalReciever("robotCommunicationEstablishedSignalRecieverModule")
        self.memoryProx = ALProxy("ALMemory", self.nao_ip, 9559)
        self.memoryProx.subscribeToEvent("RobotCommunicationEstablished","robotCommunicationEstablishedSignalRecieverModule", "robotCommunicationEstablished") #  event is case sensitive !

if __name__ == '__main__':
    rospy.init_node('robocup_message_sender', anonymous=False)
    communicator = naoqi_communicator()
    rospy.spin()
