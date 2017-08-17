#!/usr/bin/env python
from naoqi import *
import rospy
from robocup_msgs.msg import TeamCommunication

class naoqi_communicator:
    def __init__(self,team_number,player_number,nao_ip):
        self.server_ip_list = []
        self.ip_list = []
        self.server_list = []
        self.team_number = team_number
        self.player_number = player_number
        self.nao_ip = nao_ip

    def update_ip_list(self,ip_list):
        self.ip_list = ip_list
        for ip in self.ip_list:
            if ip not in self.server_ip_list:
                memoryProx = ALProxy("ALMemory", ip, 9559)
                data = (self.team_number,self.player_number,self.nao_ip)
                memoryProx.raiseEvent("RobotCommunicationEstablished",data)
                server_ip_list.append(ip)

class handshake_reciever(ALModule):
    ip_list = []
    def recieve(self, strVarName, value, strMessage):
        if data not in ip_list:
            rospy.loginfo("ip_adress:"+value+" recieved")
            ip_list.append(value)

class infrared_handshaker:
    def __init__(self):
        self.nao_ip = rospy.get_param('/nao_ip')
        self.roscore_ip = rospy.get_param('/roscore_ip')
        self.broker = ALBroker("broker",self.roscore_ip,9559,self.nao_ip,9559)
        self.irProx = ALProxy("ALInfrared")
        self.memoryProx = ALProxy("ALMemory")
        reciever_module = handshake_reciever("reciever_module")
        self.memoryProx.subscribeToEvent("InfraRedIpAdressReceived", "reciever_module", "recieve")
        team_number = rospy.get_param(rospy.get_name()+'/team_number')
        player_number = rospy.get_param(rospy.get_name()+'/player_number')
        team_communicator = naoqi_communicator()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.irProx.sendIpAddress(self.nao_ip)
            team_communicator.update_ip_list(reciever_module.ip_list)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('robocup_teamcommunication_handshaker', anonymous=False)
    shaker = infrared_handshaker()
    rospy.spin()
