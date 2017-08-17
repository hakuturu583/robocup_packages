#!/usr/bin/env python
from naoqi import *
from robocup_msgs.msg import GameState
import rospy

class game_state_publisher(ALModule):
    game_state_msg = GameState()
    game_controller_pub = rospy.Publisher('game_state',GameState,queue_size=10)
    def gc_data_changed(self, strVarName, value):
        rospy.loginfo("game controller packet number " + str(value) + " recieved")
        self.game_state_msg.game_info.state = prox.getData("gc-bridge/state")
        self.game_state_msg.packet_number = value
        self.game_state_msg.game_info.state = prox.getData("gc-bridge/state")
        self.game_state_msg.game_info.secsRemaining = prox.getData("gc-bridge/secsRemaining")
        self.game_state_msg.game_info.firstHalf = prox.getData("gc-bridge/firstHalf")
        if team_number == prox.getData("gc-bridge/team0_teamNumber"):
            self.game_state_msg.game_info.enemy_score = prox.getData("gc-bridge/team1_score")
            self.game_state_msg.game_info.enemy_teamNumber = prox.getData("gc-bridge/team1_teamNumber")
            self.game_state_msg.game_info.friend_score = prox.getData("gc-bridge/team0_score")
            self.game_state_msg.game_info.friend_teamNumber = prox.getData("gc-bridge/team0_teamNumber")
            key = "gc-bridge/team0_player" + str(player_number) + "_penalty"
            self.game_state_msg.penalty_info.penalty = prox.getData(key)
            key = "gc-bridge/team0_player" + str(player_number) + "_secsTillUnpenalised"
            self.game_state_msg.penalty_info.secsTillUnpenalised = prox.getData(key)

        if team_number == prox.getData("gc-bridge/team1_teamNumber"):
            self.game_state_msg.game_info.enemy_score = prox.getData("gc-bridge/team0_score")
            self.game_state_msg.game_info.enemy_teamNumber = prox.getData("gc-bridge/team0_teamNumber")
            self.game_state_msg.game_info.friend_score = prox.getData("gc-bridge/team1_score")
            self.game_state_msg.game_info.friend_teamNumber = prox.getData("gc-bridge/team1_teamNumber")
            key = "gc-bridge/team1_player" + str(player_number) + "_penalty"
            self.game_state_msg.penalty_info.penalty = prox.getData(key)
            key = "gc-bridge/team1_player" + str(player_number) + "_secsTillUnpenalised"
            self.game_state_msg.penalty_info.secsTillUnpenalised = prox.getData(key)

        self.game_controller_pub.publish(self.game_state_msg)

if __name__ == '__main__':
    rospy.init_node('game_state_publisher', anonymous=False)
    team_number = rospy.get_param(rospy.get_name()+'/team_number')
    player_number = rospy.get_param(rospy.get_name()+'/player_number')
    nao_ip = rospy.get_param(rospy.get_name()+'/nao_ip')
    #self.roscore_ip = rospy.get_param('roscore_ip','127.0.0.1')
    roscore_ip = os.environ.get('ROS_MASTER_URI')
    roscore_ip = roscore_ip.split('http://')[1]
    roscore_ip = roscore_ip.split(':')[0]
    broker = ALBroker("pythonBroker",roscore_ip,9999,nao_ip,9559)
    gc_bridge_Module = game_state_publisher("gc_bridge_Module")
    prox = ALProxy("ALMemory")
    prox.subscribeToEvent("gc-bridge/updatePacket","gc_bridge_Module", "gc_data_changed")
    rospy.spin()
