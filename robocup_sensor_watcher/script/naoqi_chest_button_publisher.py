#!/usr/bin/env python
from naoqi import *
import time
import rospy
import os
from robocup_msgs.msg import Event,ChestButton

# create python module
class chest_button_event_subscriber(ALModule):
    chest_pub = rospy.Publisher('chest_button',ChestButton,queue_size=10)
    chest_msg = ChestButton()
    def pythondatachanged(self, strVarName, value):
        #print "datachanged", strVarName, " ", value, " ", strMessage
        self.chest_msg.status = value
        self.chest_msg.stamp = rospy.Time.now()
        self.chest_pub.publish(self.chest_msg)

if __name__ == '__main__':
    rospy.init_node('chest_button_subscriber', anonymous=False)
    nao_ip = rospy.get_param(rospy.get_name()+'/nao_ip')
    roscore_ip = os.environ.get('ROS_MASTER_URI')
    roscore_ip = roscore_ip.split('http://')[1]
    roscore_ip = roscore_ip.split(':')[0]
    broker = ALBroker("pythonBroker",roscore_ip,9999,nao_ip,9559)
    # call method
    try:
        pythonModule = chest_button_event_subscriber("pythonModule")
        prox = ALProxy("ALMemory")
        #prox.insertData("val",1) # forbiden, data is optimized and doesn't manage callback
        prox.subscribeToEvent("ChestButtonPressed","pythonModule", "pythondatachanged") #  event is case sensitive !
    except Exception,e:
        rospy.logerr(e)
        exit(1)
    rospy.spin()
