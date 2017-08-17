#!/usr/bin/env python
import os
import rospy
from naoqi import *
from geometry_msgs.msg import PointStamped

class zmp_publisher():
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.nao_ip = rospy.get_param('/nao_ip')
        self.roscore_ip = os.environ.get('ROS_MASTER_URI')
        self.roscore_ip = self.roscore_ip.split('http://')[1]
        self.roscore_ip = self.roscore_ip.split(':')[0]
        self.broker = ALBroker("pythonBroker",self.roscore_ip,9999,self.nao_ip,9559)
        self.motionProx = ALProxy("ALMotion")
        self.memoryProx = ALProxy("ALMemory")
        self.useSensors = True
        self.com_msg = PointStamped()
        self.com_pub = rospy.Publisher("/center_of_mass",PointStamped,queue_size=10)
        self.key_list = ["Device/SubDeviceList/Face/Led/Red/Right/0Deg/Actuator/Value","Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value",
                        "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value","Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value",
                        "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value","Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value",
                        "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value","Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value"]
        while not rospy.is_shutdown():
            self.publish_com()
            #self.publish_zmp()
            self.rate.sleep()

    def publish_zmp(self):
        #LFsrFL = self.memoryProx.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
        #LFsrFR = self.memoryProx.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
        #LFsrBL = self.memoryProx.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
        #LFsrBR = self.memoryProx.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
        #RFsrFL = self.memoryProx.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
        #RFsrFR = self.memoryProx.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
        #RFsrBL = self.memoryProx.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
        #RFsrBR = self.memoryProx.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
        #rospy.loginfo( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
        #rospy.loginfo( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
        FSR = self.memoryProx.getListData(self.key_list)

    def publish_com(self):
        pos = self.motionProx.getCOM("Body", 0, self.useSensors)
        self.com_msg.header.frame_id = "torso"
        self.com_msg.header.stamp = rospy.Time.now()
        self.com_msg.point.x = pos[0]
        self.com_msg.point.y = pos[1]
        self.com_msg.point.z = pos[2]
        self.com_pub.publish(self.com_msg)

if __name__ == '__main__':
    rospy.init_node('zmp_publisher', anonymous=False)
    zmp_pub = zmp_publisher()
    rospy.spin()
