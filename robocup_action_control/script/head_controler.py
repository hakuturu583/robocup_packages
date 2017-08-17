#!/usr/bin/env python
import numpy as np
import rospy
import roslib
roslib; roslib.load_manifest('robocup_msgs')
from robocup_msgs.msg import HeadControlCommand
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
import tf
import sys,traceback

class head_controler:
    def __init__(self):
        rospy.init_node('head_controler', anonymous=True)
        self.listener = tf.TransformListener()
        self.pub_cmd_head = rospy.Publisher("/joint_angles",JointAnglesWithSpeed,queue_size=10)
        self.pub_target_look = rospy.Publisher(rospy.get_namespace()+"target_look",PointStamped,queue_size=10)
        self.state = "init"
        self.camera_frame = "CameraTop_frame"
        self.cmd_head = JointAnglesWithSpeed()
        self.sub_cmd = rospy.Subscriber(rospy.get_namespace() + "command", HeadControlCommand, self.get_command, queue_size = 10)
        self.is_initialized = False
        self.sub_joint_state = rospy.Subscriber("/joint_states", JointState, self.update_jointstate, queue_size = 10)
        self.target_look = PointStamped()
        self.gain = 0.3
        self.seq = 1

    def run(self):
        if self.is_initialized == True:
            if self.state == "init":
                self.init()
            if self.state == "stop":
                self.stop()
            if self.state == "look_around":
                self.look_around()
            if self.state == "look_at":
                #rospy.logerr("hi")
                self.look_at(self.target_look)
            self.seq = self.seq + 1

    def init(self):
        self.cmd_head.joint_names = ["HeadYaw","HeadPitch"]
        self.cmd_head.joint_angles = [0, 0]
        self.cmd_head.speed = 0.1
        self.pub_cmd_head.publish(self.cmd_head)

    def stop(self):
        self.cmd_head.joint_names = ["HeadYaw","HeadPitch"]
        self.cmd_head.joint_angles = [self.joint_state_HeadYaw, self.joint_state_HeadPitch]
        self.cmd_head.speed = 0.1
        self.pub_cmd_head.publish(self.cmd_head)

    def look_around(self):
        self.cmd_head.joint_names = ["HeadYaw","HeadPitch"]
        target_list = [0, np.pi/8, np.pi/4, np.pi/8, 0, -np.pi/8, -np.pi/4, -np.pi/8]
        target_yaw_value = target_list[(self.seq/20)%len(target_list)]
        self.cmd_head.joint_angles = [target_yaw_value, -0.1]
        self.cmd_head.speed = 0.1
        self.pub_cmd_head.publish(self.cmd_head)

    def look_at(self, target_look):
        try:
            self.pub_target_look.publish(target_look)
            transformed_point = self.listener.transformPoint(self.camera_frame, target_look)
            target_yaw_value = self.joint_state_HeadYaw + transformed_point.point.y * self.gain
            target_pitch_value = self.joint_state_HeadPitch - transformed_point.point.z * self.gain
            self.cmd_head.joint_names = ["HeadYaw","HeadPitch"]
            self.cmd_head.joint_angles = [target_yaw_value, target_pitch_value]
            self.cmd_head.speed = 0.1
            self.pub_cmd_head.publish(self.cmd_head)
        except:
            info = sys.exc_info()
            tbinfo = traceback.format_tb( info[2] )
            print 'Python Error.'.ljust( 80, '=' )
            for tbi in tbinfo:
                print tbi
            print '  %s' % str( info[1] )
            print '\n'.rjust( 80, '=' )

    def get_command(self,data):
        self.state = data.state
        self.target_look = data.target_look
        self.target_look.header.stamp = rospy.Time(0)
        #self.camera_frame = data.camera_frame

    def update_jointstate(self,data):
        self.joint_state = data
        self.joint_state_HeadYaw = data.position[0]
        self.joint_state_HeadPitch = data.position[1]
        self.is_initialized = True


if __name__ == '__main__':
    head_control = head_controler()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        head_control.run()
        r.sleep()
