#!/usr/bin/env python
import rospy
from robocup_msgs.msg import Event
from nao_interaction_msgs.msg import AudioSourceLocalization
from naoqi_bridge_msgs.msg import AudioBuffer
from geometry_msgs.msg import PoseStamped,Quaternion
from std_msgs.msg import Float32
import tf

class euler:
    def __init__(self,roll,pitch,yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    def convert_to_quarternion(self):
        q = tf.transformations.quaternion_from_euler(self.roll,self.pitch,self.yaw)
        return q

class watcher:
    def __init__(self):
        self.event_msg = Event()
        self.event_msg.node_info = rospy.get_name()
        self.event_msg.type = self.event_msg.SOUND_SOURCE_DETECTED
        self.event_msg.level = self.event_msg.NORMAL
        self.event_msg.event_name = "SOUND_SOURCE_DETECTED"
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = rospy.get_param("head_frame","Head")
        self.pose_msg.pose.position.x = 0
        self.pose_msg.pose.position.y = 0
        self.pose_msg.pose.position.z = 0
        self.event_pub = rospy.Publisher("/event",Event,queue_size=10)
        self.audio_sub = rospy.Subscriber("audio_raw",AudioBuffer,self.callback_raw)

    def callback_raw(self,data):
        a = 1

if __name__ == '__main__':
    rospy.init_node('chest_button_watcher', anonymous=False)
    hest_button_watcher = watcher()
    rospy.spin()
