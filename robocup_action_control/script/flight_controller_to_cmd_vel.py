#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class cmd_vel_publisher:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.msg = Twist()
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.callback, queue_size = 1)

    def callback(self,data):
        self.msg.linear.x = data.axes[1]
        self.msg.linear.y = data.axes[0]
        self.msg.angular.z = data.axes[3]
        self.cmd_vel_pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('flight_controller_to_cmd_vel', anonymous=True)
    pub = cmd_vel_publisher()
    rospy.spin()
