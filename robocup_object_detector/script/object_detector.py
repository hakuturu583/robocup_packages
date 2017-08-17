#!/usr/bin/env python
import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from robocup_msgs.msg import RobocupObject,Robot,SoccerBall
import rospkg
import dlib
import rospy
from dynamic_reconfigure.server import Server
from robocup_object_detector.cfg import Params_object_detectorConfig
from intersect_plane import Intersector
import csv_reader
import os
import tf
import bibs_extractor
import ball_detector

class object_detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.robocup_ball_detector = ball_detector.ball_detector()
        self.bibs_extractor = bibs_extractor.bibs_extractor()
        self.server = Server(Params_object_detectorConfig,self.config_callback)
        self.image_sub = rospy.Subscriber(rospy.get_name()+"/image_raw",Image,self.callback)
        self.image_pub = rospy.Publisher(rospy.get_name()+"/image_detected",Image,queue_size=10)
        self.object_pub = rospy.Publisher(rospy.get_name()+"/object",RobocupObject,queue_size=10)

    def config_callback(self,config,level):
        self.robocup_ball_detector.set_param(config)
        self.bibs_extractor.set_param(config)
        return config

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        robocup_object_msg = RobocupObject()
        robocup_object_msg.stamp = rospy.Time.now()
        #robocup_object_msg.stamp = data.header.stamp
        robocup_object_msg.node_info = rospy.get_name()
        #detect ball
        dets_ball = self.robocup_ball_detector.detect(cv_image)
        self.robocup_ball_detector.draw(dets_ball,cv_image)
        ball_msg = self.robocup_ball_detector.draw_3d(dets_ball,cv_image)
        if ball_msg is not None:
            robocup_object_msg.balls = ball_msg
        #detect robot
        self.bibs_extractor.extract(cv_image)
        self.bibs_extractor.bibs_friend.draw(cv_image)
        self.bibs_extractor.bibs_enemy.draw(cv_image)
        friend_robot_msg = self.bibs_extractor.bibs_friend.draw_3d(cv_image)
        enemy_robot_msg = self.bibs_extractor.bibs_enemy.draw_3d(cv_image)
        #merging message
        if friend_robot_msg is not None and enemy_robot_msg is not None:
            robot_msg = friend_robot_msg + enemy_robot_msg
            robocup_object_msg.robots = robot_msg
        if friend_robot_msg is not None and enemy_robot_msg is None:
            robot_msg = friend_robot_msg
            robocup_object_msg.robots = robot_msg
        if friend_robot_msg is None and enemy_robot_msg is not None:
            robot_msg = enemy_robot_msg
            robocup_object_msg.robots = robot_msg
        self.object_pub.publish(robocup_object_msg)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    detector = object_detector()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
